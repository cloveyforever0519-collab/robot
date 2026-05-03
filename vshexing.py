import socket
import json
import time
import math
import numpy as np

class LaneChangeController:
    def __init__(self):
        # 目标速度 20km/h
        self.target_speed_orig = 40.0 / 3.6  
        
        # --- 轨迹生成 ---
        self.xr_ref, self.yr_ref, self.thetar_ref, self.kappar_ref = self.generate_snake_path()
        self.x_end = self.xr_ref[-1]
        
        # --- 车辆物理参数 (默认值，稍后自动读取覆盖) ---
        self.L = 2.88           
        self.a = 1.49           
        self.b = 1.39           
        self.Cf = -110000.0      
        self.Cr = -95000.0      
        
        # ✨ 修复点：这里的 max_steer 必须对应轮胎的最大物理偏角(而非方向盘)。GUI里默认是40度。
        self.max_steer_angle = math.radians(40.0) 
        
        # 横向控制参数
        self.kp_ed, self.ki_ed, self.kd_ed = 0.5, 0.05, 0.1
        self.kp_ephi = 0.5
        self.ed_integral = 0.0
        self.prev_ed = 0.0  # 用于计算缺失的 D 项
        
        self.start_pose = None
        self.is_finished = False
        self.last_time = time.time()
        self.count = 0

        # 网络配置
        self.telem_addr = ("127.0.0.1", 5000)
        self.ctrl_addr = ("127.0.0.1", 5001)
        self.telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telem_sock.bind(self.telem_addr)
        self.telem_sock.setblocking(False)
        self.ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        print(f">>> 蛇行轨迹控制器已就绪。轨迹总长: {self.x_end:.1f}m")

    def generate_snake_path(self):
        # (完全保留你的原版生成逻辑)
        xr = np.arange(0, 350, 0.05)
        A = -3.5                   
        omega = 0.0045 * 2 * np.pi 
        x1 = 50.0                 
        L_trans = np.pi / (2 * omega) 
        x2 = x1 + L_trans         
        
        yr = np.zeros_like(xr)
        for i in range(len(xr)):
            x = xr[i]
            if x <= x1:
                yr[i] = 0
            elif x1 < x <= x2:
                s = (x - x1) / L_trans
                taper = 10 * s**3 - 15 * s**4 + 6 * s**5
                yr[i] = taper * A * np.sin(omega * (x - x1))
            else:
                yr[i] = A * np.sin(omega * (x - x1))
        
        thetar = np.zeros_like(xr)
        kappar = np.zeros_like(xr)
        for i in range(1, len(xr) - 1):
            dx = xr[i+1] - xr[i-1]
            dy = yr[i+1] - yr[i-1]
            ddx = xr[i+1] + xr[i-1] - 2*xr[i]
            ddy = yr[i+1] + yr[i-1] - 2*yr[i]
            thetar[i] = math.atan2(yr[i+1] - yr[i], xr[i+1] - xr[i])
            temp = dx**2 + dy**2
            if temp > 1e-9:
                kappar[i] = 4 * (dx * ddy - ddx * dy) / (temp**1.5)
        
        thetar[0], thetar[-1] = thetar[1], thetar[-2]
        kappar[0], kappar[-1] = kappar[1], kappar[-2]
        return xr, yr, thetar, kappar

    def control_step(self):
        if self.is_finished: return
        try:
            raw_data, _ = self.telem_sock.recvfrom(8192)
            telem = json.loads(raw_data.decode())
            
            # --- 数据解析 ---
            kin_node = next(v for k, v in telem.items() if "1_" in k)
            pos_xyz = next(v for k, v in kin_node.items() if "1_" in k)
            att_pry = next(v for k, v in kin_node.items() if "2_" in k)
            vel_xyz = next(v for k, v in kin_node.items() if "3_" in k)
            curr_v = math.sqrt(sum(a**2 for a in vel_xyz))
            
            curr_x, curr_y = pos_xyz[0], pos_xyz[1]
            curr_yaw = math.radians(att_pry[1]) 

            # --- 动态参数获取 ---
            dyn_params = next((v for k, v in telem.items() if "6_" in k), None)
            if dyn_params:
                self.Cf = dyn_params.get("前轮侧偏刚度_Cf", self.Cf)
                self.Cr = dyn_params.get("后轮侧偏刚度_Cr", self.Cr)
                self.L = dyn_params.get("轮距_L", self.L)
                self.a = dyn_params.get("a", self.a)
                self.b = dyn_params.get("b", self.b)
            
        except (BlockingIOError, StopIteration, Exception):
            return

        if self.start_pose is None:
            self.start_pose = {'x': curr_x, 'y': curr_y, 'yaw': curr_yaw}
            print(f"起点锁定: X={curr_x:.2f}, Y={curr_y:.2f}")
            return

        dx_global, dy_global = curr_x - self.start_pose['x'], curr_y - self.start_pose['y']
        phi_0 = self.start_pose['yaw']
        local_x = dx_global * math.cos(phi_0) + dy_global * math.sin(phi_0)
        local_y = -dx_global * math.sin(phi_0) + dy_global * math.cos(phi_0)
        local_yaw = math.atan2(math.sin(curr_yaw - phi_0), math.cos(curr_yaw - phi_0))

        dist_sq = (local_x - self.xr_ref)**2 + (local_y - self.yr_ref)**2
        idx = np.argmin(dist_sq)
        rk, ry, rtheta = self.kappar_ref[idx], self.yr_ref[idx], self.thetar_ref[idx]

        ed = -(local_x - self.xr_ref[idx]) * math.sin(rtheta) + (local_y - ry) * math.cos(rtheta)
        ephi = math.atan2(math.sin(local_yaw - rtheta), math.cos(local_yaw - rtheta))

        # --- 纵向控制 ---
        dist_to_go = self.x_end - local_x
        throttle, brake = 0.0, 0.0
        if dist_to_go < 1.0:
            brake, self.is_finished = 1.0, True
            print("\n>>> 到达蛇行绕桩终点，执行安全停止！")
        else:
            v_err = self.target_speed_orig - curr_v
            throttle = float(np.clip(0.6 * v_err, 0.0, 0.8))
            brake = float(np.clip(-0.6 * v_err, 0.0, 1.0))

        # --- 横向控制 (修复死亡摇摆) ---
        dt = max(time.time() - self.last_time, 0.02)
        self.last_time = time.time()
        
        # 1. 积分与 ✨微分 (找回你遗漏的 D 阻尼项)
        self.ed_integral = np.clip(self.ed_integral + ed * dt, -1.0, 1.0)
        ed_dot = (ed - self.prev_ed) / dt
        self.prev_ed = ed
        
        # 2. 反馈项 (加入 kd_ed，车子不再像弹簧一样晃了！)
        feedback = self.kp_ed * ed + self.ki_ed * self.ed_integral + self.kd_ed * ed_dot + self.kp_ephi * ephi
        
        # 3. 动力学前馈
        v2 = curr_v ** 2
        ku = (self.b / self.Cf) - (self.a / self.Cr)
        steer_ff = rk * (self.L + v2 * ku)
        
        # 4. 总物理弧度
        steer_rad = -feedback + steer_ff
        
        # ✨ 5. 单位换算修复：把真实物理弧度映射成 -1 到 +1 的操纵杆比例！
        steer_ratio = steer_rad / self.max_steer_angle
        final_steer = float(np.clip(steer_ratio, -1.0, 1.0))

        self.ctrl_sock.sendto(json.dumps({
            "throttle": round(throttle, 3),
            "steer": round(final_steer, 3),
            "brake": round(brake, 3)
        }).encode(), self.ctrl_addr)

        self.count += 1
        if self.count % 50 == 0:
            print("-" * 50)
            print(f"进度: {local_x:.1f}/{self.x_end:.0f}m | 速度: {curr_v*3.6:.1f} km/h")
            print(f"横偏(ed): {ed:.3f} m | 航向误差: {math.degrees(ephi):.2f} °")
            print(f"前馈转角: {steer_ff:.3f} rad | 发送指令比例: {final_steer:.3f}")

if __name__ == "__main__":
    controller = LaneChangeController()
    try:
        while not controller.is_finished:
            controller.control_step()
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n用户手动停止控制。")
