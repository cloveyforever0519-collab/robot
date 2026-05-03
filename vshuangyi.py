import socket
import json
import time
import math
import numpy as np


class LaneChangeController:
    def __init__(self):
        # 目标速度 40km/h
        self.target_speed_orig = 40.0 / 3.6

        # --- 轨迹生成 (替换为双移线路径算法) ---
        self.xr_ref, self.yr_ref, self.thetar_ref, self.kappar_ref = self.generate_double_lane_change_path()
        self.x_end = self.xr_ref[-1]

        # --- 车辆物理参数 (保持不变) ---
        self.L = 2.88  # 轴距 (meters)
        self.a = 1.49  # 质心到前轴距离
        self.b = 1.39  # 质心到后轴距离
        self.Cf = -110000.0  # 前轮侧偏刚度 (N/rad)
        self.Cr = -95000.0  # 后轮侧偏刚度 (N/rad)
        self.max_steer_angle = math.radians(450)  # 方向盘最大转角或比例换算系数

        # 横向控制参数 (保持不变)
        self.kp_ed, self.ki_ed, self.kd_ed = 0.5, 0.05, 0.1
        self.kp_ephi = 1.2
        self.ed_integral = 0.0
        self.prev_ed = 0.0

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

        print(f">>> 双移线轨迹控制器已就绪。轨迹总长: {self.x_end:.1f}m")

    def generate_double_lane_change_path(self):
        """
        核心：双移线轨迹公式实现
        """
        # 1. 基础参数
        xr = np.arange(0, 210.05, 0.05)
        shape = 2.4
        dx1 = 25.0;
        dx2 = 21.95
        dy1 = -3.5;
        dy2 = -3.5
        Xs1 = 70.0;
        Xs2 = 120.0

        # 2. 核心公式计算 yr
        z1 = shape / dx1 * (xr - Xs1) - shape / 2.0
        z2 = shape / dx2 * (xr - Xs2) - shape / 2.0
        yr = (dy1 / 2.0) * (1 + np.tanh(z1)) - (dy2 / 2.0) * (1 + np.tanh(z2))

        # 3. 计算航向角与曲率 (中心差分逻辑)
        thetar = np.zeros_like(xr)
        kappar = np.zeros_like(xr)

        for i in range(1, len(xr) - 1):
            dx = xr[i + 1] - xr[i - 1]
            dy = yr[i + 1] - yr[i - 1]
            ddx = xr[i + 1] + xr[i - 1] - 2 * xr[i]
            ddy = yr[i + 1] + yr[i - 1] - 2 * yr[i]

            # 参考航向角
            thetar[i] = math.atan2(yr[i + 1] - yr[i], xr[i + 1] - xr[i])

            # 参考曲率
            temp = dx ** 2 + dy ** 2
            if temp > 1e-9:
                kappar[i] = 4 * (dx * ddy - ddx * dy) / (temp ** 1.5)

        # 边界补全
        thetar[0], thetar[-1] = thetar[1], thetar[-2]
        kappar[0], kappar[-1] = kappar[1], kappar[-2]

        return xr, yr, thetar, kappar

    def control_step(self):
        if self.is_finished: return
        try:
            raw_data, _ = self.telem_sock.recvfrom(8192)
            telem = json.loads(raw_data.decode())

            # 数据解析
            kin_node = next(v for k, v in telem.items() if "1_" in k)
            pos_xyz = next(v for k, v in kin_node.items() if "1_" in k)
            att_pry = next(v for k, v in kin_node.items() if "2_" in k)
            vel_xyz = next(v for k, v in kin_node.items() if "3_" in k)
            curr_v = math.sqrt(sum(a ** 2 for a in vel_xyz))

            curr_x, curr_y = pos_xyz[0], pos_xyz[1]
            curr_yaw = math.radians(att_pry[1])

            # --- 新增：读取对应车辆参数与 main_gui_new.py 匹配 ---
            dyn_params = next((v for k, v in telem.items() if "6_" in k), None)
            if dyn_params:
                self.Cf = dyn_params.get("前轮侧偏刚度_Cf", self.Cf)
                self.Cr = dyn_params.get("后轮侧偏刚度_Cr", self.Cr)
                self.L = dyn_params.get("轮距_L", self.L)
                self.a = dyn_params.get("a", self.a)
                self.b = dyn_params.get("b", self.b)
            # ---------------------------------------------------

        except (BlockingIOError, StopIteration, Exception):
            return

        # 锁定起点坐标
        if self.start_pose is None:
            self.start_pose = {'x': curr_x, 'y': curr_y, 'yaw': curr_yaw}
            print(f"起点锁定: X={curr_x:.2f}, Y={curr_y:.2f}")
            return

        # 转换到局部参考系
        dx_global, dy_global = curr_x - self.start_pose['x'], curr_y - self.start_pose['y']
        phi_0 = self.start_pose['yaw']
        local_x = dx_global * math.cos(phi_0) + dy_global * math.sin(phi_0)
        local_y = -dx_global * math.sin(phi_0) + dy_global * math.cos(phi_0)
        local_yaw = math.atan2(math.sin(curr_yaw - phi_0), math.cos(curr_yaw - phi_0))

        # 寻找轨迹匹配点
        dist_sq = (local_x - self.xr_ref) ** 2 + (local_y - self.yr_ref) ** 2
        idx = np.argmin(dist_sq)

        rk, ry, rtheta = self.kappar_ref[idx], self.yr_ref[idx], self.thetar_ref[idx]

        # 误差计算
        ed = -(local_x - self.xr_ref[idx]) * math.sin(rtheta) + (local_y - ry) * math.cos(rtheta)
        ephi = math.atan2(math.sin(local_yaw - rtheta), math.cos(local_yaw - rtheta))

        # --- 纵向控制 (目标 40km/h) ---
        dist_to_go = self.x_end - local_x
        throttle, brake = 0.0, 0.0
        if dist_to_go < 1.0:
            brake, self.is_finished = 1.0, True
        else:
            v_err = self.target_speed_orig - curr_v
            throttle = float(np.clip(10 * v_err, 0.0, 0.8))
            brake = float(np.clip(-0.6 * v_err, 0.0, 1.0))

        # --- 横向控制 (PID + 动力学前馈) ---
        dt = max(time.time() - self.last_time, 0.02)
        self.last_time = time.time()
        self.ed_integral = np.clip(self.ed_integral + ed * dt, -1.0, 1.0)

        # 1. 计算反馈项 (PID)
        feedback = self.kp_ed * ed + self.ki_ed * self.ed_integral + self.kp_ephi * ephi

        # 2. 计算动力学前馈项
        v2 = curr_v ** 2
        ku = (self.b / self.Cf) - (self.a / self.Cr)
        steer_ff = rk * (self.L + v2 * ku)

        # 3. 总控制输出
        steer_rad = -feedback + steer_ff
        final_steer = float(np.clip(steer_rad, -1.0, 1.0))

        # 发送控制指令
        self.ctrl_sock.sendto(json.dumps({
            "throttle": round(throttle, 3),
            "steer": round(final_steer, 3),
            "brake": round(brake, 3)
        }).encode(), self.ctrl_addr)

        # 打印状态
        self.count += 1
        if self.count % 50 == 0:
            ephi_deg = math.degrees(ephi)
            print("-" * 50)
            print(f"进度: {local_x:.1f}/{self.x_end:.0f}m | 速度: {curr_v * 3.6:.1f} km/h")
            print(f"横偏(ed): {ed:.3f} m | 航向误差(ephi): {ephi_deg:.2f} °")
            print(f"参考路径Y: {ry:.3f} | 前馈转角: {steer_ff:.3f}")


if __name__ == "__main__":
    controller = LaneChangeController()
    try:
        while not controller.is_finished:
            controller.control_step()
            time.sleep(0.02)

        print(">>> 任务完成，正在锁死刹车...")
        for _ in range(50):
            controller.ctrl_sock.sendto(json.dumps({"throttle": 0.0, "steer": 0.0, "brake": 1.0}).encode(),
                                        controller.ctrl_addr)
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n用户手动停止控制。")
