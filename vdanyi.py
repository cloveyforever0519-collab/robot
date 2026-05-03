import socket
import json
import time
import math
import numpy as np


class LaneChangeController:
    def __init__(self):
        self.target_speed_orig = 40.0 / 3.6  # 目标 20km/h
        self.xr_ref, self.yr_ref, self.thetar_ref, self.kappar_ref = self.generate_tanh_path()
        self.x_end = self.xr_ref[-1]

        # --- 车辆物理参数 (根据实际车型调整) ---
        self.L = 2.88  # 轴距 (meters)
        self.a = 1.49  # 质心到前轴距离
        self.b = 1.39  # 质心到后轴距离
        self.Cf = -110000.0  # 前轮侧偏刚度 (N/rad)
        self.Cr = -95000.0  # 后轮侧偏刚度 (N/rad)
        self.max_steer_angle = math.radians(450)  # 方向盘最大转角或比例换算系数

        # 控制参数
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

        print(f">>> 控制器已就绪。轨迹终点: {self.x_end:.1f}m")

    def generate_tanh_path(self):
        xr = np.arange(0, 150, 0.05)
        z1 = (2.4 / 30.0) * (xr - 70.0) - 1.2
        yr = (-3.5 / 2.0) * (1 + np.tanh(z1))
        thetar = np.zeros_like(xr)
        kappar = np.zeros_like(xr)
        dx, dy = np.diff(xr), np.diff(yr)
        thetar[:-1] = np.arctan2(dy, dx)
        thetar[-1] = thetar[-2]
        # 简化版曲率计算
        for i in range(1, len(xr) - 1):
            x_d, y_d = xr[i + 1] - xr[i - 1], yr[i + 1] - yr[i - 1]
            x_dd, y_dd = xr[i + 1] + xr[i - 1] - 2 * xr[i], yr[i + 1] + yr[i - 1] - 2 * yr[i]
            kappar[i] = (x_d * y_dd - x_dd * y_d) / ((x_d ** 2 + y_d ** 2) ** 1.5 + 1e-6)
        return xr, yr, thetar, kappar

    def control_step(self):
        if self.is_finished: return
        try:
            raw_data, _ = self.telem_sock.recvfrom(8192)
            telem = json.loads(raw_data.decode())

            # --- 鲁棒性数据解析 (针对你的抓包数据) ---
            # 1. 找到动力学节点 (Key 1)
            kin_node = next(v for k, v in telem.items() if "1_" in k)

            # 2. 提取坐标 (Key 1_1)
            pos_xyz = next(v for k, v in kin_node.items() if "1_" in k)
            # 3. 提取姿态 (Key 1_2)
            att_pry = next(v for k, v in kin_node.items() if "2_" in k)
            # 4. 提取线速度 (Key 1_3) 并计算标量车速 (单位: m/s)
            vel_xyz = next(v for k, v in kin_node.items() if "3_" in k)
            curr_v = math.sqrt(sum(a ** 2 for a in vel_xyz))

            curr_x, curr_y = pos_xyz[0], pos_xyz[1]
            curr_yaw = math.radians(att_pry[1])  # 偏航角

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

        # 记录起点
        if self.start_pose is None:
            self.start_pose = {'x': curr_x, 'y': curr_y, 'yaw': curr_yaw}
            print(f"起点锁定: {curr_x}, {curr_y}")
            return

        # 坐标转换到局部参考系
        dx, dy = curr_x - self.start_pose['x'], curr_y - self.start_pose['y']
        phi_0 = self.start_pose['yaw']
        local_x = dx * math.cos(phi_0) + dy * math.sin(phi_0)
        local_y = -dx * math.sin(phi_0) + dy * math.cos(phi_0)
        local_yaw = math.atan2(math.sin(curr_yaw - phi_0), math.cos(curr_yaw - phi_0))

        # 寻找匹配点
        dist_sq = (local_x - self.xr_ref) ** 2 + (local_y - self.yr_ref) ** 2
        idx = np.argmin(dist_sq)
        rk, ry, rtheta = self.kappar_ref[idx], self.yr_ref[idx], self.thetar_ref[idx]

        # 计算误差
        ed = -(local_x - self.xr_ref[idx]) * math.sin(rtheta) + (local_y - ry) * math.cos(rtheta)
        ephi = math.atan2(math.sin(local_yaw - rtheta), math.cos(local_yaw - rtheta))

        # 纵向控制 (距离终点判断)
        dist_to_go = self.x_end - local_x
        throttle, brake = 0.0, 0.0
        if dist_to_go < 0.5:
            brake, self.is_finished = 1.0, True
        else:
            v_err = self.target_speed_orig - curr_v
            throttle = float(np.clip(0.5 * v_err, 0.0, 0.8))
            brake = float(np.clip(-0.5 * v_err, 0.0, 1.0))

        # 横向控制 (LQR简化版/PID)
        dt = max(time.time() - self.last_time, 0.02)
        self.last_time = time.time()
        self.ed_integral = np.clip(self.ed_integral + ed * dt, -1.0, 1.0)
        # 1. 计算反馈项 (PID)
        feedback = self.kp_ed * ed + self.ki_ed * self.ed_integral + self.kp_ephi * ephi

        # 2. 计算动力学前馈项 (你要求的公式)
        # 注意：此处使用 curr_v 作为 vx
        v2 = curr_v ** 2
        ku = (self.b / self.Cf) - (self.a / self.Cr)
        steer_ff = rk * (self.L + v2 * ku)

        # 3. 总控制输出 (反馈 + 前馈)
        # 注意符号：反馈项通常为负反馈，前馈随曲率方向
        # 我们假设 rk 为正则左转，steer 也要为正则左转，故用 +steer_ff
        steer_rad = -feedback + steer_ff

        # 映射到 -1 到 1 (假设控制器接受比例值)
        # 如果你的仿真器 steer 是弧度，则不需要除以 self.max_steer_angle
        final_steer = float(np.clip(steer_rad, -1.0, 1.0))

        # 发送
        self.ctrl_sock.sendto(json.dumps({
            "throttle": round(throttle, 3),
            "steer": round(final_steer, 3),
            "brake": round(brake, 3)
        }).encode(), self.ctrl_addr)

        self.count += 1
        if self.count % 20 == 0:
            print(f"速度: {curr_v * 3.6:.1f} km/h | 偏差: {ed:.2f}m | X: {local_x:.1f}m")


if __name__ == "__main__":
    controller = LaneChangeController()
    try:
        while not controller.is_finished:
            controller.control_step()
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
