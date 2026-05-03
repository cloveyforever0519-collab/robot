import socket
import json
import time
import math
import numpy as np


class SpeedAndDistanceController:
    def __init__(self):
        self.target_v = 60.0 / 3.6
        self.stop_distance = 165.0

        self.start_pose = None
        self.is_finished = False

        # 网络配置
        self.telem_addr = ("127.0.0.1", 5000)
        self.ctrl_addr = ("127.0.0.1", 5001)
        self.telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telem_sock.bind(self.telem_addr)
        self.telem_sock.setblocking(False)
        self.ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.count = 0
        print(f">>> 控制器启动：目标 80km/h，满 {self.stop_distance}m 强制停止。")

    def send_control(self, throttle, steer, brake):
        """封装发送指令的方法"""
        msg = json.dumps({
            "throttle": round(float(throttle), 3),
            "steer": round(float(steer), 3),
            "brake": round(float(brake), 3)
        }).encode()
        self.ctrl_sock.sendto(msg, self.ctrl_addr)

    def control_step(self):
        if self.is_finished:
            return

        try:
            raw_data, _ = self.telem_sock.recvfrom(8192)
            telem = json.loads(raw_data.decode())

            kin_node = next(v for k, v in telem.items() if "1_" in k)
            pos_xyz = next(v for k, v in kin_node.items() if "1_" in k)
            curr_x, curr_y = pos_xyz[0], pos_xyz[1]

            att_pry = next(v for k, v in kin_node.items() if "2_" in k)
            curr_yaw = math.radians(att_pry[1])

            vel_xyz = next(v for k, v in kin_node.items() if "3_" in k)
            curr_v = math.sqrt(sum(a ** 2 for a in vel_xyz))

            # --- 新增：读取对应车辆参数与 main_gui_new.py 匹配 ---
            dyn_params = next((v for k, v in telem.items() if "6_" in k), None)
            if dyn_params:
                self.mass = dyn_params.get("整备质量", getattr(self, 'mass', 1500.0))
            # ---------------------------------------------------

            if self.start_pose is None:
                self.start_pose = {'x': curr_x, 'y': curr_y, 'yaw': curr_yaw}
                print(f"起点记录完成。")
                return

        except (BlockingIOError, StopIteration, Exception):
            return

        # 计算进度 (Local X)
        dx = curr_x - self.start_pose['x']
        dy = curr_y - self.start_pose['y']
        phi_0 = self.start_pose['yaw']
        traveled_progress = dx * math.cos(phi_0) + dy * math.sin(phi_0)

        # --- 核心：强制停止逻辑 ---
        if traveled_progress >= self.stop_distance:
            # 1. 立即设置完成标志
            self.is_finished = True
            # 2. 发送强制刹车指令
            self.send_control(0.0, 0.0, 1.0)
            print(f"!!! 到达 {traveled_progress:.2f}m，执行强制停止。")
        else:
            # 正常 PID 纵向控制
            v_err = self.target_v - curr_v
            throttle = np.clip(0.6 * v_err, 0.0, 1.0)
            brake = np.clip(-10 * v_err, 0.0, 1.0)
            self.send_control(throttle, 0.0, brake)

        # 打印信息
        self.count += 1
        if self.count % 50 == 0:
            print(f"进度: {traveled_progress:.1f}/200m | 速度: {curr_v * 3.6:.1f}km/h")


if __name__ == "__main__":
    controller = SpeedAndDistanceController()
    try:
        # 主循环：只要没到距离就一直跑
        while not controller.is_finished:
            controller.control_step()
            time.sleep(0.02)

        # 强制停止后的“锁死”阶段：防止车辆因为惯性溜车或模拟器指令丢失
        print(">>> 任务完成，正在锁死刹车...")
        for _ in range(50):  # 持续发送 1 秒的刹车信号确保稳停
            controller.send_control(0.0, 0.0, 1.0)
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n用户手动停止。")
