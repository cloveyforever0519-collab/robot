import socket
import json
import time
import math
import numpy as np


class AccelThenDecelController:
    def __init__(self):
        # 参数设置
        self.target_v_max = 60.0 / 3.6  # 最高目标速度 (22.22 m/s)
        self.decel_rate = 0.1  # 减速平滑度 (数值越大减速越快)

        # 状态控制
        self.state = "ACCEL"  # 初始状态：加速
        self.current_target_v = self.target_v_max
        self.is_finished = False

        # 网络配置
        self.telem_addr = ("127.0.0.1", 5000)
        self.ctrl_addr = ("127.0.0.1", 5001)
        self.telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telem_sock.bind(self.telem_addr)
        self.telem_sock.setblocking(False)
        self.ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.count = 0
        print(f">>> 控制器就绪：先加速至 60km/h，随后缓慢减速至 0。")

    def control_step(self):
        if self.is_finished: return

        try:
            raw_data, _ = self.telem_sock.recvfrom(8192)
            telem = json.loads(raw_data.decode())

            # 提取当前实际速度
            kin_node = next(v for k, v in telem.items() if "1_" in k)
            vel_xyz = next(v for k, v in kin_node.items() if "3_" in k)
            curr_v = math.sqrt(sum(a ** 2 for a in vel_xyz))

            # --- 新增：读取对应车辆参数与 main_gui_new.py 匹配 ---
            dyn_params = next((v for k, v in telem.items() if "6_" in k), None)
            if dyn_params:
                self.mass = dyn_params.get("整备质量", getattr(self, 'mass', 1500.0))
            # ---------------------------------------------------

        except (BlockingIOError, StopIteration, Exception):
            return

        throttle, brake = 0.0, 0.0

        # --- 分状态控制逻辑 ---
        if self.state == "ACCEL":
            # 加速阶段
            v_err = self.target_v_max - curr_v
            if v_err > 0.1:
                throttle = float(np.clip(10 * v_err, 0.0, 1.0))
                brake = 0.0
            else:
                # 触达 80km/h，切换状态
                self.state = "DECEL"
                print("\n>>> 已达到 80km/h，开始减速...")

        elif self.state == "DECEL":
            # 减速阶段：目标速度不断下调
            if self.current_target_v > 0:
                self.current_target_v -= self.decel_rate
            else:
                self.current_target_v = 0

            v_err = self.current_target_v - curr_v

            if v_err > 0:
                throttle = 0.0  # 减速阶段原则上不给油
                brake = 0.03  # 依靠发动机制动
            else:
                throttle = 0.0
                # 根据误差给轻微刹车
                brake = float(np.clip(abs(v_err) * 0.6, 0.0, 0.4))

            # 如果目标和实际都接近 0，彻底停止
            if self.current_target_v <= 0 and curr_v < 0.1:
                brake = 1.0
                self.is_finished = True
                print("\n>>> 任务完成：车辆已停止。")

        # 发送控制
        self.ctrl_sock.sendto(json.dumps({
            "throttle": round(throttle, 3),
            "steer": 0.0,
            "brake": round(brake, 3)
        }).encode(), self.ctrl_addr)

        # 打印状态
        self.count += 1
        if self.count % 50 == 0:
            print(f"状态: {self.state} | 速度: {curr_v * 3.6:.1f} km/h | 目标: {self.current_target_v * 3.6:.1f} km/h",
                  end='\r')


if __name__ == "__main__":
    controller = AccelThenDecelController()
    try:
        while not controller.is_finished:
            controller.control_step()
            time.sleep(0.02)

        # 退出前最后补一次刹车指令
        for _ in range(10):
            controller.ctrl_sock.sendto(json.dumps({"throttle": 0, "steer": 0, "brake": 1}).encode(),
                                        controller.ctrl_addr)
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n用户中断。")
