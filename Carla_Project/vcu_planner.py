import socket
import json
import time
import math

print("========================================================")
print(" 🧠 L4 独立后台域控: 自动驾驶算法节点 (vcu_planner.py)")
print("========================================================")

# ==========================================
# 1. UDP 通信配置
# ==========================================
UDP_IP = "127.0.0.1"
UDP_PORT_RX = 5000  # 监听从 main_gui.py 发来的 26 项真值
UDP_PORT_TX = 5001  # 向 main_gui.py 发送油门/刹车/转向指令

sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    sock_rx.bind((UDP_IP, UDP_PORT_RX))
    print(f"✅ 域控大脑已上线，正在监听端口 {UDP_PORT_RX} 的底层物理真值...")
except Exception as e:
    print(f"❌ 端口绑定失败，请检查是否被占用: {e}")
    exit(1)

# ==========================================
# 2. 自动驾驶算法参数 (PID 控制器)
# ==========================================
TARGET_SPEED_KMH = 50.0  # 目标巡航速度 50km/h
Kp = 0.05                # 比例系数 (根据车重可微调)

print(f"🎯 算法任务：启动定速巡航 [{TARGET_SPEED_KMH} km/h] + 雷达紧急避障 (AEB)")

# ==========================================
# 3. 核心算法主循环
# ==========================================
try:
    while True:
        # A. 接收 26 项仿真真值
        data, addr = sock_rx.recvfrom(4096)
        telemetry = json.loads(data.decode('utf-8'))

        # B. 提取关键感知数据
        # 1. 获取运动学数据算出绝对车速
        kinematics = telemetry.get("1_刚体运动学 (Rigid Body Kinematics)", {})
        v_xyz = kinematics.get("3_线速度矢量_XYZ_米每秒", [0.0, 0.0, 0.0])
        current_speed_ms = math.sqrt(v_xyz[0]**2 + v_xyz[1]**2 + v_xyz[2]**2)
        current_speed_kmh = current_speed_ms * 3.6

        # 2. 获取雷达障碍物数据 (前方是否有车/墙)
        events = telemetry.get("4_碰撞与接触事件 (Collision & Events)", {})
        radar_targets = events.get("18b_雷达反射目标数", 0)

        # C. 决策与规划算法 (Planner)
        steer_cmd = 0.0
        throttle_cmd = 0.0
        brake_cmd = 0.0
        status_str = "🟢 正常巡航"

        # 逻辑 1：AEB 紧急主动刹车 (如果雷达反射点过多，说明快撞了)
        if radar_targets > 15:
            throttle_cmd = 0.0
            brake_cmd = 1.0  # 一脚踩死
            status_str = "🚨 障碍预警! AEB 紧急介入!"
        
        # 逻辑 2：ACC 自适应巡航 (PID 速度闭环控制)
        else:
            speed_error = TARGET_SPEED_KMH - current_speed_kmh
            
            if speed_error > 0:
                # 还没到目标速度，给油！
                throttle_cmd = min(1.0, speed_error * Kp)
                brake_cmd = 0.0
            else:
                # 超速了，松油门，轻踩刹车！
                throttle_cmd = 0.0
                brake_cmd = min(1.0, -speed_error * Kp * 0.5)

        # 打包控制指令
        control_cmd = {
            "steer": steer_cmd,
            "throttle": throttle_cmd,
            "brake": brake_cmd,
            "reverse": False,
            "hand_brake": False
        }

        # D. 下发控制指令到 Carla 物理执行器
        sock_tx.sendto(json.dumps(control_cmd).encode('utf-8'), (UDP_IP, UDP_PORT_TX))

        # E. 控制台大屏监控
        print(f"📊 车速: {current_speed_kmh:>5.1f}/{TARGET_SPEED_KMH} km/h | 📡 雷达点: {radar_targets:>3} | ⛽ 油门: {throttle_cmd:>4.2f} | 🛑 刹车: {brake_cmd:>4.2f} || {status_str}", end='\r')

except KeyboardInterrupt:
    print("\n🛑 域控制器安全下线，车辆失去自动驾驶能力！")
finally:
    sock_rx.close()
    sock_tx.close()
