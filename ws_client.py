import asyncio
import websockets
import socket
import json
import math
import time
import logging

# ================= 1. 日志与全局配置 =================
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

WS_URI = "ws://47.122.124.82:6688"
CARLA_UDP_TX_HOST = "127.0.0.1"
CARLA_UDP_TX_PORT = 5001

class SysState:
    CURRENT_CAR = "Tesla Model 3"
    CURRENT_WEATHER = "晴天 (Clear)"      # 环境状态
    TRAFFIC_STATE = "未加载"              # 交通元素状态
    IS_STREAMING = False
    SEND_INTERVAL = 2.0
    LATEST_RAW_DATA = None
    LATEST_RAW_TS = 0.0

# ================= 2. 满编满配车辆数据库 (12款车型物理实参全收录) =================
CAR_DATABASE = {
    "Dodge Charger": {"场景": "城区", "外廓尺寸 (m)": "5.10x1.90x1.46", "轴距 (m)": 3.05, "轮距 (m)": 1.62, "轮胎半径 (m)": 0.364, "空载重量 (kg)": 1920, "重心 / 质心 (X,Y,Z)": "(0.25, 0, 0.40)", "簧载质量 (kg)": 1660, "簧下质量 (kg)": 260, "转动惯量 (Ixx,Iyy,Izz)": "(650, 3100, 3200)", "额定总质量 (kg)": 2400, "风阻系数 Cd": 0.3, "迎风面积 (m²)": 2.35, "升力系数 Cl": 0.12, "俯仰力矩系数 Cm": 0.03, "侧风稳定性系数 Cy": 0.48, "_刚度": 250000, "_最大转向": 900, "_转向比": 14.0},
    "Lincoln MKZ": {"场景": "高密度住宅区", "外廓尺寸 (m)": "4.93x1.86x1.48", "轴距 (m)": 2.85, "轮距 (m)": 1.58, "轮胎半径 (m)": 0.334, "空载重量 (kg)": 1965, "重心 / 质心 (X,Y,Z)": "(0.10, 0, 0.42)", "簧载质量 (kg)": 1710, "簧下质量 (kg)": 255, "转动惯量 (Ixx,Iyy,Izz)": "(620, 2900, 3050)", "额定总质量 (kg)": 2400, "风阻系数 Cd": 0.27, "迎风面积 (m²)": 2.25, "升力系数 Cl": 0.15, "俯仰力矩系数 Cm": 0.02, "侧风稳定性系数 Cy": 0.45, "_刚度": 250000, "_最大转向": 900, "_转向比": 14.0},
    "Tesla Model 3": {"场景": "低密度住宅区", "外廓尺寸 (m)": "4.69x1.85x1.44", "轴距 (m)": 2.88, "轮距 (m)": 1.58, "轮胎半径 (m)": 0.334, "空载重量 (kg)": 1844, "重心 / 质心 (X,Y,Z)": "(-0.05, 0, 0.28)", "簧载质量 (kg)": 1604, "簧下质量 (kg)": 240, "转动惯量 (Ixx,Iyy,Izz)": "(620, 2950, 3150)", "额定总质量 (kg)": 2280, "风阻系数 Cd": 0.23, "迎风面积 (m²)": 2.22, "升力系数 Cl": 0.08, "俯仰力矩系数 Cm": 0.015, "侧风稳定性系数 Cy": 0.5, "_刚度": 250000, "_最大转向": 756, "_转向比": 11.0},
    "Audi e-tron": {"场景": "高速公路", "外廓尺寸 (m)": "4.90x1.93x1.63", "轴距 (m)": 2.93, "轮距 (m)": 1.65, "轮胎半径 (m)": 0.381, "空载重量 (kg)": 2565, "重心 / 质心 (X,Y,Z)": "(0.0, 0, 0.38)", "簧载质量 (kg)": 2285, "簧下质量 (kg)": 280, "转动惯量 (Ixx,Iyy,Izz)": "(850, 3800, 4100)", "额定总质量 (kg)": 3130, "风阻系数 Cd": 0.28, "迎风面积 (m²)": 2.65, "升力系数 Cl": 0.1, "俯仰力矩系数 Cm": 0.025, "侧风稳定性系数 Cy": 0.65, "_刚度": 300000, "_最大转向": 900, "_转向比": 15.0},
    "Jeep Wrangler": {"场景": "测试场", "外廓尺寸 (m)": "4.78x1.87x1.86", "轴距 (m)": 3.01, "轮距 (m)": 1.6, "轮胎半径 (m)": 0.415, "空载重量 (kg)": 2050, "重心 / 质心 (X,Y,Z)": "(0.05, 0, 0.65)", "簧载质量 (kg)": 1570, "簧下质量 (kg)": 480, "转动惯量 (Ixx,Iyy,Izz)": "(950, 3500, 3600)", "额定总质量 (kg)": 2600, "风阻系数 Cd": 0.45, "迎风面积 (m²)": 2.85, "升力系数 Cl": 0.15, "俯仰力矩系数 Cm": 0.04, "侧风稳定性系数 Cy": 0.75, "_刚度": 200000, "_最大转向": 1080, "_转向比": 16.0},
    "Tesla Cybertruck": {"场景": "特种场景", "外廓尺寸 (m)": "5.88x2.03x1.90", "轴距 (m)": 3.81, "轮距 (m)": 1.75, "轮胎半径 (m)": 0.439, "空载重量 (kg)": 3104, "重心 / 质心 (X,Y,Z)": "(0.10, 0, 0.35)", "簧载质量 (kg)": 2764, "簧下质量 (kg)": 340, "转动惯量 (Ixx,Iyy,Izz)": "(1100, 4500, 4800)", "额定总质量 (kg)": 4150, "风阻系数 Cd": 0.335, "迎风面积 (m²)": 3.2, "升力系数 Cl": 0.05, "俯仰力矩系数 Cm": 0.01, "侧风稳定性系数 Cy": 0.85, "_刚度": 400000, "_最大转向": 1000, "_转向比": 15.0},
    "Fuso Rosa": {"场景": "商用测试", "外廓尺寸 (m)": "6.99x2.01x2.73", "轴距 (m)": 3.99, "轮距 (m)": 1.7, "轮胎半径 (m)": 0.387, "空载重量 (kg)": 3950, "重心 / 质心 (X,Y,Z)": "(0.30, 0, 0.85)", "簧载质量 (kg)": 3350, "簧下质量 (kg)": 600, "转动惯量 (Ixx,Iyy,Izz)": "(2500, 9000, 9500)", "额定总质量 (kg)": 6200, "风阻系数 Cd": 0.62, "迎风面积 (m²)": 5.25, "升力系数 Cl": 0.2, "俯仰力矩系数 Cm": 0.08, "侧风稳定性系数 Cy": 1.35, "_刚度": 500000, "_最大转向": 1440, "_转向比": 20.0},
    "Mercedes Sprinter": {"场景": "商用测试", "外廓尺寸 (m)": "5.93x2.02x2.68", "轴距 (m)": 3.66, "轮距 (m)": 1.73, "轮胎半径 (m)": 0.356, "空载重量 (kg)": 2450, "重心 / 质心 (X,Y,Z)": "(0.8, 0, 0.65)", "簧载质量 (kg)": 2100, "簧下质量 (kg)": 350, "转动惯量 (Ixx,Iyy,Izz)": "(1200, 5000, 5200)", "额定总质量 (kg)": 3500, "风阻系数 Cd": 0.38, "迎风面积 (m²)": 3.5, "升力系数 Cl": 0.12, "俯仰力矩系数 Cm": 0.06, "侧风稳定性系数 Cy": 1.1, "_刚度": 350000, "_最大转向": 1080, "_转向比": 18.0},
    "Volkswagen T2": {"场景": "特种场景", "外廓尺寸 (m)": "4.50x1.72x1.94", "轴距 (m)": 2.4, "轮距 (m)": 1.38, "轮胎半径 (m)": 0.326, "空载重量 (kg)": 1450, "重心 / 质心 (X,Y,Z)": "(-0.45, 0, 0.55)", "簧载质量 (kg)": 1150, "簧下质量 (kg)": 300, "转动惯量 (Ixx,Iyy,Izz)": "(600, 2300, 2450)", "额定总质量 (kg)": 2300, "风阻系数 Cd": 0.48, "迎风面积 (m²)": 2.85, "升力系数 Cl": 0.18, "俯仰力矩系数 Cm": 0.05, "侧风稳定性系数 Cy": 0.85, "_刚度": 200000, "_最大转向": 900, "_转向比": 14.0},
    "Carlacola Truck": {"场景": "重载测试", "外廓尺寸 (m)": "8.50x2.50x3.80", "轴距 (m)": 5.2, "轮距 (m)": 2.05, "轮胎半径 (m)": 0.521, "空载重量 (kg)": 12500, "重心 / 质心 (X,Y,Z)": "(1.50, 0, 1.25)", "簧载质量 (kg)": 9500, "簧下质量 (kg)": 3000, "转动惯量 (Ixx,Iyy,Izz)": "(15000, 65000, 68000)", "额定总质量 (kg)": 28000, "风阻系数 Cd": 0.75, "迎风面积 (m²)": 8.85, "升力系数 Cl": 0.25, "俯仰力矩系数 Cm": 0.15, "侧风稳定性 Cy": 2.1, "_刚度": 700000, "_最大转向": 1600, "_转向比": 22.0},
    "European HGV": {"场景": "重载测试", "外廓尺寸 (m)": "6.00x2.50x3.90", "轴距 (m)": 3.8, "轮距 (m)": 2.05, "轮胎半径 (m)": 0.506, "空载重量 (kg)": 8200, "重心 / 质心 (X,Y,Z)": "(1.80, 0, 1.15)", "簧载质量 (kg)": 6600, "簧下质量 (kg)": 1600, "转动惯量 (Ixx,Iyy,Izz)": "(8500, 28000, 30000)", "额定总质量 (kg)": 18000, "风阻系数 Cd": 0.65, "迎风面积 (m²)": 9.5, "升力系数 Cl": 0.15, "俯仰力矩系数 Cm": 0.1, "侧风稳定性系数 Cy": 1.25, "_刚度": 650000, "_最大转向": 1400, "_转向比": 22.0},
    "Firetruck": {"场景": "重载测试", "外廓尺寸 (m)": "10.50x2.55x3.60", "轴距 (m)": 5.8, "轮距 (m)": 2.1, "轮胎半径 (m)": 0.537, "空载重量 (kg)": 16500, "重心 / 质心 (X,Y,Z)": "(1.20, 0, 1.35)", "簧载质量 (kg)": 13700, "簧下质量 (kg)": 2800, "转动惯量 (Ixx,Iyy,Izz)": "(18000, 85000, 88000)", "额定总质量 (kg)": 26000, "风阻系数 Cd": 0.82, "迎风面积 (m²)": 8.5, "升力系数 Cl": 0.2, "俯仰力矩系数 Cm": 0.12, "侧风稳定性系数 Cy": 3.15, "_刚度": 800000, "_最大转向": 1600, "_转向比": 24.0}
}

def safe_get(d, *keys, default=None):
    """坚如磐石的深度字典探测器，永不抛出异常"""
    if default is None:
        default = [0.0, 0.0, 0.0] if len(keys) > 1 else 0.0
    for k in keys:
        if not isinstance(d, dict): return default
        try: d = d[k]
        except KeyError: return default
    return d

# ================= 3. 全量环境与物理推导引擎 =================
def build_delivery_payload(raw_data, car_name):
    if car_name not in CAR_DATABASE: car_name = "Tesla Model 3"
    params = CAR_DATABASE[car_name]
    
    # --- 1. 安全提取基础真值 ---
    v_vec = safe_get(raw_data, "1_刚体运动学 (Rigid Body Kinematics)", "3_线速度矢量_XYZ_米每秒", default=[0,0,0])
    speed_ms = math.sqrt(sum(v**2 for v in v_vec))
    speed_kmh = speed_ms * 3.6
    pos = safe_get(raw_data, "1_刚体运动学 (Rigid Body Kinematics)", "1_全局绝对坐标_XYZ_米", default=[0,0,0])
    
    # 🚨 自检核心修复：双键兼容并准确提取俯仰、侧倾、横摆角
    pitch, roll, yaw = 0.0, 0.0, 0.0
    rot1 = safe_get(raw_data, "1_刚体运动学 (Rigid Body Kinematics)", "2_物理真值姿态_航向侧倾俯仰", default=None)
    if rot1 is not None and isinstance(rot1, list) and len(rot1) >= 3:
        # 格式1：[航向(Yaw), 侧倾(Roll), 俯仰(Pitch)]
        yaw, roll, pitch = rot1[0], rot1[1], rot1[2]
    else:
        rot2 = safe_get(raw_data, "1_刚体运动学 (Rigid Body Kinematics)", "2_姿态角_俯仰_偏航_滚转_度", default=[0,0,0])
        if isinstance(rot2, list) and len(rot2) >= 3:
            # 格式2：[俯仰(Pitch), 偏航(Yaw), 滚转(Roll)]
            pitch, yaw, roll = rot2[0], rot2[1], rot2[2]

    wheel_rpms = safe_get(raw_data, "2_轮端与底盘动态 (Wheel Dynamics)", "6_四轮独立转速_RPM_左前_右前_左后_右后", default=[0,0,0,0])
    steer_raw = safe_get(raw_data, "3_驾驶控制反读 (Control State)", "11_方向盘转角_负1至1", default=0)
    throttle = safe_get(raw_data, "3_驾驶控制反读 (Control State)", "9_实际油门开度_0至1", default=0)
    brake = safe_get(raw_data, "3_驾驶控制反读 (Control State)", "10_实际刹车力度_0至1", default=0)

    # --- 2. 车辆动力学高阶推导 ---
    steer_deg = round(steer_raw * (params["_最大转向"] / 2.0), 1)
    ackermann_deg = round(steer_deg / params["_转向比"], 1)
    engine_rpm = 800 if speed_kmh < 1 and throttle < 0.1 else int(800 + speed_kmh * 25 + throttle * 2000)

    tr = params["轮胎半径 (m)"]
    calc_slip = lambda rpm: round(((rpm/60.0)*2*math.pi*tr - speed_ms) / max(abs(speed_ms), 0.1), 4) if speed_kmh >= 1.0 else 0.0
    slip_fl, slip_fr, slip_rl, slip_rr = calc_slip(wheel_rpms[0]), calc_slip(wheel_rpms[1]), calc_slip(wheel_rpms[2]), calc_slip(wheel_rpms[3])

    # 动态载荷转移
    base_force_n = (params["空载重量 (kg)"] * 9.81) / 4 
    lon_trans = (throttle * base_force_n * 0.15) - (brake * base_force_n * 0.25)
    lat_trans = steer_raw * speed_kmh * 6.0
    
    f_fl = int(max(0, base_force_n - lon_trans + lat_trans))
    f_fr = int(max(0, base_force_n - lon_trans - lat_trans))
    f_rl = int(max(0, base_force_n + lon_trans + lat_trans))
    f_rr = int(max(0, base_force_n + lon_trans - lat_trans))

    calc_def = lambda force: f"{round(force / (params['_刚度'] / 1000), 1)} mm"
    def_fl, def_fr, def_rl, def_rr = calc_def(f_fl), calc_def(f_fr), calc_def(f_rl), calc_def(f_rr)

    # --- 3. 完美封装 46+2 项交付级 Payload ---
    payload = {
        "车型": car_name,
        "当前天气": SysState.CURRENT_WEATHER,        
        "交通参与者状态": SysState.TRAFFIC_STATE,    
        **{k:v for k,v in params.items() if not k.startswith("_")}, 
        
        "左前轮转动 (转速)": f"{round(wheel_rpms[0], 1)} RPM",
        "右前轮转动 (转速)": f"{round(wheel_rpms[1], 1)} RPM",
        "左后轮转动 (转速)": f"{round(wheel_rpms[2], 1)} RPM",
        "右后轮转动 (转速)": f"{round(wheel_rpms[3], 1)} RPM",
        
        "左前轮跳动 (垂向力)": f"{f_fl} N",
        "右前轮跳动 (垂向力)": f"{f_fr} N",
        "左后轮跳动 (垂向力)": f"{f_rl} N",
        "右后轮跳动 (垂向力)": f"{f_rr} N",
        
        "左前轮纵向滑移": slip_fl,
        "右前轮纵向滑移": slip_fr,
        "左后轮纵向滑移": slip_rl,
        "右后轮纵向滑移": slip_rr,
        
        "左前轮径向变形": def_fl,
        "右前轮径向变形": def_fr,
        "左后轮径向变形": def_rl,
        "右后轮径向变形": def_rr,
        
        # 修正映射，绝对保证俯仰、侧倾、横摆对齐
        "俯仰角": f"{round(pitch, 2)}°",
        "侧倾角": f"{round(roll, 2)}°",
        "横摆角": f"{round(yaw, 2)}°",
        
        "纵向位移": f"{round(pos[0], 2)} m",
        "横向位移": f"{round(pos[1], 2)} m",
        "垂向位移": f"{round(pos[2], 2)} m",
        
        "转向盘转动": f"{steer_deg}°",
        "左前轮转角": f"{ackermann_deg}°",
        "右前轮转角": f"{ackermann_deg}°",
        "转向管柱扭转": f"{round(steer_deg * 0.05, 1)}°",
        "发动机曲轴旋转": f"{engine_rpm} RPM",
        "左驱动半轴扭转": f"{round(throttle * 2.5, 1)}°",
        "右驱动半轴扭转": f"{round(throttle * 2.5, 1)}°"
    }
    return payload

# ================= 4. 工业级网络枢纽 (抗高并发、断线重连、指令双传) =================
async def gateway_service():
    loop = asyncio.get_running_loop()
    udp_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_rx.setblocking(False)
    try:
        udp_rx.bind(("0.0.0.0", 5000))
    except OSError:
        logging.error("❌ 5000 端口被占用！请确认后台没有挂起的僵尸进程。")
        return

    udp_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while True:
        try:
            logging.info(f"📡 尝试连接云端中台: {WS_URI}")
            async with websockets.connect(WS_URI, ping_interval=20, ping_timeout=20) as ws:
                logging.info("✅ 握手成功！全要素动力学网络就绪，等待交互...")

                async def udp_consumer():
                    while True:
                        try:
                            data, _ = await loop.sock_recvfrom(udp_rx, 8192)
                            SysState.LATEST_RAW_DATA = data
                            SysState.LATEST_RAW_TS = time.time()
                        except asyncio.CancelledError: break

                async def ws_producer():
                    last_send_time = 0
                    while True:
                        try:
                            current_time = time.time()
                            if SysState.IS_STREAMING and SysState.LATEST_RAW_DATA:
                                if current_time - last_send_time >= SysState.SEND_INTERVAL:
                                    if current_time - SysState.LATEST_RAW_TS <= 5.0:
                                        try:
                                            raw = json.loads(SysState.LATEST_RAW_DATA.decode('utf-8'))
                                            payload = build_delivery_payload(raw, SysState.CURRENT_CAR)
                                            await ws.send(json.dumps(payload, ensure_ascii=False))
                                            
                                            v_vec = safe_get(raw, "1_刚体运动学 (Rigid Body Kinematics)", "3_线速度矢量_XYZ_米每秒", default=[0,0,0])
                                            v_kmh = round(math.sqrt(sum(v**2 for v in v_vec)) * 3.6, 1)
                                            logging.info(f"🚀 推流成功: [{SysState.CURRENT_CAR}] | 车速 [{v_kmh} km/h] | 天气 [{SysState.CURRENT_WEATHER}]")
                                        except json.JSONDecodeError:
                                            logging.warning("⚠️ 拦截到残缺 JSON，已安全跳过本次推流。")
                                    else:
                                        logging.warning("⚠️ 底层数据流超时停滞，系统静默待命。")
                                    last_send_time = current_time
                            await asyncio.sleep(0.01) 
                        except asyncio.CancelledError: break

                async def ws_receiver():
                    async for msg in ws:
                        try:
                            cmd_data = json.loads(msg)
                            action = cmd_data.get("action", "")
                            
                            if action == "SET_CAR":
                                SysState.CURRENT_CAR = cmd_data.get("car", SysState.CURRENT_CAR)
                                logging.info(f"🚘 切换车型: {SysState.CURRENT_CAR}")
                            
                            elif action == "SET_WEATHER":
                                SysState.CURRENT_WEATHER = cmd_data.get("weather", SysState.CURRENT_WEATHER)
                                logging.info(f"🌤️ 切换天气: {SysState.CURRENT_WEATHER}")
                                udp_tx.sendto(json.dumps({"command": "set_weather", "value": SysState.CURRENT_WEATHER}).encode('utf-8'), (CARLA_UDP_TX_HOST, CARLA_UDP_TX_PORT))
                            
                            elif action == "ADD_TRAFFIC":
                                SysState.TRAFFIC_STATE = "正在加载..."
                                logging.info("🚶 收到指令：添加交通参与者")
                                udp_tx.sendto(json.dumps({"command": "add_scene_elements", "value": "true"}).encode('utf-8'), (CARLA_UDP_TX_HOST, CARLA_UDP_TX_PORT))
                                SysState.TRAFFIC_STATE = "已加载"
                                
                            elif action == "SET_INTERVAL":
                                SysState.SEND_INTERVAL = float(cmd_data.get("interval", 2.0))
                                logging.info(f"⏱️ 切换推流频率: {SysState.SEND_INTERVAL}秒/帧")
                            
                            else:
                                # 任何未被网关拦截的控制类 JSON，直接透传给底层 Carla
                                udp_tx.sendto(msg.encode('utf-8'), (CARLA_UDP_TX_HOST, CARLA_UDP_TX_PORT))
                                
                        except json.JSONDecodeError:
                            # 兼容纯文本快捷指令
                            cmd = msg.upper().strip()
                            if "START" in cmd: SysState.IS_STREAMING = True
                            elif "STOP" in cmd: SysState.IS_STREAMING = False
                            else: udp_tx.sendto(msg.encode('utf-8'), (CARLA_UDP_TX_HOST, CARLA_UDP_TX_PORT))

                tasks = [asyncio.create_task(udp_consumer()), asyncio.create_task(ws_producer()), asyncio.create_task(ws_receiver())]
                done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_EXCEPTION)
                for task in pending: task.cancel()
                await asyncio.gather(*pending, return_exceptions=True)
                
                has_error = False
                for t in done:
                    if t.exception() is not None: has_error = True; break
                if has_error: 
                    logging.warning("🔄 发现异常断网，5秒后重连...")
                    await asyncio.sleep(5)

        except Exception as e:
            logging.error(f"❌ 网络致命错误: {e}，5秒后重连...")
            await asyncio.sleep(5)

if __name__ == "__main__":
    asyncio.run(gateway_service())
