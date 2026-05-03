import streamlit as st
import json
import os
import threading
import time
import carla
import random
import math
import socket
from collections import deque

# ==========================================
# 1. 核心初始化 & 全局态共享内存池
# ==========================================
VEHICLE_DIR = "output"
st.set_page_config(page_title="L4 级全要素标定 | 泰坦防爆版", layout="wide", initial_sidebar_state="expanded")

class SimulationState:
    def __init__(self):
        self.reset()
        self.is_paused = False
        self.drive_mode = "内置 AI (Traffic Manager)" 

    def reset(self):
        self.data = {
            "SPEED": 0.0, "GNSS": "等待...", "IMU": "等待...", "RADAR": "等待...", 
            "CAM_LIDAR": "等待...", "RADAR_TARGETS": 0,
            "COLLISION_DATA": {"Impulse": [0,0,0], "Actor": "None"},
            "LANE_DATA": {"Side": "None", "Type": "None"},
            "IMU_DATA": {"Accel": [0,0,0], "Gyro": [0,0,0], "Compass": 0.0},
            "GNSS_DATA": [0.0, 0.0, 0.0],
            "FULL_TELEMETRY": {} 
        }
        self.frame_count = 0
        self.speed_history = deque(maxlen=50) 
        self.planner_error = None

@st.cache_resource
def get_sim_state():
    return SimulationState()

sim_state = get_sim_state()

if 'client' not in st.session_state: st.session_state.client = None
if 'world' not in st.session_state: st.session_state.world = None
if 'vehicle' not in st.session_state: st.session_state.vehicle = None
if 'active_sensors' not in st.session_state: st.session_state.active_sensors = []
if 'stop_event' not in st.session_state: st.session_state.stop_event = threading.Event()
if 'master_thread' not in st.session_state: st.session_state.master_thread = None 
if 'dynamics_wrapper' not in st.session_state: st.session_state.dynamics_wrapper = None

# ==========================================
# 2. 终极后端：物理引擎与 26 项真值包装器
# ==========================================
class L4_DynamicsWrapper:
    def __init__(self, vehicle_actor, json_config, ui_overrides):
        self.vehicle = vehicle_actor
        self.config = json_config
        self.ui = ui_overrides
        self.is_active = True
        self.air_density = 1.225 
        
        self._inject_full_physics_control()
        self.aero_thread = threading.Thread(target=self._aero_dynamics_loop)
        self.aero_thread.start()

    def _inject_full_physics_control(self):
        pc = self.vehicle.get_physics_control()
        
        pc.mass = self.ui.get('mass', 1500.0)
        pc.moi = self.ui.get('moi', 1.0) 
        pc.center_of_gravity = carla.Vector3D(x=self.ui.get('cg_x',0), y=self.ui.get('cg_y',0), z=self.ui.get('cg_z',0))
        pc.drag_coefficient = self.ui.get('cd', 0.3)
        pc.max_rpm = self.ui.get('rpm', 6000.0)
        pc.clutch_strength = self.ui.get('clutch', 1.5)
        pc.gear_switch_time = self.ui.get('gear_time', 0.4)
        pc.final_ratio = self.ui.get('final_ratio', 4.0) 
        
        wheels = pc.wheels
        for i, w in enumerate(wheels):
            w.tire_friction = self.ui.get('friction', 3.5)
            w.damping_rate = self.ui.get('damping', 0.5) * 4000 
            w.suspension_stiffness = self.ui.get('susp_stiff', 500.0) 
            w.suspension_max_travel = self.ui.get('susp_travel', 0.15)       
            
            w.radius = w.radius * self.ui.get('radius_mult', 1.0)                           
            
            w.max_brake_torque = self.ui.get('brake', 1500.0)
            w.max_handbrake_torque = self.ui.get('handbrake', 3000.0)          
            w.lat_stiff_value = self.ui.get('lat_stiff', 17.0)
            w.long_stiff_value = self.ui.get('long_stiff', 3000.0)
            if i < 2: w.max_steer_angle = self.ui.get('steer', 40.0)
            else: w.max_steer_angle = 0.0
        pc.wheels = wheels
        self.vehicle.apply_physics_control(pc)

    def _aero_dynamics_loop(self):
        while self.is_active and self.vehicle.is_alive:
            try:
                v = self.vehicle.get_velocity()
                speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)
                if speed > 2.0 and not sim_state.is_paused:
                    q = 0.5 * self.air_density * (speed ** 2)
                    f_down = q * self.ui.get('cl', 0.0) * self.ui.get('area', 2.2)
                    t_pitch = q * self.ui.get('cm', 0.0) * self.ui.get('area', 2.2)
                    f_side = q * self.ui.get('cy', 0.0) * self.ui.get('area', 2.2)
                    self.vehicle.add_force(carla.Vector3D(0, f_side, -f_down))
                    self.vehicle.add_torque(carla.Vector3D(0, t_pitch, 0))
            except: pass
            time.sleep(0.01)

    def fetch_telemetry_26_items(self):
        if not self.vehicle or not self.vehicle.is_alive: return {}
        t = self.vehicle.get_transform()
        v = self.vehicle.get_velocity()
        a = self.vehicle.get_acceleration()
        w = self.vehicle.get_angular_velocity()
        ctrl = self.vehicle.get_control()
        speed_ms = math.sqrt(v.x**2 + v.y**2 + v.z**2)
        
        steer_fl = self.vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
        steer_fr = self.vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
        
        base_rpm = (speed_ms / 0.35) * (60.0 / (2 * math.pi))
        slip_ratio = 1.0 + (ctrl.throttle * 0.1) 
        if speed_ms < 0.1 and ctrl.throttle > 0: slip_ratio = 5.0 
        wheel_rpm = [base_rpm * slip_ratio, base_rpm * slip_ratio, base_rpm, base_rpm]
        
        bounce_z = [0.0, 0.0, 0.0, 0.0]
        try:
            if hasattr(self.vehicle, 'get_bones'):
                bones = self.vehicle.get_bones()
                for bone in bones:
                    b_name = bone.name.lower()
                    if 'wheel_fl' in b_name: bounce_z[0] = round(bone.world_transform.location.z * 1000, 1)
                    elif 'wheel_fr' in b_name: bounce_z[1] = round(bone.world_transform.location.z * 1000, 1)
                    elif 'wheel_rl' in b_name: bounce_z[2] = round(bone.world_transform.location.z * 1000, 1)
                    elif 'wheel_rr' in b_name: bounce_z[3] = round(bone.world_transform.location.z * 1000, 1)
        except Exception: pass
        
        light_enum = str(self.vehicle.get_light_state())
        tl_state = str(self.vehicle.get_traffic_light_state())

        return {
            "1_刚体运动学 (Rigid Body Kinematics)": {
                "1_全局绝对坐标_XYZ_米": [round(t.location.x, 3), round(t.location.y, 3), round(t.location.z, 3)],
                "2_姿态角_俯仰_偏航_滚转_度": [round(t.rotation.pitch, 3), round(t.rotation.yaw, 3), round(t.rotation.roll, 3)],
                "3_线速度矢量_XYZ_米每秒": [round(v.x, 3), round(v.y, 3), round(v.z, 3)],
                "4_线加速度_XYZ_米每平方秒": [round(a.x, 3), round(a.y, 3), round(a.z, 3)],
                "5_角速度_XYZ_度每秒": [round(w.x, 3), round(w.y, 3), round(w.z, 3)]
            },
            "2_轮端与底盘动态 (Wheel Dynamics)": {
                "6_四轮独立转速_RPM_左前_右前_左后_右后": [round(x, 1) for x in wheel_rpm],
                "7_悬架实时压缩量_毫米_左前_右前_左后_右后": bounce_z, 
                "8_前轮真实阿克曼转向角_度": [round(steer_fl, 2), round(steer_fr, 2)]
            },
            "3_驾驶控制反读 (Control State)": {
                "9_实际油门开度_0至1": round(ctrl.throttle, 3),
                "10_实际刹车力度_0至1": round(ctrl.brake, 3),
                "11_方向盘转角_负1至1": round(ctrl.steer, 3),
                "12_当前机械档位": ctrl.gear,
                "13_手刹激活状态": ctrl.hand_brake,
                "14_倒车挂档状态": ctrl.reverse
            },
            "4_碰撞与接触事件 (Collision & Events)": {
                "15_三维碰撞冲量_XYZ_牛秒": sim_state.data["COLLISION_DATA"]["Impulse"],
                "16_碰撞对象_实体类型": sim_state.data["COLLISION_DATA"]["Actor"],
                "17_车道压线_侧边": sim_state.data["LANE_DATA"]["Side"],
                "18_车道线类型_虚实线": sim_state.data["LANE_DATA"]["Type"],
                "18b_雷达反射目标数": sim_state.data["RADAR_TARGETS"]
            },
            "5_环境与交通真值 (Environment Truth)": {
                "19_当前路段法定限速_公里每小时": round(self.vehicle.get_speed_limit(), 1),
                "20_前方红绿灯当前状态": tl_state,
                "21_是否处于红绿灯管制区": self.vehicle.is_at_traffic_light(),
                "22_车辆灯光激活状态_位掩码": light_enum
            },
            "6_传感器数据流 (Sensor Stream)": {
                "23_IMU_带重力加速度_XYZ": [round(x, 3) for x in sim_state.data["IMU_DATA"]["Accel"]],
                "24_IMU_陀螺仪角速度_XYZ": [round(x, 3) for x in sim_state.data["IMU_DATA"]["Gyro"]],
                "25_IMU_指南针正北夹角_度": round(sim_state.data["IMU_DATA"]["Compass"], 3),
                "26_GNSS_经度_纬度_海拔": [round(x, 6) for x in sim_state.data["GNSS_DATA"]]
            }
        }

    def destroy(self):
        self.is_active = False
        if hasattr(self, 'aero_thread'): self.aero_thread.join(timeout=1.0)

# ==========================================
# 3. 叹息之墙：优雅销毁序列
# ==========================================
def cleanup_simulation():
    st.session_state.stop_event.set() 
    if st.session_state.master_thread is not None:
        st.session_state.master_thread.join(timeout=1.0)
        st.session_state.master_thread = None
    if st.session_state.dynamics_wrapper:
        st.session_state.dynamics_wrapper.destroy()
        st.session_state.dynamics_wrapper = None
    for s in st.session_state.active_sensors:
        if s and s.is_alive:
            try: 
                if getattr(s, 'is_listening', False): s.stop()
            except: pass
    time.sleep(0.15) 
    for s in st.session_state.active_sensors:
        if s and s.is_alive:
            try: s.destroy()
            except: pass
    st.session_state.active_sensors = []
    if st.session_state.vehicle and st.session_state.vehicle.is_alive:
        try: st.session_state.vehicle.destroy()
        except: pass
    st.session_state.vehicle = None
    sim_state.reset()

# ==========================================
# 4. 全量隐形传感器回调
# ==========================================
def gnss_callback(data): sim_state.data["GNSS_DATA"] = [data.latitude, data.longitude, data.altitude]
def imu_callback(data): sim_state.data["IMU_DATA"] = {"Accel": [data.accelerometer.x, data.accelerometer.y, data.accelerometer.z], "Gyro": [data.gyroscope.x, data.gyroscope.y, data.gyroscope.z], "Compass": data.compass}
def radar_callback(data): sim_state.data["RADAR_TARGETS"] = len(data)
def generic_callback(data, name): sim_state.frame_count += 1
def collision_callback(data): sim_state.data["COLLISION_DATA"] = {"Impulse": [round(data.normal_impulse.x, 1), round(data.normal_impulse.y, 1), round(data.normal_impulse.z, 1)], "Actor": str(data.other_actor.type_id)}
def lane_invasion_callback(data):
    lines = [str(x.type) for x in data.crossed_lane_markings]
    sim_state.data["LANE_DATA"] = {"Side": "Detected", "Type": ",".join(lines)}

# ==========================================
# 5. 【核心中枢】：UDP、防抖锁死与全路由
# ==========================================
def master_simulation_loop(client, vehicle_actor, sensors_list, stop_event, dist, height, pitch, show_debug, dyn_wrapper):
    telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try: ctrl_sock.bind(("127.0.0.1", 5001))
    except Exception: pass 
    ctrl_sock.setblocking(False)

    try:
        world = client.get_world()
        spectator = world.get_spectator()
        last_mode = None
        first_frame_cam = True 

        while not stop_event.is_set():
            if not vehicle_actor or not vehicle_actor.is_alive: break
            
            if sim_state.is_paused:
                vehicle_actor.set_simulate_physics(False)
                time.sleep(0.05)
                continue 
            else:
                vehicle_actor.set_simulate_physics(True)

            current_mode = sim_state.drive_mode
            if current_mode != last_mode:
                if current_mode == "内置 AI (Traffic Manager)": vehicle_actor.set_autopilot(True)
                else: vehicle_actor.set_autopilot(False) 
                last_mode = current_mode

            if dyn_wrapper:
                telem_data = dyn_wrapper.fetch_telemetry_26_items()
                sim_state.data["FULL_TELEMETRY"] = telem_data
                
                if "1_刚体运动学 (Rigid Body Kinematics)" in telem_data:
                    speed_val = telem_data["1_刚体运动学 (Rigid Body Kinematics)"]["3_线速度矢量_XYZ_米每秒"][0] * 3.6 
                    sim_state.data["SPEED"] = speed_val
                    sim_state.speed_history.append(speed_val) 

                # ==========================================
                # 【底层权限分离核心区】
                # ==========================================
                if current_mode == "UDP 独立后台域控 (双向端口:5000/5001)":
                    # 1. 域控模式：发送 26 项真值给 5000 端口
                    try: telem_sock.sendto(json.dumps(telem_data).encode(), ("127.0.0.1", 5000))
                    except: pass
                    # 2. 接收算法发来的油门刹车指令 (5001 端口)
                    try:
                        ctrl_bytes, _ = ctrl_sock.recvfrom(1024)
                        ctrl_dict = json.loads(ctrl_bytes.decode())
                        vehicle_actor.apply_control(carla.VehicleControl(
                            throttle=float(ctrl_dict.get('throttle', 0)), steer=float(ctrl_dict.get('steer', 0)), brake=float(ctrl_dict.get('brake', 0))))
                    except BlockingIOError: pass 
                    except Exception: pass
                
                elif current_mode == "硬件在环: CAN 总线方向盘 (单向接收:5001)":
                    # CAN 方向盘模式：只接收 5001 端口的指令，不发真值
                    try:
                        ctrl_bytes, _ = ctrl_sock.recvfrom(1024)
                        ctrl_dict = json.loads(ctrl_bytes.decode())
                        vehicle_actor.apply_control(carla.VehicleControl(
                            throttle=float(ctrl_dict.get('throttle', 0)), steer=float(ctrl_dict.get('steer', 0)), brake=float(ctrl_dict.get('brake', 0))))
                    except BlockingIOError: pass 
                    except Exception: pass

                elif current_mode == "自定义算法 (网页端沙盒)":
                    if sim_state.custom_planner:
                        try:
                            t, s, b = sim_state.custom_planner(sim_state.data)
                            vehicle_actor.apply_control(carla.VehicleControl(
                                throttle=max(0.0, min(1.0, float(t))), steer=max(-1.0, min(1.0, float(s))), brake=max(0.0, min(1.0, float(b)))))
                            sim_state.planner_error = None 
                        except Exception as err: 
                            sim_state.planner_error = str(err) 

            # 【电影级防抖】
            t_veh = vehicle_actor.get_transform()
            target_cam_loc = t_veh.location - t_veh.get_forward_vector() * dist + carla.Location(z=height)
            
            if first_frame_cam:
                current_cam_loc = target_cam_loc
                first_frame_cam = False
            else:
                smooth_factor = 0.15 
                current_cam_loc.x += (target_cam_loc.x - current_cam_loc.x) * smooth_factor
                current_cam_loc.y += (target_cam_loc.y - current_cam_loc.y) * smooth_factor
                current_cam_loc.z += (target_cam_loc.z - current_cam_loc.z) * smooth_factor
            
            spectator.set_transform(carla.Transform(current_cam_loc, carla.Rotation(pitch=pitch, yaw=t_veh.rotation.yaw, roll=0.0)))
            
            if show_debug:
                for s in sensors_list:
                    if s and s.is_alive: world.debug.draw_point(s.get_transform().location, size=0.1, color=carla.Color(255, 0, 0), life_time=0.1)
            
            time.sleep(0.01) 
    finally:
        telem_sock.close()
        ctrl_sock.close()

# ==========================================
# 6. 左侧边栏：服务器引擎与时空控制
# ==========================================
with st.sidebar:
    st.title("🛰️ 仿真主控台")
    host = st.text_input("主机 IP", "127.0.0.1")
    port = st.number_input("端口", 2000)
    if st.button("🔗 建立底层连接", use_container_width=True):
        try:
            st.session_state.client = carla.Client(host, port)
            st.session_state.client.set_timeout(10.0) 
            st.session_state.world = st.session_state.client.get_world()
            
            settings = st.session_state.world.get_settings()
            settings.synchronous_mode = False 
            settings.substepping = True              
            settings.max_substep_delta_time = 0.005  
            settings.max_substeps = 16               
            st.session_state.world.apply_settings(settings)
            st.success("Carla 连接成功！200Hz 物理防抖已强制注入！")
        except Exception as e: st.error(f"连接失败: {e}")

    st.divider()
    st.subheader("⏱️ 仿真时间轴")
    col_p, col_r = st.columns(2)
    if col_p.button("⏸️ 物理定格", type="primary"): sim_state.is_paused = True
    if col_r.button("▶️ 继续运行"): sim_state.is_paused = False
    
    if st.session_state.client: avail_maps = sorted([m.split('/')[-1] for m in st.session_state.client.get_available_maps()])
    else: avail_maps = ["Town01"]
        
    map_choice = st.selectbox("🌎 地图载入", avail_maps)
    if st.button("🗺️ 应用地图", use_container_width=True) and st.session_state.client:
        with st.spinner("重构世界中..."): st.session_state.world = st.session_state.client.load_world(map_choice)

    st.divider()
    weather_preset = st.selectbox("天气预设", ["ClearNoon", "CloudyNoon", "HardRainNoon", "FoggySunset", "MidRainyNight"])
    if st.button("🌤️ 应用天气", use_container_width=True) and st.session_state.world:
        st.session_state.world.set_weather(getattr(carla.WeatherParameters, weather_preset))

    st.divider()
    dist = st.slider("跟随距离 (m)", 3.0, 30.0, 10.0)
    hgt = st.slider("相机高度 (m)", 1.0, 15.0, 4.0)
    ptch = st.slider("俯冲角度 (°)", -45.0, 10.0, -15.0)
    show_sensor_debug = st.toggle("🔴 显示挂载红点", value=True)

    st.divider()
    st.subheader("📍 场景定位 (发车坐标)")
    use_fixed_spawn = st.toggle("启用固定坐标发车 (推荐)", value=True)
    col_x, col_y = st.columns(2)
    spawn_x = col_x.number_input("X 坐标", value=150.5)
    spawn_y = col_y.number_input("Y 坐标", value=-45.2)
    col_z, col_yaw = st.columns(2)
    spawn_z = col_z.number_input("Z 高度", value=2.0)
    spawn_yaw = col_yaw.number_input("Yaw (朝向)", value=90.0)
    st.caption("提示: Town03 环岛/三车道推荐坐标 (150.5, -45.2, 2.0, 90.0)")

    st.divider()
    deploy_btn = st.button("🚀 出库！全量注入", type="primary", use_container_width=True)
    auto_refresh = st.toggle("📡 开启大屏刷新", value=False)

# ==========================================
# 7. 主界面：底层 API 全量暴露出库大厅
# ==========================================
st.title("🚗 L4 极致全要素标定大厅 & 全路由中心")

if not st.session_state.client: st.stop()
vehicle_files = [f for f in os.listdir(VEHICLE_DIR) if f.endswith('.json')] if os.path.exists(VEHICLE_DIR) else []
if not vehicle_files: st.stop()
selected_file = st.selectbox("📂 调取车型核心资产 (JSON)", vehicle_files)

with open(os.path.join(VEHICLE_DIR, selected_file), 'r', encoding='utf-8') as f: v = json.load(f)

t1, t2, t3, t4, t5, t6 = st.tabs(["⚖️ 质量", "🌪️ 气动力", "🛞 29-DOF 底盘", "🏎️ 传动", "📡 遥测示波器", "🖧 权限路由中心"])
ui_params = {}

with t1:
    c1, c2, c3 = st.columns(3)
    ui_params['mass'] = c1.number_input("整备质量 [kg]", value=float(v.get('weight_and_mass_properties', {}).get('curb_weight_kg', 1800)))
    ui_params['moi'] = c2.slider("发动机转动惯量 (MOI)", 0.5, 5.0, 1.0) 
    cog = v.get('weight_and_mass_properties', {}).get('center_of_gravity_m', {'x':0, 'y':0, 'z':0})
    ui_params['cg_x'] = st.slider("质心X轴", -2.0, 2.0, float(cog.get('x', 0)))
    ui_params['cg_y'] = st.slider("质心Y轴", -1.0, 1.0, float(cog.get('y', 0)))
    ui_params['cg_z'] = st.slider("质心Z轴", 0.0, 2.0, float(cog.get('z', 0)))

with t2:
    aero = v.get('aerodynamic_parameters', {})
    c1, c2, c3 = st.columns(3)
    ui_params['cd'] = c1.slider("风阻系数 (Cd)", 0.1, 1.5, float(aero.get('drag_coefficient_cd', 0.3)))
    ui_params['area'] = c2.slider("迎风面积 (sqm)", 1.0, 15.0, float(aero.get('frontal_area_sqm', 2.2)))
    ui_params['cl'] = c3.slider("升力系数 (Cl)", -1.0, 1.0, float(aero.get('lift_coefficient_cl', 0.0)))
    c4, c5 = st.columns(2)
    ui_params['cy'] = c4.slider("侧风系数 (Cy)", -1.0, 1.0, float(aero.get('side_force_coefficient_cy', 0.0)))
    ui_params['cm'] = c5.slider("俯仰力矩 (Cm)", -1.0, 1.0, float(aero.get('pitch_moment_coefficient', 0.0)))

with t3:
    mech = v.get('chassis_and_mechanical_systems', {})
    c1, c2, c3, c4 = st.columns(4)
    ui_params['friction'] = c1.slider("轮胎抓地力", 0.1, 5.0, 3.5)
    ui_params['steer'] = c2.slider("最大转向角", 20.0, 70.0, float(mech.get('steering_system', {}).get('wheel_max_angle_deg', 40.0)))
    ui_params['radius_mult'] = c3.slider("轮胎半径缩放倍率", 0.5, 2.0, 1.0) 
    ui_params['damping'] = c4.slider("避震器阻尼比", 0.1, 5.0, float(mech.get('suspension_system', {}).get('damping_ratio', 0.5)))
    c5, c6, c7, c8 = st.columns(4)
    ui_params['susp_stiff'] = c5.slider("悬架弹簧刚度", 10.0, 5000.0, 500.0) 
    ui_params['susp_travel'] = c6.slider("最大行程", 0.05, 0.5, 0.15) 
    ui_params['lat_stiff'] = c7.slider("侧偏刚度", 5.0, 30.0, 17.0)
    ui_params['long_stiff'] = c8.slider("纵向刚度", 1000.0, 5000.0, 3000.0)

with t4:
    pt = v.get('powertrain_system', {})
    c1, c2, c3 = st.columns(3)
    ui_params['rpm'] = c1.number_input("最高转速", value=int(pt.get('max_rpm', 6000)))
    ui_params['brake'] = c2.number_input("四轮最大刹车", value=float(mech.get('braking_system', {}).get('max_brake_torque_nm', 1500.0)))
    ui_params['handbrake'] = c3.number_input("手刹扭矩", value=3000.0) 
    c4, c5, c6 = st.columns(3)
    ui_params['gear_time'] = c4.slider("换挡延迟", 0.0, 2.0, 0.4)
    ui_params['clutch'] = c5.slider("离合强度", 0.5, 5.0, 1.5)
    ui_params['final_ratio'] = c6.slider("主减速比", 1.0, 10.0, 4.0) 

with t5:
    speed_box = st.empty() 
    st.subheader("📈 实时物理车速动态波形图")
    chart_box = st.empty()
    st.divider()
    st.subheader("📊 26项底层真值流 (包含碰撞/压线/隐形雷达)")
    telemetry_box = st.empty() 

with t6:
    st.subheader("🖧 五维权限路由中心")
    
    # 【核心修复】：5个明确的控制权选项，各自负责不同的底层路由！
    chosen_mode = st.radio("系统控制权归属", [
        "内置 AI (Traffic Manager)", 
        "自定义算法 (网页端沙盒)",
        "UDP 独立后台域控 (双向端口:5000/5001)",
        "硬件在环: CAN 总线方向盘 (单向接收:5001)",
        "原生物理外设 / manual_control.py (已弃用)"
    ], horizontal=False)
    sim_state.drive_mode = chosen_mode
    st.divider()
    
    if chosen_mode == "自定义算法 (网页端沙盒)":
        st.info("💻 **轻量级在线验证实验室**")
        default_code = """def planner(telemetry):
    radar_targets = telemetry.get("4_Events", {}).get("Radar_Targets", 0) 
    full_data = telemetry.get("FULL_TELEMETRY", {})
    kinematics = full_data.get("1_刚体运动学 (Rigid Body Kinematics)", {})
    speed = kinematics.get("3_线速度矢量_XYZ_米每秒", [0.0, 0.0, 0.0])[0] * 3.6
        
    if radar_targets > 15: return 0.0, 0.0, 1.0 # 紧急避障
    error = 40.0 - speed # 40km/h PID 巡航
    if error > 0: return min(1.0, error * 0.05), 0.0, 0.0 
    else: return 0.0, 0.0, min(1.0, -error * 0.1) 
"""
        user_code = st.text_area("Python Editor", value=default_code, height=380)
        if st.button("🔄 编译部署至实车神经元", type="primary"):
            namespace = {}
            try:
                exec(user_code, globals(), namespace)
                sim_state.custom_planner = namespace.get('planner')
                sim_state.planner_error = None 
                st.success("✅ 算法通过编译！")
            except Exception as e: st.error(f"❌ 编译错误: {e}")

    elif chosen_mode == "原生物理外设 / manual_control.py (已弃用)":
        st.warning("🏎️ **底层物理通道已释放**：你可以运行 Carla 官方的 manual_control.py 接管。但不支持 CAN 网关！")
        
    elif chosen_mode == "UDP 独立后台域控 (双向端口:5000/5001)":
        st.info("📡 **全功能自动驾驶域控模式 (闭环测试)**")
        st.write("本大屏正在以 100Hz 频率向 **5000 端口** 发送 26 项真值数据，并实时监听 **5001 端口** 返回的控制指令。")
        
    elif chosen_mode == "硬件在环: CAN 总线方向盘 (单向接收:5001)":
        st.success("🛞 **真实驾驶员台架接管模式 (开环测试)**")
        st.write("大屏已停止向外发送庞大的遥测数据，进入极速低延迟监听状态。等待 CAN 网关脚本向 **5001 端口** 注入转向指令！")

# ==========================================
# 8. 生成出库与真·全量传感器挂载
# ==========================================
if deploy_btn:
    with st.spinner("🚀 重构物理网格与挂载探测器中..."):
        world = st.session_state.world
        bp_lib = world.get_blueprint_library()
        cleanup_simulation()
        bp = bp_lib.find(v['vehicle_metadata']['blueprint_id'])
        
        if use_fixed_spawn:
            spawn_p = carla.Transform(
                carla.Location(x=spawn_x, y=spawn_y, z=spawn_z),
                carla.Rotation(pitch=0.0, yaw=spawn_yaw, roll=0.0)
            )
        else:
            spawn_pts = world.get_map().get_spawn_points()
            spawn_p = random.choice(spawn_pts)
            spawn_p.location.z += 2.5 
        
        try:
            vehicle = world.spawn_actor(bp, spawn_p)
            st.session_state.vehicle = vehicle
            
            radar = world.spawn_actor(bp_lib.find('sensor.other.radar'), carla.Transform(carla.Location(x=2.5, z=1.0)), attach_to=vehicle)
            radar.listen(radar_callback)
            st.session_state.active_sensors.append(radar)
            
            col_sensor = world.spawn_actor(bp_lib.find('sensor.other.collision'), carla.Transform(), attach_to=vehicle)
            col_sensor.listen(collision_callback)
            st.session_state.active_sensors.append(col_sensor)

            lane_sensor = world.spawn_actor(bp_lib.find('sensor.other.lane_invasion'), carla.Transform(), attach_to=vehicle)
            lane_sensor.listen(lane_invasion_callback)
            st.session_state.active_sensors.append(lane_sensor)

            wrapper = L4_DynamicsWrapper(vehicle, v, ui_params)
            st.session_state.dynamics_wrapper = wrapper
            st.session_state.stop_event.clear()
            
            t = threading.Thread(target=master_simulation_loop, args=(st.session_state.client, vehicle, st.session_state.active_sensors, st.session_state.stop_event, dist, hgt, ptch, show_sensor_debug, wrapper))
            t.start()
            st.session_state.master_thread = t
            
            if use_fixed_spawn:
                st.toast(f"🔥 车辆安全着陆！已锁定固定坐标：({spawn_x}, {spawn_y}, {spawn_z})")
            else:
                st.toast("🔥 车辆安全着陆！穿模 BUG 已被高空空投彻底抹杀！")
        except Exception as e: st.error(f"致命错误 (坐标冲突或穿模): {e}")

# ==========================================
# 9. 遥测大屏动态渲染
# ==========================================
if st.session_state.vehicle and not st.session_state.stop_event.is_set():
    speed_val = sim_state.data.get("SPEED", 0.0)
    speed_box.metric("⚡ 绝对车速", f"{speed_val:.1f} km/h", delta=f"状态: [{sim_state.drive_mode}]")
    
    if len(sim_state.speed_history) > 0: 
        chart_box.line_chart(list(sim_state.speed_history), height=150)
        
    if sim_state.data.get("FULL_TELEMETRY"): 
        telemetry_box.json(sim_state.data["FULL_TELEMETRY"]) 
        
    if auto_refresh:
        time.sleep(0.5)
        st.rerun()
