import streamlit as st
import json
import os
import threading
import time
import carla
import random
import math
import socket
import pandas as pd
import subprocess
import sys
from collections import deque
from typing import Dict, List

# ==========================================
# 0. 场景要素配置 & 算法工况映射 (绝对锁定版)
# ==========================================
TM_PORT = 8010
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
VEHICLE_DIR = os.path.join(BASE_DIR, "output")
FORMAL_WINDOWS_VCU_IP = "10.32.127.110"
FORMAL_UBUNTU_CARLA_IP = "10.32.127.216"

REQUIRED_COUNTS = {
    "vehicle_models": 10,
    "traffic_standards": 10,
    "barriers": 16,
    "covers": 4,
    "manholes": 1,
    "normal_vehicles": 15,
    "emergency_vehicles": 1,
    "walkers": 5,
    "bicycles": 2,
    "animals": 1
}

# 🚀 算法工况数据库
SCENARIO_DATABASE = {
    "Town01": {"pos": (-2.0, 8.0, 2.0, 90.0), "script": "vshuangyi.py", "task": "DLC 双移线紧急避险"},
    "Town02": {"pos": (3.0, 109.5, 2.0, 0), "script": "vdanyi.py", "task": "单移线避障测试"},
    "Town03": {"pos": (-42.0, 204.0, 2.0, 0.0), "script": "vjiansu.py", "task": "动态速度廓线 (减速)"},
    "Town04": {"pos": (9.0, 237.0, 2.0, -90.0), "script": "vshexing.py", "task": "长距离蛇行绕桩"},
    "Town05": {"pos": (206.6, 110.0, 2.0, -90.0), "script": "vjiasu.py", "task": "起步与定距停车 (加速)"}
}

st.set_page_config(page_title="L4 级全要素标定 | 全传感满血版", layout="wide", initial_sidebar_state="expanded")

# ==========================================
# 1. 核心初始化 & 铁壁防漏共享内存池
# ==========================================
class SimulationState:
    def __init__(self):
        self.reset()
        self.is_paused = False
        self.drive_mode = "🛞 硬件在环手动模式 (台架驾驶，无挂载)"
        self.filter_alpha = 0.15  
        self.smoothed_speed = 0.0
        self.target_ip = "127.0.0.1"
        self.scene_actors = []
        self.scene_walker_controllers = []
        self.scene_summary = None

    def reset(self):
        self.data = {
            "SPEED": 0.0, 
            "GNSS_DATA": [0.0, 0.0, 0.0], 
            "IMU_DATA": {"Accel": [0,0,0], "Gyro": [0,0,0], "Compass": 0.0}, 
            "RADAR_TARGETS": 0,
            "COLLISION_DATA": {"Impulse": [0,0,0], "Actor": "None"},
            "FULL_TELEMETRY": {} 
        }
        self.frame_count = 0
        self.speed_history = deque(maxlen=120) 
        self.smoothed_speed = 0.0

@st.cache_resource
def get_sim_state():
    return SimulationState()

sim_state = get_sim_state()

keys_to_init = ['client', 'world', 'vehicle', 'active_sensors', 'stop_event', 'master_thread', 'dynamics_wrapper', 'tracking_cam', 'algo_process', 'sim_paused', 'last_weather_key']
for key in keys_to_init:
    if key not in st.session_state:
        if key == 'active_sensors': st.session_state[key] = []
        elif key == 'stop_event': st.session_state[key] = threading.Event()
        elif key == 'sim_paused': st.session_state[key] = False
        else: st.session_state[key] = None

def clamp_float(value, min_value, max_value, default=0.0):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    return max(min_value, min(max_value, value))

def parse_bool(value):
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)

@st.cache_data(show_spinner=False)
def load_vehicle_config(path, mtime):
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)

def render_telemetry_dashboard(chart_box, speed_box, telemetry_box, sensor_box, live_mode=False):
    speed_val = sim_state.data.get("SPEED", 0.0)
    delta_text = f"接入: {sim_state.drive_mode.split(' ')[0]}" if live_mode else "状态: 待机"
    speed_box.metric("⚡ 绝对滤波车速", f"{speed_val:.1f} km/h", delta=delta_text)

    if len(sim_state.speed_history) > 0:
        df = pd.DataFrame(list(sim_state.speed_history), columns=['Speed (km/h)'])
        chart_box.line_chart(df, height=180, use_container_width=True)

    if sim_state.data.get("FULL_TELEMETRY"):
        telemetry_box.json(sim_state.data["FULL_TELEMETRY"])

    sensor_box.json({
        "📡 GNSS 绝对定位": sim_state.data["GNSS_DATA"],
        "🧭 IMU 惯导真值": sim_state.data["IMU_DATA"],
        "🛰️ 毫米波雷达目标数": sim_state.data["RADAR_TARGETS"],
        "💥 碰撞监控状态": sim_state.data["COLLISION_DATA"]
    })

def request_streamlit_rerun():
    if hasattr(st, "rerun"):
        st.rerun()
    else:
        st.experimental_rerun()

def apply_world_runtime_settings(world):
    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.substepping = True
    settings.max_substep_delta_time = 0.005
    settings.max_substeps = 16
    world.apply_settings(settings)

def build_weather_parameters(w_cond, t_day="正午"):
    w = carla.WeatherParameters()
    w.cloudiness = 0.0
    w.precipitation = 0.0
    w.precipitation_deposits = 0.0
    w.wind_intensity = 0.0
    w.fog_density = 0.0
    w.fog_distance = 0.0

    if "多云" in w_cond or "Cloudy" in w_cond:
        w.cloudiness = 80.0
    elif "小雨" in w_cond or "Rain" in w_cond:
        w.cloudiness = 80.0
        w.precipitation = 30.0
        w.precipitation_deposits = 30.0
    elif "暴雨" in w_cond or "Storm" in w_cond:
        w.cloudiness = 100.0
        w.precipitation = 90.0
        w.precipitation_deposits = 90.0
        w.wind_intensity = 80.0
    elif "大雾" in w_cond or "Fog" in w_cond:
        w.cloudiness = 50.0
        w.fog_density = 50.0
        w.fog_distance = 10.0

    if "夕阳" in t_day:
        w.sun_altitude_angle = 5.0
        w.sun_azimuth_angle = 180.0
    elif "深夜" in t_day or "Night" in w_cond:
        w.sun_altitude_angle = -90.0
        w.sun_azimuth_angle = 0.0
    else:
        w.sun_altitude_angle = 75.0
        w.sun_azimuth_angle = 180.0

    return w

def apply_weather_to_world(world, w_cond, t_day="正午"):
    if world is not None:
        world.set_weather(build_weather_parameters(w_cond, t_day))

sim_state.is_paused = st.session_state.sim_paused

# ==========================================
# 2. 物理引擎与底层真值包装器 (原汁原味，无任何干预)
# ==========================================
class L4_DynamicsWrapper:
    def __init__(self, vehicle_actor, json_config, ui_overrides, world):
        self.vehicle = vehicle_actor
        self.config = json_config
        self.ui = ui_overrides
        self.world = world
        self.is_active = True
        self.air_density = 1.225 
        
        self._inject_full_physics_control()
        self.aero_thread = threading.Thread(target=self._aero_dynamics_loop, daemon=True)
        self.aero_thread.start()

    def _inject_full_physics_control(self):
        pc = self.vehicle.get_physics_control()
        pc.mass = float(self.ui.get('mass', 1500.0))
        pc.moi = float(self.ui.get('moi', 1.0))
        pc.center_of_gravity = carla.Vector3D(x=float(self.ui.get('cg_x',0)), y=float(self.ui.get('cg_y',0)), z=float(self.ui.get('cg_z',0)))
        pc.drag_coefficient = float(self.ui.get('cd', 0.3))
        pc.max_rpm = float(self.ui.get('rpm', 6000.0))
        pc.clutch_strength = float(self.ui.get('clutch', 1.5))
        pc.gear_switch_time = float(self.ui.get('gear_time', 0.4))
        pc.final_ratio = float(self.ui.get('final_ratio', 4.0))
        
        wheels = pc.wheels
        for i, w in enumerate(wheels):
            w.tire_friction = float(self.ui.get('friction', 3.5))
            w.damping_rate = float(self.ui.get('damping', 0.5)) * 4000 
            w.suspension_stiffness = float(self.ui.get('susp_stiff', 500.0)) 
            w.suspension_max_travel = float(self.ui.get('susp_travel', 0.15))       
            w.radius = w.radius * float(self.ui.get('radius_mult', 1.0))                           
            w.max_brake_torque = float(self.ui.get('brake', 1500.0))
            w.max_handbrake_torque = float(self.ui.get('handbrake', 3000.0))          
            w.lat_stiff_value = float(self.ui.get('lat_stiff', 17.0))
            w.long_stiff_value = float(self.ui.get('long_stiff', 3000.0))
            if i < 2: w.max_steer_angle = float(self.ui.get('steer', 40.0))
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
        
        raw_speed_kmh = speed_ms * 3.6
        alpha = sim_state.filter_alpha
        sim_state.smoothed_speed = (alpha * raw_speed_kmh) + ((1.0 - alpha) * sim_state.smoothed_speed)
        
        steer_fl = self.vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
        steer_fr = self.vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
        
        base_rpm = (speed_ms / 0.35) * (60.0 / (2 * math.pi))
        slip_ratio = 1.0 + (ctrl.throttle * 0.1) 
        if speed_ms < 0.1 and ctrl.throttle > 0: slip_ratio = 5.0 
        wheel_rpm = [base_rpm * slip_ratio, base_rpm * slip_ratio, base_rpm, base_rpm]
        
        bounce_z = [0.0, 0.0, 0.0, 0.0]
        try:
            if hasattr(self.vehicle, 'get_bones'):
                for bone in self.vehicle.get_bones():
                    b_name = bone.name.lower()
                    if 'wheel_fl' in b_name: bounce_z[0] = round(bone.world_transform.location.z * 1000, 1)
                    elif 'wheel_fr' in b_name: bounce_z[1] = round(bone.world_transform.location.z * 1000, 1)
                    elif 'wheel_rl' in b_name: bounce_z[2] = round(bone.world_transform.location.z * 1000, 1)
                    elif 'wheel_rr' in b_name: bounce_z[3] = round(bone.world_transform.location.z * 1000, 1)
        except: pass
        
        light_enum = str(self.vehicle.get_light_state())
        tl_state = str(self.vehicle.get_traffic_light_state())

        mass = float(self.ui.get('mass', 1500.0))
        dyn_Cf = -110000.0 * (mass / 1500.0)
        dyn_Cr = -95000.0 * (mass / 1500.0)
        
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
            "5_环境与交通真值 (Environment Truth)": {
                "19_当前路段法定限速_公里每小时": round(self.vehicle.get_speed_limit(), 1),
                "20_前方红绿灯当前状态": tl_state,
                "21_是否处于红绿灯管制区": self.vehicle.is_at_traffic_light(),
                "22_车辆灯光激活状态_位掩码": light_enum
            },
            "7_场景要素验收 (Scene Compliance)": sim_state.scene_summary or {},
            "6_动态车辆参数": {
                "整备质量": mass,
                "前轮侧偏刚度_Cf": dyn_Cf,
                "后轮侧偏刚度_Cr": dyn_Cr,
                "轮距_L": 2.88,
                "a": 1.49,
                "b": 1.39
            }
        }

    def destroy(self):
        self.is_active = False
        if hasattr(self, 'aero_thread'): self.aero_thread.join(timeout=1.0)

# ==========================================
# 3. ✨ 四大传感器回调与环境生成
# ==========================================
def gnss_callback(data):
    sim_state.data["GNSS_DATA"] = [round(data.latitude, 5), round(data.longitude, 5), round(data.altitude, 2)]

def imu_callback(data):
    sim_state.data["IMU_DATA"] = {
        "Accel_XYZ": [round(data.accelerometer.x, 2), round(data.accelerometer.y, 2), round(data.accelerometer.z, 2)],
        "Gyro_XYZ": [round(data.gyroscope.x, 2), round(data.gyroscope.y, 2), round(data.gyroscope.z, 2)],
        "Compass": round(math.degrees(data.compass), 2)
    }

def radar_callback(data): sim_state.data["RADAR_TARGETS"] = len(data)

def collision_callback(data): 
    sim_state.data["COLLISION_DATA"] = {
        "Impulse": [round(data.normal_impulse.x, 1), round(data.normal_impulse.y, 1), round(data.normal_impulse.z, 1)], 
        "Actor": str(data.other_actor.type_id)
    }

def cleanup_scene_elements(world):
    if world is None: return
    ids = sim_state.scene_walker_controllers + sim_state.scene_actors
    for actor_id in ids:
        actor = world.get_actor(actor_id)
        if actor is not None:
            try:
                if actor.type_id.startswith("controller.ai.walker"):
                    actor.stop()
                actor.destroy()
            except Exception:
                pass
    sim_state.scene_actors = []
    sim_state.scene_walker_controllers = []
    sim_state.scene_summary = None

def cleanup_simulation():
    if st.session_state.get('algo_process'):
        try:
            st.session_state.algo_process.terminate()
            try:
                st.session_state.algo_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                st.session_state.algo_process.kill()
                st.session_state.algo_process.wait(timeout=1.0)
            st.session_state.algo_process = None
        except Exception:
            st.session_state.algo_process = None

    st.session_state.stop_event.set() 
    if st.session_state.master_thread is not None:
        st.session_state.master_thread.join(timeout=2.0)
        st.session_state.master_thread = None
    if st.session_state.dynamics_wrapper:
        st.session_state.dynamics_wrapper.destroy()
        st.session_state.dynamics_wrapper = None
    for s in st.session_state.active_sensors:
        if s and s.is_alive:
            try:
                s.stop()
            except Exception:
                pass
            try:
                s.destroy()
            except Exception:
                pass
    st.session_state.active_sensors = []
    if st.session_state.tracking_cam and st.session_state.tracking_cam.is_alive:
        try: st.session_state.tracking_cam.destroy()
        except Exception: pass
    st.session_state.tracking_cam = None
    if st.session_state.vehicle and st.session_state.vehicle.is_alive:
        try: st.session_state.vehicle.destroy()
        except Exception: pass
    st.session_state.vehicle = None
    sim_state.reset()

def cleanup_all():
    cleanup_simulation()
    if st.session_state.world:
        cleanup_scene_elements(st.session_state.world)

def ordered_unique(items: List[str]) -> List[str]:
    seen = set(); out = []
    for x in items:
        if x not in seen: seen.add(x); out.append(x)
    return out

def select_blueprints(all_ids: set, preferred: List[str], keywords: List[str] = None, limit: int = None) -> List[str]:
    selected = [x for x in preferred if x in all_ids]
    if keywords:
        selected.extend(
            x for x in sorted(all_ids)
            if any(k in x.lower() for k in keywords)
        )
    selected = ordered_unique(selected)
    return selected[:limit] if limit else selected

def actor_to_dict(actor: carla.Actor) -> Dict:
    tf = actor.get_transform()
    return {"id": actor.id, "type_id": actor.type_id, "transform": {"x": round(tf.location.x, 3), "y": round(tf.location.y, 3), "z": round(tf.location.z, 3), "pitch": round(tf.rotation.pitch, 3), "yaw": round(tf.rotation.yaw, 3), "roll": round(tf.rotation.roll, 3)}}

def offset_transform(base_tf: carla.Transform, forward=0.0, right=0.0, up=0.1, yaw_bias=0.0) -> carla.Transform:
    yaw = math.radians(base_tf.rotation.yaw)
    fx, fy = math.cos(yaw), math.sin(yaw)
    rx, ry = -math.sin(yaw), math.cos(yaw)
    loc = carla.Location(x=base_tf.location.x + fx * forward + rx * right, y=base_tf.location.y + fy * forward + ry * right, z=base_tf.location.z + up)
    rot = carla.Rotation(pitch=base_tf.rotation.pitch, yaw=base_tf.rotation.yaw + yaw_bias, roll=base_tf.rotation.roll)
    return carla.Transform(loc, rot)

def get_all_bp_ids(bp_lib: carla.BlueprintLibrary) -> List[str]: return [bp.id for bp in bp_lib.filter("*")]

def resolve_catalog(bp_lib: carla.BlueprintLibrary) -> Dict[str, List[str]]:
    all_ids = set(get_all_bp_ids(bp_lib))
    vehicle_ids = [bp.id for bp in bp_lib.filter("vehicle.*")]
    walker_ids = [bp.id for bp in bp_lib.filter("walker.pedestrian.*")]
    controller_ids = [bp.id for bp in bp_lib.filter("controller.ai.walker")]

    traffic_pref = [
        "traffic.speed_limit.30", "traffic.speed_limit.40", "traffic.speed_limit.50",
        "traffic.speed_limit.60", "traffic.speed_limit.90", "traffic.stop",
        "traffic.yield", "static.prop.trafficwarning", "static.prop.warningaccident",
        "static.prop.warningconstruction", "static.prop.trafficcone01",
        "static.prop.trafficcone02", "static.prop.trafficcone03",
        "static.prop.trafficcone04", "static.prop.directionsign",
        "static.prop.stopsign", "static.prop.yieldsign",
    ]
    traffic_standards = select_blueprints(
        all_ids,
        traffic_pref,
        keywords=["traffic.", "speed_limit", "stop", "yield", "sign", "warning"],
    )
    cover_like_ids = {
        "static.prop.ironplank", "static.prop.brokentile01", "static.prop.brokentile02",
        "static.prop.brokentile03", "static.prop.brokentile04", "static.prop.dirtdebris01",
        "static.prop.dirtdebris02", "static.prop.dirtdebris03",
    }
    traffic_standards = [x for x in traffic_standards if x not in cover_like_ids]

    barrier_pref = [
        "static.prop.streetbarrier", "static.prop.constructioncone",
        "static.prop.trafficcone01", "static.prop.trafficcone02",
        "static.prop.trafficcone03", "static.prop.trafficcone04",
        "static.prop.chainbarrier", "static.prop.chainbarrierend",
        "static.prop.warningconstruction", "static.prop.trafficwarning",
        "static.prop.warningaccident", "static.prop.plasticbarrier",
        "static.prop.roadbarrier", "static.prop.roadblock",
        "static.prop.workzone", "static.prop.barrier",
    ]
    barriers = select_blueprints(
        all_ids,
        barrier_pref,
        keywords=["barrier", "cone", "warning", "construction", "roadblock"],
    )

    cover_pref = [
        "static.prop.ironplank", "static.prop.brokentile01", "static.prop.brokentile02",
        "static.prop.brokentile03", "static.prop.brokentile04",
        "static.prop.dirtdebris01", "static.prop.dirtdebris02", "static.prop.dirtdebris03",
        "static.prop.garbage01", "static.prop.garbage02", "static.prop.garbage03",
        "static.prop.trashcan01", "static.prop.trashcan02",
    ]
    covers = select_blueprints(
        all_ids,
        cover_pref,
        keywords=["cover", "plank", "tile", "debris", "dirt", "garbage", "trash"],
    )

    manhole_pref = [
        "static.prop.manhole", "static.prop.manholecover", "static.prop.manhole_cover",
        "static.prop.draincover", "static.prop.drain_cover", "static.prop.sewercover",
        "static.prop.sewer_cover", "static.prop.utilitycover",
    ]
    manholes = select_blueprints(
        all_ids,
        manhole_pref,
        keywords=["manhole", "drain", "sewer", "utilitycover", "utility_cover"],
    )

    bicycle_pref = ["vehicle.bh.crossbike", "vehicle.diamondback.century", "vehicle.gazelle.omafiets"]
    bicycles = [x for x in bicycle_pref if x in all_ids]

    emergency_keywords = ["firetruck", "ambulance", "police"]
    emergency_vehicles = [x for x in vehicle_ids if any(k in x.lower() for k in emergency_keywords)]

    moto_keywords = ["yamaha", "harley", "vespa", "kawasaki", "bike"]
    normal_vehicles = [x for x in vehicle_ids if x not in bicycles and x not in emergency_vehicles and not any(k in x.lower() for k in moto_keywords)]

    animals = [x for x in get_all_bp_ids(bp_lib) if (any(k in x.lower() for k in ["animal", "deer", "horse"]) or x.lower().endswith(".dog") or x.lower().endswith(".cat")) and "doghouse" not in x.lower()]

    vehicle_models = ordered_unique(normal_vehicles + emergency_vehicles)

    return {
        "vehicle_models": vehicle_models, "traffic_standards": traffic_standards, "barriers": barriers,
        "covers": covers, "manholes": manholes,
        "normal_vehicles": normal_vehicles, "emergency_vehicles": emergency_vehicles, "walkers": walker_ids,
        "walker_controllers": controller_ids, "bicycles": bicycles, "animals": animals
    }

def summarize_vehicle_models(catalog: Dict[str, List[str]], normal_vehicle_result: Dict, emergency_result: Dict) -> Dict:
    spawned_model_ids = ordered_unique([a["type_id"] for a in normal_vehicle_result.get("actors", [])] + [a["type_id"] for a in emergency_result.get("actors", [])])
    return {"requested": REQUIRED_COUNTS["vehicle_models"], "available_blueprints": len(catalog.get("vehicle_models", [])), "spawned_unique_models": len(spawned_model_ids), "model_type_ids": spawned_model_ids, "satisfied": len(spawned_model_ids) >= REQUIRED_COUNTS["vehicle_models"]}

def mark_spawn_satisfaction(result: Dict, category: str) -> Dict:
    result["satisfied"] = result.get("spawned", 0) >= REQUIRED_COUNTS[category]
    return result

def build_scene_compliance_summary(summary: Dict) -> Dict:
    compliance = {}
    for key, target in REQUIRED_COUNTS.items():
        result = summary.get(key, {})
        if key == "vehicle_models":
            actual = result.get("spawned_unique_models", 0)
        else:
            actual = result.get("spawned", 0)
        compliance[key] = {
            "required": target,
            "actual": actual,
            "satisfied": actual >= target,
        }
    return {
        "all_satisfied": all(x["satisfied"] for x in compliance.values()),
        "categories": compliance,
    }

def spawn_static_objects(world: carla.World, bp_lib: carla.BlueprintLibrary, spawn_points: List[carla.Transform], bp_ids: List[str], desired_count: int, record_ids: List[int], right_bias: float) -> Dict:
    created, used, warnings = [], [], []
    if not bp_ids: return {"requested": desired_count, "available_blueprints": 0, "spawned": 0, "actors": [], "blueprints_used": [], "warnings": ["no available blueprint"]}
    random.shuffle(spawn_points)
    for i, bp_id in enumerate(bp_ids):
        if len(created) >= desired_count or not spawn_points:
            break
        base_tf = spawn_points[i % len(spawn_points)]
        tf = offset_transform(base_tf, forward=(i % 4) * 1.5, right=right_bias + (i % 3) * 1.2, up=0.1, yaw_bias=90.0)
        bp = bp_lib.find(bp_id)
        actor = world.try_spawn_actor(bp, tf)
        if actor is None:
            warnings.append(f"spawn failed: {bp_id}")
            continue
        try: actor.set_simulate_physics(False)
        except: pass
        record_ids.append(actor.id)
        used.append(bp_id)
        created.append(actor_to_dict(actor))
    if len(created) < desired_count:
        warnings.append(f"only spawned {len(created)}/{desired_count}")
    return {"requested": desired_count, "available_blueprints": len(bp_ids), "spawned": len(created), "actors": created, "blueprints_used": used, "warnings": warnings}

def spawn_vehicle_group(world: carla.World, bp_lib: carla.BlueprintLibrary, bp_ids: List[str], desired_count: int, spawn_points: List[carla.Transform], tm_port: int, role_name: str, record_ids: List[int]) -> Dict:
    created, used, warnings = [], [], []
    if not bp_ids: return {"requested": desired_count, "available_blueprints": 0, "spawned": 0, "actors": [], "blueprints_used": [], "warnings": ["no available blueprint"]}
    random.shuffle(spawn_points)
    attempts, max_attempts = 0, max(len(spawn_points) * 2, desired_count * 5)
    while len(created) < desired_count and attempts < max_attempts:
        bp_id = bp_ids[attempts % len(bp_ids)]
        tf = spawn_points[attempts % len(spawn_points)]
        bp = bp_lib.find(bp_id)
        if bp.has_attribute("role_name"): bp.set_attribute("role_name", role_name)
        if bp.has_attribute("color"):
            vals = bp.get_attribute("color").recommended_values
            if vals: bp.set_attribute("color", random.choice(vals))
        actor = world.try_spawn_actor(bp, tf)
        attempts += 1
        if actor is None: continue
        actor.set_autopilot(True, tm_port)
        try: world.wait_for_tick()
        except: time.sleep(0.05)
        record_ids.append(actor.id)
        used.append(bp_id)
        created.append(actor_to_dict(actor))
    if len(created) < desired_count: warnings.append(f"only spawned {len(created)}/{desired_count}")
    return {"requested": desired_count, "available_blueprints": len(bp_ids), "spawned": len(created), "actors": created, "blueprints_used": used, "warnings": warnings}

def spawn_bicycles_distinct(world: carla.World, bp_lib: carla.BlueprintLibrary, bicycle_bp_ids: List[str], desired_count: int, spawn_points: List[carla.Transform], tm_port: int, record_ids: List[int]) -> Dict:
    created, used, warnings = [], [], []
    if not bicycle_bp_ids: return {"requested": desired_count, "available_blueprints": 0, "spawned": 0, "actors": [], "blueprints_used": [], "warnings": ["no bicycle blueprint"]}
    target_unique = min(desired_count, len(bicycle_bp_ids))
    random.shuffle(spawn_points)
    for bp_id in bicycle_bp_ids:
        if len(created) >= target_unique: break
        bp = bp_lib.find(bp_id)
        if bp.has_attribute("role_name"): bp.set_attribute("role_name", "bicycle")
        spawned = False
        for tf in spawn_points:
            actor = world.try_spawn_actor(bp, tf)
            if actor is None: continue
            actor.set_autopilot(True, tm_port)
            try: world.wait_for_tick()
            except: time.sleep(0.05)
            record_ids.append(actor.id)
            used.append(bp_id)
            created.append(actor_to_dict(actor))
            spawned = True
            break
        if not spawned: warnings.append(f"spawn failed for bicycle blueprint: {bp_id}")
    if len(created) < desired_count: warnings.append(f"only spawned {len(created)}/{desired_count} bicycles")
    return {"requested": desired_count, "available_blueprints": len(bicycle_bp_ids), "spawned": len(created), "actors": created, "blueprints_used": used, "warnings": warnings}

def spawn_vehicle_model_fillers(world: carla.World, bp_lib: carla.BlueprintLibrary, all_vehicle_model_bp_ids: List[str], existing_model_ids: List[str], desired_unique_count: int, spawn_points: List[carla.Transform], record_ids: List[int]) -> Dict:
    created, used, warnings = [], [], []
    missing_count = desired_unique_count - len(existing_model_ids)
    if missing_count <= 0: return {"requested": desired_unique_count, "spawned": 0, "actors": [], "blueprints_used": [], "warnings": []}
    candidates = [x for x in all_vehicle_model_bp_ids if x not in existing_model_ids]
    random.shuffle(spawn_points)
    attempts, max_attempts = 0, max(len(spawn_points) * 2, missing_count * 5)
    while len(used) < missing_count and attempts < max_attempts and candidates:
        bp_id = candidates.pop(0)
        tf = spawn_points[attempts % len(spawn_points)]
        attempts += 1
        bp = bp_lib.find(bp_id)
        if bp.has_attribute("role_name"): bp.set_attribute("role_name", "vehicle_model_fill")
        actor = world.try_spawn_actor(bp, tf)
        if actor is None: continue
        try: world.wait_for_tick()
        except: time.sleep(0.05)
        record_ids.append(actor.id)
        used.append(bp_id)
        created.append(actor_to_dict(actor))
    if len(used) < missing_count: warnings.append(f"only filled {len(used)}/{missing_count} extra vehicle models")
    return {"requested": desired_unique_count, "spawned": len(used), "actors": created, "blueprints_used": used, "warnings": warnings}

def spawn_walkers(world: carla.World, bp_lib: carla.BlueprintLibrary, walker_bp_ids: List[str], controller_bp_ids: List[str], desired_count: int, record_actor_ids: List[int], record_controller_ids: List[int]) -> Dict:
    created, used, warnings = [], [], []
    if not walker_bp_ids: return {"requested": desired_count, "available_blueprints": 0, "spawned": 0, "actors": [], "blueprints_used": [], "warnings": ["no pedestrian blueprints"]}
    if not controller_bp_ids: return {"requested": desired_count, "available_blueprints": len(walker_bp_ids), "spawned": 0, "actors": [], "blueprints_used": [], "warnings": ["no walker controller blueprint"]}
    controller_bp = bp_lib.find(controller_bp_ids[0])
    attempts, max_attempts = 0, desired_count * 10
    while len(created) < desired_count and attempts < max_attempts:
        bp_id = random.choice(walker_bp_ids)
        nav_loc = world.get_random_location_from_navigation()
        attempts += 1
        if nav_loc is None: continue
        walker_tf = carla.Transform(nav_loc)
        walker_bp = bp_lib.find(bp_id)
        walker = world.try_spawn_actor(walker_bp, walker_tf)
        if walker is None: continue
        controller = world.try_spawn_actor(controller_bp, carla.Transform(), walker)
        if controller is None:
            walker.destroy()
            continue
        controller.start()
        target = world.get_random_location_from_navigation()
        if target: controller.go_to_location(target)
        controller.set_max_speed(1.2 + random.random())
        try: world.wait_for_tick()
        except: time.sleep(0.05)
        record_actor_ids.append(walker.id)
        record_controller_ids.append(controller.id)
        used.append(bp_id)
        created.append(actor_to_dict(walker))
    if len(created) < desired_count: warnings.append(f"only spawned {len(created)}/{desired_count}")
    return {"requested": desired_count, "available_blueprints": len(walker_bp_ids), "spawned": len(created), "actors": created, "blueprints_used": used, "warnings": warnings}

def spawn_animal(world: carla.World, bp_lib: carla.BlueprintLibrary, animal_bp_ids: List[str], spawn_points: List[carla.Transform], record_ids: List[int]) -> Dict:
    animal_bp_ids = [x for x in animal_bp_ids if "doghouse" not in x.lower()]
    if not animal_bp_ids: return {"requested": 1, "available_blueprints": 0, "spawned": 0, "actors": [], "blueprints_used": [], "warnings": ["no real animal blueprint in current CARLA installation"]}
    bp_id = animal_bp_ids[0]
    base_tf = random.choice(spawn_points)
    tf = offset_transform(base_tf, forward=2.0, right=6.0, up=0.1)
    bp = bp_lib.find(bp_id)
    actor = world.try_spawn_actor(bp, tf)
    if actor is None: return {"requested": 1, "available_blueprints": len(animal_bp_ids), "spawned": 0, "actors": [], "blueprints_used": [], "warnings": [f"spawn failed: {bp_id}"]}
    try:
        actor.set_simulate_physics(False)
        world.wait_for_tick()
    except: time.sleep(0.05)
    record_ids.append(actor.id)
    return {"requested": 1, "available_blueprints": len(animal_bp_ids), "spawned": 1, "actors": [actor_to_dict(actor)], "blueprints_used": [bp_id], "warnings": []}

def add_scene_elements_to_current_map(client, world):
    if client is None or world is None: return
    cleanup_scene_elements(world)
    bp_lib = world.get_blueprint_library()
    raw_spawn_points = world.get_map().get_spawn_points()
    if not raw_spawn_points: return

    # 15米防爆结界
    safe_spawn_points = []
    fixed_locs = [carla.Location(x=cfg["pos"][0], y=cfg["pos"][1], z=cfg["pos"][2]) for cfg in SCENARIO_DATABASE.values()]
    for sp in raw_spawn_points:
        if all(sp.location.distance(loc) > 15.0 for loc in fixed_locs):
            safe_spawn_points.append(sp)
    if not safe_spawn_points: safe_spawn_points = raw_spawn_points

    try: world.wait_for_tick()
    except: time.sleep(0.1)

    catalog = resolve_catalog(bp_lib)
    tm = client.get_trafficmanager(TM_PORT)
    tm.set_global_distance_to_leading_vehicle(2.5)

    actor_ids, controller_ids = [], []

    traffic_result = mark_spawn_satisfaction(spawn_static_objects(world, bp_lib, safe_spawn_points.copy(), catalog["traffic_standards"], REQUIRED_COUNTS["traffic_standards"], actor_ids, right_bias=5.0), "traffic_standards")
    barrier_result = mark_spawn_satisfaction(spawn_static_objects(world, bp_lib, safe_spawn_points.copy(), catalog["barriers"], REQUIRED_COUNTS["barriers"], actor_ids, right_bias=7.0), "barriers")
    cover_result = mark_spawn_satisfaction(spawn_static_objects(world, bp_lib, safe_spawn_points.copy(), catalog["covers"], REQUIRED_COUNTS["covers"], actor_ids, right_bias=9.0), "covers")
    manhole_result = mark_spawn_satisfaction(spawn_static_objects(world, bp_lib, safe_spawn_points.copy(), catalog["manholes"], REQUIRED_COUNTS["manholes"], actor_ids, right_bias=-3.2), "manholes")
    normal_vehicle_result = mark_spawn_satisfaction(spawn_vehicle_group(world, bp_lib, catalog["normal_vehicles"], REQUIRED_COUNTS["normal_vehicles"], safe_spawn_points.copy(), TM_PORT, "opponent", actor_ids), "normal_vehicles")
    emergency_result = mark_spawn_satisfaction(spawn_vehicle_group(world, bp_lib, catalog["emergency_vehicles"], REQUIRED_COUNTS["emergency_vehicles"], safe_spawn_points.copy(), TM_PORT, "emergency", actor_ids), "emergency_vehicles")
    bicycle_result = mark_spawn_satisfaction(spawn_bicycles_distinct(world, bp_lib, catalog["bicycles"], REQUIRED_COUNTS["bicycles"], safe_spawn_points.copy(), TM_PORT, actor_ids), "bicycles")
    walker_result = mark_spawn_satisfaction(spawn_walkers(world, bp_lib, catalog["walkers"], catalog["walker_controllers"], REQUIRED_COUNTS["walkers"], actor_ids, controller_ids), "walkers")
    animal_result = mark_spawn_satisfaction(spawn_animal(world, bp_lib, catalog["animals"], safe_spawn_points.copy(), actor_ids), "animals")

    spawned_model_ids = ordered_unique([a["type_id"] for a in normal_vehicle_result.get("actors", [])] + [a["type_id"] for a in emergency_result.get("actors", [])])
    vehicle_model_fill_result = spawn_vehicle_model_fillers(world, bp_lib, catalog["vehicle_models"], spawned_model_ids, REQUIRED_COUNTS["vehicle_models"], safe_spawn_points.copy(), actor_ids)
    vehicle_models_result = summarize_vehicle_models(catalog, {"actors": normal_vehicle_result.get("actors", []) + vehicle_model_fill_result.get("actors", [])}, emergency_result)

    sim_state.scene_actors = actor_ids
    sim_state.scene_walker_controllers = controller_ids
    scene_summary = {
        "vehicle_models": vehicle_models_result,
        "traffic_standards": traffic_result,
        "barriers": barrier_result,
        "covers": cover_result,
        "manholes": manhole_result,
        "normal_vehicles": normal_vehicle_result,
        "emergency_vehicles": emergency_result,
        "walkers": walker_result,
        "bicycles": bicycle_result,
        "animals": animal_result,
        "runtime_actor_count": len(actor_ids),
        "runtime_walker_controller_count": len(controller_ids),
    }
    scene_summary["compliance"] = build_scene_compliance_summary(scene_summary)
    sim_state.scene_summary = scene_summary

# ==========================================
# 4. 【战役枢纽】：绝对无干扰直通引擎
# ==========================================
def master_simulation_loop(client, vehicle_actor, sensors_list, stop_event, dyn_wrapper):
    telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    TARGET_PORTS = [5000, 5002, 5003] 
    
    ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ctrl_enabled = True
    try:
        ctrl_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        ctrl_sock.bind(("0.0.0.0", 5001))
    except Exception as e:
        ctrl_enabled = False
        print(f"UDP 5001 绑定失败，外部控制将暂不可用: {e}") 
    ctrl_sock.setblocking(False)

    world = client.get_world()
    spectator = world.get_spectator()
    last_mode = None
    last_pause_state = None
    ui_update_counter = 0
    tick_event_id = None

    try:
        def sync_camera_on_tick(world_snapshot):
            if not sim_state.is_paused and vehicle_actor and vehicle_actor.is_alive:
                t_veh = vehicle_actor.get_transform()
                fv = t_veh.get_forward_vector()
                box = vehicle_actor.bounding_box.extent
                cam_dist = (box.x * 2) + 4.5
                cam_height = (box.z * 2) + 1.5
                cam_x = t_veh.location.x - (fv.x * cam_dist)
                cam_y = t_veh.location.y - (fv.y * cam_dist)
                cam_z = t_veh.location.z + cam_height
                target_loc = carla.Location(x=cam_x, y=cam_y, z=cam_z)
                target_rot = carla.Rotation(pitch=-15.0, yaw=t_veh.rotation.yaw, roll=0.0)
                spectator.set_transform(carla.Transform(target_loc, target_rot))
        
        tick_event_id = world.on_tick(sync_camera_on_tick)

        while not stop_event.is_set():
            if not vehicle_actor or not vehicle_actor.is_alive: break
            
            paused = sim_state.is_paused
            if paused != last_pause_state:
                try:
                    vehicle_actor.set_simulate_physics(not paused)
                except Exception as e:
                    print(f"切换物理状态失败: {e}")
                last_pause_state = paused

            if paused:
                time.sleep(0.05)
                continue 

            current_mode = sim_state.drive_mode
            if current_mode != last_mode:
                if "内置 AI" in current_mode:
                    vehicle_actor.set_autopilot(True, TM_PORT)
                else:
                    vehicle_actor.set_autopilot(False) 
                last_mode = current_mode

            if dyn_wrapper:
                telem_data = dyn_wrapper.fetch_telemetry_26_items()
                sim_state.data["FULL_TELEMETRY"] = telem_data
                
                ui_update_counter += 1
                if ui_update_counter % 2 == 0: 
                    sim_state.speed_history.append(sim_state.smoothed_speed)
                    sim_state.data["SPEED"] = sim_state.smoothed_speed

                try: 
                    payload_bytes = json.dumps(telem_data).encode('utf-8')
                    for p in TARGET_PORTS:
                        telem_sock.sendto(payload_bytes, (sim_state.target_ip, p))
                except Exception as e:
                    print(f"UDP 遥测发送失败: {e}")
                
                if ctrl_enabled:
                    try:
                        latest_ctrl = None
                        while True:
                            ctrl_bytes, _ = ctrl_sock.recvfrom(2048)
                            latest_ctrl = ctrl_bytes
                    except BlockingIOError:
                        latest_ctrl = locals().get("latest_ctrl")

                    if latest_ctrl:
                        try:
                            ctrl_dict = json.loads(latest_ctrl.decode('utf-8'))
                            command = ctrl_dict.get("command")
                            if command == "set_weather":
                                apply_weather_to_world(world, ctrl_dict.get("value", "晴天"))
                                continue
                            if command == "add_scene_elements":
                                add_scene_elements_to_current_map(client, world)
                                continue
                            if command == "clear_scene_elements":
                                cleanup_scene_elements(world)
                                continue
                            if "内置 AI" in current_mode:
                                continue
                        
                            # ✨ 绝对直通控制，剥离所有防抖与延迟，同时限制非法输入冲击
                            vehicle_actor.apply_control(carla.VehicleControl(
                                throttle=clamp_float(ctrl_dict.get('throttle', 0), 0.0, 1.0), 
                                steer=clamp_float(ctrl_dict.get('steer', 0), -1.0, 1.0), 
                                brake=clamp_float(ctrl_dict.get('brake', 0), 0.0, 1.0),
                                reverse=parse_bool(ctrl_dict.get('reverse', False)),
                                hand_brake=parse_bool(ctrl_dict.get('hand_brake', False))
                            ))
                        except Exception as e:
                            print(f"UDP 5001 解析报错: {e}")
            
            time.sleep(0.01) 
    finally:
        if tick_event_id is not None:
            try:
                world.remove_on_tick(tick_event_id)
            except Exception:
                pass
        telem_sock.close()
        ctrl_sock.close()

# ==========================================
# 5. 左侧边栏：服务器引擎与控制流水线
# ==========================================
with st.sidebar:
    st.title("🛰️ 仿真主控流水线")
    
    # --- 1. 底层连接 ---
    st.subheader("1. 建立底层连接")
    host = st.text_input("主机 IP", "127.0.0.1")
    port = st.number_input("端口", 2000)
    if st.button("🔗 连接 Carla 引擎", use_container_width=True):
        try:
            st.session_state.client = carla.Client(host, port)
            st.session_state.client.set_timeout(10.0) 
            st.session_state.world = st.session_state.client.get_world()
            apply_world_runtime_settings(st.session_state.world)
            st.session_state.last_weather_key = None
            st.success("Carla 连接成功！底层引擎已激活！")
        except Exception as e: st.error(f"连接失败: {e}")

    st.divider()

    # --- 2. 路由中心提权 ---
    st.subheader("2. 全局路由中心 (控制流向)")
    mode_options = [
        "🤖 内置 AI 巡航模式 (纯漫游，无挂载)",
        "🧠 自动驾驶域控模式 (算法对接 | 将自动后台拉起脚本)",
        "🛞 硬件在环手动模式 (台架驾驶，无挂载)"
    ]
    default_mode_index = 2 if sim_state.drive_mode not in mode_options else mode_options.index(sim_state.drive_mode)
    sim_state.drive_mode = st.radio("请首先选择底层控制模式：", mode_options, index=default_mode_index)
    is_algo_mode = "算法对接" in sim_state.drive_mode
    if "硬件在环" in sim_state.drive_mode:
        st.success(f"正式台架 HIL 已就绪：Windows VCU {FORMAL_WINDOWS_VCU_IP} -> Ubuntu Carla {FORMAL_UBUNTU_CARLA_IP}:5001")
    elif "内置 AI" in sim_state.drive_mode:
        st.warning("当前为内置 AI 模式，外部 UDP 5001 油门/刹车/转向控制会被拦截。正式台架请切回【硬件在环手动模式】。")

    st.divider()

    # --- 3. 场景与天气矩阵 ---
    st.subheader("3. 考场与环境配置")
    if st.session_state.client: avail_maps = sorted([m.split('/')[-1] for m in st.session_state.client.get_available_maps()])
    else: avail_maps = ["Town01", "Town02", "Town03", "Town04", "Town05"]
        
    map_choice = st.selectbox("🌎 选择测试地图", avail_maps)
    map_base_name = map_choice.replace(".xodr", "")

    # ✨ 完美气象与光照矩阵
    col_w1, col_w2 = st.columns(2)
    weather_cond = col_w1.selectbox("⛅ 天气状态", ["☀️ 晴天", "☁️ 多云", "🌧️ 小雨", "⛈️ 暴雨", "雾 大雾"])
    time_of_day = col_w2.selectbox("🌞 日照时间", ["🕛 正午", "🌇 夕阳", "🌙 深夜"])

    def apply_custom_weather(world, w_cond, t_day):
        apply_weather_to_world(world, w_cond, t_day)

    weather_key = (weather_cond, time_of_day)
    if st.session_state.client and st.session_state.get('world') and st.session_state.last_weather_key != weather_key:
        apply_custom_weather(st.session_state.world, weather_cond, time_of_day)
        st.session_state.last_weather_key = weather_key

    # UI联动指示
    if is_algo_mode:
        if map_base_name in SCENARIO_DATABASE:
            cfg = SCENARIO_DATABASE[map_base_name]
            st.success(f"🎯 **算法工况匹配成功：**\n\n**任务：** {cfg['task']}\n\n💻 **将自动拉起：** `{cfg['script']}`")
        else:
            st.warning(f"⚠️ {map_base_name} 不在工况库，发车点将随机，并且不会拉起任何脚本。")
    else:
        st.info("🌍 当前为【漫游/台架模式】，车辆在目标地图随机生成，无需挂载脚本。")

    if st.button("🗺️ 应用地图与环境", use_container_width=True) and st.session_state.client:
        with st.spinner("重构世界中..."): 
            cleanup_all()
            st.session_state.world = st.session_state.client.load_world(map_choice)
            apply_world_runtime_settings(st.session_state.world)
            st.session_state.last_weather_key = None

    st.divider()
    
    # --- 4. 出库与交通 ---
    st.subheader("4. 部署与监测")
    col_a, col_b = st.columns(2)
    if col_a.button("➕ 加载交通", use_container_width=True):
        if st.session_state.client and st.session_state.world:
            with st.spinner("加载十类场景要素并执行验收统计..."):
                add_scene_elements_to_current_map(st.session_state.client, st.session_state.world)
                compliance = (sim_state.scene_summary or {}).get("compliance", {})
                if compliance.get("all_satisfied"):
                    st.success("十类场景要素全部达标！")
                else:
                    st.warning("场景要素已加载，但存在未达标项，请查看主界面验收汇总。")
        else: st.error("未连接！")
    
    if col_b.button("🧹 清空交通", use_container_width=True):
        if st.session_state.world:
            cleanup_scene_elements(st.session_state.world)
            st.success("已清场！")

    sim_state.target_ip = st.text_input("外部接收 IP (UDP多播)", sim_state.target_ip)
    
    # ✨ 核心修复：纯粹的独立暂停按钮，绝不卡循环！
    if st.button("⏸️ 暂停 / ▶️ 恢复 物理引擎", use_container_width=True):
        st.session_state.sim_paused = not st.session_state.sim_paused
        sim_state.is_paused = st.session_state.sim_paused
    
    st.info(f"引擎状态: {'⏸️ 已暂停 (定格)' if st.session_state.sim_paused else '▶️ 实时解算中'}")

    sim_state.filter_alpha = st.slider("数据防抖 (EMA)", 0.01, 1.0, 0.15)

    st.divider()
    deploy_btn = st.button("🚀 出库！执行车辆部署", type="primary", use_container_width=True)
    live_stream = st.toggle("📡 开启 20Hz 实时大屏流", value=False)

# ==========================================
# 6. 主界面：底层 API 全量暴露出库大厅
# ==========================================
st.title("🚗 L4 极致全要素标定大厅")

if not st.session_state.client: st.stop()
vehicle_files = sorted([f for f in os.listdir(VEHICLE_DIR) if f.endswith('.json')]) if os.path.exists(VEHICLE_DIR) else []
if not vehicle_files: st.stop()
selected_file = st.selectbox("📂 调取车型核心资产 (JSON)", vehicle_files)

vehicle_path = os.path.join(VEHICLE_DIR, selected_file)
v = load_vehicle_config(vehicle_path, os.path.getmtime(vehicle_path))

t1, t2, t3, t4 = st.tabs(["⚖️ 质量", "🌪️ 气动力", "🛞 底盘", "🏎️ 传动"])
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

st.divider()
c1, c2 = st.columns([3, 1])
with c1: chart_box = st.empty()
with c2: speed_box = st.empty() 

st.divider()
col_tl, col_sc = st.columns(2)
with col_tl: telemetry_box = st.empty() 
with col_sc: 
    # ✨ 专属传感器数据区
    st.markdown("#### 📡 传感器阵列真值")
    sensor_box = st.empty()

if sim_state.scene_summary:
    st.divider()
    st.markdown("#### 场景要素验收汇总")
    compliance = sim_state.scene_summary.get("compliance", {}).get("categories", {})
    label_map = {
        "vehicle_models": "车辆模型",
        "traffic_standards": "中国特色交通标准",
        "barriers": "路障",
        "covers": "覆盖物",
        "manholes": "井盖",
        "normal_vehicles": "普通对手车",
        "emergency_vehicles": "紧急对手车",
        "walkers": "行人",
        "bicycles": "自行车",
        "animals": "动物",
    }
    rows = []
    for key, item in compliance.items():
        rows.append({
            "验收项": label_map.get(key, key),
            "要求": item.get("required", 0),
            "已生成": item.get("actual", 0),
            "状态": "达标" if item.get("satisfied") else "未达标",
        })
    st.dataframe(pd.DataFrame(rows), use_container_width=True, hide_index=True)

# ==========================================
# 7. 生成出库逻辑 & 自动拉起算法 & 传感器满血挂载
# ==========================================
if deploy_btn:
    with st.spinner("🚀 重构物理网格与挂载全量探测器中..."):
        world = st.session_state.world
        bp_lib = world.get_blueprint_library()
        cleanup_simulation()
        
        bp = bp_lib.find(v['vehicle_metadata']['blueprint_id'])
        
        # 仅在算法模式下且匹配库时，强制固定点
        if is_algo_mode and map_base_name in SCENARIO_DATABASE:
            pos = SCENARIO_DATABASE[map_base_name]["pos"]
            spawn_p = carla.Transform(
                carla.Location(x=pos[0], y=pos[1], z=pos[2]),
                carla.Rotation(pitch=0.0, yaw=pos[3], roll=0.0)
            )
            st.toast(f"📍 {map_base_name} 绝对黄金坐标已锁死！")
        else:
            all_spawns = world.get_map().get_spawn_points()
            if all_spawns:
                spawn_p = random.choice(all_spawns)
                st.toast(f"🎲 已执行全地图随机发车。")
            else:
                st.error("此地图没有任何可用的生成点！")
                st.stop()
        
        try:
            vehicle = world.spawn_actor(bp, spawn_p)
            st.session_state.vehicle = vehicle
            
            box = vehicle.bounding_box.extent
            cam_dist = -(box.x * 2 + 4.5)
            cam_height = box.z * 2 + 1.5
            cam_bp = bp_lib.find('sensor.camera.rgb')
            tracking_cam = world.spawn_actor(
                cam_bp,
                carla.Transform(carla.Location(x=cam_dist, y=0, z=cam_height), carla.Rotation(pitch=-15.0)),
                attach_to=vehicle,
                attachment_type=carla.AttachmentType.SpringArmGhost
            )
            st.session_state.tracking_cam = tracking_cam
            
            # ✨ 满血传感器阵列挂载
            radar = world.spawn_actor(bp_lib.find('sensor.other.radar'), carla.Transform(carla.Location(x=2.5, z=1.0)), attach_to=vehicle)
            radar.listen(radar_callback)
            st.session_state.active_sensors.append(radar)
            
            col_sensor = world.spawn_actor(bp_lib.find('sensor.other.collision'), carla.Transform(), attach_to=vehicle)
            col_sensor.listen(collision_callback)
            st.session_state.active_sensors.append(col_sensor)

            imu_sensor = world.spawn_actor(bp_lib.find('sensor.other.imu'), carla.Transform(), attach_to=vehicle)
            imu_sensor.listen(imu_callback)
            st.session_state.active_sensors.append(imu_sensor)

            gnss_sensor = world.spawn_actor(bp_lib.find('sensor.other.gnss'), carla.Transform(), attach_to=vehicle)
            gnss_sensor.listen(gnss_callback)
            st.session_state.active_sensors.append(gnss_sensor)

            wrapper = L4_DynamicsWrapper(vehicle, v, ui_params, world)
            st.session_state.dynamics_wrapper = wrapper
            st.session_state.stop_event.clear()
            
            t = threading.Thread(target=master_simulation_loop, args=(
                st.session_state.client, vehicle, st.session_state.active_sensors, st.session_state.stop_event, wrapper))
            t.daemon = True
            t.start()
            st.session_state.master_thread = t
            st.success(f"🔥 车辆与 4 大类全要素传感器部署完成！")

            # ✨ 核心：全自动后台拉起对应的算法脚本！
            if is_algo_mode and map_base_name in SCENARIO_DATABASE:
                script_name = SCENARIO_DATABASE[map_base_name]["script"]
                try:
                    script_path = os.path.join(BASE_DIR, script_name)
                    st.session_state.algo_process = subprocess.Popen([sys.executable, script_path], cwd=BASE_DIR)
                    st.success(f"🚀 算法域控制器 `{script_name}` 已在后台成功拉起！")
                except Exception as e:
                    st.error(f"后台启动算法脚本失败，请检查脚本路径或 Python 环境配置: {e}")

        except Exception as e:
            cleanup_simulation()
            st.error(f"生成失败，已自动回收半初始化资源: {e}")

# ==========================================
# 8. 遥测大屏动态局部渲染
# ==========================================
if st.session_state.vehicle and not st.session_state.stop_event.is_set():
    if live_stream:
        render_telemetry_dashboard(chart_box, speed_box, telemetry_box, sensor_box, live_mode=True)
        time.sleep(0.05)
        request_streamlit_rerun()
    else:
        render_telemetry_dashboard(chart_box, speed_box, telemetry_box, sensor_box, live_mode=False)
