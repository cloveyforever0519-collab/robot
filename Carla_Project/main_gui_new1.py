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
from collections import deque
from typing import Dict, List

# ==========================================
# 0. 场景要素配置
# ==========================================
TM_PORT = 8010

REQUIRED_COUNTS = {
    "vehicle_models": 10,
    "traffic_standards": 10,
    "barriers": 16,
    "normal_vehicles": 15,
    "emergency_vehicles": 1,
    "walkers": 5,
    "bicycles": 2,
    "animals": 1
}

# ==========================================
# 1. 核心初始化
# ==========================================
VEHICLE_DIR = "output"
st.set_page_config(
    page_title="L4 级全要素标定 | 集成版",
    layout="wide",
    initial_sidebar_state="expanded"
)

class SimulationState:
    def __init__(self):
        self.reset()
        self.is_paused = False
        self.drive_mode = "🤖 内置 AI 巡航模式 (Carla Traffic Manager)"
        self.filter_alpha = 0.15
        self.smoothed_speed = 0.0

    def reset(self):
        self.data = {
            "SPEED": 0.0,
            "GNSS": "等待...",
            "IMU": "等待...",
            "RADAR": "等待...",
            "CAM_LIDAR": "等待...",
            "RADAR_TARGETS": 0,
            "COLLISION_DATA": {"Impulse": [0, 0, 0], "Actor": "None"},
            "LANE_DATA": {"Side": "None", "Type": "None"},
            "IMU_DATA": {"Accel": [0, 0, 0], "Gyro": [0, 0, 0], "Compass": 0.0},
            "GNSS_DATA": [0.0, 0.0, 0.0],
            "FULL_TELEMETRY": {}
        }
        self.frame_count = 0
        self.speed_history = deque(maxlen=120)
        self.smoothed_speed = 0.0

@st.cache_resource
def get_sim_state():
    return SimulationState()

sim_state = get_sim_state()

keys_to_init = [
    'client', 'world', 'vehicle', 'active_sensors', 'stop_event',
    'master_thread', 'dynamics_wrapper', 'tracking_cam',
    'scene_actors', 'scene_walker_controllers', 'scene_summary'
]
for key in keys_to_init:
    if key not in st.session_state:
        if key == 'active_sensors':
            st.session_state[key] = []
        elif key == 'stop_event':
            st.session_state[key] = threading.Event()
        elif key in ['scene_actors', 'scene_walker_controllers']:
            st.session_state[key] = []
        else:
            st.session_state[key] = None

# ==========================================
# 2. 物理包装器
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
        pc.center_of_gravity = carla.Vector3D(
            x=self.ui.get('cg_x', 0),
            y=self.ui.get('cg_y', 0),
            z=self.ui.get('cg_z', 0)
        )
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
            if i < 2:
                w.max_steer_angle = self.ui.get('steer', 40.0)
            else:
                w.max_steer_angle = 0.0
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
            except Exception:
                pass
            time.sleep(0.01)

    def fetch_telemetry_26_items(self):
        if not self.vehicle or not self.vehicle.is_alive:
            return {}
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
        if speed_ms < 0.1 and ctrl.throttle > 0:
            slip_ratio = 5.0
        wheel_rpm = [base_rpm * slip_ratio, base_rpm * slip_ratio, base_rpm, base_rpm]

        bounce_z = [0.0, 0.0, 0.0, 0.0]
        try:
            if hasattr(self.vehicle, 'get_bones'):
                for bone in self.vehicle.get_bones():
                    b_name = bone.name.lower()
                    if 'wheel_fl' in b_name:
                        bounce_z[0] = round(bone.world_transform.location.z * 1000, 1)
                    elif 'wheel_fr' in b_name:
                        bounce_z[1] = round(bone.world_transform.location.z * 1000, 1)
                    elif 'wheel_rl' in b_name:
                        bounce_z[2] = round(bone.world_transform.location.z * 1000, 1)
                    elif 'wheel_rr' in b_name:
                        bounce_z[3] = round(bone.world_transform.location.z * 1000, 1)
        except Exception:
            pass

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
            }
        }

    def destroy(self):
        self.is_active = False
        if hasattr(self, 'aero_thread'):
            self.aero_thread.join(timeout=1.0)

# ==========================================
# 3. 清理
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
                s.destroy()
            except Exception:
                pass
    st.session_state.active_sensors = []

    if st.session_state.tracking_cam and st.session_state.tracking_cam.is_alive:
        try:
            st.session_state.tracking_cam.destroy()
        except Exception:
            pass
    st.session_state.tracking_cam = None

    if st.session_state.vehicle and st.session_state.vehicle.is_alive:
        try:
            st.session_state.vehicle.destroy()
        except Exception:
            pass
    st.session_state.vehicle = None

    sim_state.reset()

def cleanup_scene_elements():
    world = st.session_state.world
    if world is None:
        return

    ids = st.session_state.scene_walker_controllers + st.session_state.scene_actors
    for actor_id in ids:
        actor = world.get_actor(actor_id)
        if actor is not None:
            try:
                actor.destroy()
            except Exception:
                pass

    st.session_state.scene_actors = []
    st.session_state.scene_walker_controllers = []
    st.session_state.scene_summary = None

def cleanup_all():
    cleanup_simulation()
    cleanup_scene_elements()

def radar_callback(data):
    sim_state.data["RADAR_TARGETS"] = len(data)

def collision_callback(data):
    sim_state.data["COLLISION_DATA"] = {
        "Impulse": [
            round(data.normal_impulse.x, 1),
            round(data.normal_impulse.y, 1),
            round(data.normal_impulse.z, 1)
        ],
        "Actor": str(data.other_actor.type_id)
    }

# ==========================================
# 4. 场景要素辅助函数（由第二个文件集成）
# ==========================================
def ordered_unique(items: List[str]) -> List[str]:
    seen = set()
    out = []
    for x in items:
        if x not in seen:
            seen.add(x)
            out.append(x)
    return out

def actor_to_dict(actor: carla.Actor) -> Dict:
    tf = actor.get_transform()
    return {
        "id": actor.id,
        "type_id": actor.type_id,
        "transform": {
            "x": round(tf.location.x, 3),
            "y": round(tf.location.y, 3),
            "z": round(tf.location.z, 3),
            "pitch": round(tf.rotation.pitch, 3),
            "yaw": round(tf.rotation.yaw, 3),
            "roll": round(tf.rotation.roll, 3)
        }
    }

def offset_transform(base_tf: carla.Transform, forward=0.0, right=0.0, up=0.1, yaw_bias=0.0) -> carla.Transform:
    yaw = math.radians(base_tf.rotation.yaw)
    fx, fy = math.cos(yaw), math.sin(yaw)
    rx, ry = -math.sin(yaw), math.cos(yaw)

    loc = carla.Location(
        x=base_tf.location.x + fx * forward + rx * right,
        y=base_tf.location.y + fy * forward + ry * right,
        z=base_tf.location.z + up
    )
    rot = carla.Rotation(
        pitch=base_tf.rotation.pitch,
        yaw=base_tf.rotation.yaw + yaw_bias,
        roll=base_tf.rotation.roll
    )
    return carla.Transform(loc, rot)

def get_all_bp_ids(bp_lib: carla.BlueprintLibrary) -> List[str]:
    return [bp.id for bp in bp_lib.filter("*")]

def resolve_catalog(bp_lib: carla.BlueprintLibrary) -> Dict[str, List[str]]:
    all_ids = set(get_all_bp_ids(bp_lib))

    vehicle_ids = [bp.id for bp in bp_lib.filter("vehicle.*")]
    walker_ids = [bp.id for bp in bp_lib.filter("walker.pedestrian.*")]
    controller_ids = [bp.id for bp in bp_lib.filter("controller.ai.walker")]

    traffic_pref = [
        "traffic.speed_limit.30",
        "traffic.speed_limit.40",
        "traffic.speed_limit.50",
        "traffic.speed_limit.60",
        "traffic.speed_limit.90",
        "traffic.stop",
        "traffic.yield",
        "static.prop.trafficwarning",
        "static.prop.warningaccident",
        "static.prop.warningconstruction",
        "static.prop.streetbarrier",
        "static.prop.constructioncone",
        "static.prop.chainbarrier",
        "static.prop.chainbarrierend",
        "static.prop.trafficcone01",
        "static.prop.trafficcone02",
    ]
    traffic_standards = ordered_unique([x for x in traffic_pref if x in all_ids])

    traffic_fill_pool = [
        "static.prop.ironplank",
        "static.prop.brokentile01",
        "static.prop.brokentile02",
        "static.prop.brokentile03",
        "static.prop.brokentile04",
        "static.prop.dirtdebris01",
        "static.prop.dirtdebris02",
        "static.prop.dirtdebris03",
    ]
    for x in traffic_fill_pool:
        if x in all_ids and x not in traffic_standards:
            traffic_standards.append(x)
        if len(traffic_standards) >= REQUIRED_COUNTS["traffic_standards"]:
            break

    barrier_pref = [
        "static.prop.streetbarrier",
        "static.prop.constructioncone",
        "static.prop.trafficcone01",
        "static.prop.trafficcone02",
        "static.prop.chainbarrier",
        "static.prop.chainbarrierend",
        "static.prop.warningconstruction",
        "static.prop.trafficwarning",
        "static.prop.warningaccident",
        "static.prop.ironplank",
        "static.prop.brokentile01",
        "static.prop.brokentile02",
        "static.prop.brokentile03",
        "static.prop.brokentile04",
        "static.prop.dirtdebris01",
        "static.prop.dirtdebris02",
        "static.prop.dirtdebris03"
    ]
    barriers = ordered_unique([x for x in barrier_pref if x in all_ids])

    bicycle_pref = [
        "vehicle.bh.crossbike",
        "vehicle.diamondback.century",
        "vehicle.gazelle.omafiets",
    ]
    bicycles = [x for x in bicycle_pref if x in all_ids]

    emergency_keywords = ["firetruck", "ambulance", "police"]
    emergency_vehicles = [
        x for x in vehicle_ids
        if any(k in x.lower() for k in emergency_keywords)
    ]

    moto_keywords = ["yamaha", "harley", "vespa", "kawasaki", "bike"]
    normal_vehicles = [
        x for x in vehicle_ids
        if x not in bicycles
        and x not in emergency_vehicles
        and not any(k in x.lower() for k in moto_keywords)
    ]

    animals = [
        x for x in get_all_bp_ids(bp_lib)
        if (
            any(k in x.lower() for k in ["animal", "deer", "horse"])
            or x.lower().endswith(".dog")
            or x.lower().endswith(".cat")
        )
        and "doghouse" not in x.lower()
    ]

    vehicle_models = ordered_unique(normal_vehicles + emergency_vehicles)

    return {
        "vehicle_models": vehicle_models,
        "traffic_standards": traffic_standards,
        "barriers": barriers,
        "normal_vehicles": normal_vehicles,
        "emergency_vehicles": emergency_vehicles,
        "walkers": walker_ids,
        "walker_controllers": controller_ids,
        "bicycles": bicycles,
        "animals": animals
    }

def summarize_vehicle_models(
    catalog: Dict[str, List[str]],
    normal_vehicle_result: Dict,
    emergency_result: Dict
) -> Dict:
    spawned_model_ids = ordered_unique(
        [a["type_id"] for a in normal_vehicle_result.get("actors", [])] +
        [a["type_id"] for a in emergency_result.get("actors", [])]
    )

    return {
        "requested": REQUIRED_COUNTS["vehicle_models"],
        "available_blueprints": len(catalog.get("vehicle_models", [])),
        "spawned_unique_models": len(spawned_model_ids),
        "model_type_ids": spawned_model_ids,
        "satisfied": len(spawned_model_ids) >= REQUIRED_COUNTS["vehicle_models"]
    }

def spawn_static_objects(
    world: carla.World,
    bp_lib: carla.BlueprintLibrary,
    spawn_points: List[carla.Transform],
    bp_ids: List[str],
    desired_count: int,
    record_ids: List[int],
    right_bias: float
) -> Dict:
    created = []
    used = []
    warnings = []

    if not bp_ids:
        return {
            "requested": desired_count,
            "available_blueprints": 0,
            "spawned": 0,
            "actors": [],
            "blueprints_used": [],
            "warnings": ["no available blueprint"]
        }

    random.shuffle(spawn_points)
    selected = bp_ids[:desired_count]

    for i, bp_id in enumerate(selected):
        base_tf = spawn_points[i % len(spawn_points)]
        tf = offset_transform(
            base_tf,
            forward=(i % 4) * 1.5,
            right=right_bias + (i % 3) * 1.2,
            up=0.1,
            yaw_bias=90.0
        )

        bp = bp_lib.find(bp_id)
        actor = world.try_spawn_actor(bp, tf)
        if actor is None:
            warnings.append(f"spawn failed: {bp_id}")
            continue

        try:
            actor.set_simulate_physics(False)
        except Exception:
            pass

        record_ids.append(actor.id)
        used.append(bp_id)
        created.append(actor_to_dict(actor))

    return {
        "requested": desired_count,
        "available_blueprints": len(bp_ids),
        "spawned": len(created),
        "actors": created,
        "blueprints_used": used,
        "warnings": warnings
    }

def spawn_vehicle_group(
    world: carla.World,
    bp_lib: carla.BlueprintLibrary,
    bp_ids: List[str],
    desired_count: int,
    spawn_points: List[carla.Transform],
    tm_port: int,
    role_name: str,
    record_ids: List[int]
) -> Dict:
    created = []
    used = []
    warnings = []

    if not bp_ids:
        return {
            "requested": desired_count,
            "available_blueprints": 0,
            "spawned": 0,
            "actors": [],
            "blueprints_used": [],
            "warnings": ["no available blueprint"]
        }

    random.shuffle(spawn_points)
    attempts = 0
    max_attempts = max(len(spawn_points) * 2, desired_count * 5)

    while len(created) < desired_count and attempts < max_attempts:
        bp_id = bp_ids[attempts % len(bp_ids)]
        tf = spawn_points[attempts % len(spawn_points)]
        bp = bp_lib.find(bp_id)

        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", role_name)
        if bp.has_attribute("color"):
            vals = bp.get_attribute("color").recommended_values
            if vals:
                bp.set_attribute("color", random.choice(vals))

        actor = world.try_spawn_actor(bp, tf)
        attempts += 1

        if actor is None:
            continue

        actor.set_autopilot(True, tm_port)

        try:
            world.wait_for_tick()
        except Exception:
            time.sleep(0.05)

        record_ids.append(actor.id)
        used.append(bp_id)
        created.append(actor_to_dict(actor))

    if len(created) < desired_count:
        warnings.append(f"only spawned {len(created)}/{desired_count}")

    return {
        "requested": desired_count,
        "available_blueprints": len(bp_ids),
        "spawned": len(created),
        "actors": created,
        "blueprints_used": used,
        "warnings": warnings
    }

def spawn_bicycles_distinct(
    world: carla.World,
    bp_lib: carla.BlueprintLibrary,
    bicycle_bp_ids: List[str],
    desired_count: int,
    spawn_points: List[carla.Transform],
    tm_port: int,
    record_ids: List[int]
) -> Dict:
    created = []
    used = []
    warnings = []

    if not bicycle_bp_ids:
        return {
            "requested": desired_count,
            "available_blueprints": 0,
            "spawned": 0,
            "actors": [],
            "blueprints_used": [],
            "warnings": ["no bicycle blueprint"]
        }

    target_unique = min(desired_count, len(bicycle_bp_ids))
    random.shuffle(spawn_points)

    for bp_id in bicycle_bp_ids:
        if len(created) >= target_unique:
            break

        bp = bp_lib.find(bp_id)
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", "bicycle")

        spawned = False
        for tf in spawn_points:
            actor = world.try_spawn_actor(bp, tf)
            if actor is None:
                continue

            actor.set_autopilot(True, tm_port)

            try:
                world.wait_for_tick()
            except Exception:
                time.sleep(0.05)

            record_ids.append(actor.id)
            used.append(bp_id)
            created.append(actor_to_dict(actor))
            spawned = True
            break

        if not spawned:
            warnings.append(f"spawn failed for bicycle blueprint: {bp_id}")

    if len(created) < desired_count:
        warnings.append(f"only spawned {len(created)}/{desired_count} bicycles with distinct blueprints")

    return {
        "requested": desired_count,
        "available_blueprints": len(bicycle_bp_ids),
        "spawned": len(created),
        "actors": created,
        "blueprints_used": used,
        "warnings": warnings
    }

def spawn_vehicle_model_fillers(
    world: carla.World,
    bp_lib: carla.BlueprintLibrary,
    all_vehicle_model_bp_ids: List[str],
    existing_model_ids: List[str],
    desired_unique_count: int,
    spawn_points: List[carla.Transform],
    record_ids: List[int]
) -> Dict:
    created = []
    used = []
    warnings = []

    missing_count = desired_unique_count - len(existing_model_ids)
    if missing_count <= 0:
        return {
            "requested": desired_unique_count,
            "spawned": 0,
            "actors": [],
            "blueprints_used": [],
            "warnings": []
        }

    candidates = [x for x in all_vehicle_model_bp_ids if x not in existing_model_ids]
    random.shuffle(spawn_points)

    attempts = 0
    max_attempts = max(len(spawn_points) * 2, missing_count * 5)

    while len(used) < missing_count and attempts < max_attempts and candidates:
        bp_id = candidates.pop(0)
        tf = spawn_points[attempts % len(spawn_points)]
        attempts += 1

        bp = bp_lib.find(bp_id)
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", "vehicle_model_fill")

        actor = world.try_spawn_actor(bp, tf)
        if actor is None:
            continue

        try:
            world.wait_for_tick()
        except Exception:
            time.sleep(0.05)

        record_ids.append(actor.id)
        used.append(bp_id)
        created.append(actor_to_dict(actor))

    if len(used) < missing_count:
        warnings.append(f"only filled {len(used)}/{missing_count} extra vehicle models")

    return {
        "requested": desired_unique_count,
        "spawned": len(used),
        "actors": created,
        "blueprints_used": used,
        "warnings": warnings
    }

def spawn_walkers(
    world: carla.World,
    bp_lib: carla.BlueprintLibrary,
    walker_bp_ids: List[str],
    controller_bp_ids: List[str],
    desired_count: int,
    record_actor_ids: List[int],
    record_controller_ids: List[int]
) -> Dict:
    created = []
    used = []
    warnings = []

    if not walker_bp_ids:
        return {
            "requested": desired_count,
            "available_blueprints": 0,
            "spawned": 0,
            "actors": [],
            "blueprints_used": [],
            "warnings": ["no pedestrian blueprints"]
        }

    if not controller_bp_ids:
        return {
            "requested": desired_count,
            "available_blueprints": len(walker_bp_ids),
            "spawned": 0,
            "actors": [],
            "blueprints_used": [],
            "warnings": ["no walker controller blueprint"]
        }

    controller_bp = bp_lib.find(controller_bp_ids[0])

    attempts = 0
    max_attempts = desired_count * 10

    while len(created) < desired_count and attempts < max_attempts:
        bp_id = random.choice(walker_bp_ids)
        nav_loc = world.get_random_location_from_navigation()
        attempts += 1

        if nav_loc is None:
            continue

        walker_tf = carla.Transform(nav_loc)
        walker_bp = bp_lib.find(bp_id)

        walker = world.try_spawn_actor(walker_bp, walker_tf)
        if walker is None:
            continue

        controller = world.try_spawn_actor(controller_bp, carla.Transform(), walker)
        if controller is None:
            walker.destroy()
            continue

        controller.start()
        target = world.get_random_location_from_navigation()
        if target:
            controller.go_to_location(target)
        controller.set_max_speed(1.2 + random.random())

        try:
            world.wait_for_tick()
        except Exception:
            time.sleep(0.05)

        record_actor_ids.append(walker.id)
        record_controller_ids.append(controller.id)
        used.append(bp_id)
        created.append(actor_to_dict(walker))

    if len(created) < desired_count:
        warnings.append(f"only spawned {len(created)}/{desired_count}")

    return {
        "requested": desired_count,
        "available_blueprints": len(walker_bp_ids),
        "spawned": len(created),
        "actors": created,
        "blueprints_used": used,
        "warnings": warnings
    }

def spawn_animal(
    world: carla.World,
    bp_lib: carla.BlueprintLibrary,
    animal_bp_ids: List[str],
    spawn_points: List[carla.Transform],
    record_ids: List[int]
) -> Dict:
    animal_bp_ids = [x for x in animal_bp_ids if "doghouse" not in x.lower()]

    if not animal_bp_ids:
        return {
            "requested": 1,
            "available_blueprints": 0,
            "spawned": 0,
            "actors": [],
            "blueprints_used": [],
            "warnings": ["no real animal blueprint in current CARLA installation"]
        }

    bp_id = animal_bp_ids[0]
    base_tf = random.choice(spawn_points)
    tf = offset_transform(base_tf, forward=2.0, right=6.0, up=0.1)

    bp = bp_lib.find(bp_id)
    actor = world.try_spawn_actor(bp, tf)
    if actor is None:
        return {
            "requested": 1,
            "available_blueprints": len(animal_bp_ids),
            "spawned": 0,
            "actors": [],
            "blueprints_used": [],
            "warnings": [f"spawn failed: {bp_id}"]
        }

    try:
        actor.set_simulate_physics(False)
        world.wait_for_tick()
    except Exception:
        time.sleep(0.05)

    record_ids.append(actor.id)
    return {
        "requested": 1,
        "available_blueprints": len(animal_bp_ids),
        "spawned": 1,
        "actors": [actor_to_dict(actor)],
        "blueprints_used": [bp_id],
        "warnings": []
    }

def add_scene_elements_to_current_map():
    if st.session_state.client is None or st.session_state.world is None:
        st.error("请先建立 CARLA 连接并加载地图")
        return

    world = st.session_state.world
    client = st.session_state.client

    cleanup_scene_elements()

    bp_lib = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        st.error("当前地图没有可用 spawn points")
        return

    try:
        world.wait_for_tick()
    except Exception:
        time.sleep(0.1)

    catalog = resolve_catalog(bp_lib)

    tm = client.get_trafficmanager(TM_PORT)
    tm.set_global_distance_to_leading_vehicle(2.5)

    actor_ids = []
    controller_ids = []

    traffic_result = spawn_static_objects(
        world, bp_lib, spawn_points.copy(),
        catalog["traffic_standards"],
        REQUIRED_COUNTS["traffic_standards"],
        actor_ids,
        right_bias=5.0
    )

    barrier_result = spawn_static_objects(
        world, bp_lib, spawn_points.copy(),
        catalog["barriers"],
        REQUIRED_COUNTS["barriers"],
        actor_ids,
        right_bias=7.0
    )

    normal_vehicle_result = spawn_vehicle_group(
        world, bp_lib,
        catalog["normal_vehicles"],
        REQUIRED_COUNTS["normal_vehicles"],
        spawn_points.copy(),
        TM_PORT,
        "opponent",
        actor_ids
    )

    emergency_result = spawn_vehicle_group(
        world, bp_lib,
        catalog["emergency_vehicles"],
        REQUIRED_COUNTS["emergency_vehicles"],
        spawn_points.copy(),
        TM_PORT,
        "emergency",
        actor_ids
    )

    bicycle_result = spawn_bicycles_distinct(
        world,
        bp_lib,
        catalog["bicycles"],
        REQUIRED_COUNTS["bicycles"],
        spawn_points.copy(),
        TM_PORT,
        actor_ids
    )

    walker_result = spawn_walkers(
        world, bp_lib,
        catalog["walkers"],
        catalog["walker_controllers"],
        REQUIRED_COUNTS["walkers"],
        actor_ids,
        controller_ids
    )

    animal_result = spawn_animal(
        world, bp_lib,
        catalog["animals"],
        spawn_points.copy(),
        actor_ids
    )

    spawned_model_ids = ordered_unique(
        [a["type_id"] for a in normal_vehicle_result.get("actors", [])] +
        [a["type_id"] for a in emergency_result.get("actors", [])]
    )

    vehicle_model_fill_result = spawn_vehicle_model_fillers(
        world,
        bp_lib,
        catalog["vehicle_models"],
        spawned_model_ids,
        REQUIRED_COUNTS["vehicle_models"],
        spawn_points.copy(),
        actor_ids
    )

    vehicle_models_result = summarize_vehicle_models(
        catalog,
        {
            "actors": normal_vehicle_result.get("actors", []) + vehicle_model_fill_result.get("actors", [])
        },
        emergency_result
    )

    st.session_state.scene_actors = actor_ids
    st.session_state.scene_walker_controllers = controller_ids
    st.session_state.scene_summary = {
        "vehicle_models": vehicle_models_result,
        "traffic_standards": traffic_result,
        "barriers": barrier_result,
        "normal_vehicles": normal_vehicle_result,
        "emergency_vehicles": emergency_result,
        "walkers": walker_result,
        "bicycles": bicycle_result,
        "animals": animal_result,
        "runtime_actor_count": len(actor_ids),
        "runtime_walker_controller_count": len(controller_ids),
    }

# ==========================================
# 5. 主循环
# ==========================================
def master_simulation_loop(client, vehicle_actor, sensors_list, stop_event, dyn_wrapper):
    telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        ctrl_sock.bind(("127.0.0.1", 5001))
    except Exception:
        pass
    ctrl_sock.setblocking(False)

    world = client.get_world()
    spectator = world.get_spectator()
    last_mode = None
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
            if not vehicle_actor or not vehicle_actor.is_alive:
                break

            if sim_state.is_paused:
                vehicle_actor.set_simulate_physics(False)
                time.sleep(0.05)
                continue
            else:
                vehicle_actor.set_simulate_physics(True)

            current_mode = sim_state.drive_mode
            if current_mode != last_mode:
                if "内置 AI" in current_mode:
                    vehicle_actor.set_autopilot(True)
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
                    telem_sock.sendto(json.dumps(telem_data).encode(), ("127.0.0.1", 5000))
                except Exception:
                    pass

                if "内置 AI" not in current_mode:
                    try:
                        ctrl_bytes, _ = ctrl_sock.recvfrom(1024)
                        ctrl_dict = json.loads(ctrl_bytes.decode())
                        vehicle_actor.apply_control(carla.VehicleControl(
                            throttle=float(ctrl_dict.get('throttle', 0)),
                            steer=float(ctrl_dict.get('steer', 0)),
                            brake=float(ctrl_dict.get('brake', 0))
                        ))
                    except BlockingIOError:
                        pass
                    except Exception:
                        pass

            time.sleep(0.01)
    finally:
        if tick_event_id is not None:
            world.remove_on_tick(tick_event_id)
        telem_sock.close()
        ctrl_sock.close()

# ==========================================
# 6. 左侧边栏
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
            st.success("Carla 连接成功！")
        except Exception as e:
            st.error(f"连接失败: {e}")

    st.divider()
    st.subheader("⏱️ 仿真引擎状态")
    engine_run = st.toggle("▶️ 物理引擎实时解算 (关闭则时间定格)", value=True)
    sim_state.is_paused = not engine_run

    st.divider()
    if st.session_state.client:
        avail_maps = sorted([m.split('/')[-1] for m in st.session_state.client.get_available_maps()])
    else:
        avail_maps = ["Town01", "Town02", "Town03", "Town04", "Town05", "Town06", "Town10HD"]

    map_choice = st.selectbox("🌎 地图载入", avail_maps)

    if st.button("🗺️ 应用地图", use_container_width=True) and st.session_state.client:
        with st.spinner("重构世界中..."):
            cleanup_all()
            st.session_state.world = st.session_state.client.load_world(map_choice)
            st.success(f"已加载地图：{map_choice}")

    add_scene_btn = st.button("➕ 添加场景要素", use_container_width=True)
    clear_scene_btn = st.button("🧹 清除场景要素", use_container_width=True)

    if add_scene_btn:
        with st.spinner("正在加载七类场景要素..."):
            add_scene_elements_to_current_map()
            st.success("七类场景要素已加载到当前地图")

    if clear_scene_btn:
        cleanup_scene_elements()
        st.success("场景要素已清除")

    st.divider()
    st.subheader("📍 场景发车策略")
    map_base_name = map_choice.replace(".xodr", "") if map_choice else "Town03"

    use_random_spawn = True
    spawn_x, spawn_y, spawn_z, spawn_yaw = 0.0, 0.0, 2.0, 0.0

    if map_base_name == "Town03":
        st.success("✅ Town03：锁定环岛黄金起跑线")
        spawn_x, spawn_y, spawn_z, spawn_yaw = 65.3, -3.8, 2.0, 180.0
        use_random_spawn = False
    elif map_base_name == "Town04":
        st.success("✅ Town04：锁定高速三车道起点")
        spawn_x, spawn_y, spawn_z, spawn_yaw = -360.0, 30.0, 2.0, 0.0
        use_random_spawn = False
    else:
        st.warning(f"⚠️ {map_base_name} 暂无定制路线，将采用【安全随机空投】")
        use_random_spawn = True

    st.divider()
    st.subheader("🎛️ 数据防抖控制 (EMA)")
    sim_state.filter_alpha = st.slider(
        "波形平滑度 (越低越平滑)",
        0.01, 1.0, 0.15,
        help="速度越快，建议调低此值以过滤物理引擎毛刺。"
    )

    st.divider()
    deploy_btn = st.button("🚀 出库！全量注入", type="primary", use_container_width=True)
    live_stream = st.toggle("📡 开启 20Hz 实时大屏流", value=False)

# ==========================================
# 7. 主界面
# ==========================================
st.title("🚗 L4 极致全要素标定大厅 & 场景要素集成版")

if not st.session_state.client:
    st.stop()

vehicle_files = [f for f in os.listdir(VEHICLE_DIR) if f.endswith('.json')] if os.path.exists(VEHICLE_DIR) else []
if not vehicle_files:
    st.warning("未找到车型 JSON 文件，请检查 output 目录。")
    st.stop()

selected_file = st.selectbox("📂 调取车型核心资产 (JSON)", vehicle_files)

with open(os.path.join(VEHICLE_DIR, selected_file), 'r', encoding='utf-8') as f:
    v = json.load(f)

t1, t2, t3, t4, t5 = st.tabs(["⚖️ 质量", "🌪️ 气动力", "🛞 底盘", "🏎️ 传动", "🖧 核心路由中心"])
ui_params = {}

with t1:
    c1, c2, c3 = st.columns(3)
    ui_params['mass'] = c1.number_input("整备质量 [kg]", value=float(v.get('weight_and_mass_properties', {}).get('curb_weight_kg', 1800)))
    ui_params['moi'] = c2.slider("发动机转动惯量 (MOI)", 0.5, 5.0, 1.0)
    cog = v.get('weight_and_mass_properties', {}).get('center_of_gravity_m', {'x': 0, 'y': 0, 'z': 0})
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
    st.subheader("🖧 核心三模路由中心")
    chosen_mode = st.radio(
        "系统数据交换与控制模式",
        [
            "🤖 内置 AI 巡航模式 (Carla Traffic Manager)",
            "🧠 自动驾驶域控模式 (算法对接 | UDP 双向 5000/5001)",
            "🛞 硬件在环手动模式 (台架对接 | UDP 双向 5000/5001)"
        ],
        horizontal=False
    )
    sim_state.drive_mode = chosen_mode

st.divider()
c1, c2 = st.columns([3, 1])
with c1:
    st.subheader("📈 实时滤波动态波形图 (防毛刺)")
    chart_box = st.empty()
with c2:
    speed_box = st.empty()

st.divider()
st.subheader("📊 26项底层真值流")
telemetry_box = st.empty()

st.divider()
st.subheader("📦 场景要素加载结果")
scene_summary_box = st.empty()
if st.session_state.scene_summary is not None:
    scene_summary_box.json(st.session_state.scene_summary)

# ==========================================
# 8. 出库与挂载
# ==========================================
if deploy_btn:
    with st.spinner("🚀 重构物理网格与挂载探测器中..."):
        world = st.session_state.world
        bp_lib = world.get_blueprint_library()

        cleanup_simulation()

        bp = bp_lib.find(v['vehicle_metadata']['blueprint_id'])

        if use_random_spawn:
            spawn_pts = world.get_map().get_spawn_points()
            spawn_p = random.choice(spawn_pts)
            spawn_p.location.z += 2.5
        else:
            spawn_p = carla.Transform(
                carla.Location(x=spawn_x, y=spawn_y, z=spawn_z),
                carla.Rotation(pitch=0.0, yaw=spawn_yaw, roll=0.0)
            )

        try:
            vehicle = world.spawn_actor(bp, spawn_p)
            st.session_state.vehicle = vehicle

            box = vehicle.bounding_box.extent
            cam_dist = -(box.x * 2 + 4.5)
            cam_height = box.z * 2 + 1.5
            cam_bp = bp_lib.find('sensor.camera.rgb')
            tracking_cam = world.spawn_actor(
                cam_bp,
                carla.Transform(
                    carla.Location(x=cam_dist, y=0, z=cam_height),
                    carla.Rotation(pitch=-15.0)
                ),
                attach_to=vehicle,
                attachment_type=carla.AttachmentType.SpringArmGhost
            )
            st.session_state.tracking_cam = tracking_cam

            radar = world.spawn_actor(
                bp_lib.find('sensor.other.radar'),
                carla.Transform(carla.Location(x=2.5, z=1.0)),
                attach_to=vehicle
            )
            radar.listen(radar_callback)
            st.session_state.active_sensors.append(radar)

            col_sensor = world.spawn_actor(
                bp_lib.find('sensor.other.collision'),
                carla.Transform(),
                attach_to=vehicle
            )
            col_sensor.listen(collision_callback)
            st.session_state.active_sensors.append(col_sensor)

            wrapper = L4_DynamicsWrapper(vehicle, v, ui_params)
            st.session_state.dynamics_wrapper = wrapper
            st.session_state.stop_event.clear()

            t = threading.Thread(
                target=master_simulation_loop,
                args=(st.session_state.client, vehicle, st.session_state.active_sensors, st.session_state.stop_event, wrapper)
            )
            t.start()
            st.session_state.master_thread = t

            if use_random_spawn:
                st.toast("🔥 车辆安全着陆！已通过随机空投避开干沟！")
            else:
                st.toast(f"🔥 车辆安全着陆！已锁定 {map_base_name} 黄金坐标！")

        except Exception as e:
            st.error(f"致命错误: {e}")

# ==========================================
# 9. 实时渲染
# ==========================================
if st.session_state.vehicle and not st.session_state.stop_event.is_set():
    if live_stream:
        with st.spinner("🔴 实时真值流传输中..."):
            while live_stream and not st.session_state.stop_event.is_set():
                speed_val = sim_state.data.get("SPEED", 0.0)
                mode_icon = sim_state.drive_mode.split(' ')[0]
                speed_box.metric("⚡ 绝对滤波车速", f"{speed_val:.1f} km/h", delta=f"接入: {mode_icon}")

                if len(sim_state.speed_history) > 0:
                    df = pd.DataFrame(list(sim_state.speed_history), columns=['Speed (km/h)'])
                    chart_box.line_chart(df, height=180, use_container_width=True)

                if sim_state.data.get("FULL_TELEMETRY"):
                    telemetry_box.json(sim_state.data["FULL_TELEMETRY"])

                if st.session_state.scene_summary is not None:
                    scene_summary_box.json(st.session_state.scene_summary)

                time.sleep(0.05)
    else:
        speed_val = sim_state.data.get("SPEED", 0.0)
        speed_box.metric("⚡ 绝对滤波车速", f"{speed_val:.1f} km/h", delta="状态: 画面定格待机")

        if len(sim_state.speed_history) > 0:
            df = pd.DataFrame(list(sim_state.speed_history), columns=['Speed (km/h)'])
            chart_box.line_chart(df, height=180, use_container_width=True)

        if sim_state.data.get("FULL_TELEMETRY"):
            telemetry_box.json(sim_state.data["FULL_TELEMETRY"])

        if st.session_state.scene_summary is not None:
            scene_summary_box.json(st.session_state.scene_summary)
