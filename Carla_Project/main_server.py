import asyncio
import json
import math
import os
import socket
import threading
import time
from copy import deepcopy
from pathlib import Path

from flask import Flask, jsonify, request, send_from_directory
import websockets


BASE_DIR = Path(__file__).resolve().parent
FRONTEND_DIR = BASE_DIR / "frontend"
ASSET_DIR = BASE_DIR / "output"


def load_bench_config():
    config_path = BASE_DIR / "bench_config.json"
    if not config_path.exists():
        return {}
    with config_path.open("r", encoding="utf-8") as file:
        return json.load(file)


BENCH_CONFIG = load_bench_config()
BENCH_PORTS = BENCH_CONFIG.get("ports", {})
BENCH_HOSTS = BENCH_CONFIG.get("bench", {})

HTTP_HOST = os.getenv("DASHBOARD_HTTP_HOST", "0.0.0.0")
HTTP_PORT = int(os.getenv("DASHBOARD_HTTP_PORT", BENCH_PORTS.get("dashboard_http", 8080)))
WS_HOST = os.getenv("DASHBOARD_WS_HOST", "0.0.0.0")
WS_PORT = int(os.getenv("DASHBOARD_WS_PORT", BENCH_PORTS.get("dashboard_ws", 8765)))

CARLA_UDP_RX_HOST = os.getenv("CARLA_DASHBOARD_RX_HOST", "0.0.0.0")
CARLA_UDP_RX_PORT = int(os.getenv("CARLA_DASHBOARD_RX_PORT", 5003))
CARLA_UDP_TX_HOST = os.getenv("CARLA_CONTROL_HOST", BENCH_HOSTS.get("ubuntu_carla_ip", "10.32.127.216"))
CARLA_UDP_TX_PORT = int(os.getenv("CARLA_CONTROL_PORT", BENCH_PORTS.get("carla_control_udp", 5001)))


BLUEPRINT_TO_FILE = {
    "vehicle.dodge.charger_2020": "sedan_dodge_charger.json",
    "vehicle.lincoln.mkz_2017": "sedan_lincoln_mkz.json",
    "vehicle.tesla.model3": "sedan_tesla_model3.json",
    "vehicle.audi.etron": "suv_audi_etron.json",
    "vehicle.jeep.wrangler_rubicon": "suv_jeep_wrangler.json",
    "vehicle.tesla.cybertruck": "suv_tesla_cyber.json",
    "vehicle.mitsubishi.fusorosa": "bus_fuso_rosa.json",
    "vehicle.mercedes.sprinter": "van_mercedes_sprinter.json",
    "vehicle.volkswagen.t2_2021": "van_volkswagen_t2.json",
    "vehicle.volkswagen.t2": "van_volkswagen_t2.json",
    "vehicle.carlamotors.carlacola": "truck_carlacola.json",
    "vehicle.carlamotors.european_hgv": "truck_european_hgv.json",
    "vehicle.carlamotors.firetruck": "truck_firetruck.json",
}

FILENAME_TO_DISPLAY_NAME = {
    "sedan_dodge_charger.json": "Dodge Charger",
    "sedan_lincoln_mkz.json": "Lincoln MKZ",
    "sedan_tesla_model3.json": "Tesla Model 3",
    "suv_audi_etron.json": "Audi e-tron",
    "suv_jeep_wrangler.json": "Jeep Wrangler",
    "suv_tesla_cyber.json": "Tesla Cybertruck",
    "bus_fuso_rosa.json": "Fuso Rosa",
    "van_mercedes_sprinter.json": "Mercedes Sprinter",
    "van_volkswagen_t2.json": "Volkswagen T2",
    "truck_carlacola.json": "Carlacola Truck",
    "truck_european_hgv.json": "European HGV",
    "truck_firetruck.json": "Firetruck",
}

WEATHER_LABELS = {
    "clear": "晴天 (Clear)",
    "cloudy": "多云 (Cloudy)",
    "rain": "小雨 (Rain)",
    "storm": "暴雨 (Storm)",
    "fog": "大雾 (Fog)",
    "night": "深夜 (Night)",
}


def deep_merge(base, updates):
    merged = deepcopy(base)
    for key, value in updates.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def clamp_float(value, minimum, maximum, default=0.0):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    return max(minimum, min(maximum, value))


def parse_bool(value):
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def safe_get(data, *keys, default=None):
    current = data
    for key in keys:
        if not isinstance(current, dict):
            return default
        current = current.get(key)
        if current is None:
            return default
    return current


def vector_norm(values):
    if not isinstance(values, (list, tuple)) or len(values) < 3:
        return 0.0
    return math.sqrt(sum(float(v or 0.0) ** 2 for v in values[:3]))


def load_vehicle_file(filename):
    path = ASSET_DIR / filename
    with path.open("r", encoding="utf-8") as file:
        return json.load(file)


def vehicle_summary(filename):
    data = load_vehicle_file(filename)
    meta = data.get("vehicle_metadata", {})
    mass = data.get("weight_and_mass_properties", {})
    aero = data.get("aerodynamic_parameters", {})
    chassis = data.get("chassis_and_mechanical_systems", {})
    return {
        "id": meta.get("blueprint_id", filename),
        "file": filename,
        "name": FILENAME_TO_DISPLAY_NAME.get(filename, meta.get("official_name", filename)),
        "officialName": meta.get("official_name", ""),
        "category": meta.get("category", ""),
        "massKg": mass.get("curb_weight_kg"),
        "gvwrKg": mass.get("gross_vehicle_weight_rating_kg"),
        "cd": aero.get("drag_coefficient_cd"),
        "steeringRatio": chassis.get("steering_system", {}).get("steering_ratio"),
    }


class SimState:
    def __init__(self):
        self.lock = threading.Lock()
        self.telemetry = {}
        self.dashboard = self._initial_dashboard()
        self.current_vehicle_config = {}
        self.selected_vehicle = "Tesla Model 3"
        self.weather = WEATHER_LABELS["clear"]
        self.traffic_state = "未加载"
        self.mode = "待机"
        self.latest_raw_ts = 0.0
        self.latest_control = {
            "steer": 0.0,
            "throttle": 0.0,
            "brake": 0.0,
            "reverse": False,
            "hand_brake": False,
        }
        self.scene_compliance = {}

    def _initial_dashboard(self):
        return {
            "status": "waiting_for_carla",
            "updatedAt": 0.0,
            "vehicle": "Tesla Model 3",
            "weather": WEATHER_LABELS["clear"],
            "traffic": "未加载",
            "speedKmh": 0.0,
            "gear": "P",
            "steer": 0.0,
            "throttle": 0.0,
            "brake": 0.0,
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "attitude": {"pitch": 0.0, "yaw": 0.0, "roll": 0.0},
            "wheelRpm": [0.0, 0.0, 0.0, 0.0],
            "wheelSteer": [0.0, 0.0],
            "wheelLoad": [0.0, 0.0, 0.0, 0.0],
            "slip": [0.0, 0.0, 0.0, 0.0],
            "radarTargets": 0,
            "collision": {"Impulse": [0, 0, 0], "Actor": "None"},
            "gnss": [0.0, 0.0, 0.0],
            "imu": {},
            "speedLimit": 0.0,
            "trafficLight": "Unknown",
            "autopilot": False,
            "raw": {},
        }

    def update_telemetry(self, raw):
        now = time.time()
        dashboard = build_dashboard_payload(raw, self)
        dashboard["updatedAt"] = now
        dashboard["status"] = "live"
        with self.lock:
            self.telemetry = raw
            self.dashboard = dashboard
            self.latest_raw_ts = now

    def update_control(self, control):
        with self.lock:
            self.latest_control = control

    def snapshot(self):
        with self.lock:
            data = deepcopy(self.dashboard)
            if self.latest_raw_ts and time.time() - self.latest_raw_ts > 2.5:
                data["status"] = "stale"
            data["control"] = deepcopy(self.latest_control)
            data["mode"] = self.mode
            return data


state = SimState()
app = Flask(__name__, static_folder=None)
udp_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def build_dashboard_payload(raw, sim_state):
    kinematics = safe_get(raw, "1_刚体运动学 (Rigid Body Kinematics)", default={})
    wheel = safe_get(raw, "2_轮端与底盘动态 (Wheel Dynamics)", default={})
    control = safe_get(raw, "3_驾驶控制反读 (Control State)", default={})
    environment = safe_get(raw, "5_环境与交通真值 (Environment Truth)", default={})
    scene_compliance = safe_get(raw, "7_场景要素验收 (Scene Compliance)", default={})

    v_xyz = kinematics.get("3_线速度矢量_XYZ_米每秒", [0.0, 0.0, 0.0])
    speed_kmh = vector_norm(v_xyz) * 3.6
    position = kinematics.get("1_全局绝对坐标_XYZ_米", [0.0, 0.0, 0.0])
    attitude = kinematics.get("2_姿态角_俯仰_偏航_滚转_度", [0.0, 0.0, 0.0])
    wheel_rpm = wheel.get("6_四轮独立转速_RPM_左前_右前_左后_右后", [0, 0, 0, 0])

    throttle = clamp_float(control.get("9_实际油门开度_0至1"), 0.0, 1.0)
    brake = clamp_float(control.get("10_实际刹车力度_0至1"), 0.0, 1.0)
    steer = clamp_float(control.get("11_方向盘转角_负1至1"), -1.0, 1.0)
    gear = control.get("12_当前机械档位", 0)
    reverse = parse_bool(control.get("14_倒车挂档状态", False))

    collision = raw.get("COLLISION_DATA") or raw.get("碰撞监控状态") or {"Impulse": [0, 0, 0], "Actor": "None"}
    radar_targets = raw.get("RADAR_TARGETS", 0)
    gnss = raw.get("GNSS_DATA", [0.0, 0.0, 0.0])
    imu = raw.get("IMU_DATA", {})

    return {
        "vehicle": sim_state.selected_vehicle,
        "weather": sim_state.weather,
        "traffic": sim_state.traffic_state,
        "speedKmh": round(speed_kmh, 1),
        "gear": "R" if reverse else ("D" if gear != 0 else "N"),
        "steer": steer,
        "throttle": throttle,
        "brake": brake,
        "position": {
            "x": round(float(position[0]), 2) if len(position) > 0 else 0.0,
            "y": round(float(position[1]), 2) if len(position) > 1 else 0.0,
            "z": round(float(position[2]), 2) if len(position) > 2 else 0.0,
        },
        "attitude": {
            "pitch": round(float(attitude[0]), 2) if len(attitude) > 0 else 0.0,
            "yaw": round(float(attitude[1]), 2) if len(attitude) > 1 else 0.0,
            "roll": round(float(attitude[2]), 2) if len(attitude) > 2 else 0.0,
        },
        "wheelRpm": [round(float(v or 0.0), 1) for v in wheel_rpm[:4]],
        "wheelSteer": wheel.get("8_前轮真实阿克曼转向角_度", [0.0, 0.0]),
        "wheelLoad": estimate_wheel_loads(speed_kmh, steer, throttle, brake),
        "slip": estimate_slip(wheel_rpm, speed_kmh),
        "radarTargets": int(radar_targets or 0),
        "collision": collision,
        "gnss": gnss,
        "imu": imu,
        "speedLimit": environment.get("19_当前路段法定限速_公里每小时", 0.0),
        "trafficLight": environment.get("20_前方红绿灯当前状态", "Unknown"),
        "sceneCompliance": scene_compliance,
        "autopilot": sim_state.mode == "智驾",
        "raw": raw,
    }


def estimate_wheel_loads(speed_kmh, steer, throttle, brake):
    base = 4600
    lon = throttle * 450 - brake * 650
    lat = steer * speed_kmh * 9
    return [
        round(max(0, base - lon + lat)),
        round(max(0, base - lon - lat)),
        round(max(0, base + lon + lat)),
        round(max(0, base + lon - lat)),
    ]


def estimate_slip(wheel_rpm, speed_kmh):
    if speed_kmh < 1.0:
        return [0.0, 0.0, 0.0, 0.0]
    speed_ms = speed_kmh / 3.6
    tire_radius = 0.334
    result = []
    for rpm in list(wheel_rpm[:4]) + [0.0] * max(0, 4 - len(wheel_rpm)):
        wheel_speed = (float(rpm or 0.0) / 60.0) * 2 * math.pi * tire_radius
        result.append(round((wheel_speed - speed_ms) / max(abs(speed_ms), 0.1), 4))
    return result[:4]


def send_carla_command(payload):
    udp_tx.sendto(json.dumps(payload, ensure_ascii=False).encode("utf-8"), (CARLA_UDP_TX_HOST, CARLA_UDP_TX_PORT))


def carla_udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((CARLA_UDP_RX_HOST, CARLA_UDP_RX_PORT))
    except OSError as exc:
        print(f"UDP {CARLA_UDP_RX_PORT} 绑定失败，大屏只能显示离线状态: {exc}")
        return

    print(f"正在监听 Carla 真值 UDP {CARLA_UDP_RX_HOST}:{CARLA_UDP_RX_PORT}")
    while True:
        try:
            data, _ = sock.recvfrom(65535)
            raw = json.loads(data.decode("utf-8"))
            state.update_telemetry(raw)
        except Exception as exc:
            print(f"解析 Carla 真值失败: {exc}")
            time.sleep(0.05)


@app.route("/")
def index():
    html = (FRONTEND_DIR / "index.html").read_text(encoding="utf-8")
    config = {
        "wsPort": WS_PORT,
        "httpPort": HTTP_PORT,
        "udpRxPort": CARLA_UDP_RX_PORT,
        "udpTxPort": CARLA_UDP_TX_PORT,
    }
    html = html.replace("__DASHBOARD_CONFIG__", json.dumps(config, ensure_ascii=False))
    return html


@app.route("/<path:path>")
def frontend_static(path):
    return send_from_directory(FRONTEND_DIR, path)


@app.route("/api/state", methods=["GET"])
def api_state():
    return jsonify(state.snapshot())


@app.route("/api/vehicles", methods=["GET"])
def api_vehicles():
    vehicles = []
    for path in sorted(ASSET_DIR.glob("*.json")):
        if path.name in FILENAME_TO_DISPLAY_NAME:
            vehicles.append(vehicle_summary(path.name))
    return jsonify({"vehicles": vehicles})


@app.route("/api/spawn", methods=["POST"])
def spawn_command():
    req = request.get_json(silent=True) or {}
    blueprint = req.get("vehicle_blueprint") or req.get("blueprint")
    filename = req.get("file") or BLUEPRINT_TO_FILE.get(blueprint)

    if filename not in FILENAME_TO_DISPLAY_NAME:
        return jsonify({"status": "error", "msg": f"未知车型配置: {blueprint or filename}"}), 404

    try:
        base_config = load_vehicle_file(filename)
    except FileNotFoundError:
        return jsonify({"status": "error", "msg": f"找不到底层配置文件: {filename}"}), 404

    params = req.get("params", {})
    merged = deep_merge(base_config, params if isinstance(params, dict) else {})
    display_name = FILENAME_TO_DISPLAY_NAME[filename]

    with state.lock:
        state.current_vehicle_config = merged
        state.selected_vehicle = display_name
        state.mode = req.get("mode", state.mode)

    return jsonify({
        "status": "success",
        "msg": "Vehicle config loaded",
        "vehicle": display_name,
        "blueprint": merged.get("vehicle_metadata", {}).get("blueprint_id"),
    })


@app.route("/api/control", methods=["POST"])
def api_control():
    req = request.get_json(silent=True) or {}
    command = {
        "steer": clamp_float(req.get("steer", 0.0), -1.0, 1.0),
        "throttle": clamp_float(req.get("throttle", 0.0), 0.0, 1.0),
        "brake": clamp_float(req.get("brake", 0.0), 0.0, 1.0),
        "reverse": parse_bool(req.get("reverse", False)),
        "hand_brake": parse_bool(req.get("hand_brake", False)),
    }
    state.update_control(command)
    send_carla_command(command)
    return jsonify({"status": "success", "command": command})


@app.route("/api/mode", methods=["POST"])
def api_mode():
    req = request.get_json(silent=True) or {}
    mode = req.get("mode", "待机")
    if mode not in {"待机", "手驾", "智驾"}:
        return jsonify({"status": "error", "msg": "mode must be 待机/手驾/智驾"}), 400
    with state.lock:
        state.mode = mode
    return jsonify({"status": "success", "mode": mode})


@app.route("/api/weather", methods=["POST"])
def api_weather():
    req = request.get_json(silent=True) or {}
    weather_key = req.get("weather", "clear")
    label = WEATHER_LABELS.get(weather_key, weather_key)
    with state.lock:
        state.weather = label
    send_carla_command({"command": "set_weather", "value": label})
    return jsonify({"status": "success", "weather": label})


@app.route("/api/traffic", methods=["POST"])
def api_traffic():
    req = request.get_json(silent=True) or {}
    enabled = parse_bool(req.get("enabled", True))
    with state.lock:
        state.traffic_state = "已加载" if enabled else "未加载"
    send_carla_command({"command": "add_scene_elements" if enabled else "clear_scene_elements", "value": enabled})
    return jsonify({"status": "success", "traffic": state.traffic_state})


async def telemetry_broadcaster(websocket):
    print("大屏 WebSocket 已连接")
    try:
        while True:
            await websocket.send(json.dumps(state.snapshot(), ensure_ascii=False))
            await asyncio.sleep(0.05)
    except websockets.exceptions.ConnectionClosed:
        print("大屏 WebSocket 已断开")


def run_flask():
    app.run(host=HTTP_HOST, port=HTTP_PORT, threaded=True)


async def run_ws():
    async with websockets.serve(telemetry_broadcaster, WS_HOST, WS_PORT, ping_interval=20, ping_timeout=20):
        await asyncio.Future()


if __name__ == "__main__":
    threading.Thread(target=carla_udp_listener, daemon=True).start()
    threading.Thread(target=run_flask, daemon=True).start()

    print("=" * 72)
    print(f"L4 安卓大屏后端已启动")
    print(f"HTTP 页面: http://<Ubuntu_IP>:{HTTP_PORT}/")
    print(f"WebSocket: ws://<Ubuntu_IP>:{WS_PORT}")
    print(f"Carla 真值输入 UDP: {CARLA_UDP_RX_PORT}")
    print(f"Carla 控制输出 UDP: {CARLA_UDP_TX_HOST}:{CARLA_UDP_TX_PORT}")
    print("=" * 72)
    asyncio.run(run_ws())
