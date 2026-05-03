"""
Offline/online sanity checks for the final Carla L4 HIL bench.

The checks intentionally protect the verified base:
  - Windows virtual VCU uses CANalyst-II channel 0 only.
  - Cockpit_Control 0x116 byte packing remains locked.
  - Official GBK DBC is present and matches the expected message.
  - Dashboard, frontend, and scene requirement files are present.
"""

import argparse
import importlib.util
import json
import socket
import sys
from pathlib import Path


BASE_DIR = Path(__file__).resolve().parent
CONFIG_PATH = BASE_DIR / "bench_config.json"


def load_config():
    with CONFIG_PATH.open("r", encoding="utf-8") as file:
        return json.load(file)


def ok(name, detail=""):
    print(f"[OK] {name}{': ' + detail if detail else ''}")


def fail(name, detail):
    print(f"[FAIL] {name}: {detail}")
    return False


def read_text(path, encodings=("utf-8", "gbk")):
    data = path.read_bytes()
    last_error = None
    for encoding in encodings:
        try:
            return data.decode(encoding)
        except UnicodeDecodeError as exc:
            last_error = exc
    raise last_error


def check_file_exists(path, name):
    if path.exists():
        ok(name, str(path.relative_to(BASE_DIR)))
        return True
    return fail(name, f"missing {path}")


def check_config(config):
    passed = True
    expected_counts = {
        "vehicle_models": 10,
        "traffic_standards": 10,
        "barriers": 16,
        "covers": 4,
        "manholes": 1,
        "normal_vehicles": 15,
        "emergency_vehicles": 1,
        "walkers": 5,
        "bicycles": 2,
        "animals": 1,
    }
    actual_counts = config.get("scene_requirements", {})
    for key, expected in expected_counts.items():
        if actual_counts.get(key) != expected:
            passed = fail("scene requirement", f"{key} expected {expected}, got {actual_counts.get(key)}")
        else:
            ok("scene requirement", f"{key}={expected}")

    can_cfg = config.get("can", {})
    if can_cfg.get("channel") != 0:
        passed = fail("CAN channel lock", f"expected channel 0, got {can_cfg.get('channel')}")
    else:
        ok("CAN channel lock", "channel 0 only")

    if str(can_cfg.get("message_id", "")).lower() != "0x116":
        passed = fail("CAN message id", f"expected 0x116, got {can_cfg.get('message_id')}")
    else:
        ok("CAN message id", "0x116")
    return passed


def check_dbc(config):
    dbc_path = BASE_DIR / config["can"]["dbc_path"]
    if not check_file_exists(dbc_path, "official DBC"):
        return False
    text = read_text(dbc_path, encodings=("gbk", "utf-8"))
    required = [
        "BO_ 278 Cockpit_Control: 8 VCU",
        "SG_ Cockpit_ACC",
        "SG_ Cockpit_Beak",
        "SG_ Cockpit_Gear",
        "SG_ Cockpit_EPS_Angle",
        "SG_ Cockpit_Key_XbW",
        "SG_ Cockpit_LED_Ready",
    ]
    passed = True
    for token in required:
        if token not in text:
            passed = fail("DBC token", token)
        else:
            ok("DBC token", token)
    return passed


def load_virtual_vcu_module():
    spec = importlib.util.spec_from_file_location("virtual_vcu", BASE_DIR / "virtual_vcu.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def check_virtual_vcu():
    passed = True
    source = (BASE_DIR / "virtual_vcu.py").read_text(encoding="utf-8")
    forbidden = ["channel=1", '"channel": 1', "'channel': 1", "CANALYST_CHANNEL_1"]
    for token in forbidden:
        if token in source:
            passed = fail("virtual_vcu channel red line", f"found {token}")

    vcu = load_virtual_vcu_module()
    vectors = [
        (0.0, [0x01, 0x1E, 0x00, 0x00, 0x00, 0x02, 0x01, 0x00]),
        (1.0, [0x01, 0x1E, 0x00, 0x01, 0xF4, 0x02, 0x01, 0x00]),
        (-1.0, [0x01, 0x1E, 0x00, 0xFE, 0x0C, 0x02, 0x01, 0x00]),
    ]
    for steer, expected in vectors:
        frame = vcu.ControlFrame(steer_norm=steer, throttle_pct=30, brake_pct=0)
        data = list(vcu.build_cockpit_control_bytes(frame))
        if data != expected:
            passed = fail("0x116 byte pack", f"steer={steer} got={data} expected={expected}")
        else:
            ok("0x116 byte pack", f"steer={steer}")
    if passed:
        ok("virtual_vcu channel red line", "no channel-1 open path detected")
    return passed


def check_python_sources():
    required_files = [
        "main_gui_new.py",
        "main_server.py",
        "virtual_vcu.py",
        "frontend/index.html",
        "frontend/styles.css",
        "frontend/app.js",
        "requirements_dashboard.txt",
        "README_formal_hil_bench.md",
        "README_android_dashboard.md",
    ]
    return all(check_file_exists(BASE_DIR / item, "required file") for item in required_files)


def udp_probe(host, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.5)
    try:
        sock.sendto(b'{"bench_check": true}', (host, port))
        ok("UDP probe", f"sent to {host}:{port}")
        return True
    except OSError as exc:
        return fail("UDP probe", f"{host}:{port} {exc}")
    finally:
        sock.close()


def tcp_probe(host, port, label):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(1.0)
    try:
        sock.connect((host, port))
        ok(label, f"{host}:{port}")
        return True
    except OSError as exc:
        return fail(label, f"{host}:{port} {exc}")
    finally:
        sock.close()


def main(argv):
    parser = argparse.ArgumentParser(description="Check final Carla L4 HIL bench files and protocol locks.")
    parser.add_argument("--online", action="store_true", help="Also probe configured UDP/TCP ports.")
    args = parser.parse_args(argv)

    config = load_config()
    checks = [
        check_file_exists(CONFIG_PATH, "bench config"),
        check_config(config),
        check_dbc(config),
        check_virtual_vcu(),
        check_python_sources(),
    ]

    if args.online:
        ubuntu_host = config["bench"]["ubuntu_carla_ip"]
        ports = config["ports"]
        checks.append(udp_probe(ubuntu_host, ports["carla_control_udp"]))
        checks.append(tcp_probe(ubuntu_host, ports["dashboard_http"], "dashboard HTTP"))
        checks.append(tcp_probe(ubuntu_host, ports["dashboard_ws"], "dashboard WebSocket"))

    if all(checks):
        print("\nBENCH CHECK PASSED")
        return 0
    print("\nBENCH CHECK FAILED")
    return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
