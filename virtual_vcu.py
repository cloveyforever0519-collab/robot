"""
Virtual VCU for the final Carla L4 HIL bench.

This script is intended to run on the Windows bench host:
  - Host IP: 10.32.127.110
  - Python: 32-bit Anaconda environment can32
  - CAN hardware: CANalyst-II with WinUSB/libusb takeover

Hard rules preserved here:
  - Open only one CANalyst-II channel: channel 0.
  - Never attempt channel 1 or dual-channel communication.
  - Blind-send CAN 0x116 and mirror the same control over UDP.
  - Cockpit_Control Byte 0 is locked to D gear.
  - Cockpit_Key_XbW and Ready bits are locked on.
  - Cockpit_EPS_Angle uses explicit Motorola/big-endian signed 16-bit bytes.
"""

import argparse
import json
import math
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path


BASE_DIR = Path(__file__).resolve().parent


def load_bench_config():
    config_path = BASE_DIR / "bench_config.json"
    if not config_path.exists():
        return {}
    with config_path.open("r", encoding="utf-8") as file:
        return json.load(file)


BENCH_CONFIG = load_bench_config()

DEFAULT_CARLA_HOST = BENCH_CONFIG.get("bench", {}).get("ubuntu_carla_ip", "10.32.127.216")
DEFAULT_CARLA_PORT = int(BENCH_CONFIG.get("ports", {}).get("carla_control_udp", 5001))
DEFAULT_CAN_BITRATE = int(BENCH_CONFIG.get("can", {}).get("bitrate", 500000))
DEFAULT_SEND_HZ = float(BENCH_CONFIG.get("can", {}).get("send_hz", 20.0))

CAN_ID_COCKPIT_CONTROL = 0x116
CANALYST_CHANNEL_0 = 0

GEAR_D = 0x01
XBW_ENABLE_MASK = 0x02
READY_ON = 0x01


@dataclass
class ControlFrame:
    steer_norm: float
    throttle_pct: int
    brake_pct: int
    reverse: bool = False
    hand_brake: bool = False


def clamp(value, low, high):
    return max(low, min(high, value))


def build_cockpit_control_bytes(control: ControlFrame) -> bytes:
    """
    Build Cockpit_Control (0x116) according to the locked bench protocol.

    Byte 0: Cockpit_Gear, locked to D = 0x01.
    Byte 1: Cockpit_ACC, 0~100.
    Byte 2: Cockpit_Beak, 0~100.
    Byte 3/4: Cockpit_EPS_Angle, signed 16-bit Motorola/big-endian.
    Byte 5: Cockpit_Key_XbW bit mask, locked to 0x02.
    Byte 6: Cockpit_LED_Ready, locked to 0x01.
    Byte 7: reserved.
    """
    steer_norm = clamp(float(control.steer_norm), -1.0, 1.0)
    throttle_pct = int(clamp(int(control.throttle_pct), 0, 100))
    brake_pct = int(clamp(int(control.brake_pct), 0, 100))

    physical_angle = int(steer_norm * 500)
    angle_hex = physical_angle & 0xFFFF
    byte3 = (angle_hex >> 8) & 0xFF
    byte4 = angle_hex & 0xFF

    return bytes([
        GEAR_D,
        throttle_pct & 0xFF,
        brake_pct & 0xFF,
        byte3,
        byte4,
        XBW_ENABLE_MASK,
        READY_ON,
        0x00,
    ])


def build_udp_payload(control: ControlFrame) -> dict:
    return {
        "steer": clamp(float(control.steer_norm), -1.0, 1.0),
        "throttle": clamp(float(control.throttle_pct) / 100.0, 0.0, 1.0),
        "brake": clamp(float(control.brake_pct) / 100.0, 0.0, 1.0),
        "reverse": bool(control.reverse),
        "hand_brake": bool(control.hand_brake),
    }


def micro_sine_control(start_time: float) -> ControlFrame:
    t = time.time() - start_time
    steer_norm = 0.05 * math.sin(t * 1.0)
    throttle_pct = int(30 + 3 * math.cos(t * 0.5))
    return ControlFrame(
        steer_norm=steer_norm,
        throttle_pct=throttle_pct,
        brake_pct=0,
        reverse=False,
        hand_brake=False,
    )


def open_canalyst_channel_0(bitrate: int):
    """
    Open CANalyst-II channel 0 only.

    The final bench uses a 32-bit Anaconda environment and WinUSB/libusb.
    Different python-can canalystii builds expose slightly different keyword
    names, so we try channel-0-only variants. No code path opens channel 1.
    """
    import can

    attempts = [
        {"interface": "canalystii", "channel": CANALYST_CHANNEL_0, "bitrate": bitrate},
        {"bustype": "canalystii", "channel": CANALYST_CHANNEL_0, "bitrate": bitrate},
        {"interface": "canalystii", "channel": 0, "bitrate": bitrate},
    ]

    last_error = None
    for kwargs in attempts:
        try:
            return can.interface.Bus(**kwargs)
        except Exception as exc:
            last_error = exc

    raise RuntimeError(f"CANalyst-II channel 0 open failed: {last_error}")


def send_can_frame(bus, data: bytes):
    import can

    msg = can.Message(
        arbitration_id=CAN_ID_COCKPIT_CONTROL,
        data=data,
        is_extended_id=False,
    )
    bus.send(msg)


def run(args):
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    carla_addr = (args.host, args.port)

    bus = None
    if not args.udp_only:
        try:
            bus = open_canalyst_channel_0(args.bitrate)
            print("CANalyst-II channel 0 opened. Channel 1 is intentionally untouched.")
        except Exception as exc:
            print(f"CAN unavailable, continuing UDP-only: {exc}")
            if args.require_can:
                raise

    interval = 1.0 / args.hz
    start_time = time.time()
    seq = 0

    print("=" * 78)
    print("Virtual VCU is running")
    print(f"UDP target: {args.host}:{args.port}")
    print(f"CAN ID: 0x{CAN_ID_COCKPIT_CONTROL:03X}, channel: 0 only, bitrate: {args.bitrate}")
    print("Protocol lock: Byte0=D, Byte5=0x02, Byte6=0x01, EPS big-endian")
    print("=" * 78)

    try:
        while True:
            control = micro_sine_control(start_time)
            can_data = build_cockpit_control_bytes(control)
            udp_payload = build_udp_payload(control)

            udp_sock.sendto(json.dumps(udp_payload).encode("utf-8"), carla_addr)

            if bus is not None:
                try:
                    send_can_frame(bus, can_data)
                except Exception as exc:
                    print(f"\nCAN send failed: {exc}")
                    if args.require_can:
                        raise

            if seq % max(1, int(args.hz)) == 0:
                hex_bytes = " ".join(f"{b:02X}" for b in can_data)
                print(
                    f"steer={udp_payload['steer']:+.4f} "
                    f"throttle={udp_payload['throttle']:.2f} "
                    f"brake={udp_payload['brake']:.2f} "
                    f"CAN[0x116]={hex_bytes}",
                    end="\r",
                )

            seq += 1
            time.sleep(interval)

    except KeyboardInterrupt:
        print("\nVirtual VCU stopped by user.")
    finally:
        udp_sock.close()
        if bus is not None:
            try:
                bus.shutdown()
            except Exception:
                pass


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Final bench Virtual VCU for Carla HIL.")
    parser.add_argument("--host", default=DEFAULT_CARLA_HOST, help="Ubuntu Carla host IP.")
    parser.add_argument("--port", type=int, default=DEFAULT_CARLA_PORT, help="Ubuntu Carla UDP control port.")
    parser.add_argument("--hz", type=float, default=DEFAULT_SEND_HZ, help="Send rate in Hz.")
    parser.add_argument("--bitrate", type=int, default=DEFAULT_CAN_BITRATE, help="CAN bitrate.")
    parser.add_argument("--udp-only", action="store_true", help="Skip CANalyst-II and send UDP only.")
    parser.add_argument("--require-can", action="store_true", help="Exit if CANalyst-II channel 0 cannot open.")
    return parser.parse_args(argv)


if __name__ == "__main__":
    run(parse_args(sys.argv[1:]))
