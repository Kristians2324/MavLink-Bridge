"""Two-way MAVLink <-> Godot bridge.

This bridge:
  1. Listens for MAVLink telemetry from ArduPilot on udpin:127.0.0.1:14550
  2. Forwards selected telemetry packets to Godot over UDP on 127.0.0.1:14551
  3. Listens for JSON control inputs from Godot on a non-blocking UDP socket
  4. Converts Godot axes (Throttle, Yaw, Pitch, Roll) from -1.0..1.0 to PWM
     values and sends RC channel overrides back to the vehicle

Usage:
  python Bridge.py --mavlink-port 14550 --godot-host 127.0.0.1 --godot-port 14551
  python Bridge.py --control-port 14552

Install dependency:
  pip install pymavlink
"""

from __future__ import annotations

import argparse
import json
import socket
import time
from typing import Any, Dict, Optional

from pymavlink import mavutil


DEFAULT_MAVLINK_PORT = 14550
DEFAULT_GODOT_PORT = 14551
DEFAULT_CONTROL_PORT = 14552


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Two-way MAVLink bridge for Godot")
    parser.add_argument("--mavlink-port", type=int, default=DEFAULT_MAVLINK_PORT, help="UDP port to listen for MAVLink")
    parser.add_argument("--godot-host", default="127.0.0.1", help="Godot UDP host")
    parser.add_argument("--godot-port", type=int, default=DEFAULT_GODOT_PORT, help="Godot UDP port for telemetry")
    parser.add_argument(
        "--control-port",
        type=int,
        default=DEFAULT_CONTROL_PORT,
        help="UDP port to listen for Godot control JSON (use a separate port from telemetry)",
    )
    return parser.parse_args()


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def axis_to_pwm(value: float) -> int:
    """Map normalized axis input (-1.0..1.0) to PWM (1000..2000)."""
    value = clamp(float(value), -1.0, 1.0)
    pwm = 1500 + (value * 500)
    return int(round(clamp(pwm, 1000, 2000)))


def parse_control_packet(data: bytes) -> Optional[Dict[str, Any]]:
    try:
        decoded = data.decode("utf-8").strip()
        if not decoded:
            return None
        payload = json.loads(decoded)
        if not isinstance(payload, dict):
            return None
        return payload
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None


def setup_vehicle(mav: mavutil.mavfile) -> None:
    """Perform the basic arming / mode setup after heartbeat."""
    # Try to set a sensible guided/manual-capable mode before arming.
    # If the vehicle rejects a mode, we continue and still allow overrides.
    try:
        mav.set_mode_apm("GUIDED")
    except Exception:
        try:
            mav.set_mode_apm("STABILIZE")
        except Exception:
            pass

    try:
        mav.arducopter_arm()
        mav.motors_armed_wait(timeout=10)
    except Exception as exc:
        print(f"Warning: arming sequence failed or timed out: {exc}")


def send_rc_override(mav: mavutil.mavfile, channels: Dict[int, int]) -> None:
    chan1 = channels.get(1, 65535)
    chan2 = channels.get(2, 65535)
    chan3 = channels.get(3, 65535)
    chan4 = channels.get(4, 65535)
    chan5 = channels.get(5, 65535)
    chan6 = channels.get(6, 65535)
    chan7 = channels.get(7, 65535)
    chan8 = channels.get(8, 65535)
    mav.mav.rc_channels_override_send(
        mav.target_system,
        mav.target_component,
        chan1,
        chan2,
        chan3,
        chan4,
        chan5,
        chan6,
        chan7,
        chan8,
    )


def main() -> None:
    args = parse_args()

    telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    godot_addr = (args.godot_host, args.godot_port)

    control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    control_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    control_socket.bind(("127.0.0.1", args.control_port))
    control_socket.setblocking(False)

    print(f"Waiting for MAVLink on udpin:127.0.0.1:{args.mavlink_port} ...")
    mav = mavutil.mavlink_connection(f"udpin:127.0.0.1:{args.mavlink_port}")

    mav.wait_heartbeat()
    print(
        f"Connected to system {mav.target_system}, component {mav.target_component}. "
        f"Forwarding telemetry to {args.godot_host}:{args.godot_port} and controls from 127.0.0.1:{args.control_port}"
    )

    setup_vehicle(mav)

    last_status = 0.0
    last_override = 0.0
    current_channels: Dict[int, int] = {1: 1500, 2: 1500, 3: 1000, 4: 1500}

    while True:
        # Drain any inbound Godot control packets without blocking telemetry.
        while True:
            try:
                data, _addr = control_socket.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError as exc:
                print(f"Control socket error: {exc}")
                break

            print(f"Raw control packet from Godot: {data!r}")

            control = parse_control_packet(data)
            if not control:
                continue

            throttle = control.get("Throttle", 0.0)
            yaw = control.get("Yaw", 0.0)
            pitch = control.get("Pitch", 0.0)
            roll = control.get("Roll", 0.0)

            current_channels[3] = axis_to_pwm(throttle)
            current_channels[4] = axis_to_pwm(yaw)
            current_channels[2] = axis_to_pwm(pitch)
            current_channels[1] = axis_to_pwm(roll)

            send_rc_override(mav, current_channels)
            last_override = time.time()

        msg = mav.recv_match(blocking=False)
        if msg is None:
            if time.time() - last_override > 0.25:
                # Keep overrides alive so the sim continues honoring the latest controls.
                send_rc_override(mav, current_channels)
                last_override = time.time()
            time.sleep(0.005)
            continue

        msg_type = msg.get_type()
        payload: Dict[str, Any] = {"type": msg_type, "timestamp": time.time()}

        if msg_type == "HEARTBEAT":
            payload.update(
                {
                    "base_mode": int(msg.base_mode),
                    "system_status": int(msg.system_status),
                }
            )
        elif msg_type == "ATTITUDE":
            payload.update(
                {
                    "roll": float(msg.roll),
                    "pitch": float(msg.pitch),
                    "yaw": float(msg.yaw),
                }
            )
        elif msg_type == "GLOBAL_POSITION_INT":
            payload.update(
                {
                    "lat": int(msg.lat) / 1e7,
                    "lon": int(msg.lon) / 1e7,
                    "alt": int(msg.alt) / 1000.0,
                    "relative_alt": int(msg.relative_alt) / 1000.0,
                    "vx": int(msg.vx) / 100.0,
                    "vy": int(msg.vy) / 100.0,
                    "vz": int(msg.vz) / 100.0,
                }
            )
        else:
            continue

        telemetry_socket.sendto(json.dumps(payload).encode("utf-8"), godot_addr)

        if time.time() - last_status > 2.0:
            print(f"Forwarded {msg_type}: {payload}")
            last_status = time.time()


if __name__ == "__main__":
    main()