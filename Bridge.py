"""Minimal MAVLink -> Godot bridge.

This script is intentionally small: it listens for MAVLink packets from an
ArduPilot vehicle (Copter/Plane/etc.) and forwards a compact JSON message to a
Godot client over UDP.

Default flow:
  1. MAVLink source sends to udp:0.0.0.0:14550
  2. This bridge listens on 14550
  3. The bridge forwards JSON packets to Godot on 127.0.0.1:14551

Install dependency:
  pip install pymavlink

Example run:
  python Bridge.py --mavlink-port 14550 --godot-host 127.0.0.1 --godot-port 14551
"""

from __future__ import annotations

import argparse
import json
import socket
import time

from pymavlink import mavutil


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Minimal MAVLink bridge for Godot testing")
    parser.add_argument("--mavlink-port", type=int, default=14550, help="UDP port to listen for MAVLink")
    parser.add_argument("--godot-host", default="127.0.0.1", help="Godot UDP host")
    parser.add_argument("--godot-port", type=int, default=14551, help="Godot UDP port")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    godot_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    godot_addr = (args.godot_host, args.godot_port)

    print(f"Waiting for MAVLink on udp:0.0.0.0:{args.mavlink_port} ...")
    mav = mavutil.mavlink_connection(f"udp:0.0.0.0:{args.mavlink_port}")
    mav.wait_heartbeat()
    print(
        f"Connected to system {mav.target_system}, component {mav.target_component}. "
        f"Forwarding to {args.godot_host}:{args.godot_port}"
    )

    last_status = 0.0

    while True:
        msg = mav.recv_match(blocking=False)
        if msg is None:
            time.sleep(0.01)
            continue

        msg_type = msg.get_type()
        payload = {"type": msg_type, "timestamp": time.time()}

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

        godot_socket.sendto(json.dumps(payload).encode("utf-8"), godot_addr)

        # Avoid spamming the console too much.
        if time.time() - last_status > 2.0:
            print(f"Forwarded {msg_type}: {payload}")
            last_status = time.time()


if __name__ == "__main__":
    main()
