"""CLI entry point for driving a Robotiq gripper with a joystick.

This module is intended to be installed via PyPI and exposed as a console
script via `pyproject.toml` (e.g. `pyrobotiqgripper-joystick`).

It is intentionally minimal and keeps dependencies (pygame) only when run as
an executable script / console command.
"""

from __future__ import annotations

import argparse
import logging
import sys
from typing import List, Optional

import pygame
from pynput import mouse

from . import RobotiqGripper
from .constants import AUTO_DETECTION, GRIPPER_MODE_RTU, GRIPPER_MODE_RTU_VIA_TCP

class MouseJoystick:
    def __init__(self):
        self.x = 0

        # Start listener in background
        self.listener = mouse.Listener(on_move=self._on_move)
        self.listener.start()

    def _on_move(self, x, y):
        self.x = x  # store latest mouse X

    def get_axis(self, axis=None):
        # Lazy import to avoid dependency if unused
        import pyautogui

        screen_width, _ = pyautogui.size()

        # Normalize to [-1, 1]
        value=(self.x / screen_width) * 2 - 1
        value = max(-1, min(1, value))  # clamp to [-1, 1]

        return value

def map_0_255(x: float) -> int:

    """Map a joystick axis value from [-1, 1] to [0, 255]."""
    return int((x + 1) * 255 / 2)


def main(argv: Optional[List[str]] = None) -> int:
    """Entry point for the joystick CLI."""
    parser = argparse.ArgumentParser(
        prog="pyrobotiqgripper-joystick",
        description="Control a Robotiq gripper using a joystick (pygame).",
    )

    parser.add_argument(
        "--joystick-id",
        type=int,
        default=0,
        help="Joystick ID to use (default: %(default)s). -1 to control with mouse",
    )
    parser.add_argument(
        "--connection-type",
        choices=[GRIPPER_MODE_RTU, GRIPPER_MODE_RTU_VIA_TCP],
        default=GRIPPER_MODE_RTU,
        help="Connection type for the gripper (default: %(default)s)",
    )
    parser.add_argument(
        "--device-id",
        type=int,
        default=9,
        help="Modbus device ID for the gripper (default: %(default)s)",
    )
    parser.add_argument(
        "--com-port",
        default=AUTO_DETECTION,
        help="COM port for RTU connection (default: %(default)s)",
    )
    parser.add_argument(
        "--gripper-type",
        default="2F85",
        help="Type of gripper (default: %(default)s)",
    )
    parser.add_argument(
        "--tcp-host",
        default="10.0.0.153",
        help="TCP host/IP for the gripper Modbus TCP gateway (default: %(default)s)",
    )
    parser.add_argument(
        "--tcp-port",
        type=int,
        default=54321,
        help="TCP port for the gripper Modbus TCP gateway (default: %(default)s)",
    )
    parser.add_argument(
        "--axis",
        type=int,
        default=3,
        help="Joystick axis index to read for position (default: %(default)s)",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug logging.",
    )
    parser.add_argument(
        "--min-speed-pos-delta",
        type=int,
        default=5,
        help="Minimum speed position delta for real-time move (default: %(default)s)",
    )
    parser.add_argument(
        "--max-speed-pos-delta",
        type=int,
        default=100,
        help="Maximum speed position delta for real-time move (default: %(default)s)",
    )
    parser.add_argument(
        "--continuous-grip",
        action="store_true",
        default=True,
        help="Enable continuous grip for real-time move (default: %(default)s)",
    )
    parser.add_argument(
        "--no-continuous-grip",
        action="store_false",
        dest="continuous_grip",
        help="Disable continuous grip for real-time move",
    )
    parser.add_argument(
        "--auto-lock",
        action="store_true",
        default=True,
        help="Enable auto lock for real-time move (default: %(default)s)",
    )
    parser.add_argument(
        "--no-auto-lock",
        action="store_false",
        dest="auto_lock",
        help="Disable auto lock for real-time move",
    )
    parser.add_argument(
        "--minimal-motion",
        type=int,
        default=2,
        help="Minimal motion for real-time move (default: %(default)s)",
    )
    parser.add_argument(
        "--verbose",
        type=int,
        default=0,
        help="Verbose level for real-time move (default: %(default)s). 1 prints executed commands, 2 prints all commands.",
    )

    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )

    logging.info("Initializing pygame joystick")
    pygame.init()

    js=None

    if args.joystick_id == -1:
        logging.info("Joystick ID -1 selected, using mouse position for control")
        js = MouseJoystick()
    else:

        pygame.joystick.init()

        if pygame.joystick.get_count() <= args.joystick_id:
            logging.error("Joystick ID %d not found. Available joysticks: %d", args.joystick_id, pygame.joystick.get_count())
            return 1

        js = pygame.joystick.Joystick(args.joystick_id)
        js.init()

    logging.info(
        "Connecting to Robotiq gripper at %s:%s (type: %s, device_id: %s)",
        args.tcp_host,
        args.tcp_port,
        args.connection_type,
        args.device_id,
    )

    gripper = RobotiqGripper(
        connection_type=args.connection_type,
        tcp_host=args.tcp_host,
        tcp_port=args.tcp_port,
        com_port=args.com_port,
        device_id=args.device_id,
        gripper_type=args.gripper_type,
        debug=args.debug
    )

    gripper.connect()
    gripper.activate()
    gripper.start()
    gripper.calibrate_speed()
    print("Autolock option : ",args.auto_lock)

    try:
        while True:

            pygame.event.pump()
            joy_value = js.get_axis(args.axis)
            pos = map_0_255(joy_value)
            gripper.realTimeMove(
                pos,
                minSpeedPosDelta=args.min_speed_pos_delta,
                maxSpeedPosDelta=args.max_speed_pos_delta,
                continuousGrip=args.continuous_grip,
                autoLock=args.auto_lock,
                minimalMotion=args.minimal_motion,
                verbose=args.verbose,
            )
    except KeyboardInterrupt:
        logging.info("Stopping joystick control")
    finally:
        try:
            gripper.disconnect()
        except Exception:
            # Some transports may not support explicit disconnect.
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
