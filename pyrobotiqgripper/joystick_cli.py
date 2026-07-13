"""CLI entry point for driving a Robotiq gripper with a joystick.

This module is intended to be installed via PyPI and exposed as a console
script via `pyproject.toml` (e.g. `pyrobotiqgripper-joystick`).

It is intentionally minimal and keeps dependencies (pygame) only when run as
an executable script / console command.
"""

from __future__ import annotations

import argparse
import logging
from typing import List, Optional

import pygame

from . import RobotiqGripper
from .constants import AUTO_DETECTION, GRIPPER_MODE_RTU, GRIPPER_MODE_RTU_VIA_TCP
from .mouse_joystick import *


def main(argv: Optional[List[str]] = None) -> int:
    """Entry point for the joystick CLI."""
    parser = argparse.ArgumentParser(
        prog="pyrobotiqgripper-joystick",
        description="Control a Robotiq gripper using a joystick (pygame) or mouse.",
    )

    common_group = parser.add_argument_group("Common options")
    common_group.add_argument(
        "--joystick-id",
        type=int,
        default=0,
        help="Joystick ID to use (default: %(default)s). -1 to control with mouse",
    )
    common_group.add_argument(
        "--connection-type",
        choices=[GRIPPER_MODE_RTU, GRIPPER_MODE_RTU_VIA_TCP],
        default=GRIPPER_MODE_RTU,
        help="Connection type for the gripper (default: %(default)s)",
    )
    common_group.add_argument(
        "--device-id",
        type=int,
        default=9,
        help="Modbus device ID for the gripper (default: %(default)s)",
    )
    common_group.add_argument(
        "--com-port",
        default=AUTO_DETECTION,
        help="COM port for RTU connection (default: %(default)s)",
    )
    common_group.add_argument(
        "--gripper-type",
        default="2F85",
        help="Type of gripper (default: %(default)s)",
    )
    common_group.add_argument(
        "--tcp-host",
        default="10.0.0.153",
        help="TCP host/IP for the gripper Modbus TCP gateway (default: %(default)s)",
    )
    common_group.add_argument(
        "--tcp-port",
        type=int,
        default=54321,
        help="TCP port for the gripper Modbus TCP gateway (default: %(default)s)",
    )
    common_group.add_argument(
        "--motion-axis",
        type=int,
        default=0,
        help="Joystick axis index to read for the position/speed control input. "
             "Defaults to 3 for a joystick, or %d (AXIS_X) for mouse control." % AXIS_X,
    )
    common_group.add_argument(
        "--force-axis",
        type=int,
        default=1,
        help="Joystick axis index to read for the force control input. "
             "Defaults to 4 for a joystick, or %d (AXIS_Y) for mouse control." % AXIS_Y,
    )
    common_group.add_argument(
        "--deadzone",
        type=float,
        default=0.10,
        help="Mouse control only: dead zone as a fraction (0-1) of each screen "
             "dimension, centered on the middle of the screen (default: %(default)s)",
    )
    common_group.add_argument(
        "--control-method",
        choices=["position", "speed"],
        default="position",
        help="Control method to use: 'position' to control the gripper in" \
        "position or 'speed' to control the gripper from its speed (default: %(default)s)",
    )
    common_group.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug logging.",
    )
    common_group.add_argument(
        "--verbose",
        type=int,
        default=0,
        help="Verbose level for gripper control calls (default: %(default)s). "
             "1 prints executed commands (real-time move) or force/speed/position "
             "after each call (push-pull).",
    )

    position_group = parser.add_argument_group("Position control options")
    position_group.add_argument(
        "--min-speed-pos-delta",
        type=int,
        default=5,
        help="Minimum speed position delta for real-time move (default: %(default)s)",
    )
    position_group.add_argument(
        "--max-speed-pos-delta",
        type=int,
        default=100,
        help="Maximum speed position delta for real-time move (default: %(default)s)",
    )
    position_group.add_argument(
        "--continuous-grip",
        action="store_true",
        default=True,
        help="Enable continuous grip for real-time move (default: %(default)s)",
    )
    position_group.add_argument(
        "--no-continuous-grip",
        action="store_false",
        dest="continuous_grip",
        help="Disable continuous grip for real-time move",
    )
    position_group.add_argument(
        "--auto-lock",
        action="store_true",
        default=True,
        help="Enable auto lock for real-time move (default: %(default)s)",
    )
    position_group.add_argument(
        "--no-auto-lock",
        action="store_false",
        dest="auto_lock",
        help="Disable auto lock for real-time move",
    )
    position_group.add_argument(
        "--minimal-motion",
        type=int,
        default=2,
        help="Minimal motion for real-time move (default: %(default)s)",
    )
    position_group.add_argument(
        "--detection-duration",
        type=float,
        default=0.5,
        help="Duration used to detect if an object is inside the gripper.",
    )

    push_pull_group = parser.add_argument_group("Push-pull control options")
    push_pull_group.add_argument(
        "--force-nudge-threshold",
        type=int,
        default=20,
        help="Minimum increase in requested force (0-255) needed to retry closing/opening "
             "while an object is latched (default: %(default)s)",
    )
    push_pull_group.add_argument(
        "--speed-nudge-threshold",
        type=int,
        default=20,
        help="Minimum increase in requested speed (0-255) needed to retry closing/opening "
             "while an object is latched (default: %(default)s)",
    )

    args = parser.parse_args(argv)

    if args.motion_axis is None:
        args.motion_axis = AXIS_X if args.joystick_id == -1 else 3
    if args.force_axis is None:
        args.force_axis = AXIS_Y if args.joystick_id == -1 else 4

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )

    logging.info("Initializing pygame joystick")
    pygame.init()

    js=None

    if args.joystick_id == -1:
        logging.info("Joystick ID -1 selected, using mouse position for control")
        js = MouseJoystick(deadzone=args.deadzone)
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
    gripper.open()

    if args.auto_lock :
        print("Autolock option : ",args.auto_lock)
    logging.info("Using control method: %s", args.control_method)

    try:
        while True:

            pygame.event.pump()
            motion_value = js.get_axis(args.motion_axis)

            if args.control_method == "speed":
                force_value = -js.get_axis(args.force_axis)
                gripper.realTimeSpeedMove(
                    motion_value,
                    forceSignal=force_value,
                    forceNudgeThreshold=args.force_nudge_threshold,
                    speedNudgeThreshold=args.speed_nudge_threshold,
                    verbose=args.verbose,
                )
            else:
                gripper.realTimePositionMove(
                    
                    motion_value,
                    minSpeedPosDelta=args.min_speed_pos_delta,
                    maxSpeedPosDelta=args.max_speed_pos_delta,
                    continuousGrip=args.continuous_grip,
                    autoLock=args.auto_lock,
                    minimalMotion=args.minimal_motion,
                    verbose=args.verbose,
                    objectDetectionDuration=args.detection_duration
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
