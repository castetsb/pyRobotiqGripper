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
from .constants import *
from .mouse_joystick import *

from .bipper import Bipper


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
        "--control-axis",
        type=int,
        default=0,
        help="Joystick axis index to read for the position/speed control input. "
             "Defaults to 3 for a joystick, or %d (AXIS_X) for mouse control." % AXIS_X,
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
        "--bipper",
        action="store_true",
        help="Enable a bipper that provide audio force feedback.",
    )
    common_group.add_argument(
        "--grip-speed",
        type=int,
        default=None,
        help="Speed parameter use to secure a grip when an object is detected."\
        "If used, grip_force and grip_speed have to be both set.",
    )
    common_group.add_argument(
        "--grip-force",
        type=int,
        default=None,
        help="Force parameter use to secure a grip when an object is detected."\
        "If used, grip_force and grip_speed have to be both set.",
    )

    common_group.add_argument(
        "--verbose",
        type=int,
        default=0,
        help="Verbose level for gripper control calls (default: %(default)s). "
             "1 prints executed commands (real-time move) or force/speed/position "
             "after each call (push-pull).",
    )
    common_group.add_argument(
        "--controlBuffer",
        type=float,
        default=0.05,
        help="Portion of control signal use as deadzone. The deadzone is"\
        "at the bottom and at the top of the control range for the position"\
        "method and at the center of the control range for the speed method."\
        " (default: %(default)s). ",
    )

    position_group = parser.add_argument_group("Position control options")

    position_group.add_argument(
        "--speedLowerControlThreshold",
        type=int,
        default=10,
        help="In position control method the speed is function of the "\
        "difference between current position and target position. "\
        "The function is ramp that start at speedLowerControlThreshold and "\
        "finish at speedUpperControlThreshold"
    )
    position_group.add_argument(
        "--speedUpperControlThreshold",
        type=int,
        default=30,
        help="In position control method the speed is function of the "\
        "difference between current position and target position. "\
        "The function is ramp that start at speedLowerControlThreshold and "\
        "finish at speedUpperControlThreshold"
    )

    common_group.add_argument(
        "--visual-tool",
        action="store_true",
        help="Open a live gripper state visualization window (requires the "
             "optional PyQt5 and PyQtChart packages). Displayed signals are "
             "selectable from the window itself; the timeline window is "
             "fixed at 5 seconds.",
    )

    args = parser.parse_args(argv)

    if args.control_axis is None:
        args.control_axis = AXIS_X if args.joystick_id == -1 else 3

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )

    logging.info("Initializing pygame joystick")
    pygame.init()

    js=None

    if args.joystick_id == -1:
        logging.info("Joystick ID -1 selected, using mouse position for control")
        js = MouseJoystick(deadzone=0)
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

    logging.info("Using control method: %s", args.control_method)

    if args.bipper :
        bipper = Bipper()
        bipper.min_beep_rate = 1
        bipper.max_beep_rate = 12
        bipper.start()
        bipper.volume = 0

    visualizer = None
    if args.visual_tool:
        try:
            from .visualizer import GripperVisualizer
        except ImportError as exc:
            logging.error("Cannot start the visualization tool: %s", exc)
            return 1
        visualizer = GripperVisualizer(gripper)
        visualizer.start()

    try:
        while True:

            pygame.event.pump()
            control_value = js.get_axis(args.control_axis)

            if (args.control_method == "position") and (args.joystick_id == -1):
                control_value = (control_value + 1)/2

            if args.control_method == "speed":
                gripper.realTimeSpeedMove(controlSignal=control_value,
                                          controlBuffer=args.controlBuffer,
                                          gripSpeed=args.grip_speed,
                                          gripForce=args.grip_force,
                                          verbose=args.verbose)
                if args.bipper:
                    if gripper.realTimeSpeedMove_Mode() in [REALTIME_SPEED_MOVE_MODE_OBJECT_DETECTED,REALTIME_SPEED_MOVE_MODE_FORCE_DEACTIVATED]:
                        bipper.input_signal = gripper.force()/255
                        bipper.audio_pitch = 400
                        bipper.volume = 0.3
                    elif gripper.realTimeSpeedMove_Mode() in [REALTIME_SPEED_MOVE_MODE_FORCE_ACTIVATED]:
                        bipper.input_signal = gripper.force()/255
                        bipper.audio_pitch = 900
                        bipper.volume = 0.3
                    else:
                        bipper.volume = 0
            else:
                gripper.realTimePositionMove(controlSignal=control_value,
                                             controlBuffer=args.controlBuffer,
                                             speedLowerControlThreshold=args.speedLowerControlThreshold,
                                             speedUpperControlThreshold=args.speedUpperControlThreshold,
                                             gripSpeed=args.grip_speed,
                                             gripForce=args.grip_force,
                                             verbose=args.verbose)
                
                if args.bipper:
                    if gripper.realTimePositionMove_Mode() in [REALTIME_POSITION_MOVE_MODE_OBJECT_DETECTED_CLOSING,
                                                              REALTIME_POSITION_MOVE_MODE_OBJECT_DETECTED_OPENING,
                                                              REALTIME_POSITION_MOVE_MODE_FORCE_DEACTIVATED_CLOSING,
                                                              REALTIME_POSITION_MOVE_MODE_FORCE_DEACTIVATED_OPENING]:
                        bipper.input_signal = gripper.force()/255
                        bipper.audio_pitch = 400
                        bipper.volume = 0.3
                    elif gripper.realTimePositionMove_Mode() in [REALTIME_POSITION_MOVE_MODE_FORCE_ACTIVATED_CLOSING,
                                                                REALTIME_POSITION_MOVE_MODE_FORCE_ACTIVATED_OPENING]:
                        bipper.input_signal = gripper.force()/255
                        bipper.audio_pitch = 900
                        bipper.volume = 0.3
                    else:
                        bipper.volume = 0
                
    except KeyboardInterrupt:
        logging.info("Stopping joystick control")
    finally:
        if visualizer is not None:
            visualizer.stop()
        try:
            gripper.disconnect()
        except Exception:
            # Some transports may not support explicit disconnect.
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
