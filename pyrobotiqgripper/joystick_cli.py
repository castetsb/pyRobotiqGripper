"""CLI entry point for driving a Robotiq gripper with a joystick.

This module is intended to be installed via PyPI and exposed as a console
script via `pyproject.toml` (e.g. `pyrobotiqgripper-joystick`).

It is intentionally minimal and keeps dependencies (pygame) only when run as
an executable script / console command.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import threading
import time
from typing import List, Optional

import pygame
from pynput import keyboard

from . import RobotiqGripper
from .constants import *
from .mouse_joystick import *

from .bipper import Bipper


def _print_shutdown_notice() -> None:
    """Print a one-off notice that shutdown may take a while.

    A single synchronous print, not a background thread: the shutdown delay
    is suspected to come from GIL contention between background threads
    (visualizer, mouse listener) during teardown, so adding yet another
    concurrently-running thread here risks making that worse rather than
    giving useful feedback (as observed: the countdown thread itself never
    got a chance to print).
    """
    print(
        "\nStopping... closing the visualization tool can take a while. "
        "Please wait.",
        flush=True,
    )


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
        choices=["position-raw","position", "speed"],
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
        default=20,
        help="In position control method the speed is function of the "\
        "difference between current position and target position. "\
        "The function is ramp that start at speedLowerControlThreshold and "\
        "finish at speedUpperControlThreshold (default: %(default)s)."
    )
    position_group.add_argument(
        "--speedUpperControlThreshold",
        type=int,
        default=100,
        help="In position control method the speed is function of the "\
        "difference between current position and target position. "\
        "The function is ramp that start at speedLowerControlThreshold and "\
        "finish at speedUpperControlThreshold (default: %(default)s)."
    )

    common_group.add_argument(
        "--visual-tool",
        action="store_true",
        help="Open a live gripper state visualization window (requires the "
             "optional PySide6 package). Displayed signals are selectable "
             "from the window itself; the timeline window is fixed at 5 "
             "seconds.",
    )
    common_group.add_argument(
        "--mouse-visual-tool",
        action="store_true",
        help="Open a live overlay bar (requires the optional PySide6 "
             "package) spanning the top of the screen, tracking the mouse's "
             "live X position with a marker. While using --control-method "
             "position, colored control zones (deadzone / force-ramp range) "
             "are also shown once an object has been detected. Only valid "
             "with --joystick-id -1.",
    )

    args = parser.parse_args(argv)

    if args.control_axis is None:
        args.control_axis = AXIS_X if args.joystick_id == -1 else 3

    if args.mouse_visual_tool and args.joystick_id != -1:
        print("--mouse-visual-tool requires --joystick-id -1", file=sys.stderr)
        return 1

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )

    logging.info("Initializing pygame joystick")
    pygame.init()

    js=None

    if args.joystick_id == -1:
        logging.info("Joystick ID -1 selected, using mouse position for control")
        mouse_output_range = "unsigned" if args.control_method in ("position", "position-raw") else "signed"
        js = MouseJoystick(deadzone=0, output_range=mouse_output_range)
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

    mouse_visualizer = None
    mouse_screen_width = None
    if args.mouse_visual_tool:
        try:
            mouse_visualizer = MouseJoystickVisualizer()
        except ImportError as exc:
            logging.error("Cannot start the mouse visualization tool: %s", exc)
            return 1
        mouse_visualizer.start()
        import pyautogui
        mouse_screen_width, _ = pyautogui.size()

    last_position_move_mode = None

    # Esc is the primary way to stop: checked as a plain flag at the top of
    # every loop iteration, independent of OS signal delivery. Ctrl+C remains
    # a fallback (caught below), but on Windows it's only ever acted on
    # between bytecode instructions -- if the main thread is stuck inside a
    # single blocking call (e.g. real Modbus/serial I/O) when it's pressed,
    # delivery waits until that call returns.
    quit_requested = threading.Event()

    def _on_key_press(key):
        if key == keyboard.Key.esc:
            quit_requested.set()
            return False  # stop the listener

    keyboard_listener = keyboard.Listener(on_press=_on_key_press)
    keyboard_listener.start()

    logging.info("Press Esc to stop (Ctrl+C also works as a fallback)")

    try:
        while not quit_requested.is_set():

            pygame.event.pump()
            control_value = js.get_axis(args.control_axis)

            if mouse_visualizer is not None:
                mouse_visualizer.set_marker_position(js.x / mouse_screen_width)

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
            elif args.control_method == "position":
                gripper.realTimePositionMove(controlSignal=control_value,
                                             controlBuffer=args.controlBuffer,
                                             speedLowerControlThreshold=args.speedLowerControlThreshold,
                                             speedUpperControlThreshold=args.speedUpperControlThreshold,
                                             gripSpeed=args.grip_speed,
                                             gripForce=args.grip_force,
                                             verbose=args.verbose)

                if mouse_visualizer is not None:
                    position_move_mode = gripper.realTimePositionMove_Mode()
                    if position_move_mode != last_position_move_mode:
                        last_position_move_mode = position_move_mode
                        mouse_visualizer.clear_control_zones()
                        cb = args.controlBuffer
                        if position_move_mode in [REALTIME_POSITION_MOVE_MODE_FORCE_DEACTIVATED_CLOSING,
                                                   REALTIME_POSITION_MOVE_MODE_FORCE_ACTIVATED_CLOSING]:
                            mouse_visualizer.add_control_zone(0.0, cb, "cyan")
                            mouse_visualizer.add_control_zone(cb, 2 * cb, "green")
                        elif position_move_mode in [REALTIME_POSITION_MOVE_MODE_FORCE_DEACTIVATED_OPENING,
                                                     REALTIME_POSITION_MOVE_MODE_FORCE_ACTIVATED_OPENING]:
                            mouse_visualizer.add_control_zone(1 - cb, 1.0, "cyan")
                            mouse_visualizer.add_control_zone(1 - 2 * cb, 1 - cb, "green")
                        elif position_move_mode in [REALTIME_POSITION_MOVE_MODE_FREEMOVE]:
                            mouse_visualizer.add_control_zone(0.0, cb, "cyan")
                            mouse_visualizer.add_control_zone(1 - cb, 1.0, "cyan")
                        elif position_move_mode == REALTIME_POSITION_MOVE_MODE_OBJECT_DETECTED_CLOSING:
                            mouse_visualizer.add_control_zone(0.0, cb, (255,215,215))
                        elif position_move_mode == REALTIME_POSITION_MOVE_MODE_OBJECT_DETECTED_OPENING:
                            mouse_visualizer.add_control_zone(1 - cb, 1.0, (255,215,215))
                        else:
                            raise ValueError("This mode is not current managed by the mouse visualizer : ",position_move_mode)


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
            else:
                positionCommand = int(max(0,min(255,control_value*255)))
                speedCommmand = args.grip_speed
                if speedCommmand is None:
                    speedCommmand = 255
                forceCommand = args.grip_force
                if forceCommand is None:
                    forceCommand = 255
                gripper.move(positionCommand,speedCommmand,forceCommand,wait=False,readStatus=True)
                
    except KeyboardInterrupt:
        pass
    finally:
        logging.info(
            "Stopping joystick control (%s)",
            "Esc pressed" if quit_requested.is_set() else "Ctrl+C",
        )
        if visualizer is not None or mouse_visualizer is not None:
            _print_shutdown_notice()

        t_shutdown_start = time.monotonic()

        # Release OS-level hooks first and as early as possible: left
        # running, their callbacks compete for the GIL with whatever the
        # slower teardown below is doing, making input feel laggy system-wide
        # for as long as that takes.
        t0 = time.monotonic()
        keyboard_listener.stop()
        logging.info("Keyboard listener stopped in %.2fs", time.monotonic() - t0)

        if args.joystick_id == -1:
            t0 = time.monotonic()
            js.stop()
            logging.info("Mouse listener stopped in %.2fs", time.monotonic() - t0)

        if mouse_visualizer is not None:
            t0 = time.monotonic()
            mouse_visualizer.stop()
            logging.info("Mouse visualizer stopped in %.2fs", time.monotonic() - t0)

        if visualizer is not None:
            t0 = time.monotonic()
            visualizer.stop()
            logging.info("Visualizer stopped in %.2fs", time.monotonic() - t0)

        if visualizer is not None or mouse_visualizer is not None:
            t0 = time.monotonic()
            from .qt_app_host import get_qt_app_host
            get_qt_app_host().shutdown()
            logging.info("Shared Qt event loop stopped in %.2fs", time.monotonic() - t0)

        t0 = time.monotonic()
        try:
            gripper.disconnect()
        except Exception:
            # Some transports may not support explicit disconnect.
            pass
        logging.info("Gripper disconnected in %.2fs", time.monotonic() - t0)

        logging.info("Total shutdown time: %.2fs", time.monotonic() - t_shutdown_start)

    if visualizer is not None or mouse_visualizer is not None:
        # PySide6's QApplication must live on the main thread. The shared
        # qt_app_host runs it on a background thread instead, to keep this
        # thread free for the control loop above, which works fine at
        # runtime but reliably segfaults during Python's normal interpreter
        # shutdown (regardless of teardown order). Everything that matters
        # (gripper disconnect, bipper stop) has already run above, so skip
        # that shutdown sequence entirely.
        logging.info("Exiting via os._exit(0) to skip PySide6's shutdown segfault")
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
