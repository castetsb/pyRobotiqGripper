"""pyRobotiqGripper: Python Driver for Robotiq Grippers via Modbus RTU/TCP

pyRobotiqGripper is a Python library designed to facilitate control of Robotiq
grippers using Modbus RTU communication via serial port or Modbus TCP over Ethernet.

This module provides documentation in two formats:

- Docstrings: Embedded within the code for easy access.
- Online Documentation: Extensive documentation available at
    <https://pyrobotiqgripper.readthedocs.io/en/latest/>.
"""

# Package metadata
__author__ = "Benoit CASTETS"
__email__ = "opensourceeng@robotiq.com"
__license__ = "Apache License, Version 2.0"
__url__ = "https://github.com/castetsb/pyRobotiqGripper"
__version__ = "2.0.6"

# Main class
from .gripper import RobotiqGripper

# Constants
from .constants import (
    BAUDRATE,
    BYTESIZE,
    PARITY,
    STOPBITS,
    TIMEOUT,
    AUTO_DETECTION,
    GRIPPER_2F_VMAX,
    GRIPPER_2F_VMIN,
    NO_OBJECT_DETECTED,
    OBJECT_DETECTED_WHILE_OPENING,
    OBJECT_DETECTED_WHILE_CLOSING,
    GRIP_NOT_REQUESTED,
    GRIP_REQUESTED,
    GRIP_VALIDATED,
    GRIPPER_VMAX,
    GRIPPER_VMIN,
    NO_COMMAND,
    WRITE_READ_COMMAND,
    READ_COMMAND,
    COM_TIME,
)

# Exceptions
from .exceptions import (
    RobotiqGripperError,
    GripperConnectionError,
    GripperNotActivatedError,
    GripperNotCalibratedError,
    GripperTimeoutError,
    GripperPositionError,
    GripperCalibrationError,
    GripperCommunicationError,
    UnsupportedGripperTypeError,
)

# Public API
__all__ = [
    # Main class
    "RobotiqGripper",
    # Constants
    "BAUDRATE",
    "BYTESIZE",
    "PARITY",
    "STOPBITS",
    "TIMEOUT",
    "AUTO_DETECTION",
    "GRIPPER_2F_VMAX",
    "GRIPPER_2F_VMIN",
    "NO_OBJECT_DETECTED",
    "OBJECT_DETECTED_WHILE_OPENING",
    "OBJECT_DETECTED_WHILE_CLOSING",
    "GRIP_NOT_REQUESTED",
    "GRIP_REQUESTED",
    "GRIP_VALIDATED",
    "GRIPPER_VMAX",
    "GRIPPER_VMIN",
    "NO_COMMAND",
    "WRITE_READ_COMMAND",
    "READ_COMMAND",
    "COM_TIME",
    # Exceptions
    "RobotiqGripperError",
    "GripperConnectionError",
    "GripperNotActivatedError",
    "GripperNotCalibratedError",
    "GripperTimeoutError",
    "GripperPositionError",
    "GripperCalibrationError",
    "GripperCommunicationError",
    "UnsupportedGripperTypeError",
]