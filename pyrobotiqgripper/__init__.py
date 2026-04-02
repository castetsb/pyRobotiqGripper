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
__version__ = "3.0.3"
__project__ = 'pyrobotiqgripper'

# Main class
from .gripper import RobotiqGripper

# Constants
from .constants import *

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
    GripperFaultError,
    GripperValidationError,
    UnsupportedGripperTypeError,
)

# Public API
__all__ = [
    # Main class
    "RobotiqGripper",
    # Constants
    #"BAUDRATE",
    #"BYTESIZE",
    #"PARITY",
    #"STOPBITS",
    #"TIMEOUT",
    
    # Exceptions
    "RobotiqGripperError",
    "GripperConnectionError",
    "GripperNotActivatedError",
    "GripperNotCalibratedError",
    "GripperTimeoutError",
    "GripperPositionError",
    "GripperCalibrationError",
    "GripperCommunicationError",
    "GripperFaultError",
    "GripperValidationError",
    "UnsupportedGripperTypeError",
]