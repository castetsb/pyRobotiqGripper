"""pyRobotiqGripper: Python Driver for Robotiq Grippers via Modbus RTU/TCP

pyRobotiqGripper is a Python library designed to facilitate control of Robotiq
grippers using Modbus RTU communication via serial port or Modbus TCP over Ethernet.

This module provides documentation in two formats:

- Docstrings: Embedded within the code for easy access.
- Online Documentation: Extensive documentation available at
    <https://pyrobotiqgripper.readthedocs.io/en/latest/>.
"""

from email.utils import parseaddr
from importlib.metadata import PackageNotFoundError, metadata as _pkg_metadata

# The distribution name, needed to look up our own installed metadata below.
__project__ = 'pyrobotiqgripper'

# All other metadata is defined once, in pyproject.toml; read it back from
# the installed package metadata instead of duplicating it here.
try:
    _metadata = _pkg_metadata(__project__)
    __version__ = _metadata["Version"]
    __author__ = _metadata["Author"]
    __email__ = parseaddr(_metadata["Author-email"])[1]
    __url__ = next(
        (url.split(", ", 1)[1] for url in _metadata.get_all("Project-URL", [])
         if url.split(", ", 1)[0].strip().lower() == "homepage"),
        None,
    )
    __license__ = next(
        (c.rsplit("::", 1)[1].strip() for c in _metadata.get_all("Classifier", [])
         if c.startswith("License ::")),
        None,
    )
except PackageNotFoundError:
    __version__ = __author__ = __email__ = __url__ = __license__ = "unknown"

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
    #Constants
    "REGISTER_DIC",
    "BAUDRATE",
    "BYTESIZE",
    "PARITY",
    "STOPBITS",
    "TIMEOUT",
    "AUTO_DETECTION",
    "GRIPPER_MODE_RTU_VIA_TCP",
    "GRIPPER_MODE_RTU", 
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