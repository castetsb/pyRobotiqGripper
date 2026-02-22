"""pyRobotiqGripper: Python Driver for Robotiq Grippers via Modbus RTU

pyRobotiqGripper is a Python library designed to facilitate control of Robotiq\
grippers using Modbus RTU communication via serial port.

This module provides documentation in two formats:

- Docstrings: Embedded within the code for easy access.
- Online Documentation: Extensive documentation available at\
    <https://pyrobotiqgripper.readthedocs.io/en/latest/>.
"""

#General information
__author__  = "Benoit CASTETS"
__email__   = "opensourceeng@robotiq.com"
__license__ = "Apache License, Version 2.0"
__url__ = "https://github.com/castetsb/pyRobotiqGripper"
__version__ = "2.0.3"

from .gripper import RobotiqGripper

__all__ = [
    "RobotiqGripper"
]