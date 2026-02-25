"""Constants for pyRobotiqGripper package.

This module contains all configuration constants used throughout the package,
including communication parameters, gripper limits, and status codes.
"""

#Constants
BAUDRATE=115200
BYTESIZE=8
PARITY="N"
STOPBITS=1
TIMEOUT=0.2
AUTO_DETECTION="auto"

GRIPPER_2F_VMAX = 332  # max speed of the 2F gripper in steps per second
GRIPPER_2F_VMIN = 68   # min speed of the 2F gripper in steps per second

NO_OBJECT_DETECTED = 0
OBJECT_DETECTED_WHILE_OPENING = 1
OBJECT_DETECTED_WHILE_CLOSING = 2

GRIP_NOT_REQUESTED = 0
GRIP_REQUESTED = 1
GRIP_VALIDATED = 2

GRIPPER_VMAX = 332  # max speed in steps per second
GRIPPER_VMIN = 68   # min speed in steps per second

NO_COMMAND =0
WRITE_READ_COMMAND = 1
READ_COMMAND = 2

COM_TIME = 0.016 #Approximative time needed to make one communication with the gripper

GRIPPER_MODE_RTU_VIA_TCP = "RTU_VIA_TCP"
GRIPPER_MODE_RTU = "RTU"