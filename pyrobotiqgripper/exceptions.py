"""Custom exceptions for pyRobotiqGripper package."""


class RobotiqGripperError(Exception):
    """Base exception for all pyRobotiqGripper errors."""
    pass


class GripperConnectionError(RobotiqGripperError):
    """Raised when connection to gripper fails or is lost."""
    pass


class GripperNotActivatedError(RobotiqGripperError):
    """Raised when an action is requested but gripper is not activated."""
    
    def __init__(self):
        super().__init__(
            "Gripper must be activated before requesting an action. "
            "Call activate() or resetActivate() first."
        )


class GripperNotCalibratedError(RobotiqGripperError):
    """Raised when mm-based operations are used without calibration."""
    
    def __init__(self):
        super().__init__(
            "The gripper must be calibrated before using mm-based positioning. "
            "Call calibrate(closemm, openmm) first."
        )


class GripperTimeoutError(RobotiqGripperError):
    """Raised when gripper operation exceeds timeout."""
    
    def __init__(self, operation: str = "Operation", timeout: float = 10):
        super().__init__(
            f"{operation} did not complete within {timeout} seconds."
        )


class GripperPositionError(RobotiqGripperError):
    """Raised when an invalid position is requested."""
    
    def __init__(self, position: int, min_val: int = 0, max_val: int = 255):
        super().__init__(
            f"Position {position} is out of range [{min_val}, {max_val}]."
        )


class GripperCalibrationError(RobotiqGripperError):
    """Raised when calibration data is invalid."""
    pass


class GripperCommunicationError(RobotiqGripperError):
    """Raised when Modbus communication fails."""
    pass


class UnsupportedGripperTypeError(RobotiqGripperError):
    """Raised when an unsupported gripper type is specified."""
    
    def __init__(self, gripper_type: str):
        super().__init__(
            f"Gripper type '{gripper_type}' is not supported."
        )
