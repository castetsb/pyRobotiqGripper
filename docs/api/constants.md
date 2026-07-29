# Constants

Every constant below is documented at its definition in
`pyrobotiqgripper/constants.py`; this page just pulls those docstrings in,
grouped by topic.

<div class="constants-page" markdown="1">

## Communication settings

::: pyrobotiqgripper.constants.BAUDRATE
::: pyrobotiqgripper.constants.BYTESIZE
::: pyrobotiqgripper.constants.PARITY
::: pyrobotiqgripper.constants.STOPBITS
::: pyrobotiqgripper.constants.TIMEOUT
::: pyrobotiqgripper.constants.AUTO_DETECTION
::: pyrobotiqgripper.constants.GRIPPER_MODE_RTU
::: pyrobotiqgripper.constants.GRIPPER_MODE_RTU_VIA_TCP
::: pyrobotiqgripper.constants.COM_TIME

## History buffer

::: pyrobotiqgripper.constants.MAX_HISTORY

## Gripper status register values

::: pyrobotiqgripper.constants.GSTA_NOT_ACTIVATED
::: pyrobotiqgripper.constants.GSTA_ACTIVATION_IN_PROGRESS
::: pyrobotiqgripper.constants.GSTA_ACTIVATED
::: pyrobotiqgripper.constants.GGTO_STOPPED_OR_ACTIVATING
::: pyrobotiqgripper.constants.GGTO_GO_TO_REQUESTED_POSITION
::: pyrobotiqgripper.constants.GOBJ_IN_MOTION
::: pyrobotiqgripper.constants.GOBJ_DETECTED_WHILE_OPENING
::: pyrobotiqgripper.constants.GOBJ_DETECTED_WHILE_CLOSING
::: pyrobotiqgripper.constants.GOBJ_AT_POSITION
::: pyrobotiqgripper.constants.GACT_RESET
::: pyrobotiqgripper.constants.GACT_ACTIVATE

## Gripper command register values

::: pyrobotiqgripper.constants.RGTO_STOP
::: pyrobotiqgripper.constants.RGTO_GO_TO_REQUESTED_POSITION
::: pyrobotiqgripper.constants.RACT_DESACTIVATE
::: pyrobotiqgripper.constants.RACT_ACTIVATE

## Estimated object detection (eOBJ) values

Computed by the gripper from position/speed history (not a raw Modbus
register) and stored in the `eOBJ` history column.

::: pyrobotiqgripper.constants.EOBJ_IN_MOTION
::: pyrobotiqgripper.constants.EOBJ_AT_POSITION
::: pyrobotiqgripper.constants.EOBJ_DETECTED_WHILE_OPENING
::: pyrobotiqgripper.constants.EOBJ_DETECTED_WHILE_OPENING_STUCK_ON_RELEASE
::: pyrobotiqgripper.constants.EOBJ_STUCK_AT_FULL_OPENING
::: pyrobotiqgripper.constants.EOBJ_DETECTED_WHILE_OPENING_SLIPPING
::: pyrobotiqgripper.constants.EOBJ_DETECTED_WHILE_CLOSING
::: pyrobotiqgripper.constants.EOBJ_DETECTED_WHILE_CLOSING_STUCK_ON_RELEASE
::: pyrobotiqgripper.constants.EOBJ_STUCK_AT_FULL_CLOSING
::: pyrobotiqgripper.constants.EOBJ_DETECTED_WHILE_CLOSING_SLIPPING
::: pyrobotiqgripper.constants.EOBJ_CALCULATION_IMPOSSIBLE

## Realtime position move modes

State values of
[`realTimePositionMove_Mode`][pyrobotiqgripper.RobotiqGripper.realTimePositionMove_Mode].

::: pyrobotiqgripper.constants.REALTIME_POSITION_MOVE_MODE_FREEMOVE
::: pyrobotiqgripper.constants.REALTIME_POSITION_MOVE_MODE_OBJECT_DETECTED_CLOSING
::: pyrobotiqgripper.constants.REALTIME_POSITION_MOVE_MODE_FORCE_DEACTIVATED_CLOSING
::: pyrobotiqgripper.constants.REALTIME_POSITION_MOVE_MODE_FORCE_ACTIVATED_CLOSING
::: pyrobotiqgripper.constants.REALTIME_POSITION_MOVE_MODE_OBJECT_DETECTED_OPENING
::: pyrobotiqgripper.constants.REALTIME_POSITION_MOVE_MODE_FORCE_DEACTIVATED_OPENING
::: pyrobotiqgripper.constants.REALTIME_POSITION_MOVE_MODE_FORCE_ACTIVATED_OPENING
::: pyrobotiqgripper.constants.REALTIME_POSITION_MOVE_MODE_SECURE
::: pyrobotiqgripper.constants.REALTIME_POSITION_IN_LOWER_BUFFER
::: pyrobotiqgripper.constants.REALTIME_POSITION_IN_ACTIVATION_BUFFER
::: pyrobotiqgripper.constants.REALTIME_POSITION_IN_UPPER_BUFFER
::: pyrobotiqgripper.constants.REALTIME_POSITION_POSITION_DELTA_REFERENCE_LAST_AT_POSITION
::: pyrobotiqgripper.constants.REALTIME_POSITION_POSITION_DELTA_REFERENCE_CURRENT_POSITION

## Realtime speed move modes

State values of
[`realTimeSpeedMove_Mode`][pyrobotiqgripper.RobotiqGripper.realTimeSpeedMove_Mode].

::: pyrobotiqgripper.constants.REALTIME_SPEED_MOVE_MODE_FREEMOVE
::: pyrobotiqgripper.constants.REALTIME_SPEED_MOVE_MODE_OBJECT_DETECTED
::: pyrobotiqgripper.constants.REALTIME_SPEED_MOVE_MODE_FORCE_DEACTIVATED
::: pyrobotiqgripper.constants.REALTIME_SPEED_MOVE_MODE_FORCE_ACTIVATED
::: pyrobotiqgripper.constants.REALTIME_SPEED_MOVE_MODE_SECURE

## History table column indices

Column indices into the numpy arrays returned by
[`commandHistory`][pyrobotiqgripper.RobotiqGripper.commandHistory],
[`statusHistoryNumpy`][pyrobotiqgripper.RobotiqGripper.statusHistoryNumpy], and
[`historyNumpy`][pyrobotiqgripper.RobotiqGripper.historyNumpy] (command and
status merged).

::: pyrobotiqgripper.constants.TIME
::: pyrobotiqgripper.constants.RARD
::: pyrobotiqgripper.constants.RATR
::: pyrobotiqgripper.constants.RGTO
::: pyrobotiqgripper.constants.RACT
::: pyrobotiqgripper.constants.RPR
::: pyrobotiqgripper.constants.RSP
::: pyrobotiqgripper.constants.RFR
::: pyrobotiqgripper.constants.GOBJ
::: pyrobotiqgripper.constants.GSTA
::: pyrobotiqgripper.constants.GGTO
::: pyrobotiqgripper.constants.GACT
::: pyrobotiqgripper.constants.KFLT
::: pyrobotiqgripper.constants.GFLT
::: pyrobotiqgripper.constants.GPR
::: pyrobotiqgripper.constants.GPO
::: pyrobotiqgripper.constants.GCU
::: pyrobotiqgripper.constants.EOBJ
::: pyrobotiqgripper.constants.M_GOBJ
::: pyrobotiqgripper.constants.M_GSTA
::: pyrobotiqgripper.constants.M_GGTO
::: pyrobotiqgripper.constants.M_GACT
::: pyrobotiqgripper.constants.M_KFLT
::: pyrobotiqgripper.constants.M_GFLT
::: pyrobotiqgripper.constants.M_GPR
::: pyrobotiqgripper.constants.M_GPO
::: pyrobotiqgripper.constants.M_GCU
::: pyrobotiqgripper.constants.M_EOBJ

## Column name / index lookup dictionaries

::: pyrobotiqgripper.constants.COMMAND_HISTORY_COLUMNS_ID_2_NAME
::: pyrobotiqgripper.constants.COMMAND_HISTORY_COLUMNS_NAME_2_ID
::: pyrobotiqgripper.constants.STATUS_HISTORY_COLUMNS_ID_2_NAME
::: pyrobotiqgripper.constants.STATUS_HISTORY_COLUMNS_NAME_2_ID
::: pyrobotiqgripper.constants.HISTORY_COLUMNS_ID_2_NAME
::: pyrobotiqgripper.constants.HISTORY_COLUMNS_NAME_2_ID

## Register reference

::: pyrobotiqgripper.constants.REGISTER_DIC

</div>
