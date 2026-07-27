(api)=

# API Reference

## RobotiqGripper

```{eval-rst}
.. autoclass:: pyrobotiqgripper.RobotiqGripper
   :show-inheritance:
   :noindex:
```

### Setup

```{eval-rst}
.. automethod:: pyrobotiqgripper.RobotiqGripper.connect
.. automethod:: pyrobotiqgripper.RobotiqGripper.disconnect
.. automethod:: pyrobotiqgripper.RobotiqGripper.reset
.. automethod:: pyrobotiqgripper.RobotiqGripper.activate
.. automethod:: pyrobotiqgripper.RobotiqGripper.start
.. automethod:: pyrobotiqgripper.RobotiqGripper.stop
.. automethod:: pyrobotiqgripper.RobotiqGripper.calibrate_bit
.. automethod:: pyrobotiqgripper.RobotiqGripper.calibrate_speed
.. automethod:: pyrobotiqgripper.RobotiqGripper.calibrate_mm
```

### Control

```{eval-rst}
.. automethod:: pyrobotiqgripper.RobotiqGripper.open
.. automethod:: pyrobotiqgripper.RobotiqGripper.close
.. automethod:: pyrobotiqgripper.RobotiqGripper.open_mm
.. automethod:: pyrobotiqgripper.RobotiqGripper.close_mm
.. automethod:: pyrobotiqgripper.RobotiqGripper.move
.. automethod:: pyrobotiqgripper.RobotiqGripper.move_mm
.. automethod:: pyrobotiqgripper.RobotiqGripper.moveToCurrentPosition
```

### Realtime Control

```{eval-rst}
.. automethod:: pyrobotiqgripper.RobotiqGripper.realTimePositionMove
.. automethod:: pyrobotiqgripper.RobotiqGripper.realTimePositionMove_Mode
.. automethod:: pyrobotiqgripper.RobotiqGripper.realTimeSpeedMove
.. automethod:: pyrobotiqgripper.RobotiqGripper.realTimeSpeedMove_Mode
```

### Status

```{eval-rst}
.. automethod:: pyrobotiqgripper.RobotiqGripper.isActivated
.. automethod:: pyrobotiqgripper.RobotiqGripper.isStarted
.. automethod:: pyrobotiqgripper.RobotiqGripper.is_bit_calibrated
.. automethod:: pyrobotiqgripper.RobotiqGripper.is_mm_calibrated
.. automethod:: pyrobotiqgripper.RobotiqGripper.positioningResolution
.. automethod:: pyrobotiqgripper.RobotiqGripper.is_speed_calibrated
.. automethod:: pyrobotiqgripper.RobotiqGripper.gripper_vmax_bits
.. automethod:: pyrobotiqgripper.RobotiqGripper.gripper_vmin_bits
.. automethod:: pyrobotiqgripper.RobotiqGripper.minSpeedMmSecond
.. automethod:: pyrobotiqgripper.RobotiqGripper.maxSpeedMmSecond
.. automethod:: pyrobotiqgripper.RobotiqGripper.speedResolutionBit
.. automethod:: pyrobotiqgripper.RobotiqGripper.speedResolutionMm
.. automethod:: pyrobotiqgripper.RobotiqGripper.positionCommand
.. automethod:: pyrobotiqgripper.RobotiqGripper.position
.. automethod:: pyrobotiqgripper.RobotiqGripper.position_mm
.. automethod:: pyrobotiqgripper.RobotiqGripper.lastMoveTime
.. automethod:: pyrobotiqgripper.RobotiqGripper.lastMoveDirection
.. automethod:: pyrobotiqgripper.RobotiqGripper.speed
.. automethod:: pyrobotiqgripper.RobotiqGripper.force
.. automethod:: pyrobotiqgripper.RobotiqGripper.objectDetection
.. automethod:: pyrobotiqgripper.RobotiqGripper.printObjectDetection
.. automethod:: pyrobotiqgripper.RobotiqGripper.evaluateGrip
.. automethod:: pyrobotiqgripper.RobotiqGripper.readStatus
.. automethod:: pyrobotiqgripper.RobotiqGripper.lastStatusReadTime
.. automethod:: pyrobotiqgripper.RobotiqGripper.status
.. automethod:: pyrobotiqgripper.RobotiqGripper.printStatus
.. automethod:: pyrobotiqgripper.RobotiqGripper.commandHistoryPanda
.. automethod:: pyrobotiqgripper.RobotiqGripper.statusHistoryPanda
.. automethod:: pyrobotiqgripper.RobotiqGripper.historyPanda
.. automethod:: pyrobotiqgripper.RobotiqGripper.commandHistory
.. automethod:: pyrobotiqgripper.RobotiqGripper.statusHistoryNumpy
.. automethod:: pyrobotiqgripper.RobotiqGripper.historyNumpy
```

## Additional Tools

These tools are not required to control a gripper: they support experimenting
with and debugging the realtime control features (see
{doc}`Realtime usage <realtime>`).

### Mouse Joystick

```{eval-rst}
.. autoclass:: pyrobotiqgripper.mouse_joystick.MouseJoystick
   :members:
   :show-inheritance:
```

```{data} pyrobotiqgripper.mouse_joystick.AXIS_X
:annotation:

Axis index for the horizontal mouse position, for use with
{meth}`~pyrobotiqgripper.mouse_joystick.MouseJoystick.get_axis`.
```

```{data} pyrobotiqgripper.mouse_joystick.AXIS_Y
:annotation:

Axis index for the vertical mouse position, for use with
{meth}`~pyrobotiqgripper.mouse_joystick.MouseJoystick.get_axis`.
```

### Joystick Visual Tool

Requires the optional `PySide6` package, included in the `all` extra
(`uv add "pyrobotiqgripper[all]"`). Generic over any joystick-like object
exposing `get_axis(axis)` -- {class}`~pyrobotiqgripper.mouse_joystick.MouseJoystick`
or a real `pygame.joystick.Joystick`.

```{eval-rst}
.. autoclass:: pyrobotiqgripper.joystick_visual_tool.JoystickVisualTool
   :members:
   :show-inheritance:
```

### Gripper Visualizer

Requires the optional `PySide6` package, included in the `all` extra
(`uv add "pyrobotiqgripper[all]"`).

```{eval-rst}
.. autoclass:: pyrobotiqgripper.visualizer.GripperVisualizer
   :members:
   :show-inheritance:
```

```{data} pyrobotiqgripper.visualizer.BOUNDED_SIGNALS
:annotation:

History column names selectable on the 0-255 bounded chart
(`gPO`, `rPR`, `rSP`, `rFR`, `gPR`, `gCU`).
```

```{data} pyrobotiqgripper.visualizer.DEFAULT_BOUNDED_SIGNALS
:annotation:

Signals checked by default on the bounded chart when none are given:
actual position, commanded speed and commanded force.
```

```{data} pyrobotiqgripper.visualizer.STATE_SIGNALS
:annotation:

History column names selectable on the state chart, carrying small
enumerated / flag register values (`gOBJ`, `gSTA`, `gGTO`, `gACT`,
`kFLT`, `gFLT`, `rARD`, `rATR`, `rGTO`, `rACT`).
```

```{data} pyrobotiqgripper.visualizer.DEFAULT_STATE_SIGNALS
:annotation:

Signals checked by default on the state chart when none are given:
object detection.
```

### Bipper

Requires the optional `sounddevice` package, included in the `all` extra
(`uv add "pyrobotiqgripper[all]"`).

Plays a continuous tone whose beep rate speeds up as
{attr}`~pyrobotiqgripper.bipper.Bipper.input_signal` increases towards
`1.0`. Used by the Joystick CLI's `--bipper` option to give an audio cue
of grip force during realtime control.

```{eval-rst}
.. autoclass:: pyrobotiqgripper.bipper.Bipper
   :members:
   :show-inheritance:
```

### Joystick CLI

Console script (`pyrobotiqgripper-joystick`) used to drive a gripper with a
joystick or mouse from the command line. See
{ref}`the Joystick CLI usage guide <joystick-cli-feature>` for installation
and examples; run `pyrobotiqgripper-joystick --help` for the full list of
options.

```{eval-rst}
.. autofunction:: pyrobotiqgripper.joystick_cli.main
```

## Constants

Every constant below is documented at its definition in
`pyrobotiqgripper/constants.py`; this page just pulls those docstrings in,
grouped by topic.

```{currentmodule} pyrobotiqgripper.constants
```

### Communication settings

```{eval-rst}
.. autodata:: BAUDRATE
.. autodata:: BYTESIZE
.. autodata:: PARITY
.. autodata:: STOPBITS
.. autodata:: TIMEOUT
.. autodata:: AUTO_DETECTION
.. autodata:: GRIPPER_MODE_RTU
.. autodata:: GRIPPER_MODE_RTU_VIA_TCP
.. autodata:: COM_TIME
```

### History buffer

```{eval-rst}
.. autodata:: MAX_HISTORY
```

### Gripper status register values (gSTA / gGTO / gOBJ / gACT)

```{eval-rst}
.. autodata:: GSTA_NOT_ACTIVATED
.. autodata:: GSTA_ACTIVATION_IN_PROGRESS
.. autodata:: GSTA_ACTIVATED
.. autodata:: GGTO_STOPPED_OR_ACTIVATING
.. autodata:: GGTO_GO_TO_REQUESTED_POSITION
.. autodata:: GOBJ_IN_MOTION
.. autodata:: GOBJ_DETECTED_WHILE_OPENING
.. autodata:: GOBJ_DETECTED_WHILE_CLOSING
.. autodata:: GOBJ_AT_POSITION
.. autodata:: GACT_RESET
.. autodata:: GACT_ACTIVATE
```

### Gripper command register values (rGTO / rACT)

```{eval-rst}
.. autodata:: RGTO_STOP
.. autodata:: RGTO_GO_TO_REQUESTED_POSITION
.. autodata:: RACT_DESACTIVATE
.. autodata:: RACT_ACTIVATE
```

### Estimated object detection (eOBJ) values

Computed by the gripper from position/speed history (not a raw Modbus
register) and stored in the `eOBJ` history column.

```{eval-rst}
.. autodata:: EOBJ_IN_MOTION
.. autodata:: EOBJ_AT_POSITION
.. autodata:: EOBJ_DETECTED_WHILE_OPENING
.. autodata:: EOBJ_DETECTED_WHILE_OPENING_STUCK_ON_RELEASE
.. autodata:: EOBJ_STUCK_AT_FULL_OPENING
.. autodata:: EOBJ_DETECTED_WHILE_OPENING_SLIPPING
.. autodata:: EOBJ_DETECTED_WHILE_CLOSING
.. autodata:: EOBJ_DETECTED_WHILE_CLOSING_STUCK_ON_RELEASE
.. autodata:: EOBJ_STUCK_AT_FULL_CLOSING
.. autodata:: EOBJ_DETECTED_WHILE_CLOSING_SLIPPING
.. autodata:: EOBJ_CALCULATION_IMPOSSIBLE
```

### Realtime position move modes

State values of {meth}`~pyrobotiqgripper.RobotiqGripper.realTimePositionMove_Mode`.

```{eval-rst}
.. autodata:: REALTIME_POSITION_MOVE_MODE_FREEMOVE
.. autodata:: REALTIME_POSITION_MOVE_MODE_OBJECT_DETECTED_CLOSING
.. autodata:: REALTIME_POSITION_MOVE_MODE_FORCE_DEACTIVATED_CLOSING
.. autodata:: REALTIME_POSITION_MOVE_MODE_FORCE_ACTIVATED_CLOSING
.. autodata:: REALTIME_POSITION_MOVE_MODE_OBJECT_DETECTED_OPENING
.. autodata:: REALTIME_POSITION_MOVE_MODE_FORCE_DEACTIVATED_OPENING
.. autodata:: REALTIME_POSITION_MOVE_MODE_FORCE_ACTIVATED_OPENING
.. autodata:: REALTIME_POSITION_MOVE_MODE_SECURE
.. autodata:: REALTIME_POSITION_IN_LOWER_BUFFER
.. autodata:: REALTIME_POSITION_IN_ACTIVATION_BUFFER
.. autodata:: REALTIME_POSITION_IN_UPPER_BUFFER
.. autodata:: REALTIME_POSITION_POSITION_DELTA_REFERENCE_LAST_AT_POSITION
.. autodata:: REALTIME_POSITION_POSITION_DELTA_REFERENCE_CURRENT_POSITION
```

### Realtime speed move modes

State values of {meth}`~pyrobotiqgripper.RobotiqGripper.realTimeSpeedMove_Mode`.

```{eval-rst}
.. autodata:: REALTIME_SPEED_MOVE_MODE_FREEMOVE
.. autodata:: REALTIME_SPEED_MOVE_MODE_OBJECT_DETECTED
.. autodata:: REALTIME_SPEED_MOVE_MODE_FORCE_DEACTIVATED
.. autodata:: REALTIME_SPEED_MOVE_MODE_FORCE_ACTIVATED
.. autodata:: REALTIME_SPEED_MOVE_MODE_SECURE
```

### History table column indices

Column indices into the numpy arrays returned by
{meth}`~pyrobotiqgripper.RobotiqGripper.commandHistory`,
{meth}`~pyrobotiqgripper.RobotiqGripper.statusHistoryNumpy`, and
{meth}`~pyrobotiqgripper.RobotiqGripper.historyNumpy` (command and status
merged).

```{eval-rst}
.. autodata:: TIME
.. autodata:: RARD
.. autodata:: RATR
.. autodata:: RGTO
.. autodata:: RACT
.. autodata:: RPR
.. autodata:: RSP
.. autodata:: RFR
.. autodata:: GOBJ
.. autodata:: GSTA
.. autodata:: GGTO
.. autodata:: GACT
.. autodata:: KFLT
.. autodata:: GFLT
.. autodata:: GPR
.. autodata:: GPO
.. autodata:: GCU
.. autodata:: EOBJ
.. autodata:: M_GOBJ
.. autodata:: M_GSTA
.. autodata:: M_GGTO
.. autodata:: M_GACT
.. autodata:: M_KFLT
.. autodata:: M_GFLT
.. autodata:: M_GPR
.. autodata:: M_GPO
.. autodata:: M_GCU
.. autodata:: M_EOBJ
```

### Column name / index lookup dictionaries

```{eval-rst}
.. autodata:: COMMAND_HISTORY_COLUMNS_ID_2_NAME
   :annotation:
.. autodata:: COMMAND_HISTORY_COLUMNS_NAME_2_ID
   :annotation:
.. autodata:: STATUS_HISTORY_COLUMNS_ID_2_NAME
   :annotation:
.. autodata:: STATUS_HISTORY_COLUMNS_NAME_2_ID
   :annotation:
.. autodata:: HISTORY_COLUMNS_ID_2_NAME
   :annotation:
.. autodata:: HISTORY_COLUMNS_NAME_2_ID
   :annotation:
```

### Register reference

```{eval-rst}
.. autodata:: REGISTER_DIC
   :annotation:
```
