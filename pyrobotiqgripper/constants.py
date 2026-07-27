"""Constants for pyRobotiqGripper package.

This module contains all configuration constants used throughout the package,
including communication parameters, gripper limits, and status codes.
"""

from typing import Final
# Numpy data table columns id

# The command sent to the gripper are stored in a numpy table.
# The columns of this table are defined here with their corresponding index.

# Table indexes
# Those constant make it easy to select table columns by name instead of by
# index. For example, to select the column corresponding to the rACT register,
# you can use the constant RACT instead of the integer 4.

#Constants

# --- Communication settings ------------------------------------------------

BAUDRATE=115200
"""Default serial baudrate used for Modbus RTU communication with the gripper."""

BYTESIZE=8
"""Serial data bit size used for Modbus RTU communication with the gripper."""

PARITY="N"
"""Serial parity setting used for Modbus RTU communication (``"N"`` = none)."""

STOPBITS=1
"""Serial stop bit count used for Modbus RTU communication with the gripper."""

TIMEOUT=0.2
"""Default communication timeout, in seconds, for a Modbus request/response."""

AUTO_DETECTION="auto"
"""Sentinel value for ``com_port``: auto-detect the gripper's serial port
instead of specifying one explicitly."""

#GRIPPER_2F85_VMAX = 150  # mm/s.
#GRIPPER_2F85_VMIN = 20   # mm/s.

#GRIPPER_2F140_VMAX = 250  # mm/s.
#GRIPPER_2F140_VMIN = 30   # mm/s.

#GRIPPER_HANDE_VMAX = 150  # mm/s.
#GRIPPER_HANDE_VMIN = 20   # mm/s.

COM_TIME = 0.016
"""Approximate time, in seconds, needed for one communication round-trip
with the gripper. Documented for reference; not currently consumed by any
calculation in this package."""

GRIPPER_MODE_RTU_VIA_TCP = "RTU_VIA_TCP"
"""``connection_type`` value: Modbus RTU tunnelled over a TCP gateway."""

GRIPPER_MODE_RTU = "RTU"
"""``connection_type`` value: direct Modbus RTU over a serial port."""

# --- History buffer ---------------------------------------------------------

MAX_HISTORY = 500
"""Size, in rows, of the command/status history ring buffers
(:meth:`~pyrobotiqgripper.RobotiqGripper.commandHistory`,
:meth:`~pyrobotiqgripper.RobotiqGripper.statusHistoryNumpy`). Gives ~5s of
history at a typical 100Hz control loop."""

# --- gSTA / gGTO / gOBJ / gACT status register values -----------------------
# Mirror the value spaces documented in REGISTER_DIC below. Some of these
# (GSTA_NOT_ACTIVATED, GSTA_ACTIVATION_IN_PROGRESS, GGTO_STOPPED_OR_ACTIVATING,
# GACT_RESET, GACT_ACTIVATE) exist to name the full register value space but
# are not directly compared against anywhere in this package's own code.

GSTA_NOT_ACTIVATED = 0
"""``gSTA`` value: gripper in reset (or automatic release) state."""

GSTA_ACTIVATION_IN_PROGRESS = 1
"""``gSTA`` value: gripper activation in progress."""

GSTA_ACTIVATED = 3
"""``gSTA`` value: gripper activation completed. Compared against in
:meth:`~pyrobotiqgripper.RobotiqGripper.isActivated`."""

GGTO_STOPPED_OR_ACTIVATING = 0
"""``gGTO`` value: gripper stopped, or performing activation/automatic release."""

GGTO_GO_TO_REQUESTED_POSITION = 1
"""``gGTO`` value: gripper going to the requested position. Compared against
(alongside the ``rGTO`` command echo) in
:meth:`~pyrobotiqgripper.RobotiqGripper.isStarted`."""

GOBJ_IN_MOTION = 0
"""``gOBJ`` value: fingers in motion towards the requested position, no
object detected. Polling-loop exit condition in the gripper's internal
``_waitComplete``."""

GOBJ_DETECTED_WHILE_OPENING = 1
"""``gOBJ`` value: fingers stopped due to a contact while opening, before
reaching the requested position. Drives the realtime move state machines
(:meth:`~pyrobotiqgripper.RobotiqGripper.realTimePositionMove`,
:meth:`~pyrobotiqgripper.RobotiqGripper.realTimeSpeedMove`) and
:meth:`~pyrobotiqgripper.RobotiqGripper.evaluateGrip`."""

GOBJ_DETECTED_WHILE_CLOSING = 2
"""``gOBJ`` value: fingers stopped due to a contact while closing, before
reaching the requested position. Drives the realtime move state machines
(:meth:`~pyrobotiqgripper.RobotiqGripper.realTimePositionMove`,
:meth:`~pyrobotiqgripper.RobotiqGripper.realTimeSpeedMove`) and
:meth:`~pyrobotiqgripper.RobotiqGripper.evaluateGrip`."""

GOBJ_AT_POSITION = 3
"""``gOBJ`` value: fingers at the requested position, no object detected (or
object lost/dropped). Checked in
:meth:`~pyrobotiqgripper.RobotiqGripper.evaluateGrip` and
:meth:`~pyrobotiqgripper.RobotiqGripper.objectDetection`."""

# --- Estimated object detection (eOBJ) values -------------------------------
# Computed by the gripper's internal ``_estimateObjectDetection`` from
# position/speed history (not a raw Modbus register), and stored in the
# ``EOBJ`` history column alongside every status read.

EOBJ_IN_MOTION = 0
"""``eOBJ`` value: default/fallback -- actively closing the position error,
no object condition detected."""

EOBJ_AT_POSITION = 3
"""``eOBJ`` value: position error resolved (``|gPR - gPO|`` within tolerance)."""

EOBJ_DETECTED_WHILE_OPENING = 1
"""``eOBJ`` value: consistently demanding motion while opening, axis
immobile, not at a mechanical edge, not stuck-on-release -- the estimated
analogue of ``gOBJ`` 1, derived from history rather than the register bit."""

EOBJ_DETECTED_WHILE_OPENING_STUCK_ON_RELEASE = 4
"""``eOBJ`` value: demanding motion while opening and immobile, but the
previous state shows an object was already held while closing -- i.e. stuck
trying to release."""

EOBJ_STUCK_AT_FULL_OPENING = 5
"""``eOBJ`` value: immobile at the minimum position boundary while still
being commanded further open."""

EOBJ_DETECTED_WHILE_OPENING_SLIPPING = 9
"""``eOBJ`` value: consistently demanding motion while opening and the axis
is still moving despite already holding an object -- the object is
compressing/slipping."""

EOBJ_DETECTED_WHILE_CLOSING = 2
"""``eOBJ`` value: consistently demanding motion while closing, axis
immobile, not at a mechanical edge, not stuck-on-release -- the estimated
analogue of ``gOBJ`` 2, derived from history rather than the register bit."""

EOBJ_DETECTED_WHILE_CLOSING_STUCK_ON_RELEASE = 6
"""``eOBJ`` value: demanding motion while closing and immobile, but the
previous state shows an object was already held while opening -- i.e. stuck
trying to release."""

EOBJ_STUCK_AT_FULL_CLOSING = 7
"""``eOBJ`` value: immobile at the maximum position boundary while still
being commanded further closed."""

EOBJ_DETECTED_WHILE_CLOSING_SLIPPING = 8
"""``eOBJ`` value: consistently demanding motion while closing and the axis
is still moving despite already holding an object -- the object is
compressing/slipping."""

EOBJ_CALCULATION_IMPOSSIBLE = -1
"""``eOBJ`` value: the estimate could not be computed (missing speed
calibration, too little/uninitialized history, or a communication gap)."""

# --- rGTO / rACT command register values ------------------------------------

RGTO_STOP = 0
"""``rGTO`` command value: stop. Written by
:meth:`~pyrobotiqgripper.RobotiqGripper.stop`."""

RGTO_GO_TO_REQUESTED_POSITION = 1
"""``rGTO`` command value: go to the requested position. Written by
:meth:`~pyrobotiqgripper.RobotiqGripper.start` and
:meth:`~pyrobotiqgripper.RobotiqGripper.move` (with ``start=True``)."""

GACT_RESET = 0
"""``gACT`` status value: gripper reset."""

GACT_ACTIVATE = 1
"""``gACT`` status value: gripper activation."""

RACT_DESACTIVATE = 0
"""``rACT`` command value: deactivate the gripper."""

RACT_ACTIVATE = 1
"""``rACT`` command value: activate the gripper. Written by
:meth:`~pyrobotiqgripper.RobotiqGripper.activate` and
:meth:`~pyrobotiqgripper.RobotiqGripper.move`."""

# --- realTimePositionMove state machine -------------------------------------

REALTIME_POSITION_MOVE_MODE_FREEMOVE = 0
"""``realTimePositionMove`` mode: normal joystick-follows-signal motion, no
object held."""

REALTIME_POSITION_MOVE_MODE_OBJECT_DETECTED_CLOSING = 100
"""``realTimePositionMove`` mode: an object was just detected while closing;
holding position, waiting for the control signal to return toward the
deadzone before force-release logic engages."""

REALTIME_POSITION_MOVE_MODE_FORCE_DEACTIVATED_CLOSING = 101
"""``realTimePositionMove`` mode: signal is back in the deadzone after a
closing detection; armed, waiting for the signal to push past the deadzone
again to (re-)apply grip force."""

REALTIME_POSITION_MOVE_MODE_FORCE_ACTIVATED_CLOSING = 102
"""``realTimePositionMove`` mode: actively nudging force upward to hold/
increase grip while an object remains detected while closing."""

REALTIME_POSITION_MOVE_MODE_OBJECT_DETECTED_OPENING = 200
"""``realTimePositionMove`` mode: an object was just detected while opening;
holding position, waiting for the control signal to return toward the
deadzone before force-release logic engages."""

REALTIME_POSITION_MOVE_MODE_FORCE_DEACTIVATED_OPENING = 201
"""``realTimePositionMove`` mode: signal is back in the deadzone after an
opening detection; armed, waiting for the signal to push past the deadzone
again to (re-)apply grip force."""

REALTIME_POSITION_MOVE_MODE_FORCE_ACTIVATED_OPENING = 202
"""``realTimePositionMove`` mode: actively nudging force upward to hold/
increase grip while an object remains detected while opening."""

REALTIME_POSITION_MOVE_MODE_SECURE = 300
"""``realTimePositionMove`` mode: entered instead of the
``OBJECT_DETECTED_*`` modes when no ``gripSpeed``/``gripForce`` override is
configured; drives the gripper further into the detected object
proportionally to signal until it releases back to ``FREEMOVE``."""

REALTIME_POSITION_IN_LOWER_BUFFER = -2
"""Reserved: sentinel for a below-range position-buffer classification. Not
currently used by :meth:`~pyrobotiqgripper.RobotiqGripper.realTimePositionMove`."""

REALTIME_POSITION_IN_ACTIVATION_BUFFER = -1
"""Reserved: sentinel for an activation-range position-buffer
classification. Not currently used by
:meth:`~pyrobotiqgripper.RobotiqGripper.realTimePositionMove`."""

REALTIME_POSITION_IN_UPPER_BUFFER = 256
"""Reserved: sentinel for an above-range position-buffer classification. Not
currently used by :meth:`~pyrobotiqgripper.RobotiqGripper.realTimePositionMove`."""

REALTIME_POSITION_POSITION_DELTA_REFERENCE_LAST_AT_POSITION = 0
"""Reserved: sentinel selecting "last at-position" as the reference point
for a position-delta calculation. Not currently used by
:meth:`~pyrobotiqgripper.RobotiqGripper.realTimePositionMove`."""

REALTIME_POSITION_POSITION_DELTA_REFERENCE_CURRENT_POSITION = 1
"""Reserved: sentinel selecting "current position" as the reference point
for a position-delta calculation. Not currently used by
:meth:`~pyrobotiqgripper.RobotiqGripper.realTimePositionMove`."""

# --- realTimeSpeedMove state machine -----------------------------------------

REALTIME_SPEED_MOVE_MODE_FREEMOVE = 0
"""``realTimeSpeedMove`` mode: normal speed-follows-signal motion, no object
held."""

REALTIME_SPEED_MOVE_MODE_OBJECT_DETECTED = 100
"""``realTimeSpeedMove`` mode: an object was just detected (closing or
opening); holding, waiting for the signal to return to near-zero deadzone."""

REALTIME_SPEED_MOVE_MODE_FORCE_DEACTIVATED = 101
"""``realTimeSpeedMove`` mode: signal is back near zero after detection;
armed, waiting for renewed signal magnitude to reapply force."""

REALTIME_SPEED_MOVE_MODE_FORCE_ACTIVATED = 102
"""``realTimeSpeedMove`` mode: actively nudging speed/force to hold/increase
grip while an object remains detected."""

REALTIME_SPEED_MOVE_MODE_SECURE = 300
"""``realTimeSpeedMove`` mode: fallback engagement mode (no ``gripSpeed``/
``gripForce`` override) driving further into the object based on signal
sign/magnitude."""

# --- Command history table column indices -----------------------------------
# Column indices into the numpy array returned by
# :meth:`~pyrobotiqgripper.RobotiqGripper.commandHistory`.

TIME = 0
"""Command/status history column index: time, in seconds."""
RARD = 1
"""Command history column index: ``rARD`` (automatic release direction) register."""
RATR = 2
"""Command history column index: ``rATR`` (automatic release type) register."""
RGTO = 3
"""Command history column index: ``rGTO`` (go-to command) register."""
RACT = 4
"""Command history column index: ``rACT`` (activation command) register."""
RPR = 5
"""Command history column index: ``rPR`` (requested position) register."""
RSP = 6
"""Command history column index: ``rSP`` (requested speed) register."""
RFR = 7
"""Command history column index: ``rFR`` (requested force) register."""

# The following dictionaries allow to convert between column names and their
# corresponding index.

COMMAND_HISTORY_COLUMNS_ID_2_NAME= {0:"time",
                          1:"rARD",
                          2:"rATR",
                          3:"rGTO",
                          4:"rACT",
                          5:"rPR",
                          6:"rSP",
                          7:"rFR"}
"""Maps :meth:`~pyrobotiqgripper.RobotiqGripper.commandHistory` column index
to register name."""

COMMAND_HISTORY_COLUMNS_NAME_2_ID={name: id for id, name in COMMAND_HISTORY_COLUMNS_ID_2_NAME.items()}
"""Inverse of :data:`COMMAND_HISTORY_COLUMNS_ID_2_NAME`: register name to
:meth:`~pyrobotiqgripper.RobotiqGripper.commandHistory` column index."""

# Status retrieved from the gripper are also stored in a numpy table. The
# columns of this table are defined here with their corresponding index.

# the following constants make it easy to select table columns by name instead
# of by index. For example, to select the column corresponding to the gSTA
# register, you can use the constant GSTA instead of the integer 2.

# --- Status history table column indices ------------------------------------
# Column indices into the numpy array returned by
# :meth:`~pyrobotiqgripper.RobotiqGripper.statusHistoryNumpy`. TIME is
# redefined here with the same value as the command table's TIME above --
# harmless, since both tables share the same time column semantics.

TIME = 0
"""Command/status history column index: time, in seconds."""
GOBJ = 1
"""Status history column index: ``gOBJ`` (object detection) register."""
GSTA = 2
"""Status history column index: ``gSTA`` (activation status) register."""
GGTO = 3
"""Status history column index: ``gGTO`` (go-to status) register."""
GACT = 4
"""Status history column index: ``gACT`` (activation status) register."""
KFLT = 5
"""Status history column index: ``kFLT`` (controller fault) register."""
GFLT = 6
"""Status history column index: ``gFLT`` (gripper fault) register."""
GPR = 7
"""Status history column index: ``gPR`` (requested position echo) register."""
GPO = 8
"""Status history column index: ``gPO`` (actual position) register."""
GCU = 9
"""Status history column index: ``gCU`` (motor current) register."""
EOBJ = 10
"""Status history column index: ``eOBJ``, the estimated object detection
value computed from history -- not an actual gripper register (see the
:data:`EOBJ_IN_MOTION` group above)."""

# The following dictionaries allow to convert between column names and their
# corresponding index.
STATUS_HISTORY_COLUMNS_ID_2_NAME= {0:"time",
                          1:"gOBJ",
                          2:"gSTA",
                          3:"gGTO",
                          4:"gACT",
                          5:"kFLT",
                          6:"gFLT",
                          7:"gPR",
                          8:"gPO",
                          9:"gCU",
                          10:"eOBJ"}
"""Maps :meth:`~pyrobotiqgripper.RobotiqGripper.statusHistoryNumpy` column
index to register name."""

STATUS_HISTORY_COLUMNS_NAME_2_ID={name: id for id, name in STATUS_HISTORY_COLUMNS_ID_2_NAME.items()}
"""Inverse of :data:`STATUS_HISTORY_COLUMNS_ID_2_NAME`: register name to
:meth:`~pyrobotiqgripper.RobotiqGripper.statusHistoryNumpy` column index."""

# To make it easy to check gripper command history and status history at the same
# time, pyRobotigGripper can join the command history and status history tables into a single table.

# The following constants are the indexes of the status columns in this merged table.

HISTORY_COLUMNS_ID_2_NAME={0:"time",
                 1:"rARD",
                 2:"rATR",
                 3:"rGTO",
                 4:"rACT",
                 5:"rPR",
                 6:"rSP",
                 7:"rFR",
                 8:"gOBJ",
                 9:"gSTA",
                 10:"gGTO",
                 11:"gACT",
                 12:"kFLT",
                 13:"gFLT",
                 14:"gPR",
                 15:"gPO",
                 16:"gCU",
                 17:"eOBJ"}
"""Maps :meth:`~pyrobotiqgripper.RobotiqGripper.historyNumpy` (command and
status columns merged into one table) column index to register name."""

# The following dictionary is the inverse of HISTORY_COLUMNS_ID_2_NAME,
# allowing to convert from column names to their corresponding index.

HISTORY_COLUMNS_NAME_2_ID={name: id for id, name in HISTORY_COLUMNS_ID_2_NAME.items()}
"""Inverse of :data:`HISTORY_COLUMNS_ID_2_NAME`: register name to
:meth:`~pyrobotiqgripper.RobotiqGripper.historyNumpy` column index."""

# The following constants are the indexes of the status columns in the merged table.
# Computed as the command table's column count (minus 1, since the shared
# ``time`` column isn't duplicated) plus the status table's own column index.

M_GOBJ = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + GOBJ
"""Merged history table (:meth:`~pyrobotiqgripper.RobotiqGripper.historyNumpy`)
column index for ``gOBJ``. Used e.g. in
:meth:`~pyrobotiqgripper.RobotiqGripper.evaluateGrip`."""
M_GSTA = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + GSTA
"""Merged history table column index for ``gSTA``."""
M_GGTO = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + GGTO
"""Merged history table column index for ``gGTO``."""
M_GACT = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + GACT
"""Merged history table column index for ``gACT``."""
M_KFLT = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + KFLT
"""Merged history table column index for ``kFLT``."""
M_GFLT = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + GFLT
"""Merged history table column index for ``gFLT``."""
M_GPR = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + GPR
"""Merged history table column index for ``gPR``."""
M_GPO = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + GPO
"""Merged history table column index for ``gPO``. Used e.g. in
:meth:`~pyrobotiqgripper.RobotiqGripper.evaluateGrip`."""
M_GCU = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + GCU
"""Merged history table column index for ``gCU``."""
M_EOBJ = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + EOBJ
"""Merged history table column index for ``eOBJ``."""


def _build_register_dic():
    """Builds a dictionary containing all input and output registers of the
    Robotiq gripper. The dictionary contains a description of the signification
    of each register value.

    The structure of the dictionary is as follows:
    {
        "gOBJ": {0: "Fingers are in motion towards requested position. No object detected.",
                 1: "Fingers have stopped due to a contact while opening before requested position. Object detected opening.",
                 2: "Fingers have stopped due to a contact while closing before requested position. Object detected closing.",
                 3: "Fingers are at requested position. No object detected or object has been loss / dropped."},
        "gSTA": {0: "Gripper is in reset ( or automatic release ) state. See Fault Status if Gripper is activated.",
                 1: "Activation in progress.",
                 3: "Activation is completed."},
        ...
    }
    """
    register_dic = {}
    #input register variable
    register_dic.update({"gOBJ":{},
                                "gSTA":{},
                                "gGTO":{},
                                "gACT":{},
                                "kFLT":{},
                                "gFLT":{},
                                "gPR":{},
                                "gPO":{},
                                "gCU":{},
                                "eOBJ":{}})
    
    #gOBJ
    gOBJdic=register_dic["gOBJ"]
    
    gOBJdic[0]="Fingers are in motion towards requested position. No object detected."
    gOBJdic[1]="Fingers have stopped due to a contact while opening before requested position. Object detected opening."
    gOBJdic[2]="Fingers have stopped due to a contact while closing before requested position. Object detected closing."
    gOBJdic[3]="Fingers are at requested position. No object detected or object has been loss / dropped."
    
    #gSTA
    gSTAdic=register_dic["gSTA"]
    
    gSTAdic[0]="Gripper is in reset ( or automatic release ) state. See Fault Status if Gripper is activated."
    gSTAdic[1]="Activation in progress."
    gSTAdic[3]="Activation is completed."
    
    #gGTO
    gGTOdic=register_dic["gGTO"]
    
    gGTOdic[0]="Stopped (or performing activation / automatic release)."
    gGTOdic[1]="Go to Position Request."
    gGTOdic[2]="Unknown status"
    gGTOdic[3]="Unknown status"
    
    #gACT
    gACTdic=register_dic["gACT"]
    
    gACTdic[0]="Gripper reset."
    gACTdic[1]="Gripper activation."
    
    #kFLT
    kFLTdic=register_dic["kFLT"]
    i=0
    while i<256:
        kFLTdic[i]=i
        i+=1
    
    #See your optional Controller Manual (input registers & status).
    
    #gFLT
    gFLTdic=register_dic["gFLT"]
    i=0
    while i<256:
        gFLTdic[i]=i
        i+=1
    gFLTdic[0]="No fault (LED is blue)"
    gFLTdic[5]="Priority faults (LED is blue). Action delayed, activation (reactivation) must be completed prior to performing the action."
    gFLTdic[7]="Priority faults (LED is blue). The activation bit must be set prior to action."
    gFLTdic[8]="Minor faults (LED continuous red). Maximum operating temperature exceeded, wait for cool-down."
    gFLTdic[9]="Minor faults (LED continuous red). No communication during at least 1 second."
    gFLTdic[10]="Major faults (LED blinking red/blue) - Reset is required (rising edge on activation bit rACT needed). Under minimum operating voltage."
    gFLTdic[11]="Major faults (LED blinking red/blue) - Reset is required (rising edge on activation bit rACT needed). Automatic release in progress."
    gFLTdic[12]="Major faults (LED blinking red/blue) - Reset is required (rising edge on activation bit rACT needed). Internal fault; contact support@robotiq.com."
    gFLTdic[13]="Major faults (LED blinking red/blue) - Reset is required (rising edge on activation bit rACT needed). Activation fault, verify that no interference or other error occurred."
    gFLTdic[14]="Major faults (LED blinking red/blue) - Reset is required (rising edge on activation bit rACT needed). Overcurrent triggered."
    gFLTdic[15]="Major faults (LED blinking red/blue) - Reset is required (rising edge on activation bit rACT needed). Automatic release completed."
    
    #gPR
    gPRdic=register_dic["gPR"]
    
    i=0
    while i<256:
        gPRdic[i]="Echo of the requested position for the Gripper:{}/255".format(i)
        i+=1
    
    #gPO
    gPOdic=register_dic["gPO"]
    i=0
    while i<256:
        gPOdic[i]="Actual position of the Gripper obtained via the encoders:{}/255".format(i)
        i+=1
    
    #gCU
    gCUdic=register_dic["gCU"]
    i=0
    while i<256:
        current=i*10
        gCUdic[i]="The current is read instantaneously from the motor drive, approximate current: {} mA".format(current)
        i+=1

    ######################################################################
    #output register variable
    register_dic.update({"rARD":{},
                                "rATR":{},
                                "rGTO":{},
                                "rACT":{},
                                "rPR":{},
                                "rSP":{},
                                "rFR":{}})
    
    rARDdic=register_dic["rARD"]

    rARDdic[0]="Closingauto-release"
    rARDdic[1]="Openingauto-release"

    rATRdic=register_dic["rATR"]

    rATRdic[0]="Normal"
    rATRdic[1]="Emergency auto-release"

    rGTOdic=register_dic["rGTO"]

    rGTOdic[0]="Stop."
    rGTOdic[1]="Gotorequestedposition."

    rACTdic=register_dic["rACT"]

    rACTdic[0]="DeactivateGripper"
    rACTdic[1]="ActivateGripper(muststayonafteractivationroutineiscompleted)."

    rPRdic=register_dic["rPR"]
    i=0
    while i<256:
        position=i*10
        rPRdic[i]="The target position for the Gripper's fingers is set to {}/255".format(position)
        i+=1

    rSPdic=register_dic["rSP"]
    i=0
    while i<256:
        speed=i*10
        rSPdic[i]="The Gripper closing or opening speed is set to {}/255".format(speed)
        i+=1

    rFRdic=register_dic["rFR"]
    i=0
    while i<256:
        force=i*10
        rFRdic[i]="The final gripping force for the Gripper is set to {}/255".format(force)
        i+=1

    eOBJdic=register_dic["eOBJ"]

    eOBJdic[EOBJ_AT_POSITION]="Fingers are at requested position. No object detected or object has been loss / dropped."
    eOBJdic[EOBJ_DETECTED_WHILE_CLOSING]="Fingers have stopped due to a contact while closing before requested position. Object detected closing."
    eOBJdic[EOBJ_DETECTED_WHILE_CLOSING_STUCK_ON_RELEASE]="Stuck on release after detecting an object while closing"
    eOBJdic[EOBJ_DETECTED_WHILE_OPENING]="Fingers have stopped due to a contact while opening before requested position. Object detected opening."
    eOBJdic[EOBJ_DETECTED_WHILE_OPENING_STUCK_ON_RELEASE]="Stuck on release after detectin an object while opening"
    eOBJdic[EOBJ_IN_MOTION]="Fingers are in motion towards requested position. No object detected."
    eOBJdic[EOBJ_STUCK_AT_FULL_CLOSING]="Stuck in the extrem close position"
    eOBJdic[EOBJ_STUCK_AT_FULL_OPENING]="Stuck in the extrem open position"

    return register_dic


REGISTER_DIC: Final[dict] = _build_register_dic()
"""Dictionary mapping every input/output register name to a ``{code: description}``
dict of human-readable text for each of that register's values.

Each top-level key is a register name; its value is a dict from raw integer
code to description.

Input registers (``g`` / ``k`` prefix):

- ``gOBJ`` -- object detection status: 0 fingers in motion (no object
  detected), 1 stopped while opening (object detected), 2 stopped while
  closing (object detected), 3 at requested position (no object detected, or
  object lost/dropped).
- ``gSTA`` -- gripper status: 0 reset/automatic release, 1 activation in
  progress, 3 activation completed.
- ``gGTO`` -- go-to status: 0 stopped/performing activation or release, 1 go
  to position requested.
- ``gACT`` -- activation status: 0 gripper reset, 1 gripper activation.
- ``kFLT`` -- controller fault codes (0-255).
- ``gFLT`` -- gripper fault codes (0-255, with specific fault text for
  indices 0, 5, 7-15).
- ``gPR`` -- echo of requested positions (0-255).
- ``gPO`` -- actual positions read from the encoders (0-255).
- ``gCU`` -- instantaneous current from the motor drive (0-255, in mA).

Output registers (``r`` prefix):

- ``rARD`` -- automatic release status: 0 closing auto-release, 1 opening
  auto-release.
- ``rATR`` -- automatic release type: 0 normal, 1 emergency auto-release.
- ``rGTO`` -- go-to command status: 0 stop, 1 go to requested position.
- ``rACT`` -- activation command: 0 deactivate gripper, 1 activate gripper
  (must stay on until the activation routine completes).
- ``rPR`` -- target positions for the gripper's fingers (0-255).
- ``rSP`` -- gripper closing/opening speed (0-255).
- ``rFR`` -- final gripping force (0-255).
"""
