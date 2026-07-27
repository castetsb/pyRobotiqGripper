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


BAUDRATE=115200

BYTESIZE=8

PARITY="N"

STOPBITS=1

TIMEOUT=0.2

AUTO_DETECTION="auto"

#GRIPPER_2F85_VMAX = 150  # mm/s.
#GRIPPER_2F85_VMIN = 20   # mm/s.

#GRIPPER_2F140_VMAX = 250  # mm/s.
#GRIPPER_2F140_VMIN = 30   # mm/s.

#GRIPPER_HANDE_VMAX = 150  # mm/s.
#GRIPPER_HANDE_VMIN = 20   # mm/s.

GRIP_NOT_REQUESTED = 0
GRIP_REQUESTED = 1
GRIP_VALIDATED = 2

NO_COMMAND =0
WRITE_READ_COMMAND = 1
READ_COMMAND = 2

COM_TIME = 0.016 #Approximative time needed to make one communication with the gripper

GRIPPER_MODE_RTU_VIA_TCP = "RTU_VIA_TCP"

GRIPPER_MODE_RTU = "RTU"

MAX_HISTORY = 500 #Command/status history buffer size. Gives ~5s of history
#at a typical 100Hz control loop.

GSTA_NOT_ACTIVATED = 0
GSTA_ACTIVATION_IN_PROGRESS = 1
GSTA_ACTIVATED = 3

GGTO_STOPPED_OR_ACTIVATING = 0
GGTO_GO_TO_REQUESTED_POSITION = 1

GOBJ_IN_MOTION = 0
GOBJ_DETECTED_WHILE_OPENING = 1
GOBJ_DETECTED_WHILE_CLOSING = 2
GOBJ_AT_POSITION = 3

#Estimated object detection
EOBJ_IN_MOTION = 0
EOBJ_AT_POSITION = 3

EOBJ_DETECTED_WHILE_OPENING = 1
EOBJ_DETECTED_WHILE_OPENING_STUCK_ON_RELEASE = 4
EOBJ_STUCK_AT_FULL_OPENING = 5
EOBJ_DETECTED_WHILE_OPENING_SLIPPING = 9

EOBJ_DETECTED_WHILE_CLOSING = 2
EOBJ_DETECTED_WHILE_CLOSING_STUCK_ON_RELEASE = 6
EOBJ_STUCK_AT_FULL_CLOSING = 7
EOBJ_DETECTED_WHILE_CLOSING_SLIPPING = 8

EOBJ_CALCULATION_IMPOSSIBLE = -1

RGTO_STOP = 0
RGTO_GO_TO_REQUESTED_POSITION = 1

GACT_RESET = 0
GACT_ACTIVATE = 1

RACT_DESACTIVATE = 0
RACT_ACTIVATE =1

GRIP_EVALUATION_NO_GRIP = 0
GRIP_EVALUATION_STABLE_GRIP =1
GRIP_EVALUATION_SLIPPING = 2
GRIP_EVALUATION_LOST =3

REALTIME_POSITION_MOVE_MODE_FREEMOVE = 0
REALTIME_POSITION_MOVE_MODE_OBJECT_DETECTED_CLOSING = 100
REALTIME_POSITION_MOVE_MODE_FORCE_DEACTIVATED_CLOSING = 101
REALTIME_POSITION_MOVE_MODE_FORCE_ACTIVATED_CLOSING = 102
REALTIME_POSITION_MOVE_MODE_OBJECT_DETECTED_OPENING = 200
REALTIME_POSITION_MOVE_MODE_FORCE_DEACTIVATED_OPENING = 201
REALTIME_POSITION_MOVE_MODE_FORCE_ACTIVATED_OPENING = 202
REALTIME_POSITION_MOVE_MODE_SECURE = 300
REALTIME_POSITION_IN_LOWER_BUFFER = -2
REALTIME_POSITION_IN_ACTIVATION_BUFFER = -1
REALTIME_POSITION_IN_UPPER_BUFFER = 256
REALTIME_POSITION_POSITION_DELTA_REFERENCE_LAST_AT_POSITION = 0
REALTIME_POSITION_POSITION_DELTA_REFERENCE_CURRENT_POSITION = 1

REALTIME_SPEED_MOVE_MODE_FREEMOVE = 0
REALTIME_SPEED_MOVE_MODE_OBJECT_DETECTED = 100
REALTIME_SPEED_MOVE_MODE_FORCE_DEACTIVATED = 101
REALTIME_SPEED_MOVE_MODE_FORCE_ACTIVATED = 102
REALTIME_SPEED_MOVE_MODE_SECURE = 300

#Command table
TIME = 0
RARD = 1
RATR = 2
RGTO = 3
RACT = 4
RPR = 5
RSP = 6
RFR = 7

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

# The following dictionary is the inverse of COMMAND_HISTORY_COLUMNS_ID_2_NAME,
# allowing to convert from column names to their corresponding index.

COMMAND_HISTORY_COLUMNS_NAME_2_ID={name: id for id, name in COMMAND_HISTORY_COLUMNS_ID_2_NAME.items()}

# Status retrieved from the gripper are also stored in a numpy table. The
# columns of this table are defined here with their corresponding index.

# the following constants make it easy to select table columns by name instead
# of by index. For example, to select the column corresponding to the gSTA
# register, you can use the constant GSTA instead of the integer 2.

#Status table
TIME = 0 #Double define but it is not a problem since the definition is the same for both tables
GOBJ = 1
GSTA = 2
GGTO = 3
GACT = 4
KFLT = 5
GFLT = 6
GPR = 7
GPO = 8
GCU = 9
EOBJ = 10 #estiamted object detection, this is a calculated object detection. Not available in gripper register.

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

# The following dictionary is the inverse of STATUS_HISTORY_COLUMNS_ID_2_NAME,
# allowing to convert from column names to their corresponding index.
STATUS_HISTORY_COLUMNS_NAME_2_ID={name: id for id, name in STATUS_HISTORY_COLUMNS_ID_2_NAME.items()}

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

# The following dictionary is the inverse of HISTORY_COLUMNS_ID_2_NAME,
# allowing to convert from column names to their corresponding index.

HISTORY_COLUMNS_NAME_2_ID={name: id for id, name in HISTORY_COLUMNS_ID_2_NAME.items()}

# The following constants are the indexes of the status columns in the merged table.

M_GOBJ = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + GOBJ
M_GSTA = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + GSTA
M_GGTO = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + GGTO
M_GACT = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + GACT
M_KFLT = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + KFLT
M_GFLT = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + GFLT
M_GPR = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + GPR
M_GPO = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + GPO
M_GCU = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + GCU
M_EOBJ = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + EOBJ


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
"""
Dictionary containing all input and output registers for the Robotiq gripper.

Each top-level key represents a register group:

Input registers (`g` / `k` prefix):
- gOBJ : Object detection status
    - 0: Fingers in motion, no object detected
    - 1: Fingers stopped while opening, object detected
    - 2: Fingers stopped while closing, object detected
    - 3: Fingers at requested position, no object detected or lost/dropped
- gSTA : Gripper status
    - 0: Reset / automatic release
    - 1: Activation in progress
    - 3: Activation completed
- gGTO : Go-to status
    - 0: Stopped / performing activation or release
    - 1: Go to position requested
- gACT : Activation status
    - 0: Gripper reset
    - 1: Gripper activation
- kFLT : Controller fault codes (0–255)
- gFLT : Gripper fault codes (0–255, specific faults for indices 0, 5, 7–15)
- gPR  : Echo of requested positions (0–255)
- gPO  : Actual positions read from encoders (0–255)
- gCU  : Instantaneous current from motor drive (0–255, in mA)

Output registers (`r` prefix):
- rARD : Automatic release status
    - 0: Closing auto-release
    - 1: Opening auto-release
- rATR : Automatic release type
    - 0: Normal
    - 1: Emergency auto-release
- rGTO : Go-to command status
    - 0: Stop
    - 1: Go to requested position
- rACT : Activation command
    - 0: Deactivate gripper
    - 1: Activate gripper (must stay on until routine completes)
- rPR  : Target positions for gripper fingers (0–255)
- rSP  : Speed of gripper movement (0–255)
- rFR  : Final gripping force (0–255)

This dictionary is mapping integer codes to human-readable descriptions for every register.
"""
