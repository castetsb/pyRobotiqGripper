"""Constants for pyRobotiqGripper package.

This module contains all configuration constants used throughout the package,
including communication parameters, gripper limits, and status codes.
"""
#Numpy data table columns id

#Command table
TIME = 0
RARD = 1
RATR = 2
RGTO = 3
RACT = 4
RPR = 5
RSP = 6
RFR = 7

COMMAND_HISTORY_COLUMNS_ID_2_NAME= {0:"time",
                          1:"rARD",
                          2:"rATR",
                          3:"rGTO",
                          4:"rACT",
                          5:"rPR",
                          6:"rSP",
                          7:"rFR"}

COMMAND_HISTORY_COLUMNS_NAME_2_ID={name: id for id, name in COMMAND_HISTORY_COLUMNS_ID_2_NAME.items()}


#Status table
TIME = 0
GOBJ = 1
GSTA = 2
GGTO = 3
GACT = 4
KFLT = 5
GFLT = 6
GPR = 7
GPO = 8
GCU = 9

M_GOBJ = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + GOBJ
M_GSTA = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + GSTA
M_GGTO = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + GGTO
M_GACT = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + GACT
M_KFLT = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + KFLT
M_GFLT = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + GFLT
M_GPR = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + GPR
M_GPO = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + GPO
M_GCU = len(COMMAND_HISTORY_COLUMNS_ID_2_NAME) -1 + GCU

STATUS_HISTORY_COLUMNS_ID_2_NAME= {0:"time",
                          1:"gOBJ",
                          2:"gSTA",
                          3:"gGTO",
                          4:"gACT",
                          5:"kFLT",
                          6:"gFLT",
                          7:"gPR",
                          8:"gPO",
                          9:"gCU"}
STATUS_HISTORY_COLUMNS_NAME_2_ID={name: id for id, name in STATUS_HISTORY_COLUMNS_ID_2_NAME.items()}

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
                 16:"gCU"}

HISTORY_COLUMNS_NAME_2_ID={name: id for id, name in HISTORY_COLUMNS_ID_2_NAME.items()}


def _build_register_dic():
    """Function use to build the register dictionnary
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
                                "gCU":{}})
    
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
    return register_dic



#: Dictionary containing all input and output registers for the Robotiq gripper.
#: 
#: Each top-level key represents a register group:
#: 
#: Input registers (`g` / `k` prefix):
#: - gOBJ : Object detection status
#:     - 0: Fingers in motion, no object detected
#:     - 1: Fingers stopped while opening, object detected
#:     - 2: Fingers stopped while closing, object detected
#:     - 3: Fingers at requested position, no object detected or lost/dropped
#: - gSTA : Gripper status
#:     - 0: Reset / automatic release
#:     - 1: Activation in progress
#:     - 3: Activation completed
#: - gGTO : Go-to status
#:     - 0: Stopped / performing activation or release
#:     - 1: Go to position requested
#: - gACT : Activation status
#:     - 0: Gripper reset
#:     - 1: Gripper activation
#: - kFLT : Controller fault codes (0–255)
#: - gFLT : Gripper fault codes (0–255, specific faults for indices 0, 5, 7–15)
#: - gPR  : Echo of requested positions (0–255)
#: - gPO  : Actual positions read from encoders (0–255)
#: - gCU  : Instantaneous current from motor drive (0–255, in mA)
#: 
#: Output registers (`r` prefix):
#: - rARD : Automatic release status
#:     - 0: Closing auto-release
#:     - 1: Opening auto-release
#: - rATR : Automatic release type
#:     - 0: Normal
#:     - 1: Emergency auto-release
#: - rGTO : Go-to command status
#:     - 0: Stop
#:     - 1: Go to requested position
#: - rACT : Activation command
#:     - 0: Deactivate gripper
#:     - 1: Activate gripper (must stay on until routine completes)
#: - rPR  : Target positions for gripper fingers (0–255)
#: - rSP  : Speed of gripper movement (0–255)
#: - rFR  : Final gripping force (0–255)
#: 
#: This dictionary is mapping integer codes to human-readable descriptions for every register.
REGISTER_DIC = _build_register_dic()

#Constants

#: Default baudrate of the gripper use by Robotiq gripper.
BAUDRATE=115200

#: Byte size use by Robotiq gripper
BYTESIZE=8

#: Parity use by Robotiq gripper
PARITY="N"

#: Stop bits used by Robotiq gripper
STOPBITS=1

#: Default timeout use for communication with Robotiq gripper
TIMEOUT=0.2

#: Automatically detect the USB port on which the gripper connected.
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

#: Set communication to be RTU via TCP
GRIPPER_MODE_RTU_VIA_TCP = "RTU_VIA_TCP"

#: Set communication to be RTU
GRIPPER_MODE_RTU = "RTU"

MAX_HISTORY = 50

GSTA_NOT_ACTIVATED = 0
GSTA_ACTIVATION_IN_PROGRESS = 1
GSTA_ACTIVATED = 3

GGTO_STOPPED_OR_ACTIVATING = 0
GGTO_GO_TO_REQUESTED_POSITION = 1

GOBJ_IN_MOTION = 0
GOBJ_DETECTED_WHILE_OPENING = 1
GOBJ_DETECTED_WHILE_CLOSING = 2
GOBJ_AT_POSITION = 3

RGTO_STOP = 0
RGTO_GO_TO_REQUESTED_POSITION = 1

GACT_RESET = 0
GACT_ACTIVATE = 1

RACT_DESACTIVATE = 0
RACT_ACTIVATE =1