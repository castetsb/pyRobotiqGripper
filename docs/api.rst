.. _api:

API Reference
=============

RobotiqGripper
--------------

.. autoclass:: pyrobotiqgripper.RobotiqGripper
   :noindex:
   :show-inheritance:

Initialization
~~~~~~~~~~~~~~

.. automethod:: pyrobotiqgripper.RobotiqGripper.__init__

Setup
~~~~~

.. automethod:: pyrobotiqgripper.RobotiqGripper.connect
.. automethod:: pyrobotiqgripper.RobotiqGripper.disconnect
.. automethod:: pyrobotiqgripper.RobotiqGripper.reset
.. automethod:: pyrobotiqgripper.RobotiqGripper.activate
.. automethod:: pyrobotiqgripper.RobotiqGripper.start
.. automethod:: pyrobotiqgripper.RobotiqGripper.stop
.. automethod:: pyrobotiqgripper.RobotiqGripper.calibrate_bit
.. automethod:: pyrobotiqgripper.RobotiqGripper.calibrate_speed
.. automethod:: pyrobotiqgripper.RobotiqGripper.calibrate_mm

Control
~~~~~~~

.. automethod:: pyrobotiqgripper.RobotiqGripper.open
.. automethod:: pyrobotiqgripper.RobotiqGripper.close
.. automethod:: pyrobotiqgripper.RobotiqGripper.move
.. automethod:: pyrobotiqgripper.RobotiqGripper.move_mm
.. automethod:: pyrobotiqgripper.RobotiqGripper.realTimeMove

Status
~~~~~~

.. automethod:: pyrobotiqgripper.RobotiqGripper.isActivated
.. automethod:: pyrobotiqgripper.RobotiqGripper.isStarted
.. automethod:: pyrobotiqgripper.RobotiqGripper.is_bit_calibrated
.. automethod:: pyrobotiqgripper.RobotiqGripper.is_mm_calibrated
.. automethod:: pyrobotiqgripper.RobotiqGripper.is_speed_calibrated
.. automethod:: pyrobotiqgripper.RobotiqGripper.gripper_vmax_bits
.. automethod:: pyrobotiqgripper.RobotiqGripper.gripper_vmin_bits
.. automethod:: pyrobotiqgripper.RobotiqGripper.positionCommand
.. automethod:: pyrobotiqgripper.RobotiqGripper.position
.. automethod:: pyrobotiqgripper.RobotiqGripper.position_mm
.. automethod:: pyrobotiqgripper.RobotiqGripper.speed
.. automethod:: pyrobotiqgripper.RobotiqGripper.force
.. automethod:: pyrobotiqgripper.RobotiqGripper.objectDetection
.. automethod:: pyrobotiqgripper.RobotiqGripper.readStatus
.. automethod:: pyrobotiqgripper.RobotiqGripper.status
.. automethod:: pyrobotiqgripper.RobotiqGripper.printStatus   
.. automethod:: pyrobotiqgripper.RobotiqGripper.commandHistory
.. automethod:: pyrobotiqgripper.RobotiqGripper.statusHistory
.. automethod:: pyrobotiqgripper.RobotiqGripper.history

Constants
---------

.. currentmodule:: pyrobotiqgripper
.. data:: BAUDRATE
   :annotation:

   Default baudrate of the gripper use by Robotiq gripper.

.. data:: BYTESIZE
   :annotation:

   Byte size use by Robotiq gripper

.. data:: PARITY
   :annotation:

   Parity use by Robotiq gripper

.. data:: STOPBITS
   :annotation:

   Stop bits used by Robotiq gripper

.. data:: TIMEOUT
   :annotation:

   Default timeout use for communication with Robotiq gripper

.. data:: AUTO_DETECTION
   :annotation:

   Automatically detect the USB port on which the gripper connected.

.. data:: GRIPPER_MODE_RTU_VIA_TCP
   :annotation:

   Set communication to be RTU via TCP

.. data:: GRIPPER_MODE_RTU
   :annotation:

   Set communication to be RTU

.. data:: REGISTER_DIC
   :annotation:

   
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
   