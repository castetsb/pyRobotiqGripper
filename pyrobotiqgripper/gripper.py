"""Core Robotiq Gripper class for controlling via Modbus RTU/TCP."""

# Optional dependency: pandas is only required for history DataFrame helpers.

# Meno: Windows / Linux: Ctrl + K then Ctrl + 0 to folds all foldable regions (functions,classes,etc.) in Visual code studio

import numpy as np
from pymodbus.client import ModbusSerialClient, ModbusTcpClient
from pymodbus.framer import FramerType
import time
import serial
import serial.tools.list_ports
from .utils import *
from .constants import *
from .exceptions import *
import logging
import multiprocessing
import warnings

def _get_pandas():
    """Lazy import pandas and provide clear error when missing."""
    try:
        import pandas as pd
        return pd
    except ImportError as exc:
        raise ImportError(
            "pandas is required only for RobotiqGripper history methods "
            "(commandHistory, statusHistory, history). Install pandas via "
            "`pip install pandas` or avoid calling these methods."
        ) from exc

class RobotiqGripper( ):
    """Class use to control Robotiq grippers (2F85, 2F140 or hande).

    This class provides methods to initialize, open, close, and monitor the gripper.

    Physical connection
    -------------------    
    The physical connection with the gripper can done in 2 ways:
    - Connected to the PC via the USB/RS485 adapter.
    - Connected to the UR robot: In this case the UR RS485 URCAP needs to be installed
    on the robot controller.

    Modbus RTU/TCP communication
    ----------------------------
    Modbus RTU function code supported by robotiq gripper

    =======================================  ====================
    Description                              Modbus function code
    =======================================  ====================
    Read registers                           4
    Write registers                          16
    Master read & write multiple registers   23
    =======================================  ====================
    
    For more information for gripper communication please check gripper manual
    on Robotiq website.
    https://robotiq.com/support/2f-85-2f-140

    .. note::
        This class cannot be use to control epick, 3F or powerpick.
    """    
    def __init__(self,
                 com_port: str = AUTO_DETECTION,
                 device_id: int=9,
                 connection_type: str = GRIPPER_MODE_RTU,
                 tcp_host: str = "127.0.0.1",
                 tcp_port: int = 54321,
                 baudrate = BAUDRATE,
                 debug: bool = False,
                 **kwargs):
        """Create a RobotiqGripper object which can be use to control Robotiq
        grippers using modbus RTU protocol USB/RS485 connection.
        
        Args:
            com_port (str): COM port to which the gripper is connected.
                If "auto" (or equal to the constant AUTO_DETECTION), the library will try 
                to find the COM port to which the gripper is connected.
                On Windows, COM ports are named COM1, COM2, etc. On Linux, COM ports are
                named /dev/ttyUSB0, /dev/ttyUSB1, etc. Default is AUTO_DETECTION.
            device_id (int): Address of the gripper (integer) usually 9.
            gripper_type (str): Type of the gripper. Currently only "2F" is supported.
                Default is "2F".
            connection_type (str): Type of connection to the gripper.
                "RTU" (or equal to the constant GRIPPER_MODE_RTU) for direct Modbus RTU
                connection (e.g. via USB/RS485 adapter). "RTU_VIA_TCP" (or equal to the
                constant GRIPPER_MODE_RTU_VIA_TCP) for Modbus RTU connection via TCP
                (e.g. when using the UR RS485 URCAP). Default is "RTU".
            tcp_host (str): Host IP address for TCP connection. Default is "127.0.0.1"
            tcp_port (int): Port number for TCP connection. Default is 54321.
            baudrate (int): Gripper communication baudrate.
            debug (bool): If True, enable debug logging for Modbus communication.
                Default is False.
        
        Examples:

            Gripper connected at PC USB port:
            
            >>> import pyrobotiqgripper as rq
            >>> gripper = rq.RobotiqGripper(connection_type=rq.GRIPPER_MODE_RTU)
            >>> gripper.connect()
            >>> gripper.activate()
            >>> gripper.start()
            >>> gripper.open()
            >>> gripper.close()
            >>> gripper.move(100) #Move at position 100 in bit
            >>> print(gripper.position()) #Print gripper position in bit
            >>> gripper.calibrate_mm(closemm=0,openmm=85) #Calibrate the gripper with 0mm when closed and 85mm when open
            >>> gripper.move_mm(50) #Move at position 50mm
            >>> gripper.printStatus() #Print gripper status information in the python terminal
            >>> print(gripper.positionmm()) #Print gripper position in mm
            
            Gripper connected to UR robot via RS495 URCAP (RTU over TCP):
            
            >>> import pyrobotiqgripper as rq
            >>> gripper = rq.RobotiqGripper(connection_type=rq.GRIPPER_MODE_RTU_VIA_TCP,tcp_host="192.168.1.100")
            >>> gripper.connect()
            >>> gripper.activate()
            >>> gripper.start()
            >>> gripper.open()
        """
        #Public properties
        ##################

        #: COM port to which the gripper is connected.
        #: If "auto" (or equal to the constant AUTO_DETECTION), the library will try 
        #: to find the COM port to which the gripper is connected.
        #: On Windows, COM ports are named COM1, COM2, etc. On Linux, COM ports are
        #: named /dev/ttyUSB0, /dev/ttyUSB1, etc. Default is AUTO_DETECTION.
        self.com_port=com_port
        self.baudrate=baudrate

        self.device_id=device_id
        self.connection_type=connection_type
        self.tcp_host=tcp_host
        self.tcp_port=tcp_port
        self.debug = debug
        #Maximum allowed time to perform and action
        self.timeOut=5

        #Private properties
        ###################
        #Client managing gripper commmunication
        self._client=self._create_modbus_client()

        self.connect()


        #Status retrieval historical data
        self._commandHistory=np.ones((MAX_HISTORY,len(COMMAND_HISTORY_COLUMNS_ID_2_NAME)))*-1

        self._statusHistory=np.ones((MAX_HISTORY,len(STATUS_HISTORY_COLUMNS_ID_2_NAME)))*-1
        
        
        self._is_bit_calibrated=False
        #Say if the gripper is calibrated to be controlled in mm
        self._is_mm_calibrated=False
        #Say if the gripper is calibrated to be estimate position from speed
        self._is_speed_calibrated=False
        
        #Distance between the fingers when gripper is closed
        self._closemm=None
        #Position in bit when gripper is closed
        self._closebit=None
        #Distance between the fingers when gripper is open
        self._openmm=None
        #Position in bit when gripper is open
        self._openbit=None
        
        #Linear coefficient to link bit and distance between fingers
        #mm=self._aCoef*bit+self._bCoef
        self._aCoef=None #positioning resolution
        self._bCoef=None
        
        self._gripper_vmax_bits=None #Speed in bit per second with gripper
        #speed parameter at 255
        self._gripper_vmin_bits=None #Speed in bit per second with gripper
        #speed parameter at 0

        #Last direction of movement of the gripper.
        # 1 if closing, -1 if opening, 0 if unknown
        self._lastMoveDirection = 0
        self._lastMoveTime = None

        self._lastObjectDetectionTime = None

        self._last_at_position_status_position = None


        self._realtimePositionMove_NudgeBaseline = None
        self._realTimePositionMove_ForceCommand = None
        self._realtimePositionMove_force_mode_start_force =None
        self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_FREEMOVE


        self._realtimeSpeedMove_NudgeBaseline = None
        self._realTimeSpeedMove_ForceCommand = None
        self._realtimeSpeedMove_force_mode_start_force =None
        self._realtimeSpeedMove_Mode = REALTIME_SPEED_MOVE_MODE_FREEMOVE

        #Initialisation
        ###############
        self._configure_logging()
        self.readStatus()

    ####################################################
    ### PRIVATE FUNCTIONS
    ###################################################
    
    #SETUP FUNCTIONS
    def _probe_port_process(self, port, device_id, return_dict, timeout=1):
        """Probe a serial port to check if a gripper is connected.

        This method attempts to connect to a Modbus device on the specified port
        and reads a register to verify if it's a gripper.

        Parameters:
        -----------
        port : str
            The serial port to probe (e.g., 'COM1', '/dev/ttyUSB0').
        device_id : int
            The Modbus device ID to use for communication.
        return_dict : dict
            A shared dictionary to store the result of the probe.
        timeout : float, optional
            Timeout for the connection attempt in seconds. Default is 1.
        """
        try:
            client = ModbusSerialClient(
                port=port,
                baudrate=self.baudrate,
                parity='N',
                stopbits=1,
                bytesize=8,
                timeout=timeout
            )

            print(f"Trying to connect to {port}...")
            if not client.connect():
                err = f"Fail to connect to modbus RTU device on {port}."
                print(err)
                return_dict["success"] = False
                return_dict["error"] = err
                return

            result = client.read_input_registers(
                address=2000,
                count=1,
                device_id=device_id
            )

            if result is None:
                err = f"No response from {port} (register read returned None)"
                print(err)
                return_dict["success"] = False
                return_dict["error"] = err
                return

            if hasattr(result, 'isError') and result.isError():
                error_code = getattr(result, 'exception_code', 'unknown')
                err = f"Modbus error on {port}: {error_code}"
                print(err)
                return_dict["success"] = False
                return_dict["error"] = err
                return
                
            if hasattr(result, 'registers') and len(result.registers) > 0:
                return_dict["success"] = True
            else:
                print(f"Invalid register response from {port}: {result}")
                return_dict["success"] = False

        except Exception as e:
            error_msg = f"Connection failed on port {port}: {type(e).__name__}: {str(e)}"
            print(error_msg)
            logging.error(error_msg)  # Also log for debugging
            return_dict["success"] = False


        finally:
            try:
                client.close()
            except:
                pass
    
    def _configure_logging(self):
        """Configure logging for Modbus communication based on the debug flag."""
        logger = logging.getLogger("pymodbus")

        if self.debug:
            logging.basicConfig(
                level=logging.DEBUG,
                format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
            )
            logger.setLevel(logging.DEBUG)
        else:
            logger.setLevel(logging.CRITICAL)  # Effectively disables it
    
    def _create_modbus_client(self):
        """Factory method to create appropriate Modbus client."""
        
        if self.connection_type == GRIPPER_MODE_RTU_VIA_TCP:
            return ModbusTcpClient(
                host=self.tcp_host,
                port=self.tcp_port,
                timeout=1,
                framer=FramerType.RTU)
        elif self.connection_type == GRIPPER_MODE_RTU:
            if self.com_port == AUTO_DETECTION:
                self.com_port = self._autoConnect()
                
            
            return ModbusSerialClient(
                port=self.com_port,
                baudrate=self.baudrate,
                parity='N',
                stopbits=1,
                bytesize=8,
                timeout=1)

    def _autoConnect(self):
        """Automatically detect the COM port to which the gripper is connected.

        This method scans available serial ports and attempts to communicate with
        a gripper on each one to find the correct port.

        Returns:
        --------
        str
            The COM port where the gripper was detected.

        Raises:
        ------
        GripperConnectionError
            If no gripper is detected on any available port.
        """
        ports = serial.tools.list_ports.comports()
        manager = multiprocessing.Manager()  # Create ONCE

        for port in ports:
            print(f"Testing: {port.device}")

            return_dict = manager.dict()

            p = multiprocessing.Process(
                target=self._probe_port_process,
                args=(port.device, self.device_id, return_dict)
                )

            p.start()
            p.join(3.0)  # HARD TIMEOUT (1 second)

            if p.is_alive():
                print(f"Hard timeout — killing process on {port.device}")
                p.terminate()
                p.join()
                print(f"Probe timed out on {port.device}; no port response from probe worker. This means the port may not be a Modbus device or is very slow.")
                continue

            if p.exitcode is not None and p.exitcode != 0:
                print(f"Probe process for {port.device} exited with code {p.exitcode}")

            if return_dict.get("success", False):
                print(f"Gripper detected on {port.device}")
                return port.device

            reason = return_dict.get("error", "none")
            print(f"No gripper response on {port.device} (reason: {reason})")


        raise GripperConnectionError(
            f"No gripper detected on any of {len(list(serial.tools.list_ports.comports()))} "
            f"available ports. Tested ports: "
            f"{', '.join([p.device for p in serial.tools.list_ports.comports()])}. "
            f"Please check: 1) Gripper is powered, 2) USB cable connected, "
            f"3) Device ID matches (expected {self.device_id})"
        )

    #COMMUNICATION FUNCTIONS

    #Write only: Modbus function code 16 (Write multiple registers)

    def _writeA(self, rARD, rATR, rGTO, rACT):
        """Write activation and action control parameters. "A" means Action.

        Args:
            rARD (int): Automatic release status
                0: Closing auto-release
                1: Opening auto-release
            rATR (int): Automatic release type
                0: Normal
                1: Emergency auto-release
            rGTO (int): Go-to action command.
                0: Stop
                1: Go to requested position
            rACT (int): Activation command.
                0: Deactivate gripper
                1: Activate gripper (must stay on until routine completes)
        """
        action = (rARD << 13) | (rATR << 12) | (rGTO << 11) | rACT<<8

        res=self._client.write_registers(address=1000,
                                         values=[action],
                                         device_id=self.device_id)
        
        # Check result
        if res.isError():
            raise GripperCommunicationError("Write failed")
        
        t=floor_to_ms(time.monotonic())
        command={"time":t,"rARD":rARD,"rATR":rATR,"rGTO":rGTO,"rACT":rACT}

        self._completeAndSaveCommand(command)

    def _writeP(self,position):
        """Write position in the command register.

        Args:
            position (int):
                The position to move the gripper to in bits. Integer between 0 and 255.
        """
        res=self._client.write_registers(address=1001,
                                 values=[position],
                                 device_id=self.device_id)

        # Check result
        if res.isError():
            raise GripperCommunicationError("Write failed")
        t=floor_to_ms(time.monotonic())
        command={"time":t,"rPR":position}

        self._completeAndSaveCommand(command)
    
    def _writeSF(self,speed, force):
        """Write speed and force in the command register.

        Args:
            speed (int):
                The speed of the gripper movement. Integer between 0 and 255.
            force (int):
                The force of the gripper movement. Integer between 0 and 255.
        """
        res=self._client.write_registers(address=1002,
                                 values=[(speed << 8) | force],
                                  device_id=self.device_id)

        # Check result
        if res.isError():
            raise GripperCommunicationError("Write failed")
        t=floor_to_ms(time.monotonic())
        command={"time":t,"rSP":speed,"rFR":force}

        self._completeAndSaveCommand(command)

    def _writePSF(self,position, speed, force):
        """Write position, speed and force in the command register.

        Args:
            position (int):
                The position to move the gripper to in bits. Integer between 0 and 255.
            speed (int):
                The speed of the gripper movement. Integer between 0 and 255.
            force (int):
                The force of the gripper movement. Integer between 0 and 255.
        """
        res=self._client.write_registers(address=1001,
                                 values=[position, (speed << 8) | force],
                                  device_id=self.device_id)

        # Check result
        if res.isError():
            raise GripperCommunicationError("Write failed")
        t=floor_to_ms(time.monotonic())
        command={"time":t,"rPR":position, "rSP":speed,"rFR":force}

        self._completeAndSaveCommand(command)
    
    def _writeAPSF(self, rARD, rATR, rGTO, rACT, position, speed, force):
        """Write Action Position Speed Force in the command register.

        Args:
            rARD (int): Automatic release status
                0: Closing auto-release
                1: Opening auto-release
            rATR (int): Automatic release type
                0: Normal
                1: Emergency auto-release
            rGTO (int): Go-to action command.
                0: Stop
                1: Go to requested position
            rACT (int): Activation command.
                0: Deactivate gripper
                1: Activate gripper (must stay on until routine completes)
            position (int):
                The position to move the gripper to in bits. Integer between 0 and 255.
            speed (int):
                The speed of the gripper movement. Integer between 0 and 255.
            force (int):
                The force of the gripper movement. Integer between 0 and 255.
        """
        action = (rARD << 13) | (rATR << 12) | (rGTO << 11) | rACT<<8

        res=self._client.write_registers(address=1000,
                                         values=[action,
                                                 position,
                                                 (speed << 8) | force],
                                         device_id=self.device_id)
        
        # Check result
        if res.isError():
            raise GripperCommunicationError("Write failed")
        
        t=floor_to_ms(time.monotonic())

        command={"time":t,"rARD":rARD,"rATR":rATR,"rGTO":rGTO,"rACT":rACT,"rPR":position,"rSP":speed,"rFR":force}

        self._completeAndSaveCommand(command)

    #Read and write: Modbus function code 23 (Read/Write multiple registers)

    def _writeAreadStatus(self, rARD, rATR, rGTO, rACT):
        """Write action parameters in the command register and read the gripper
        status in a single Modbus transaction."""
        
        action = (rARD << 13) | (rATR << 12) | (rGTO << 11) | (rACT<<8)

        result=self._client.readwrite_registers(read_address=2000,
                                        read_count=3,
                                        write_address=1000,
                                        values=[action],
                                        device_id=self.device_id)
        
        # Check result
        if result.isError():
            raise GripperCommunicationError("Write failed")

        registers=result.registers
        t=floor_to_ms(time.monotonic())

        self._saveStatus(t,registers,readWrite=True)
        command={"time":t,"rARD":rARD,"rATR":rATR,"rGTO":rGTO,"rACT":rACT}

        self._completeAndSaveCommand(command)

    def _writePreadStatus(self,position):
        """Write position in the command register and read the gripper\
        status in a single Modbus transaction.
        
        Args:
            position (int):
                The position to move the gripper to in bits. Integer between 0 and 255.
        """
        result=self._client.readwrite_registers(read_address=2000,
                                        read_count=3,
                                        write_address=1001,
                                        values=[position],
                                        device_id=self.device_id)
        registers=result.registers
        t=floor_to_ms(time.monotonic())

        self._saveStatus(t,registers,readWrite=True)
        command={"time":t,"rPR":position}

        self._completeAndSaveCommand(command)

    def _writePSFreadStatus(self,position, speed, force):
        """Write position, speed and force in the command register and read the gripper\
        status in a single Modbus transaction.
        
        Args:
            position (int):
                The position to move the gripper to in bits. Integer between 0 and 255.
            speed (int):
                The speed of the gripper movement. Integer between 0 and 255.
            force (int):
                The force of the gripper movement. Integer between 0 and 255.
        """
        res=self._client.readwrite_registers(read_address=2000,
                                        read_count=3,
                                        write_address=1001,
                                        values=[position, (speed << 8) | force],
                                        device_id=self.device_id)

        # Check result
        if res.isError():
            raise GripperCommunicationError("Write failed")
        
        registers=res.registers

        t=floor_to_ms(time.monotonic())
        self._saveStatus(t,registers,readWrite=True)

        
        command={"time":t,"rPR":position, "rSP":speed,"rFR":force}

        self._completeAndSaveCommand(command)

    def _writeAPSFreadStatus(self,rARD, rATR, rGTO, rACT, position, speed, force):
        """Write Action Position Speed Force in the command register and read the gripper\
        status in a single Modbus transaction.
        
        Args:
            rARD (int): Automatic release status
                0: Closing auto-release
                1: Opening auto-release
            rATR (int): Automatic release type
                0: Normal
                1: Emergency auto-release
            rGTO (int): Go-to action command.
                0: Stop
                1: Go to requested position
            rACT (int): Activation command.
                0: Deactivate gripper
                1: Activate gripper (must stay on until routine completes)
            position (int):
                The position to move the gripper to in bits. Integer between 0 and 255.
            speed (int):
                The speed of the gripper movement. Integer between 0 and 255.
            force (int):
                The force of the gripper movement. Integer between 0 and 255.
        """
        action = (rARD << 13) | (rATR << 12) | (rGTO << 11) | (rACT<<8)

        res=self._client.readwrite_registers(read_address = 2000,
                                             read_count = 3,
                                             write_address = 1000,
                                             values = [action,
                                                       position,
                                                       speed << 8 | force],
                                             device_id = self.device_id)
        # Check result
        if res.isError():
            raise GripperCommunicationError("Write failed")
        
        t = floor_to_ms(time.monotonic())
        registers = res.registers
        self._saveStatus(t,registers,readWrite=True)

        command={"time":t,
                 "rARD":rARD,
                 "rATR":rATR,
                 "rGTO":rGTO,
                 "rACT":rACT,
                 "rPR":position,
                 "rSP":speed,
                 "rFR":force}

        self._completeAndSaveCommand(command)
 
    #DATA SAVING FUNCTIONS
    def _saveStatus(self,t,statusRegisters,readWrite):
        """Save the gripper status register values in status history.

        Args:
            t (float):
                Time of the status is second
            statusRegisters (int) :
                Status registers
            readWrite (boolean) :
                Whether the status was read after a write command or not. This is
                to manage the object detection status while is reset following read
                write command.
        """
        #########################################
        #Register 2000
        #First Byte: gripperStatus
        #Second Byte: RESERVED
        
        #First Byte: gripperStatus
        gripperStatusReg0=(statusRegisters[0] >> 8) & 0b11111111 #xxxxxxxx00000000
        #########################################
        #Object detection

        #if readWrite:
        #    gOBJ=-1
        #else:

        gOBJ=(gripperStatusReg0 >> 6) & 0b11 #xx000000
        if ((gOBJ == GOBJ_DETECTED_WHILE_CLOSING) or (gOBJ == GOBJ_DETECTED_WHILE_OPENING)) and (self._lastObjectDetectionTime==0):
            self._lastObjectDetectionTime = t
        else:
            self._lastObjectDetectionTime = 0
        
        #Gripper status
        gSTA=(gripperStatusReg0 >> 4) & 0b11 #00xx0000
        #Action status. echo of rGTO (go to bit)
        gGTO=(gripperStatusReg0 >> 3) & 0b1 #0000x000
        #Activation status
        gACT=gripperStatusReg0 & 0b00000001 #0000000x
        
        #########################################
        #Register 2001
        #First Byte: Fault status
        #Second Byte: Pos request echo
        
        #First Byte: fault status
        faultStatusReg2= (statusRegisters[1] >>8)  & 0b11111111 #xxxxxxxx00000000
        #########################################
        #Universal controler
        kFLT=(faultStatusReg2 >> 4) & 0b1111 #xxxx0000
        #Fault
        gFLT=faultStatusReg2 & 0b00001111 #0000xxxx

        if gFLT in [7,8,10,11,12,13,14,15]:
            print(f"gFLT = {gFLT}")
            print("Gripper status history:")
            print(self.statusHistory())
            raise GripperFaultError(REGISTER_DIC["gFLT"][gFLT])

        
        #if gFLT in [5,9]:
        #    warnings.warn(REGISTER_DIC["gFLT"][9],UserWarning, stacklevel=2)

        #########################################
        #Second Byte: Pos request echo
        posRequestEchoReg3=statusRegisters[1] & 0b11111111 #00000000xxxxxxxx
        #########################################
        #Echo of request position
        gPR=posRequestEchoReg3
        
        #########################################
        #Register 2002
        #First Byte: Position
        #Second Byte: Current
        
        #First Byte: Position
        positionReg4=(statusRegisters[2] >> 8) & 0b11111111 #xxxxxxxx00000000

        #########################################
        #Actual position of the gripper
        gPO=positionReg4

        if gOBJ == GOBJ_AT_POSITION:
            self._last_at_position_status_position = gPO
        
        #########################################
        #Second Byte: Current
        currentReg5=statusRegisters[2] & 0b0000000011111111 #00000000xxxxxxxx
        #########################################
        #Current
        gCU=currentReg5

        eOBJ = None

        self._statusHistory[:-1,:]=self._statusHistory[1:,:]
        self._statusHistory[-1,:]=[t,gOBJ,gSTA,gGTO,gACT,kFLT,gFLT,gPR,gPO,gCU,eOBJ]

        eOBJ = self._estimateObjectDetection()
        self._statusHistory[-1,EOBJ]=eOBJ

        #Save last move time and direction
        pastPosition = self._statusHistory[-2,GPO]
        currentPosition = self._statusHistory[-1,GPO]

        if pastPosition != currentPosition:
            self._lastMoveTime = t
            self._lastMoveDirection= np.sign(currentPosition-pastPosition)
        
  
    def _completeAndSaveCommand(self,command):
        """Complete a partial command dictionary and save it to history.

        Parameters:
        -----------
        command : dict
            A partial command dictionary to complete and save.
        """
        self._complete_command(command)

        self._commandHistory[:-1,:]=self._commandHistory[1:,:]
        self._commandHistory[-1,:]= [command["time"],
                                    command["rARD"],
                                    command["rATR"],
                                    command["rGTO"],
                                    command["rACT"],
                                    command["rPR"],
                                    command["rSP"],
                                    command["rFR"]]
    
    #DATA PROCESSING FUNCTIONS

    def _mmToBit(self,mm):
        """Convert a mm gripper opening into gripper position paramter bit value

        .. note::
            Calibration is needed to use this function.\n
            Execute the function calibrate at least 1 time before using this function.
        """
        if not self.is_mm_calibrated():
            raise GripperNotCalibratedError()

        bit=(mm-self._bCoef)/self._aCoef

        if bit>255:
            bit=255
        elif bit<0:
            bit=0
        
        return bit
    
    def _bitTomm(self,bit):
        """Convert gripper actual position out parameter in mm opening.

        Returns:
        --------
        mm : float
            Gripper position converted in mm
        
        .. note::
            Calibration is needed to use this function.\n
            Execute the function calibrate at least 1 time before using this function.
        """
        if not self.is_mm_calibrated():
            raise GripperNotCalibratedError()
        mm=self._aCoef*bit+self._bCoef

        if mm>self._openmm:
            mm=self._openmm
        elif mm<self._closemm:
            mm=self._closemm
        
        return mm
    
    def _positionEstimation(self,startPosition,requestedPosition,speed,elapsedTime):
        """
        Estimate what will be the gripper position after an "elapsedTime" knowing the\
        start position of the motion, the requested position and the speed
        
        Parameters:
        -----------
        startPosition : int
            Position in bits from which start the motion.
        requestedPosition : int
            Position in bits where the gripper is requested to move.
        speed : int
            Speed of the gripper in bits.
        elapsedTime : float
            Elapsed time in seconds from the start position to the estimated position

        Returns:
        --------
        estimatedPosition : int
            Estimated position of the gripper.
        """
        if not self.is_speed_calibrated():
            raise GripperCalibrationError("The speed calibration is required to estimate gripper position.")

        #Calculate predicted position
        positionDelta = requestedPosition - startPosition
        
        direction = np.sign(positionDelta)

        motion = int(direction * float(self.gripper_vmin_bits() + (self.gripper_vmax_bits() - self.gripper_vmin_bits()) * speed / 255) * elapsedTime)
        
        estimatedPosition = 0
        
        if abs(positionDelta) < abs(motion):
            #Last position request was reachable within the elapsed time
            estimatedPosition = requestedPosition
        else:
            #Last position was not reachable within elapsed time
            estimatedPosition = startPosition + motion

        return estimatedPosition
    
    def _convert_speedParameter_2_bitPerSecond(self,speed):
        """Return the corresponding position bits/s speed for a speed value in bit.
        
        Parameters:
        -----------
        speed : int
            Gripper speed in bits
        
        Returns:
        --------
        bitPerSecond : float
            Position variation in bits/s
        """
        if not self.is_speed_calibrated():
            raise GripperCalibrationError("Gripper need to be calibrater in order to use use _convert_speedParameter_2_bitPerSecond function")

        bitPerSecond=self.gripper_vmin_bits() + (float(self.gripper_vmax_bits()-self.gripper_vmin_bits())/255)*speed
        if bitPerSecond<0:
            raise GripperValidationError("Negative bit per second value {} for speed {}. Gripper_vmax_bits: {} and Gripper_vmin_bits: {}".format(bitPerSecond,speed,self.gripper_vmax_bits(),self.gripper_vmin_bits()))
        return bitPerSecond
    
    def _convert_bitPerSecond_2_speedParameter(self,speed):
        """Return the corresponding speed value in bit for a speed value in bits/s.
        
        Parameters:
        -----------
        speed : int
            Gripper speed in bits/s
        
        Returns:
        --------
        bitPerSecond : float
            Speed value in bit
        """

        if not self.is_speed_calibrated():
            raise GripperCalibrationError("Gripper need to be calibrater in order to use use _convert_bitPerSecond_2_speedParameter function")


        speedValue = (speed - self.gripper_vmin_bits()) / (float(self.gripper_vmax_bits()-self.gripper_vmin_bits())/255)

        speedValue = int(max(0,min(speedValue,255)))

        return speedValue
    
    def _mergeHistory(self):
        """Merge command and status history arrays based on time.

        Returns:
        --------
        numpy.ndarray
            A merged array containing aligned command and status data.
        """
        #Merge using time property
        history=array_merge_on_first_column(self._commandHistory,self._statusHistory)
        #Filled forward missing command value
        columns_to_fill=[RARD,RGTO,RACT,RPR,RSP,RFR]
        array_forward_fill_columns(history,columns_to_fill,missing_value=-1)
        return history
    
    def _fill_gPO(self,history):
        """
        Fill NaN values of gPO using speed, target, and elapsed time.
        """
        df=history
        time_col="time"
        pos_col="gPO"
        target_col="rPR"
        speed_col="rSP"

        if not self.is_speed_calibrated():
            raise GripperCalibrationError("The gripper need to be speed activated to be able to estimate gripper position.")


        #Sort table with time ascending values
        df = df.sort_values(by=time_col).reset_index(drop=True)
        
        for i in range(1, len(df)):
            if np.isnan(df.loc[i, pos_col]):
                
                prev_gPO = df.loc[i-1, pos_col]
                prev_rPR = df.loc[i-1, target_col]
                speed = df.loc[i-1, speed_col]
                
                # If we still don't have enough info, skip
                if np.isnan(prev_gPO) or np.isnan(prev_rPR) or np.isnan(speed):
                    #print("prev_gPO : ",prev_gPO)
                    #print("prev_rPR : ",prev_rPR)
                    #print("speed : ",speed)
                    #warnings.warn("Not enough information to estimate position")
                    continue
                
                # Time difference
                dt = df.loc[i, time_col] - df.loc[i-1, time_col]
                
                # Direction
                direction = np.sign(prev_rPR - prev_gPO)
                
                # Movement
                delta = direction * self._convert_speedParameter_2_bitPerSecond(speed) * dt
                new_pos = int(prev_gPO + delta)
                
                # Clamp to target (avoid overshoot)
                if direction > 0:
                    new_pos = min(new_pos, prev_rPR)
                elif direction < 0:
                    new_pos = max(new_pos, prev_rPR)
                
                if new_pos > self._closebit:
                    new_pos = self._closebit
                if new_pos < self._openbit:
                    new_pos = self._openbit
                
                df.loc[i, pos_col] = new_pos
        df=df.sort_values(by="time", ascending=False).reset_index(drop=True)
        return df

    def _travelTime(self,startPosition,endPosition,speed):
        """Return the time need to travel from a position to another at a given speed.
        
        Parameters:
        -----------
        startPosition : int
            Start position in bits.
        endPosition : int
            End position in bits.
        speed : int
            Gripper speed in bits.

        Returns:
        --------
        travelTime : float
            Time needed to travel from start to end position.
        """
        posBitPerSecond = self._convert_speedParameter_2_bitPerSecond(speed)
        travelTime = abs(float(endPosition-startPosition))/posBitPerSecond
        return travelTime

    def _complete_command(self,command: dict) -> dict:
        """
        Complete a partial record using the first row of the DataFrame.
        
        Parameters:
            partial_record (dict): A dictionary with some values missing.
            
        Returns:
            dict: A complete record with missing values filled from the first row of df.
        """

        if "time" not in command.keys():
            raise GripperValidationError("Time is required to complete the command record.")

        for key in ["rARD", "rATR", "rGTO", "rACT", "rPR", "rSP", "rFR"]:
            if key not in command.keys():
                arrayID=COMMAND_HISTORY_COLUMNS_NAME_2_ID[key]
                command[key] = self._commandHistory[-1, arrayID]
        for key in COMMAND_HISTORY_COLUMNS_NAME_2_ID.keys():
            if key not in command.keys():
                command[key] = -1

    #TIME FUNCTIONS

    def _waitComplete(self):
        """Wait until the gripper has completed its motion or detect an object.

        This method blocks until the gripper reaches the target position or detects an object.
        """
        startTime = time.time()
        self.readStatus()
        gOBJ=self._statusHistory[-1,GOBJ]
        while gOBJ == GOBJ_IN_MOTION and (time.time() - startTime) < self.timeOut:
            self.readStatus()
            gOBJ=self._statusHistory[-1,GOBJ]
            
    ####################################################
    ### PUBLIC FUCNTIONS
    ###################################################
    # SETUP
    
    def connect(self):
        """Connect to the gripper. If the connection is already established, do nothing.
        """
        if not self._client.connect():
            raise GripperConnectionError("Failed to connect to the gripper. Please check the connection and try again.")
    
    def disconnect(self):
        """Disconnect from the gripper. If the connection is already closed, do nothing.
        """
        self._client.close()
    
    def reset(self):
        """Reset the gripper (clear previous activation and calibration if any)
        """
        #Reset the gripper
        self._client.write_registers(1000,[0,0,0],device_id=self.device_id)

        t=time.monotonic()
        command={"time": t,
                 "rARD": 0,
                 "rATR": 0,
                 "rGTO": 0,
                 "rACT": 0,
                 "rPR": 0,
                 "rSP": 0,
                 "rFR": 0}
                 

        self._completeAndSaveCommand(command)

        self._is_bit_calibrated=False
        self._is_speed_calibrated=False
        self._is_mm_calibrated=False

        self.readStatus()

    def activate(self,reset= True, start=True,refreshStatus=True):
        """Activate the gripper if it is not already active.

        Args:
            reset (bool): Whether to reset the gripper before activation.
            start (bool): Whether to start the gripper motion immediately after
                activation (gGTO set to 1).
            refreshStatus (bool): Whether to refresh the gripper status before the
                activation process. The status information is used to determine if
                the gripper is already activated. Setting this parameter to False
                can make the activation faster if you are sure the gripper status
                is up to date.

        .. warning::
            When you execute this function, the gripper will fully open and close.
            During this operation, the gripper must be able to move freely.
            Do not place any object inside the gripper.
        """
        
        #Turn the variable which indicate that the gripper is processing
        #an action to True
        if reset:
            self.reset()


        t=floor_to_ms(time.monotonic())
        command={"time": t,
                 "rACT": RACT_ACTIVATE}
        if not self.isActivated(refreshStatus=refreshStatus):
            #Activate the gripper
            #rACT=1 Activate Gripper (must stay on after activation routine is
            #completed).
            
            if start:
                res=self._client.write_registers(1000,[0b0000100100000000],device_id=self.device_id)
                command["rGTO"]=RGTO_GO_TO_REQUESTED_POSITION
                if res.count !=1:
                    raise GripperCommunicationError("Failed to write register")
            else:
                res=self._client.write_registers(1000,[0b0000000100000000],device_id=self.device_id)
            
            if res.isError():
                raise GripperCommunicationError("Fail to write gripper register")

            #Waiting for activation to complete
            activationStartTime=floor_to_ms(time.monotonic())
            activationTime=0

            while not self.isActivated(refreshStatus=True) and (activationTime < self.timeOut):
                activationTime = floor_to_ms(time.monotonic()) - activationStartTime
            if activationTime > self.timeOut:
                raise GripperTimeoutError("Activation", self.timeOut)
            
            self.readStatus()
            self._completeAndSaveCommand(command)
        else:
            res = None
            if start:
                res=self._client.write_registers(1000,[0b0000100100000000],device_id=self.device_id)
                command["rGTO"]=RGTO_GO_TO_REQUESTED_POSITION
                self._completeAndSaveCommand(command)
                self.readStatus()
            if res is not None and res.isError():
                raise GripperCommunicationError("Failed to write register")
    
    def start(self,refreshStatus=True,readStatus=True):
        """Start the gripper motion.

        The GTO bit is set to 1. The gripper will move to the position command
        if it is not already there.

        Args:
            refreshStatus (bool): Whether to refresh the gripper status before
                the start process. The status information is used to determine
                if the gripper is already started. Setting this parameter to
                False can make the start process faster if you are sure the
                gripper status is up to date.
            readStatus (bool): Whether to read the gripper status while sending
                the start command.
        """
        rARD=0
        rATR=0
        rGTO=RGTO_GO_TO_REQUESTED_POSITION
        rATC=0
        if not self.isStarted(refreshStatus=refreshStatus):
            if self.isActivated(refreshStatus=refreshStatus):
                rATC=1
                if readStatus:
                    self._writeAreadStatus(rARD=rARD,
                                           rATR=rATR,
                                           rGTO=rGTO,
                                           rACT=rATC
                                           )
                else:
                    self._writeA(rARD=rARD,
                                rATR=rATR,
                                rGTO=rGTO,
                                rACT=rATC
                                )
            else:
                rATC=0
                if readStatus:
                    self._writeAreadStatus(rARD=rARD,
                                           rATR=rATR,
                                           rGTO=rGTO,
                                           rACT=rATC
                                           )
                else:
                    
                    self._writeA(rARD=rARD,
                                rATR=rATR,
                                rGTO=rGTO,
                                rACT=rATC
                                )
    
    def stop(self,refreshStatus=True,readStatus=True):
        """Gripper stop moving. GTO bit is set to 0 but the position command is not
        changed. The gripper will stop at its current position and hold it.

        Args:
            refreshStatus (bool, optional): Whether to refresh the gripper status before
                deciding whether to preserve the activation bit. Defaults to True.
            readStatus (bool, optional): If True, read fresh gripper status in the same
                Modbus transaction as the stop write (function code 23). If False, only
                the stop is written (function code 16). Defaults to True.
        """
        rACT = RACT_ACTIVATE if self.isActivated(refreshStatus=refreshStatus) else RACT_DESACTIVATE
        if readStatus:
            self._writeAreadStatus(rARD=0, rATR=0, rGTO=RGTO_STOP, rACT=rACT)
        else:
            self._writeA(rARD=0, rATR=0, rGTO=RGTO_STOP, rACT=rACT)
    
    def calibrate_bit(self,
                      openbit=None,
                      closebit=None):
        """Calibrate the maximum and minimum position bit values of the gripper.

        This function sets the reference positions for the gripper in bits.
        If no parameters are provided, the gripper will perform a full open
        followed by a full close to measure the maximum and minimum positions.

        Args:
            openbit (int, optional): Position value in bits when the gripper is open.
            closebit (int, optional): Position value in bits when the gripper is closed.

        .. warning::
            If no parameters are provided, the gripper will make a full open
            followed by a full close to measure the maximum and minimum positions
            in bits. Ensure the gripper can move freely during this operation.
        """
        if (openbit is None) or (closebit is None):
             
            #Search for open and close position values
            #and search for min and max speed in bits/s

            self.open(speed=255,force=0,wait=True)
            self._openbit=self.position()

            self.close(speed=255,force=0,wait=True)
            self._closebit=self.position()

        else:
            self._openbit=openbit
            self._closebit=closebit
        
        self._is_bit_calibrated =True

    def calibrate_speed(self, minSpeedClosingTime=None, maxSpeedClosingTime=None):
        """Calibrate gripper speed to estimate gripper position over time.

        If no parameters are provided, the gripper will perform a full-speed
        closing and a slow-speed closing to evaluate its motion. Bit calibration
        will also be performed.

        Args:
            minSpeedClosingTime (float, optional): Time in seconds for the gripper
                to move from fully open to fully closed at minimum speed.
            maxSpeedClosingTime (float, optional): Time in seconds for the gripper
                to move from fully open to fully closed at maximum speed.

        .. warning::
            If no parameters are provided, the gripper will perform a full-speed
            closing and a slow-speed closing to evaluate its motion. Ensure the
            gripper can move freely during this operation.
        """
        if (minSpeedClosingTime is None) or (maxSpeedClosingTime is None):
            self.open(speed=255,force=0,wait=True)
            self._openbit=self.position()

            startTime=floor_to_ms(time.monotonic())
            self.close(speed=255,force=0,wait=True)
            elapsedTime=floor_to_ms(time.monotonic())-startTime
            self._closebit=self.position()
            self._gripper_vmax_bits= (self._closebit - self._openbit)/elapsedTime

            self.open(speed=255,force=0,wait=True)

            startTime=floor_to_ms(time.monotonic())
            self.close(speed=0,force=0,wait=True)
            elapsedTime=floor_to_ms(time.monotonic())-startTime
            self._gripper_vmin_bits= (self._closebit - self._openbit)/elapsedTime

            self._is_bit_calibrated=True
        else:
            if not self.is_bit_calibrated():
                raise GripperCalibrationError("You have to execute calibrate_bit() before calibrating speed with input parameters")
            self._gripper_vmin_bits=(self._closebit - self._openbit)/minSpeedClosingTime
            self._gripper_vmax_bits=(self._closebit - self._openbit)/maxSpeedClosingTime
        self._is_speed_calibrated = True
    
    def calibrate_mm(self,
                     closemm,
                     openmm):
        """Calibrate the gripper for millimeter positioning.

        Once this calibration is done, it is possible to control the gripper
        using millimeters.

        Args:
            closemm (float): Distance between the fingers when the gripper is fully closed.
            openmm (float): Distance between the fingers when the gripper is fully open.

        .. warning::
            Bit calibration is required before executing this function. If the bit
            calibration is not done, the function `calibrate_bit()` will be executed
            automatically before calibrating in millimeters.
        """
        if not self.is_bit_calibrated():
            self.calibrate_bit()

        self._closemm=closemm
        self._openmm=openmm
        
        self._aCoef=(closemm-openmm)/(self._closebit-self._openbit)
        self._bCoef=(openmm*self._closebit-self._openbit*closemm)/(self._closebit-self._openbit)

        self._is_mm_calibrated=True
    
    #ACTIONS
    
    def open(self,speed=255,force=255,wait=True,readStatus=True,refreshStatus=False):
        """Open the gripper.

        Args:
            speed (int, optional): Gripper speed between 0 and 255. Defaults to 255.
            force (int, optional): Gripper force between 0 and 255. Defaults to 255.
            wait (bool, optional): If True, the function waits until the gripper
                reaches the requested position or detects an object. Defaults to True.
            readStatus (bool, optional): If True, the gripper status is read after
                sending the command. Defaults to True.
            refreshStatus (bool, optional): If True, the gripper status is refreshed
                before sending the command. The status information is used to determine
                if the gripper is already activated. Setting this parameter to False
                can make the command process faster if the gripper status is up to date.
                Defaults to False.
        """
        #Check if the gripper is activated
        self.move(0,speed,force,wait=wait,readStatus=readStatus,refreshStatus=refreshStatus)
    
    def close(self,speed=255,force=255,wait=True,readStatus=True,refreshStatus=False, start =False):
        """Close the gripper.

        Args:
            speed (int, optional): Gripper speed between 0 and 255. Defaults to 255.
            force (int, optional): Gripper force between 0 and 255. Defaults to 255.
            wait (bool, optional): If True, the function waits until the gripper
                reaches the requested position or detects an object. Defaults to True.
            readStatus (bool, optional): If True, the gripper status is read after
                sending the command. Defaults to True.
            refreshStatus (bool, optional): If True, the gripper status is refreshed
                before sending the command. The status information is used to determine
                if the gripper is already activated. Setting this parameter to False
                can make the command process faster if the gripper status is up to date.
                Defaults to False.
            start (bool, optional): If True, the activation and go-to bits are combined
                with this move in a single Modbus transaction instead of requiring a
                separate start() call first. The gripper must already be activated
                (activate() called at least once). Defaults to False.
        """
        self.move(255,speed,force,wait=wait,readStatus=readStatus,refreshStatus=refreshStatus,start=start)
    
    def move(self,position,speed=255,force=255,wait=True,readStatus=True,refreshStatus=False,start=False):
        """Move gripper fingers to the requested position with specified speed and force.

        Args:
            position (int): Target position of the gripper. Must be between 0 and 255,
                where 0 represents fully open and 255 represents fully closed.
            speed (int, optional): Gripper speed between 0 and 255. Defaults to 255.
            force (int, optional): Gripper force between 0 and 255. Defaults to 255.
            wait (bool, optional): If True, the function waits until the gripper
                reaches the requested position or detects an object. Defaults to True.
            readStatus (bool, optional): If True, the gripper status is read after
                sending the command. Defaults to True.
            refreshStatus (bool, optional): If True, the gripper status is refreshed
                before sending the command. The status information is used to determine
                if the gripper is already activated. Setting this parameter to False
                can make the command process faster if the gripper status is up to date.
                Defaults to False.
            start (bool, optional): If True, the activation and go-to bits are combined
                with this move in a single Modbus transaction instead of requiring a
                separate start() call first. The gripper must already be activated
                (activate() called at least once). Defaults to False.

        Examples:
            Combine start() and move() in a single Modbus transaction:

            >>> gripper.activate()
            >>> gripper.move(200, speed=150, force=100, start=True)
        """
        if refreshStatus:
            self.readStatus()

        speed_value=0
        if speed is None:
            if self.speed() is not None:
                speed_value = self.speed()
            else:
                speed_value = 0
        else:
            speed_value = speed

        force_value=0
        if force is None:
            if self.force() is not None:
                force_value = self.force()
            else:
                force_value = 0
        else:
            force_value = force

        #Check if the gripper is activated
        if not start:
            if not self.isActivated(refreshStatus=False):
                raise GripperNotActivatedError()
            if not self.isStarted(refreshStatus=False):
                raise GripperNotStartedError()
        elif not self.isActivated(refreshStatus=False):
            raise GripperNotActivatedError()
        
        #Check input value
        if position>255 or position<0:
            raise GripperPositionError(position)

        position=int(position)

        if start:
            speed_value=int(speed_value)
            force_value=int(force_value)
            if readStatus:
                self._writeAPSFreadStatus(rARD=0,
                                          rATR=0,
                                          rGTO=RGTO_GO_TO_REQUESTED_POSITION,
                                          rACT=RACT_ACTIVATE,
                                          position=position,
                                          speed=speed_value,
                                          force=force_value)
            else:
                self._writeAPSF(rARD=0,
                                rATR=0,
                                rGTO=RGTO_GO_TO_REQUESTED_POSITION,
                                rACT=RACT_ACTIVATE,
                                position=position,
                                speed=speed_value,
                                force=force_value)
        elif (speed is None) and (force is None):
            if readStatus:
                self._writePreadStatus(position)
            else:
                self._writeP(position)

        else:
            speed_value=int(speed_value)
            force_value=int(force_value)
            if readStatus:
                self._writePSFreadStatus(position,speed_value,force_value)
            else:
                self._writePSF(position,speed_value,force_value)

        if wait:
            self._waitComplete()
    
    def move_mm(self,positionmm,speed=255,force=255,wait=True,readStatus=True,refreshStatus=False):
        """Move the gripper to the requested opening in millimeters.

        Args:
            positionmm (float): Target gripper opening in millimeters.
            speed (int, optional): Gripper speed between 0 and 255. Defaults to 255.
            force (int, optional): Gripper force between 0 and 255. Defaults to 255.
            wait (bool, optional): If True, the function waits until the gripper
                reaches the requested position or detects an object. Defaults to True.
            readStatus (bool, optional): If True, the gripper status is read after
                sending the command. Defaults to True.
            refreshStatus (bool, optional): If True, the gripper status is refreshed
                before sending the command. The status information is used to determine
                if the gripper is already activated. Setting this parameter to False
                can make the command process faster if the gripper status is up to date.
                Defaults to False.

        .. note::
            Millimeter calibration is required to use this function.
            Execute the function `calibrate_mm()` before using this function.
        """

        if not self.is_mm_calibrated():
            raise GripperNotCalibratedError()

        if  positionmm>self._openmm:
            raise GripperValidationError("The maximum opening is {} mm but the given value is {} mm".format(self._openmm,positionmm))
        if positionmm<self._closemm:
            raise GripperValidationError("The minimum closing is {} mm but the given value is {} mm".format(self._openmm,positionmm))
        
        position=int(self._mmToBit(positionmm))
        self.move(position,speed,force,wait,readStatus,refreshStatus)

    
    def _realTimePositionMove_freeMotion(self,
                                         controlSignal,
                                         controlBuffer=0.05,
                                         speedLowerControlThreshold=10,
                                         speedUpperControlThreshold=30):
        """
        Args:
            controlSignal (float): Analogic control signal in range [0,1]
            controlBuffer (float): Dimension of the upper and lower range of the
                control signal. Express in percentage of total control range.
            speedLowerControlThreshold (int): Position delta (in bits) below which
                the commanded speed is 0.
            speedUpperControlThreshold (int): Position delta (in bits) above which
                the commanded speed is saturated at 255.
        """
        positionCommand = None
        speedCommand = None
        forceCommand = None

        controlBuffer = max(0,min(1,controlBuffer))

        #Position command
        #################
        if controlSignal < controlBuffer:
            positionCommand = 0
        elif controlSignal <= (1 - controlBuffer):
            positionControlFunction = get_line_function(controlBuffer,0,1-controlBuffer,255)
            positionCommand = int(max(0,min(255,positionControlFunction(controlSignal))))
        else:
            positionCommand = 255

        #Speed command
        ###############
        positionDelta = None
        #positionDelta = abs(self._last_at_position_status_position - positionCommand)
        positionDelta = abs(self.position(refreshStatus=False) - positionCommand)
        
        if controlSignal < controlBuffer:
            speedCommand = 255
        elif controlSignal <= (1 - controlBuffer):
            speedControlFunction = make_curve_function(speedLowerControlThreshold,speedUpperControlThreshold)
            speedCommand = int(max(0,min(255,speedControlFunction(positionDelta))))
        else:
            speedCommand = 255

        #Force command
        ##############
        forceCommand = speedCommand
        
        #Motion
        #######
        if self.objectDetection(refreshStatus=False) == GOBJ_DETECTED_WHILE_CLOSING:
            # If previous object status indicate that an object was detected.
            # The move is done with high force and speed to ease the release of
            # the gripper.
            positionCommand = max(0,self.position(refreshStatus=False)-15)
            speedCommand=255
            forceCommand=255
            self.move(positionCommand,speedCommand,forceCommand,wait=True)
        elif self.objectDetection(refreshStatus=False) == GOBJ_DETECTED_WHILE_OPENING:
            positionCommand = min(255,self.position(refreshStatus=False)+15)
            speedCommand=255
            forceCommand=255
            self.move(positionCommand,speedCommand,forceCommand,wait=True)
        
        else:
            self.move(positionCommand,speedCommand,forceCommand,wait=False)

        return positionCommand, speedCommand, forceCommand

    def _realtimePositionMove_forceMode(self,controlSignal,controlBuffer=0.1):
        """
        """
        positionCommand = None
        speedCommand = None
        forceCommand = None

        # Position command

        if self.objectDetection(refreshStatus=False) == GOBJ_DETECTED_WHILE_CLOSING:
            positionCommand = 255
        else:
            positionCommand = 0
        
        # Speed command
        if self.objectDetection(refreshStatus=False) == GOBJ_DETECTED_WHILE_CLOSING:
            if controlSignal < controlBuffer:
                speedCommand = 0
            else:
                speedControlFunction=get_line_function(controlBuffer,0,1,255)
                speedCommand = speedControlFunction(controlSignal)
        else:
            if controlSignal > (1-controlBuffer):
                speedCommand = 0
            else:
                speedControlFunction = get_line_function(0,255,1-controlBuffer,0)
                speedCommand = speedControlFunction(controlSignal)

        
        speedCommand = int(max(0,min(255,speedCommand)))

        
        if speedCommand <= self._realtimePositionMove_NudgeBaseline:
            speedCommand = None
        else:
            self._realtimePositionMove_NudgeBaseline = speedCommand
        
        forceCommand = speedCommand

        # Motion
        if speedCommand is not None:
            self.stop(refreshStatus=False)
            self.move(positionCommand,speedCommand,forceCommand,start=True)
        else:
            self.readStatus()
        
        return positionCommand, speedCommand, forceCommand
    
    def _realtimeSpeedMove_forceMode(self,controlSignal,controlBuffer=0.1):
        """
        """
        positionCommand = None
        speedCommand = None
        forceCommand = None

        # Position command
        if self.objectDetection(refreshStatus=False) == GOBJ_DETECTED_WHILE_CLOSING:
            positionCommand = 255
        else:
            positionCommand = 0
        
        # Speed command
        if abs(controlSignal) < 2*controlBuffer:
            speedCommand = 0
        else:
            speedControlFunction=get_line_function(2*controlBuffer,0,1,255)
            speedCommand = speedControlFunction(controlSignal)

        speedCommand = int(max(0,min(255,speedCommand)))

        if speedCommand <= self._realtimeSpeedMove_NudgeBaseline:
            speedCommand = None
        else:
            self._realtimeSpeedMove_NudgeBaseline = speedCommand
        
        forceCommand = speedCommand

        # Motion
        if speedCommand is not None:
            self.stop(refreshStatus=False)
            self.move(positionCommand,speedCommand,forceCommand,wait=True,start=True)
        else:
            self.readStatus()

        return positionCommand, speedCommand, forceCommand
    
    def realTimePositionMove_Mode(self):
        """Return the mode of the realTimePositionMove function
        
        Return:
            control_mode (int): Control mode code. See constants for details.
                REALTIME_POSITION_MOVE_MODE_FREEMOVE = 0
                REALTIME_POSITION_MOVE_MODE_OBJECT_DETECTED_CLOSING = 100
                REALTIME_POSITION_MOVE_MODE_FORCE_DEACTIVATED_CLOSING = 101
                REALTIME_POSITION_MOVE_MODE_FORCE_ACTIVATED_CLOSING = 102
                REALTIME_POSITION_MOVE_MODE_OBJECT_DETECTED_OPENING = 200
                REALTIME_POSITION_MOVE_MODE_FORCE_DEACTIVATED_OPENING = 201
                REALTIME_POSITION_MOVE_MODE_FORCE_ACTIVATED_OPENING = 202
        
        """

        return self._realtimePositionMove_Mode

    def realTimeSpeedMove_Mode(self):
        """Return the mode of the realTimeSpeedMove function"""

        return self._realtimeSpeedMove_Mode

    def _realTimePositionMove_secureGrip(self,controlSignal,controlBuffer,speed=255,force=255):
        """Secure the grip on the object after object detection.
        """
        positionCommand= None
        speedCommand = None
        forceCommand = None

        commandFunction = make_ramp_function(controlBuffer,1-controlBuffer,0,255)
        command = commandFunction(controlSignal)
        command = int(max(0,min(255,command)))

        requestFurtherClosing = (self.objectDetection(refreshStatus=False)==GOBJ_DETECTED_WHILE_CLOSING) and (command > self.position(refreshStatus=False))
        requestFurtherOpening = (self.objectDetection(refreshStatus=False)==GOBJ_DETECTED_WHILE_OPENING) and (command < self.position(refreshStatus=False))

        if requestFurtherClosing:
            positionCommand= 255
            speedCommand = speed
            forceCommand = force
            isSecured = (self.positionCommand() == positionCommand) and (self.speed()==speedCommand) and (self.force()==forceCommand)
            if not isSecured:
                self.stop()
                self.move(positionCommand,speedCommand,forceCommand,wait=True,start=True)
        elif requestFurtherOpening:
            positionCommand= 0
            speedCommand = speed
            forceCommand = force
            isSecured = (self.positionCommand() == positionCommand) and (self.speed()==speedCommand) and (self.force()==forceCommand)
            if not isSecured:
                self.stop()
                self.move(positionCommand,speedCommand,forceCommand,wait=True,start=True)
        else:
            positionCommand=None
            forceCommand=None
            speedCommand=None
        
        return positionCommand,forceCommand,speedCommand

    def realTimePositionMove(self,
                             controlSignal,
                             controlBuffer=0.05,
                             speedLowerControlThreshold=10,
                             speedUpperControlThreshold=30,
                             gripSpeed=None,
                             gripForce=None,
                             verbose=0):
        """Move the gripper in real time to the requested position.

        Meant to be called at high frequency (e.g. 100 Hz) in a non-blocking
        control loop for remote operation application.

        Args:
            controlSignal: Analogic position control signal in range [0,1]
            controlBuffer: Dimension of the signal deadzones express in percentage of the
                the control signal. Deadzones are positionned at the bottom and
                the top of the control range.
            speedLowerControlThreshold: The speed is function of the difference
                between current position and target position. The function is ramp
                that start at speedLowerControlThreshold and finish at
                speedUpperControlThreshold.
            speedUpperControlThreshold: The speed is function of the difference
                between current position and target position. The function is ramp
                that start at speedLowerControlThreshold and finish at
                speedUpperControlThreshold.
            gripSpeed: Speed parameter use to secure a grip when an object is
                detected. If used, gripForce and gripSpeed have to be both set.
            gripForce: Force parameter use to secure a grip when an object is
                detected. If used, gripForce and gripSpeed have to be both set.
            verbose (int, optional): Verbose level for debug.

        Examples:
            Make a loop to control the gripper using a joystick with pygame

            >>> import pygame
            >>> import time
            >>> import pyrobotiqgripper as rq
            >>> grip = rq.RobotiqGripper()
            >>> pygame.init()
            >>> pygame.joystick.init()
            >>> js = pygame.joystick.Joystick(0)
            >>> js.init()
            >>> while True:
            >>>     positionSignal = js.get_axis(0)
            >>>     pygame.event.pump()
            >>>     grip.realTimePositionMove(positionSignal)
        """
        controlledGripForce=False
        if (gripForce is None) and (gripSpeed is None):
            controlledGripForce=True
        elif (gripForce is not None) and (gripSpeed is not None):
            gripSpeed = int(max(0,min(255,gripSpeed)))
            gripForce = int(max(0,min(255,gripForce)))
            controlledGripForce=False
        else:
            raise RobotiqGripperError("gripSpeed and gripForce must be both None OR both with a value in the range ")


        #Control signal conditionning
        controlSignal=max(0,min(controlSignal,1))

        # Command
        ##########

        ## Position Command
        positionCommand = None
        
        #Speed command
        speedCommand = None

        # Force command
        forceCommand = None


        #####################
        # Mode shifting and shared variable updated
        #####################

        # 0
        ###
        if self._realtimePositionMove_Mode == REALTIME_POSITION_MOVE_MODE_FREEMOVE:
            if controlledGripForce:
                if self.objectDetection(refreshStatus=False) == GOBJ_DETECTED_WHILE_CLOSING:
                    #Refresh because the object detection status from the last command may be outdated
                    #It is possible that last command was a release request from a grip.
                    #In such case gOBJ will still indicate that an object is detected
                    #while the new status is that there is no object.
                    if self.objectDetection(refreshStatus=True) == GOBJ_DETECTED_WHILE_CLOSING:
                        self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_OBJECT_DETECTED_CLOSING
                    else:
                        self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_FREEMOVE
                elif self.objectDetection(refreshStatus=False) == GOBJ_DETECTED_WHILE_OPENING:
                    if self.objectDetection(refreshStatus=True) == GOBJ_DETECTED_WHILE_OPENING:
                        self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_OBJECT_DETECTED_OPENING
                    else:
                        self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_FREEMOVE
                else:
                    self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_FREEMOVE
            else:
                if self.objectDetection(refreshStatus=False) in [GOBJ_DETECTED_WHILE_CLOSING,GOBJ_DETECTED_WHILE_OPENING]:
                    if self.objectDetection(refreshStatus=True) in [GOBJ_DETECTED_WHILE_CLOSING,GOBJ_DETECTED_WHILE_OPENING]:
                        self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_SECURE
                    else:
                        self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_FREEMOVE
                else:
                    self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_FREEMOVE
        ###########################
        ###########################
        
        # 100
        #####
        elif self._realtimePositionMove_Mode == REALTIME_POSITION_MOVE_MODE_OBJECT_DETECTED_CLOSING:
            if controlSignal < controlBuffer:
                self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_FORCE_DEACTIVATED_CLOSING
            else:
                self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_OBJECT_DETECTED_CLOSING
        
        # 101
        #####
        elif self._realtimePositionMove_Mode == REALTIME_POSITION_MOVE_MODE_FORCE_DEACTIVATED_CLOSING:
            if controlSignal > controlBuffer:
                self._realtimePositionMove_force_mode_start_force = self.force()
                self._realtimePositionMove_NudgeBaseline = self.force()
                self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_FORCE_ACTIVATED_CLOSING
            else:
                self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_FORCE_DEACTIVATED_CLOSING
        
        # 102
        #####
        elif self._realtimePositionMove_Mode == REALTIME_POSITION_MOVE_MODE_FORCE_ACTIVATED_CLOSING:
            if self.objectDetection(refreshStatus=False) == GOBJ_DETECTED_WHILE_CLOSING:
                if controlSignal < controlBuffer:
                    #Compare the value of the force when force mode have been activated and the current force value of the gripper
                    if self.force() > self._realtimePositionMove_force_mode_start_force:
                        self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_FORCE_DEACTIVATED_CLOSING
                    else:
                        self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_FREEMOVE
                else:
                    self._realtimePositionMove_NudgeBaseline = self.force()
                    self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_FORCE_ACTIVATED_CLOSING
            else:
                self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_FREEMOVE
        
        ###########################
        ###########################

        # 200
        #####
        elif self._realtimePositionMove_Mode == REALTIME_POSITION_MOVE_MODE_OBJECT_DETECTED_OPENING:
            if controlSignal > (1-controlBuffer):
                self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_FORCE_DEACTIVATED_OPENING
            else:
                self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_OBJECT_DETECTED_OPENING
        
        # 201
        #####
        elif self._realtimePositionMove_Mode == REALTIME_POSITION_MOVE_MODE_FORCE_DEACTIVATED_OPENING:
            if controlSignal < (1-controlBuffer):
                self._realtimePositionMove_force_mode_start_force = self.force()
                self._realtimePositionMove_NudgeBaseline = self.force()
                self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_FORCE_ACTIVATED_OPENING
            else:
                self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_FORCE_DEACTIVATED_OPENING
        
        # 202
        #####
        elif self._realtimePositionMove_Mode == REALTIME_POSITION_MOVE_MODE_FORCE_ACTIVATED_OPENING:
            if self.objectDetection(refreshStatus=False) == GOBJ_DETECTED_WHILE_OPENING:
                if controlSignal > (1-controlBuffer):
                    #Compare the value of the force when force mode have been activated and the current force value of the gripper
                    if self.force() > self._realtimePositionMove_force_mode_start_force:
                        self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_FORCE_DEACTIVATED_OPENING
                    else:
                        self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_FREEMOVE
                else:
                    self._realtimePositionMove_NudgeBaseline = self.force()
                    self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_FORCE_ACTIVATED_OPENING
            else:
                self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_FREEMOVE
        
        # 300
        #####
        elif self._realtimePositionMove_Mode == REALTIME_POSITION_MOVE_MODE_SECURE:
            if self.objectDetection(refreshStatus=False) in [GOBJ_DETECTED_WHILE_CLOSING,GOBJ_DETECTED_WHILE_OPENING]:
                commandFunction = make_ramp_function(controlBuffer,1-controlBuffer,0,255)
                command = int(max(0,min(255,commandFunction(controlSignal))))
                furtherGripIn = (self.objectDetection(refreshStatus=False) == GOBJ_DETECTED_WHILE_CLOSING) and (command > self.position())
                furtherGripOut = (self.objectDetection(refreshStatus=False) == GOBJ_DETECTED_WHILE_OPENING) and (command < self.position())
                if furtherGripIn or furtherGripOut:
                    self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_SECURE
                else:
                    self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_FREEMOVE
            else:
                self._realtimePositionMove_Mode = REALTIME_POSITION_MOVE_MODE_FREEMOVE
        else:
            raise RobotiqGripperError("realtimepositionmove mode not supported")

        
        #############
        # Motion logic
        #############

        # 0
        ###
        if self._realtimePositionMove_Mode == REALTIME_POSITION_MOVE_MODE_FREEMOVE:
            #Reset speed and force base lines whihc are use in force control mode
            positionCommand, speedCommand, forceCommand = self._realTimePositionMove_freeMotion(controlSignal=controlSignal,
                                                                                                controlBuffer=controlBuffer,
                                                                                                speedLowerControlThreshold=speedLowerControlThreshold,
                                                                                                speedUpperControlThreshold=speedUpperControlThreshold)

        ###########################
        ###########################
        
        # 100
        #####
        elif self._realtimePositionMove_Mode == REALTIME_POSITION_MOVE_MODE_OBJECT_DETECTED_CLOSING:
            self.readStatus() #Not needed but worst doing a status refresh
        # 101
        #####
        elif self._realtimePositionMove_Mode == REALTIME_POSITION_MOVE_MODE_FORCE_DEACTIVATED_CLOSING:
            self.readStatus() #Not needed but worst doing a status refresh
        # 102
        #####
        elif self._realtimePositionMove_Mode == REALTIME_POSITION_MOVE_MODE_FORCE_ACTIVATED_CLOSING:
            positionCommand, speedCommand, forceCommand = self._realtimePositionMove_forceMode(controlSignal=controlSignal,controlBuffer=2*controlBuffer)
        
        ###########################
        ###########################

        # 200
        #####
        elif self._realtimePositionMove_Mode == REALTIME_POSITION_MOVE_MODE_OBJECT_DETECTED_OPENING:
            self.readStatus() #Not needed but worst doing a status refresh
        # 201
        #####
        elif self._realtimePositionMove_Mode == REALTIME_POSITION_MOVE_MODE_FORCE_DEACTIVATED_OPENING:
            self.readStatus() #Not needed but worst doing a status refresh
        # 202
        #####
        elif self._realtimePositionMove_Mode == REALTIME_POSITION_MOVE_MODE_FORCE_ACTIVATED_OPENING:
            positionCommand, speedCommand, forceCommand = self._realtimePositionMove_forceMode(controlSignal=controlSignal,controlBuffer=2*controlBuffer)
        
        # 300
        #####
        
        elif self._realtimePositionMove_Mode == REALTIME_POSITION_MOVE_MODE_SECURE:
            positionCommand, speedCommand, forceCommand = self._realTimePositionMove_secureGrip(controlSignal=controlSignal,controlBuffer=controlBuffer,speed=gripSpeed,force=gripForce)
        
        else:
            raise RobotiqGripperError("Realtime position move mode unknown: ",self._realtimePositionMove_Mode)

        if verbose==1:
            if None not in [positionCommand,speedCommand,forceCommand]:
                deltaTime = self._statusHistory[-1,TIME] - self._statusHistory[-2,TIME]
                frequency = None
                if deltaTime > 0 :
                    frequency = int(1/deltaTime)
                print("Frequency : ", frequency,
                    " Time : ",f"{self._statusHistory[-1,TIME]:.3f}",
                    " Signal : ",f"{controlSignal:.2f}",
                    " mode : ",self._realtimePositionMove_Mode,
                    " gPO : ", self.position(False),
                    " gOBJ : ",self.objectDetection(False),
                    " P: ",positionCommand,
                    " F: ",forceCommand,
                    " S: ",speedCommand)

    def moveToCurrentPosition(self, speed = None, force = None):
        """Move the gripper to its current position. This has the effect to stop
        the gripper while keeping it started.
        It is more flexible than jsut stopping the gripper.
        
        args:
            speed (int):
                Gripper speed parameter value to move to current position
            force (int):
                Gripper force parameter value to move to current position
        """
        #Stop the gripper
        self.stop(readStatus=False)

        #Position command
        #The position command is set to fresh gripper position value
        positionCommand = self.position(refreshStatus=True)
        
        #Speed command
        speedCommand = 0
        if speed is None:
            speedCommand = self.speed()
        else:
            speedCommand = speed
        
        forceCommand = 0
        if force is None:
            forceCommand = self.force()
        else:
            forceCommand = force

        self.move(positionCommand,
                  speedCommand,
                  forceCommand,
                  start=True,
                  wait=False)
        
        if not self.isStarted():
            print("speed: ",speed)
            print("force: ",force)
            raise GripperNotStartedError()
    
    def _realTimeSpeedMove_secureGrip(self,controlSignal,controlBuffer,speed=255,force=255):
        """Secure the grip on the object after object detection.
        """
        positionCommand= None
        speedCommand = None
        forceCommand = None

        requestFurtherClosing = (self.objectDetection(refreshStatus=False) == GOBJ_DETECTED_WHILE_CLOSING) and (controlSignal>controlBuffer)
        requestFurtherOpening = (self.objectDetection(refreshStatus=False) == GOBJ_DETECTED_WHILE_OPENING) and (controlSignal<-controlBuffer)

        if requestFurtherClosing:
            positionCommand= 255
            speedCommand = speed
            forceCommand = force
            isSecured = (self.positionCommand() == positionCommand) and (self.speed()==speedCommand) and (self.force()==forceCommand)
            if not isSecured:
                self.stop()
                self.move(positionCommand,speedCommand,forceCommand,wait=True,start=True)
        elif requestFurtherOpening:
            positionCommand= 0
            speedCommand = speed
            forceCommand = force
            isSecured = (self.positionCommand() == positionCommand) and (self.speed()==speedCommand) and (self.force()==forceCommand)
            if not isSecured:
                self.stop()
                self.move(positionCommand,speedCommand,forceCommand,wait=True,start=True)
        else:
            positionCommand=None
            forceCommand=None
            speedCommand=None
        
        return positionCommand,forceCommand,speedCommand

    def realTimeSpeedMove(self,
                     controlSignal,
                     controlBuffer=0.05,
                     gripSpeed=None,
                     gripForce=None,
                     verbose=0):
        """Move the gripper with a speed signal.

        Meant to be called at high frequency (e.g. 100 Hz) in a non-blocking
        control loop for remote operation application.

        Args:
            controlSignal (float): Analogic signal in the range [-1, 1] use to
                control gripper speed. Positive closes the gripper, negative
                opens it, 0 holds the current position. Magnitude controls the
                commanded speed.
            controlBuffer (float): controlSignal deadzone express in percentage
                of the control range (Joystick position range between -1 and 1).
                The deadzone is positionned at the center of the control range.
            gripSpeed: Speed parameter use to secure a grip when an object is
                detected. If used, gripForce and gripSpeed have to be both set.
            gripForce: Force parameter use to secure a grip when an object is
                detected. If used, gripForce and gripSpeed have to be both set.
            verbose (int, optional): Verbose level for debug.

        Examples:
            Make a loop to control the gripper using a joystick with pygame

            >>> import pygame
            >>> import time
            >>> import pyrobotiqgripper as rq
            >>> grip = rq.RobotiqGripper()
            >>> pygame.init()
            >>> pygame.joystick.init()
            >>> js = pygame.joystick.Joystick(0)
            >>> js.init()
            >>> while True:
            >>>     speedSignal = js.get_axis(0)
            >>>     pygame.event.pump()
            >>>     grip.realTimeSpeedMove(speedSignal)
        """
        controlledGripForce=False
        if (gripForce is None) and (gripSpeed is None):
            controlledGripForce=True
        elif (gripForce is not None) and (gripSpeed is not None):
            gripSpeed = int(max(0,min(255,gripSpeed)))
            gripForce = int(max(0,min(255,gripForce)))
            controlledGripForce=False
        else:
            raise RobotiqGripperError("gripSpeed and gripForce must be both None OR both with a value in the range ")
        
        positionCommand=None
        speedCommand=None
        forceCommand=None

        ###########################################
        # Mode shifting and shared variable updated
        ###########################################

        # 0
        ###
        if self._realtimeSpeedMove_Mode == REALTIME_SPEED_MOVE_MODE_FREEMOVE:
            if controlledGripForce:
                if self.objectDetection(refreshStatus=False) in [GOBJ_DETECTED_WHILE_CLOSING,GOBJ_DETECTED_WHILE_OPENING]:
                    if self.objectDetection(refreshStatus=True) in [GOBJ_DETECTED_WHILE_CLOSING,GOBJ_DETECTED_WHILE_OPENING]:
                        self._realtimeSpeedMove_Mode = REALTIME_SPEED_MOVE_MODE_OBJECT_DETECTED
                    else:
                        self._realtimeSpeedMove_Mode = REALTIME_SPEED_MOVE_MODE_FREEMOVE
                else:
                    self._realtimeSpeedMove_Mode = REALTIME_SPEED_MOVE_MODE_FREEMOVE
            else:
                if self.objectDetection(refreshStatus=False) in [GOBJ_DETECTED_WHILE_CLOSING,GOBJ_DETECTED_WHILE_OPENING]:
                    if self.objectDetection(refreshStatus=True) in [GOBJ_DETECTED_WHILE_CLOSING,GOBJ_DETECTED_WHILE_OPENING]:
                        self._realtimeSpeedMove_Mode = REALTIME_SPEED_MOVE_MODE_SECURE
                    else:
                        self._realtimeSpeedMove_Mode = REALTIME_SPEED_MOVE_MODE_FREEMOVE
                else:
                    self._realtimeSpeedMove_Mode = REALTIME_SPEED_MOVE_MODE_FREEMOVE
        # 100
        #####
        elif self._realtimeSpeedMove_Mode == REALTIME_SPEED_MOVE_MODE_OBJECT_DETECTED:
            if abs(controlSignal) < controlBuffer:
                self._realtimeSpeedMove_NudgeBaseline = self.force()
                self._realtimeSpeedMove_Mode = REALTIME_SPEED_MOVE_MODE_FORCE_DEACTIVATED
            else:
                self._realtimeSpeedMove_Mode = REALTIME_SPEED_MOVE_MODE_OBJECT_DETECTED
        
        # 101
        #####
        elif self._realtimeSpeedMove_Mode == REALTIME_SPEED_MOVE_MODE_FORCE_DEACTIVATED:
            if controlSignal > controlBuffer:
                self._realtimeSpeedMove_force_mode_start_force = self.force()
                self._realtimeSpeedMove_NudgeBaseline = self.force()
                self._realtimeSpeedMove_Mode = REALTIME_SPEED_MOVE_MODE_FORCE_ACTIVATED
            else:
                self._realtimeSpeedMove_Mode = REALTIME_SPEED_MOVE_MODE_FORCE_DEACTIVATED
        
        # 102
        #####
        elif self._realtimeSpeedMove_Mode == REALTIME_SPEED_MOVE_MODE_FORCE_ACTIVATED:
            if self.objectDetection(refreshStatus=False) in [GOBJ_DETECTED_WHILE_CLOSING,GOBJ_DETECTED_WHILE_OPENING]:
                if abs(controlSignal) < controlBuffer:
                    #Compare the value of the force when force mode have been activated and the current force value of the gripper
                    if self.force() > self._realtimeSpeedMove_force_mode_start_force:
                        self._realtimeSpeedMove_Mode = REALTIME_SPEED_MOVE_MODE_FORCE_DEACTIVATED
                    else:
                        self._realtimeSpeedMove_Mode = REALTIME_SPEED_MOVE_MODE_FREEMOVE
                else:
                    self._realtimeSpeedMove_NudgeBaseline = self.force()
                    self._realtimeSpeedMove_Mode = REALTIME_SPEED_MOVE_MODE_FORCE_ACTIVATED
            else:
                #Clear object detection
                self.stop(refreshStatus=False)
                self.moveToCurrentPosition(0,0)
                self._realtimeSpeedMove_Mode = REALTIME_SPEED_MOVE_MODE_FREEMOVE
        # 300
        #####
        elif self._realtimeSpeedMove_Mode == REALTIME_SPEED_MOVE_MODE_SECURE:
            if self.objectDetection(refreshStatus=False) in [GOBJ_DETECTED_WHILE_CLOSING,GOBJ_DETECTED_WHILE_OPENING]:
                furtherGripIn = (self.objectDetection(refreshStatus=False) == GOBJ_DETECTED_WHILE_CLOSING) and (controlSignal>controlBuffer*2)
                furtherGripOut = (self.objectDetection(refreshStatus=False) == GOBJ_DETECTED_WHILE_OPENING) and (controlSignal<-controlBuffer*2)
                if furtherGripIn or furtherGripOut:
                    self._realtimeSpeedMove_Mode = REALTIME_SPEED_MOVE_MODE_SECURE
                else:
                    self._realtimeSpeedMove_Mode = REALTIME_SPEED_MOVE_MODE_FREEMOVE
            else:
                self._realtimeSpeedMove_Mode = REALTIME_SPEED_MOVE_MODE_FREEMOVE
        else:
            raise RobotiqGripperError("realtimepositionmove mode not supported")

        ##############
        # Motion logic
        ##############

        # 0
        ###
        if self._realtimeSpeedMove_Mode == REALTIME_SPEED_MOVE_MODE_FREEMOVE:
            #Reset speed and force base lines whihc are use in force control mode
            positionCommand, speedCommand, forceCommand = self._realTimeSpeedMove_freeMotion(controlSignal=controlSignal,controlBuffer=controlBuffer)
        
        ###########################
        ###########################
        
        # 100
        #####
        elif self._realtimeSpeedMove_Mode == REALTIME_SPEED_MOVE_MODE_OBJECT_DETECTED:
            self.readStatus() #Not needed but worst doing a status refresh
        
        # 101
        #####
        elif self._realtimeSpeedMove_Mode == REALTIME_SPEED_MOVE_MODE_FORCE_DEACTIVATED:
            self.readStatus() #Not needed but worst doing a status refresh
        
        # 102
        #####
        elif self._realtimeSpeedMove_Mode == REALTIME_SPEED_MOVE_MODE_FORCE_ACTIVATED:
            positionCommand, speedCommand, forceCommand = self._realtimeSpeedMove_forceMode(controlSignal=max(0,min(1,controlSignal)),controlBuffer=controlBuffer)
    
        # 300
        #####
        elif self._realtimeSpeedMove_Mode == REALTIME_SPEED_MOVE_MODE_SECURE:
            positionCommand, speedCommand, forceCommand = self._realTimeSpeedMove_secureGrip(controlSignal=controlSignal,controlBuffer=controlBuffer,speed=gripSpeed,force=gripForce)
        
        else:
            raise RobotiqGripperError("Realtime position move mode unknown: ",self._realtimeSpeedMove_Mode)

        if verbose==1:
            deltaTime = self._statusHistory[-1,TIME] - self._statusHistory[-2,TIME]
            frequency = None
            if deltaTime >0:
                frequency = int(1/(self._statusHistory[-1,TIME] - self._statusHistory[-2,TIME]))
            print("Frequency : ", frequency,
                " Time : ",f"{self._statusHistory[-1,TIME]:.3f}",
                " Signal : ",f"{controlSignal:.2f}",
                " mode : ",self._realtimeSpeedMove_Mode,
                " gPO : ", self.position(False),
                " gOBJ : ",self.objectDetection(False),
                " P: ",positionCommand,
                " F: ",forceCommand,
                " S: ",speedCommand)



    def _realTimeSpeedMove_freeMotion(self,controlSignal,controlBuffer=0.1):
        """
        """
        positionCommand = None
        speedCommand = None
        forceCommand = None

        controlSignal = max(-1,min(1,controlSignal))

        if self.objectDetection(refreshStatus=False) == GOBJ_DETECTED_WHILE_CLOSING:
            # If previous object status indicate that an object was detected.
            # The move is done with high force and speed to ease the release of
            # the gripper.
            positionCommand = max(0,self.position(refreshStatus=False)-15)
            speedCommand=255
            forceCommand=255
            self.move(positionCommand,speedCommand,forceCommand,wait=True)
        elif self.objectDetection(refreshStatus=False) == GOBJ_DETECTED_WHILE_OPENING:
            positionCommand = min(255,self.position(refreshStatus=False)+15)
            speedCommand=255
            forceCommand=255
            self.move(positionCommand,speedCommand,forceCommand,wait=True)

        elif  abs(controlSignal)< controlBuffer:
            #Gripper request to stay at current position
            speedCommand = 0
            forceCommand = 0

            if self.objectDetection(refreshStatus=True)!=GOBJ_AT_POSITION:
                self.stop(refreshStatus=False)
                self.moveToCurrentPosition(speedCommand,forceCommand)

            positionCommand = self.position(refreshStatus=False)
        else:
            #Position
            if controlSignal > 0:
                positionCommand = 255
            else:
                positionCommand = 0

            #Speed
            speedControlFunction = make_ramp_function(controlBuffer*2*2,1,0,255)
            speedCommand = int(max(0,min(255,speedControlFunction(abs(controlSignal)))))

            #Force
            forceCommand = speedCommand

            self.move(positionCommand,speedCommand,forceCommand,wait=False)

        return positionCommand, speedCommand, forceCommand

    def isActivated(self, refreshStatus=True):
        """Check whether the gripper is activated.

        Args:
            refreshStatus (bool, optional): Whether to refresh the gripper status before
                checking the activation state. The status information is used to
                determine if the gripper is already activated. Setting this parameter
                to False can make the check faster if the gripper status is up to date.
                Defaults to True.

        Returns:
            bool: True if the gripper is activated, False if not activated,
            None if there is no available information to confirm gripper activation status
        """
        if refreshStatus:
            self.readStatus()
        gSTA = self._statusHistory[-1,GSTA]

        res=None

        if gSTA == -1:
            warnings.warn("Status history is empty. Activation status unknown.",UserWarning, stacklevel=2)
            res=None
            return res

        res=False
        if gSTA == GSTA_ACTIVATED:
            res=True
        else:
            res=False
        
        return res

    def isStarted(self,refreshStatus=True):
        """Check whether the gripper is started.

        Returns:
            bool: True if the gripper is started, False if not started, None
            is there is not enough information to confirm if the gripper is started.
        """
        if refreshStatus:
            self.readStatus()
        
        rGTO=self._commandHistory[-1,RGTO]
        gGTO=self._statusHistory[-1,GGTO]

        lastCommandTime = self._commandHistory[-1,TIME]
        lastStatusTime = self._statusHistory[-1,TIME]

        res=None

        if (gGTO == -1) and (rGTO == -1):
            warnings.warn("Status history is empty. Action status unknown.",UserWarning, stacklevel=2)
            res = None
            return res
        
        res=False
        if lastCommandTime > lastStatusTime:
            if rGTO == RGTO_GO_TO_REQUESTED_POSITION:
                res=True
        elif lastCommandTime < lastStatusTime:
            if gGTO == GGTO_GO_TO_REQUESTED_POSITION:
                res=True
        else:
            if (gGTO == GGTO_GO_TO_REQUESTED_POSITION) or (rGTO == RGTO_GO_TO_REQUESTED_POSITION):
                res = True
        
        return res
    
    def is_bit_calibrated(self):
        """Check whether the gripper is bit calibrated (ie. the gripper know
        its position in bit when fully open and fully closed).

        Returns:
            bool: True if the gripper is bit calibrated, False otherwise.
        """
        return self._is_bit_calibrated

    def is_mm_calibrated(self):
        """Check whether the gripper millimeter (mm) calibration is done.

        Returns:
            bool: True if the gripper is mm calibrated, False otherwise.
        """
        return self._is_mm_calibrated

    def positioningResolution(self):
        """Return the gripper positionning resolution in mm/bit. The number of
        mm the gripper will move if its position parameter vary of 1 bit.
        """
        if not self.is_mm_calibrated():
            raise GripperCalibrationError("The gripper is not mm calibrated")
        return abs(self._aCoef)

    def speedResolutionBit(self):
        """Return the speed resultion in gripper position bit/s per gripper
        speed bit. Correspond to the speed increase if speed parameter vary of
        1 bit.
        """
        return (self.gripper_vmax_bits() - self.gripper_vmin_bits())/255
    
    def speedResolutionMm(self):
        """Return the speed resultion in mm/s per bit. This correspond the 
        speed increase in mm/s if the speed parameter vary of 1 bit
        """
        return self.speedResolutionBit() * self.positioningResolution()

    def is_speed_calibrated(self):
        """Check whether the gripper speed calibration is done.

        Returns:
            bool: True if the gripper speed is calibrated, False otherwise.
        """
        return self._is_speed_calibrated

    def gripper_vmax_bits(self):
        """Return the maximum gripper speed in bits per second.

        Returns:
            int: Maximum gripper speed in bits per second.
        """
        if not self.is_speed_calibrated():
            raise GripperCalibrationError("The gripper is not speed calibrated")
        return self._gripper_vmax_bits
    
    def gripper_vmin_bits(self):
        """Return the minimum gripper speed in bits per second.

        Returns:
            int: Minimum gripper speed in bits per second.
        """

        if not self.is_speed_calibrated():
            raise GripperCalibrationError("The gripper is not speed calibrated")
        return self._gripper_vmin_bits
    
    def open_mm(self):
        """Return distance between fingers in open position
        """
        if not self.is_mm_calibrated():
            raise GripperCalibrationError("The gripper is not mm calibrated")
        
        return self._openmm()
    
    def close_mm(self):
        """Return distance between fingers in open position
        """
        if not self.is_mm_calibrated():
            raise GripperCalibrationError("The gripper is not mm calibrated")
        
        return self._closemm
    
    def positionCommand(self):
        """Return the last commanded position value.

        Returns:
            int | None: The last position command value (0-255), or None if the history is empty.
        """
        value = self._commandHistory[-1,RPR]
        if value == -1:
            warnings.warn("Command history is empty. Last set position is unknown.",UserWarning, stacklevel=2)
            return None
        return value
    
    def position(self,refreshStatus=True):
        """Return the current position of the gripper in bits.

        Args:
            refreshStatus (bool, optional): Whether to refresh the gripper status before
                getting the position. The status information is used to determine the
                current position. Setting this parameter to False can make the retrieval
                faster if the gripper status is already up to date. Defaults to True.

        Returns:
            int | None: Current gripper position in bits (0-255), or None if history is empty.
        """
        if refreshStatus:
            self.readStatus()
        res = self._statusHistory[-1,GPO]
        if res ==-1:
            warnings.warn("Status history is empty. Last position is unknown.",UserWarning, stacklevel=2)
            return None
        return int(res)

    def position_mm(self,refreshStatus=True):
        """Return the current position of the gripper in millimeters.

        Args:
            refreshStatus (bool, optional): Whether to refresh the gripper status before
                getting the position. The status information is used to determine the
                current position. Setting this parameter to False can make the retrieval
                faster if the gripper status is already up to date. Defaults to True.

        Returns:
            float: Current gripper position in millimeters.

        .. note::
            Calibration is required to use this function.
            Execute the calibration function at least once before using this function.
        """
        if not self.is_mm_calibrated():
            raise GripperNotCalibratedError()
        
        res=self._bitTomm(self.position(refreshStatus=refreshStatus))
        return res

    def speed(self):
        """Return the last set speed parameter value.

        Returns:
        --------
        int or None
            The last speed value set (0-255), or None if history is empty.
        """
        value=self._commandHistory[-1, RSP]
        if (value == -1) or (value is None):
            warnings.warn("Command history is empty. Last set speed is unknown.",UserWarning, stacklevel=2)
            return None
        return value
    
    def force(self):
        """Return the last set force paramater value.

        Returns:
            int | None: The last force value set (0-255), or None if the history is empty.
        """
        value = self._commandHistory[-1, RFR]
        if (value == -1) or (value is None):
            warnings.warn("Command history is empty. Last set force is unknown.",UserWarning, stacklevel=2)
            return None
        return value
    
    def minSpeedMmSecond(self):
        """Return the minimum gripper speed in mm/s
        """
        if not self.is_speed_calibrated():
            raise GripperCalibrationError("The gripper is not speed calibrated")
        return self.gripper_vmin_bits() * self.positioningResolution()

    def maxSpeedMmSecond(self):
        """Return the maximum gripper speed in mm/s
        """
        if not self.is_speed_calibrated():
            raise GripperCalibrationError("The gripper is not speed calibrated")
        return self.gripper_vmax_bits() * self.positioningResolution()

    def objectDetection(self,refreshStatus = True):
        """Return object detection code gOBJ

        Even if the refreshStatus is not requested, it will be done if the
        status have never been retrieved.

        Args:
            refreshSatus (boolean):
                Indicate if the object detection status is directly retrieved
                from the gripper or if it is taken from the last previously
                read status.

        Returns:
            int: Object detection status code.
                - 0: Fingers in motion, no object detected
                - 1: Fingers stopped while opening, object detected
                - 2: Fingers stopped while closing, object detected
                - 3: Fingers at requested position, no object detected or lost/dropped
        """

        if refreshStatus:
            self.readStatus()
        
        gOBJ = self._statusHistory[-1,GOBJ]
        
        return gOBJ
    
    def _estimateObjectDetection(self, window_time=0.1, trajectory_window_time=0.5, epsilon=2):
        """Estimates the object detection and stuck state of the gripper.

        Analyses the history buffer using a dual-window architecture and incorporates
        advanced slip/squeeze tracking alongside speed-proportional dynamic deadband scaling.

        Args:
            window_time (float, optional): Short window for evaluation of axis immobility 
                and active demand. Defaults to 0.1.
            trajectory_window_time (float, optional): Larger historical window specifically 
                used to detect broad path variations/oscillations in gPR. Defaults to 0.5.
            epsilon (int, optional): Position deadband in bits. Defaults to 2.

        Returns:
            int: The calculated system state id based on external configuration maps.
        """
        if not self.is_speed_calibrated():
            return EOBJ_CALCULATION_IMPOSSIBLE

        statusHistory = self.statusHistoryNumpy()
        Vmin = self.gripper_vmin_bits()
        minPos = self._openbit
        maxPos = self._closebit
        edge_margin = epsilon

        # 1. Quick Guard: Table must have at least 2 rows to read a previous state
        if len(statusHistory) < 2:
            return EOBJ_CALCULATION_IMPOSSIBLE

        # 2. Extract the previous valid state from the second-to-last line
        previous_state = int(statusHistory[-2, EOBJ])

        # 3. Initialization Guard
        if np.any(statusHistory[-1] == -1):
            return EOBJ_CALCULATION_IMPOSSIBLE

        time_array = statusHistory[:, TIME]
        
        # 4. History Depth Guard (Must handle the largest required window)
        valid_times = time_array[time_array != -1]
        if len(valid_times) == 0:
            return EOBJ_CALCULATION_IMPOSSIBLE
            
        current_time = valid_times[-1]
        total_elapsed_history = current_time - valid_times[0]
        
        # Guard against the maximum required history depth
        max_required_window = max(window_time, trajectory_window_time)
        if total_elapsed_history < max_required_window:
            return EOBJ_IN_MOTION

        # 5. Extract Broad Trajectory Window for Reversal Detection
        trajectory_start_time = current_time - trajectory_window_time
        traj_samples = statusHistory[time_array >= trajectory_start_time]
        gPR_traj = traj_samples[:, GPR].astype(int)
        
        # Calculate steps across the larger trajectory timeline
        gPR_steps_large = np.diff(gPR_traj)
        target_is_reversing = np.any(gPR_steps_large > 0) and np.any(gPR_steps_large < 0)

        # 6. Extract Short Diagnostic Window for Stuck/Immobility Detection
        window_start_time = current_time - window_time
        samples_in_window = statusHistory[time_array >= window_start_time]
        num_samples = len(samples_in_window)
        
        # 7. Communication drop check
        comm_threshold = int(window_time / 0.02)
        if num_samples < comm_threshold:
            return EOBJ_CALCULATION_IMPOSSIBLE
            
        # Extract small window vectors
        gPR_win = samples_in_window[:, GPR].astype(int)
        gPO_win = samples_in_window[:, GPO].astype(int)
        t_win   = samples_in_window[:, TIME]
        
        dt = np.diff(t_win)
        dt = np.where(dt == 0, 1e-6, dt)
        velocity = np.abs(np.diff(gPO_win)) / dt
        
        # --- Speed-Proportional Dynamic Tracking Deadband Scaling ---
        # 1. Calculate the actual mean velocity of the target profile (gPR) in bits/second
        target_velocity_profile = np.abs(np.diff(gPR_win)) / dt
        mean_target_velocity = np.mean(target_velocity_profile)
        
        # 2. Convert speed into a dynamic bit buffer: (bits/second) * (seconds) = bits
        expected_lag_buffer = int(np.ceil(mean_target_velocity * window_time))
        
        # 3. Add the speed-proportional buffer to your baseline threshold
        dynamic_epsilon = epsilon + expected_lag_buffer
        # -------------------------------------------------------------
        
        raw_error = gPR_win[1:] - gPO_win[1:]
        
        # Evaluate directional demand inside small window using the speed-proportional dynamic epsilon
        all_positive = np.all(raw_error > dynamic_epsilon)   # Loop wants to CLOSE
        all_negative = np.all(raw_error < -dynamic_epsilon)  # Loop wants to OPEN
        is_consistently_demanding = all_positive or all_negative
        
        is_immobile = np.all(velocity < Vmin)
        current_gPO = gPO_win[-1]

        # Check if the error is completely resolved (At position) using original epsilon
        if np.abs(gPR_win[-1] - current_gPO) <= epsilon:
            return EOBJ_AT_POSITION

        # Define context groups to determine if an object is already known to be held
        is_holding_closing_family = previous_state in (
            EOBJ_DETECTED_WHILE_CLOSING, 
            EOBJ_DETECTED_WHILE_CLOSING_SLIPPING
        )
        is_holding_opening_family = previous_state in (
            EOBJ_DETECTED_WHILE_OPENING, 
            EOBJ_DETECTED_WHILE_OPENING_SLIPPING
        )

        # 8. Evaluate Immobile / Stuck Scenarios
        if is_consistently_demanding and is_immobile:
            
            # Use the broader trajectory analysis to filter out target weaving/oscillations
            if target_is_reversing:
                return EOBJ_IN_MOTION
            
            # Scenario A: Handled Edge Boundaries (Normal mechanical limits)
            is_at_min_edge = current_gPO <= (minPos + edge_margin)
            is_at_max_edge = current_gPO >= (maxPos - edge_margin)
            
            if (is_at_min_edge and all_negative) or (is_at_max_edge and all_positive):
                return EOBJ_AT_POSITION

            # Scenario B: Stuck AT an extreme edge trying to move OUTward
            if is_at_min_edge and all_positive:
                return EOBJ_STUCK_AT_FULL_OPENING
            if is_at_max_edge and all_negative:
                return EOBJ_STUCK_AT_FULL_CLOSING

            # Scenario C: Context-Aware State Evaluation
            if previous_state == EOBJ_CALCULATION_IMPOSSIBLE:
                return EOBJ_CALCULATION_IMPOSSIBLE

            # Define release-stuck families
            was_stuck_closing_release = previous_state == EOBJ_DETECTED_WHILE_CLOSING_STUCK_ON_RELEASE
            was_stuck_opening_release = previous_state == EOBJ_DETECTED_WHILE_OPENING_STUCK_ON_RELEASE

            # --- CASE 1: Loop demands CLOSING ---
            if all_positive:
                if was_stuck_opening_release or is_holding_opening_family:
                    return EOBJ_DETECTED_WHILE_OPENING_STUCK_ON_RELEASE
                return EOBJ_DETECTED_WHILE_CLOSING

            # --- CASE 2: Loop demands OPENING ---
            if all_negative:
                if was_stuck_closing_release or is_holding_closing_family:
                    return EOBJ_DETECTED_WHILE_CLOSING_STUCK_ON_RELEASE
                return EOBJ_DETECTED_WHILE_OPENING

        # 9. Squeeze / Slip Scenario Handling
        elif is_consistently_demanding and not is_immobile:
            
            if all_positive and is_holding_closing_family:
                return EOBJ_DETECTED_WHILE_CLOSING_SLIPPING
                
            if all_negative and is_holding_opening_family:
                return EOBJ_DETECTED_WHILE_OPENING_SLIPPING

        # Default fallback: Axis is actively clearing error tracking normally
        return EOBJ_IN_MOTION
    
    def printObjectDetection(self,refreshStatus=True):
        """Print object detection status in a human readable way
        """
        gOBJ=self.objectDetection(refreshStatus=refreshStatus)

        print(REGISTER_DIC["gOBJ"][gOBJ])

    def commandHistory(self):
        """Return the gripper command history as a numpy array.

        Warning:
            This function keep reference of the commandHistory
            source array. If you want to edit the value of the return numpy array
            make a copy of it.
            commandHistory().copy()

        Returns:
            numpy array: A numpy array containing the status history.

                Columns are

                0 -> "time"
                1 -> "rARD"
                2 -> "rATR"
                3 -> "rGTO"
                4 -> "rACT"
                5 -> "rPR"
                6 -> "rSP"
                7 -> "rFR"

                The constant dictionnaries can be use to converter column id
                into parameter name or column name into column id.

                COMMAND_HISTORY_COLUMNS_ID_2_NAME
                COMMAND_HISTORY_COLUMNS_NAME_2_ID
        """


        return self._commandHistory

    def commandHistoryPanda(self):
        """Return the gripper command history as a pandas DataFrame.

        Returns:
            pd.DataFrame: A DataFrame containing the command history with columns for
                time and various command registers (rARD, rATR, etc.).
        """
        pd = _get_pandas()
        columns = [COMMAND_HISTORY_COLUMNS_ID_2_NAME[i] for i in range(self._commandHistory.shape[1])]
        df = pd.DataFrame(self._commandHistory, columns=columns)
        return df

   #Read only: Modbus function code 4 (Read input registers)
    def readStatus(self):
        """Retrieve gripper output register information and save it in the status history.
        """
        #Read 3 16bits registers starting from register 2000
        res = self._client.read_input_registers(2000,
                                                count=3,
                                                device_id=self.device_id)
        
        # Check result
        if res.isError():
            raise GripperCommunicationError("Read failed")

        registers=res.registers
        t = floor_to_ms(time.monotonic())
        self._saveStatus(t, registers, readWrite=False)
    
    def status(self, refreshStatus=True):
        """Return the current gripper status as a dictionary.

        Args:
            refreshStatus (bool, optional): Whether to read fresh status from the gripper.
                Defaults to True.

        Returns:
            dict: A dictionary containing current status values for all registers.
        """
        if refreshStatus:
            self.readStatus()
        status={}
        status["time"]=self._statusHistory[-1,TIME]
        status["gOBJ"]=self._statusHistory[-1,GOBJ]
        status["gSTA"]=self._statusHistory[-1,GSTA]
        status["gGTO"]=self._statusHistory[-1,GGTO]
        status["gACT"]=self._statusHistory[-1,GACT]
        status["kFLT"]=self._statusHistory[-1,KFLT]
        status["gFLT"]=self._statusHistory[-1,GFLT]
        status["gPR"]=self._statusHistory[-1,GPR]
        status["gPO"]=self._statusHistory[-1,GPO]
        status["gCU"]=self._statusHistory[-1,GCU]
        return status

    def printStatus(self, refreshStatus=True):
        """Print gripper status info in the Python terminal.

        Args:
            refreshStatus (bool, optional): Whether to read fresh status from the gripper
                before printing. Defaults to False. If you are sure that the status
                information is up to date, setting this parameter to False can make
                the printing faster.

        Examples:
            >>> grip.move(100)
            >>> grip.print_status()

            Output::

                ======================================================================
                                    GRIPPER STATUS
                ======================================================================

                gOBJ : 3
                └─ Fingers are at requested position. No object detected or object has been lost / dropped.

                gSTA : 3
                └─ Activation is completed.

                gGTO : 1
                └─ Go to Position Request.

                gACT : 1
                └─ Gripper activation.

                kFLT : 0
                └─ 0

                gFLT : 9
                └─ Minor faults (LED continuous red). No communication during at least 1 second.

                gPR  : 100
                └─ Echo of the requested position for the Gripper: 100/255

                gPO  : 100
                └─ Actual position of the Gripper obtained via the encoders: 100/255

                gCU  : 0
                └─ The current is read instantaneously from the motor drive, approximate current: 0 mA

                ======================================================================
        """
        
        status=self.status(refreshStatus=refreshStatus)

        # Print header
        print("\n" + "=" * 70)
        print(" " * 20 + "GRIPPER STATUS")
        print("=" * 70)
        
        # Find the longest key for alignment
        max_key_length = max(len(key) for key in status.keys() if key != "time")
        
        # Print each status item with description
        for key, value in status.items():
            if key != "time":
                # Print key and value with alignment
                print(f"\n{key:<{max_key_length}} : {value}")
                # Print description with indentation
                description = REGISTER_DIC[key][value]
                print(f"  └─ {description}")
        
        print("\n" + "=" * 70 + "\n")

    def statusHistoryNumpy(self):
        """Return the gripper status history as a numpy array.

        Warning:
            This function keep reference of the statusHistory
            source array. If you want to edit the value of the return numpy array
            make a copy of it.
            statusHistoryNumpy().copy()

        Returns:
            numpy array: A numpy array containing the status history.

                Columns are

                0 -> "time"
                1 -> "gOBJ"
                2 -> "gSTA"
                3 -> "gGTO"
                4 -> "gACT"
                5 -> "kFLT"
                6 -> "gFLT"
                7 -> "gPR"
                8 -> "gPO"
                9 -> "gCU"

                The constant dictionnaries can be use to converter column id
                into parameter name or column name into column id.

                STATUS_HISTORY_COLUMNS_ID_2_NAME
                STATUS_HISTORY_COLUMNS_NAME_2_ID
        """
        return self._statusHistory
    
    def statusHistoryPanda(self):
        """Return the gripper status history as a pandas DataFrame.

        Returns:
            pd.DataFrame: A DataFrame containing the status history with columns for
                time and various status registers (gOBJ, gSTA, etc.).
        """
        pd = _get_pandas()
        columns = [STATUS_HISTORY_COLUMNS_ID_2_NAME[i] for i in range(self._statusHistory.shape[1])]
        df = pd.DataFrame(self._statusHistory, columns=columns)
        return df
    
    def historyNumpy(self):
        """Return the merged command and status history as a numpy array.
        The first line of the array corresponds to the odlest command or
        status entry, the last line corresponds to the most recent command or
        status entry.

        Returns:
            numpy array: A numpy array containing the merged history.

                Columns are

                0 -> "time"
                1 -> "rARD"
                2 -> "rATR"
                3 -> "rGTO"
                4 -> "rACT"
                5 -> "rPR"
                6 -> "rSP"
                7 -> "rFR"
                8 -> "gOBJ"
                9 -> "gSTA"
                10 -> "gGTO"
                11 -> "gACT"
                12 -> "kFLT"
                13 -> "gFLT"
                14 -> "gPR"
                15 -> "gPO"
                16 -> "gCU"

                The constant dictionnaries can be use to converter column id
                into parameter name or column name into column id.

                HISTORY_COLUMNS_ID_2_NAME
                HISTORY_COLUMNS_NAME_2_ID
        """
        return self._mergeHistory()

    def historyPanda(self):
        """Return the merged command and status history as a pandas DataFrame.
        The first line of the DataFrame corresponds to the odlest command or
        status entry, the last line corresponds to the most recent command or
        status entry.

        Returns:
            pd.DataFrame: A DataFrame containing the merged history with columns for
                time, commands, and status values.
        """
        pd = _get_pandas()
        mergedHistory = self._mergeHistory()
        #Build column headers for the merged history DataFrame
        columns = [HISTORY_COLUMNS_ID_2_NAME[i] for i in range(mergedHistory.shape[1])]
        df = pd.DataFrame(mergedHistory, columns=columns)
        return df

    def lastStatusReadTime(self):
        t = self._statusHistory[-1,TIME]
        return t

    def lastMoveTime(self):
        """ Return the time of gripper last move"""
        return self._lastMoveTime
    
    def lastMoveDirection(self):
        """ Return the last direction of movement of the gripper based gripper
        position history

        Returns:
            int: Last direction of movement. 1 for closing, -1 for opening, 0 if unknown.
        """

        return self._lastMoveDirection
