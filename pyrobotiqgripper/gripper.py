"""Core Robotiq Gripper class for controlling via Modbus RTU/TCP."""

# Optional dependency: pandas is only required for history DataFrame helpers.

#Meno: Windows / Linux: Ctrl + K then Ctrl + 0 to folds all foldable regions (functions,classes,etc.) in Visual code studio
from click import command
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
                 debug: bool = False,
                 **kwargs):
        """Create a RobotiqGripper object which can be use to control Robotiq\
        grippers using modbus RTU protocol USB/RS485 connection.
        
        Parameters:
        -----------
        com_port : str
            COM port to which the gripper is connected.\
            If "auto" (or equal to the constant AUTO_DETECTION), the library will try \
            to find the COM port to which the gripper is connected.\
            On Windows, COM ports are named COM1, COM2, etc. On Linux, COM ports are\
            named /dev/ttyUSB0, /dev/ttyUSB1, etc. Default is AUTO_DETECTION.
        device_id : int
            Address of the gripper (integer) usually 9.
        gripper_type : str
            Type of the gripper. Currently only "2F" is supported.\
            Default is "2F".
        connection_type : str
            Type of connection to the gripper.\
            "RTU" (or equal to the constant GRIPPER_MODE_RTU) for direct Modbus RTU connection (e.g. via USB/RS485\
            adapter). "RTU_VIA_TCP" (or equal to the constant GRIPPER_MODE_RTU_VIA_TCP) for Modbus RTU connection via TCP\
            (e.g. when using the UR RS485 URCAP). Default is "RTU".
        tcp_host : str
            Host IP address for TCP connection. Default is "127.0.0.1"
        tcp_port : int
            Port number for TCP connection. Default is 54321.
        debug : bool
            If True, enable debug logging for Modbus\
            communication. Default is False.
        
        Examples
        --------
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
        self.com_port=com_port
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
        '''
        self._statusHistory=pd.DataFrame(columns=["time",#Time of the status
                                                  "gOBJ",#Object detection
                                                  "gSTA",#Gripper status
                                                  "gGTO",#Action status. echo of rGTO (go to bit)
                                                  "gACT",#Activation status
                                                  "kFLT",#Universal controler
                                                  "gFLT",#Fault
                                                  "gPR",#Echo of request position
                                                  "gPO",#Actual position of the gripper
                                                  "gCU"])#Current
        
        #Command historical data
        self._commandHistory=pd.DataFrame(columns=["time",#time of the command
                                                   "rARD",#Auto-releasedirection (0: close, 1:open)
                                                   "rATR",#Auto-release trigger
                                                   "rGTO",#If 1 move the gripper ot requested position
                                                   "rACT",#Activate the gripper
                                                   "rPR",#Position request
                                                   "rSP",#Speed request
                                                   "rFR"])#Force request
        '''
        
        #Attribute to monitore if the gripper is processing an action
        self._processing=False
        
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
        self._aCoef=None
        self._bCoef=None
        
        self._gripper_vmax_bits=None #Speed in bit per second
        self._gripper_vmin_bits=None #Speed in bit per second

        #Initialisation
        ###############
        self._configure_logging()
        self.readStatus()

    ####################################################
    ### PRIVATE FUCNTIONS
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
                baudrate=115200,
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
                baudrate=BAUDRATE,
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
    def _writePreadStatus(self,position):
        """Write position in the command register and read the gripper\
        status in a single Modbus transaction.
        
        Parameters:
        -----------
        position: int
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
        
        Parameters:
        -----------
        position: int
            The position to move the gripper to in bits. Integer between 0 and 255.
        speed: int
            The speed of the gripper movement. Integer between 0 and 255.
        force: int
            The force of the gripper movement. Integer between 0 and 255.
        """
        result=self._client.readwrite_registers(read_address=2000,
                                        read_count=3,
                                        write_address=1001,
                                        values=[position, speed * 0b100000000 + force],
                                        device_id=self.device_id)
        registers=result.registers
        t=floor_to_ms(time.monotonic())
        self._saveStatus(t,registers,readWrite=True)

        
        command={"time":t,"rPR":position, "rSP":speed,"rFR":force}

        self._completeAndSaveCommand(command)
    
    def _writePSF(self,position, speed, force):
        """Write position, speed and force in the command register.

        Parameters:
        -----------
        position: int
            The position to move the gripper to in bits. Integer between 0 and 255.
        speed: int
            The speed of the gripper movement. Integer between 0 and 255.
        force: int
            The force of the gripper movement. Integer between 0 and 255.
        """
        res=self._client.write_registers(address=1001,
                                 values=[position, speed * 0b100000000 + force],
                                  device_id=self.device_id)

        # Check result
        if res.isError():
            raise GripperCommunicationError("Write failed")
        t=floor_to_ms(time.monotonic())
        command={"time":t,"rPR":position, "rSP":speed,"rFR":force}

        self._completeAndSaveCommand(command)
    
    def _writeP(self,position):
        """Write position in the command register.

        Parameters:
        -----------
        position: int
            The position to move the gripper to in bits. Integer between 0 and 255.
        """
        res=self._client.write_registers(address=1001,
                                 values=[position],
                                 device_id=self.device_id)

        # Check result
        if res.isError():
            raise GripperCommunicationError("Write failed")
        t=floor_to_ms(time.monotonic())
        command={"rPR":position}

        self._completeAndSaveCommand(command)
    
    def _writeSF(self,speed, force):
        """Write speed and force in the command register.

        Parameters:
        -----------
        speed: int
            The speed of the gripper movement. Integer between 0 and 255.
        force: int
            The force of the gripper movement. Integer between 0 and 255.
        """
        res=self._client.write_registers(address=1002,
                                 values=[speed * 0b100000000 + force],
                                  device_id=self.device_id)

        # Check result
        if res.isError():
            raise GripperCommunicationError("Write failed")
        
        t=floor_to_ms(time.monotonic())
        command={"rSP":speed,"rFR":force}

        self._completeAndSaveCommand(command)
        
    #DATA SAVING FUNCTIONS
    def _saveStatus(self,t,statusRegisters,readWrite):
        """Save the gripper status register values in status history.

        Parameters:
        -----------
        t:
            Time of the status
        statusRegisters :
            Status register
        """
        #########################################
        #Register 2000
        #First Byte: gripperStatus
        #Second Byte: RESERVED
        
        #First Byte: gripperStatus
        gripperStatusReg0=(statusRegisters[0] >> 8) & 0b11111111 #xxxxxxxx00000000
        #########################################
        #Object detection
        if readWrite:
            gOBJ=-1
        else:
            gOBJ=(gripperStatusReg0 >> 6) & 0b11 #xx000000
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

        
        if gFLT in [5,9]:
            warnings.warn(REGISTER_DIC["gFLT"][9],UserWarning, stacklevel=2)

        
        
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
        
        #########################################
        #Second Byte: Current
        currentReg5=statusRegisters[2] & 0b0000000011111111 #00000000xxxxxxxx
        #########################################
        #Current
        gCU=currentReg5

        self._statusHistory[:-1,:]=self._statusHistory[1:,:]
        self._statusHistory[-1,:]=[t,gOBJ,gSTA,gGTO,gACT,kFLT,gFLT,gPR,gPO,gCU]
  
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
        """Convert a mm gripper opening in bit opening.

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
        """Convert a bit gripper opening in mm opening.

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
    
    def _bitPerSecond(self,speed):
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
        bitPerSecond=self.gripper_vmin_bits() + (float(self.gripper_vmax_bits()-self.gripper_vmin_bits())/255)*speed
        if bitPerSecond<0:
            raise GripperValidationError("Negative bit per second value {} for speed {}. Gripper_vmax_bits: {} and Gripper_vmin_bits: {}".format(bitPerSecond,speed,self.gripper_vmax_bits(),self.gripper_vmin_bits()))
        return bitPerSecond
    
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
                delta = direction * self._bitPerSecond(speed) * dt
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
        posBitPerSecond = self._bitPerSecond(speed)
        travelTime = abs(float(endPosition-startPosition))/posBitPerSecond
        return travelTime

    def _commandFilter(self,
                       current_time,
                       current_rPR,
                       minSpeedPosDelta=5,
                       maxSpeedPosDelta=55,
                       continuousGrip=True,
                       autoLock=True,
                       minimalMotion=1,
                       verbose=0,
                       refreshStatus=False
                       ):
        """"Filter the gripper command to perform a smooth and safe motion of the gripper.\
        The command filter is based on the gripper command history and the gripper status.
        
        Parameters:
        -----------
        t0_RequestTime : float
            Request time.
        t0_RequestPosition : int
            Requested position.
        commandHistory : dict
            Command history.
        status : dict
            Gripper status.
        minSpeedPosDelta : int
            Minimum speed position delta.
        maxSpeedPosDelta : int
            Maximum speed position delta.
        continuousGrip : bool
            Whether to continuously grip.
        autoLock : bool
            Whether to automatically lock.
        minimalMotion : int
            Minimal motion.
        verbose : int
            Verbose level to print. 1 print all executed command. 2 print all commands.

        Returns:
        --------
        command : dict
            Filtered command.
        """
        if refreshStatus:
            self.readStatus()

        #t1: previous command
        #t0: next command to come

        #Object detection

        history = self._mergeHistory()
        
        prev_time=history[-1,TIME]
        prev_cOBJ=self.objectDetection(history,duration=0.2,tolerance=3)
        prev_gPO=history[-1,M_GPO]
        prev_rPR=history[-1,RPR]
        prev_rSP=history[-1,RSP]
        prev_rFR=history[-1,RFR]

        forceMin = continuousGrip * 1
        command = {}

        dt = current_time - prev_time

        command["execution"] = NO_COMMAND
        command["rPR"] = 0
        command["rSP"] = 0
        command["rFR"] = forceMin
        command["wait"] = False
        command["comment"] = ""
        
        cPO = self._positionEstimation(prev_gPO,
                                       prev_rPR,
                                       prev_rSP,
                                       dt)
        
        # Check if object detected
        if prev_cOBJ in [GOBJ_DETECTED_WHILE_OPENING, GOBJ_DETECTED_WHILE_CLOSING]:
            # Object detected
            full_grip_applied = (prev_rPR in [0, 255] and prev_rSP == 255 and prev_rFR == 255)
            if not full_grip_applied:
                # Apply full grip
                if prev_cOBJ == GOBJ_DETECTED_WHILE_CLOSING:
                    command["execution"] = WRITE_READ_COMMAND
                    command["rPR"] = 255
                    command["rSP"] = 255
                    command["rFR"] = 255
                    command["wait"] = True
                    command["comment"] = "Object detected while closing, apply full grip"
                else:  # GOBJ_DETECTED_WHILE_OPENING
                    command["execution"] = WRITE_READ_COMMAND
                    command["rPR"] = 0
                    command["rSP"] = 255
                    command["rFR"] = 255
                    command["wait"] = True
                    command["comment"] = "Object detected while opening, apply full release"
            else:
                # Full grip applied
                # Check if further gripping requested
                if ((prev_cOBJ == GOBJ_DETECTED_WHILE_CLOSING and current_rPR >= prev_gPO) or
                    (prev_cOBJ == GOBJ_DETECTED_WHILE_OPENING and current_rPR <= prev_gPO)):
                    # Further gripping
                    command["execution"] = READ_COMMAND
                    command["rPR"] = None
                    command["rSP"] = None
                    command["rFR"] = None
                    command["wait"] = False
                    command["comment"] = "Full grip applied, further gripping requested, do nothing"
                else:
                    # Away from grip or release
                    command["execution"] = WRITE_READ_COMMAND
                    command["rPR"] = current_rPR
                    command["rSP"] = 255
                    command["rFR"] = 255
                    command["wait"] = True
                    command["comment"] = "Object detected, going to release position"
        else:
            # No object detected
            if current_rPR in [0, 255] and not (prev_rPR in [0, 255] and prev_rSP == 255 and prev_rFR == 255):
                command["execution"] = WRITE_READ_COMMAND
                command["rPR"] = current_rPR
                command["rSP"] = 255
                command["rFR"] = 255
                command["wait"] = True
                command["comment"] = f"No object detected, trigger full grip to {current_rPR}"

            elif ((abs(prev_gPO - current_rPR) <= minimalMotion)
                or (prev_rPR > self._closebit and current_rPR > self._closebit)
                or (prev_rPR < self._openbit and current_rPR < self._openbit)):
                command["execution"] = READ_COMMAND
                command["rPR"] = None
                command["rSP"] = None
                command["rFR"] = None
                command["wait"] = False
                command["comment"] = "Requested position close to current position or both in extreme position, do nothing"
            else:
                # Adjust speed based on distance
                posDelta = abs(prev_gPO - current_rPR)
                if posDelta <= minSpeedPosDelta:
                    speed = 0
                elif posDelta >= maxSpeedPosDelta:
                    speed = 255
                else:
                    speed = int((posDelta - minSpeedPosDelta) / (maxSpeedPosDelta - minSpeedPosDelta) * 255)
                force = forceMin
                wait = False
                comment = f"No object detected, move with adjusted speed {speed}"


                if (prev_rPR==0 and current_rPR > self._openbit) or (prev_rPR==255 and current_rPR < self._closebit):
                    force = 255
                    speed = 255
                    wait =True
                    comment = f"Move out of a full close of a full opening"

                
                # Check if requesting extreme position and previous was not full grip
                command["execution"] = WRITE_READ_COMMAND
                command["rPR"] = current_rPR
                command["rSP"] = speed
                command["rFR"] = force
                command["wait"] = wait
                command["comment"] = comment
        if verbose ==1:
            if command["execution"]==WRITE_READ_COMMAND:
                print(f"Time: {current_time:.3f} | ReqPos: {current_rPR:3d} | cPO: {cPO}| ObjDet: {prev_cOBJ} | CmdExe: {command['execution']:1d} | CmdPos: {command['rPR'] or 0:3d} | CmdSpd: {command['rSP'] or 0:3d} | CmdFrc: {command['rFR'] or 0:3d} | CmdWait: {command['wait'] or 0:.3f} | Comment: {command['comment']}")
        if verbose ==2:
            print(f"Time: {current_time:.3f} | ReqPos: {current_rPR:3d} | cPO: {cPO} | ObjDet: {prev_cOBJ} | CmdExe: {command['execution']:1d} | CmdPos: {command['rPR'] or 0:3d} | CmdSpd: {command['rSP'] or 0:3d} | CmdFrc: {command['rFR'] or 0:3d} | CmdWait: {command['wait'] or 0:.3f} | Comment: {command['comment']}")


        return command

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

        for key in ["rPR", "rSP", "rFR"]:
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
    #SETUP
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
        """If not already activated, activate the gripper.

        Parameters:
        -----------
        reset : bool
            Whether to reset the gripper before activation. Default is True.
        start : bool
            Whether to start the gripper motion immediately after activation. Default is True.
        refreshStatus : bool
            Whether to refresh the gripper status before the activation process. Default is True.

        .. warning::
            When you execute this function the gripper is going to fully open\
            and close. During this operation the gripper must be able to freely\
            move. Do not place object inside the gripper.
        """
        
        #Turn the variable which indicate that the gripper is processing
        #an action to True
        if reset:
            self.reset()

        self._processing=True


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
            
        self._processing=False
    
    def start(self,refreshStatus=True):
        """Gripper start moving. GTO bit is set to 1 but the position command is not\
        changed. The gripper will move to the position command if it is not already there."""
        t=floor_to_ms(time.monotonic())
        command={"time":t,
                 "rARD":0,
                 "rATR":0,
                 "rGTO":RGTO_GO_TO_REQUESTED_POSITION
                 }
        if not self.isStarted(refreshStatus=refreshStatus):
            if self.isActivated(refreshStatus=refreshStatus):
                res=self._client.write_registers(1000,[0b0000100100000000],device_id=self.device_id)
                command["rACT"]=RACT_ACTIVATE
                if res.isError():
                    raise GripperCommunicationError("Communication to set rGTO bit failed")
            else:
                res=self._client.write_registers(1000,[0b0000100000000000],device_id=self.device_id)
                command["rACT"]=RACT_DESACTIVATE
                if res.isError():
                    raise GripperCommunicationError("Communication to set rGTO bit failed")
            self._completeAndSaveCommand(command)
            self.readStatus()
    
    def stop(self):
        """Gripper stop moving. GTO bit is set to 0 but the position command is not\
        changed. The gripper will stop at its current position and hold it."""
        if self.isActivated:
            self._client.write_registers(1000,[0b0000000100000000,0,0],device_id=self.device_id)
        else:
            self._client.write_registers(1000,[0b0000000000000000,0,0],device_id=self.device_id)
    
    def calibrate_bit(self,
                      openbit=None,
                      closebit=None):
        """Calibrate the maximum and minimum position bit value

        openbit : int
            Position value in bits when the gripper is open
        closebit : int
            Position value in bits when the gripper is closed

        .. warning::
        If no parametesr are provided, the gripper will make a full open followed by a\
        full close to measure the maximum and minium position in bit.
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

    def calibrate_speed(self,minSpeedClosingTime=None,maxSpeedClosingTime=None):
        """Calibrate gripper speed to be able to estimate gripper position over time

        If no parameters are provided, the gripper will do a closing at full speed and\
        a closing at slow speed to evaluate. the bit calibration will also be performed.

        Parameters :
        ------------
        minSpeedClosingTime : float
            Time (s) is takes for the gripper to move from a full open position to a full close position at the minimum speed.
        maxSpeedClosingTime : float
            Time (s) is takes for the gripper to move from a full open position to a full close position at the maximum speed.
        
        .. warning::
        If no parameters are provided, the gripper will do a closing at full speed and a closing at slow speed to evaluate 
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
        """Calibrate the gripper for mm positionning.
        
        Once the calibration is done it is possible to control the gripper in\
        mm.

        Parameters:
        -----------
        closemm : float
            Distance between the fingers when the gripper is fully closed.
        openmm : float
            Distance between the fingers when the gripper is fully open.       
        """
        if not self.is_bit_calibrated():
            raise GripperCalibrationError("Execute the function calibrate_bit() before executing the function calibrate_mm()")

        self._closemm=closemm
        self._openmm=openmm
        
        self._aCoef=(closemm-openmm)/(self._closebit-self._openbit)
        self._bCoef=(openmm*self._closebit-self._openbit*closemm)/(self._closebit-self._openbit)

        self._is_mm_calibrated=True

    #ACTIONS
    def open(self,speed=255,force=255,wait=True,readStatus=True,refreshStatus=False):
        """Open the gripper
        
        Parameters:
        -----------
        speed : int
            Gripper speed between 0 and 255. Default is 255.
        force : int
            Gripper force between 0 and 255. Default is 255.
        """
        #Check if the gripper is activated
        self.move(0,speed,force,wait=wait,readStatus=readStatus,refreshStatus=refreshStatus)
    
    def close(self,speed=None,force=None,wait=True,readStatus=True,refreshStatus=False):
        """Close the gripper.

        Parameters:
        -----------
        speed : int
            Gripper speed between 0 and 255. Default is None.
        force : int
            Gripper force between 0 and 255. Default is None.
        """
        self.move(255,speed,force,wait=wait,readStatus=readStatus,refreshStatus=refreshStatus)
    
    def move(self,position,speed=None,force=None,wait=True,readStatus=True,refreshStatus=False):
        """Move gripper fingers to the requested position with determined speed and force.
        
        Parameters:
        -----------
        position : int
            Position of the gripper. Integer between 0 and 255.\
            0 being the open position and 255 being the close position.
        speed : int
            Gripper speed between 0 and 255. Default is 255.
        force : int
            Gripper force between 0 and 255. Default is 255.
        wait : bool
            If True, the function wait until the gripper\
            reach the requested position or detect an object. Default is False.
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
        if not self.isActivated(refreshStatus=False):
            print("Status history:")
            print(self._statusHistory)
            raise GripperNotActivatedError()
        if not self.isStarted(refreshStatus=False):
            raise GripperNotStartedError()
        #Check input value
        if position>255 or position<0:
            raise GripperPositionError(position)
        
        position=int(position)

        if (speed is None) and (force is None):
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
        
        self._processing=True
        if wait:
            self._waitComplete()
        self._processing=False
    
    def move_mm(self,positionmm,speed=None,force=None,wait=True,readStatus=True,refreshStatus=False):
        """Go to the requested opening expressed in mm

        Parameters:
        -----------
        positionmm : float
            Gripper opening in mm.
        speed : int
            Gripper speed between 0 and 255. Default is 255.
        force : int
            Gripper force between 0 and 255. Default is 255.
        
        .. note::
            Calibration is needed to use this function.\n
            Execute the function calibrate at least 1 time before using this function.
        """

        if not self.is_mm_calibrated():
            raise GripperNotCalibratedError()

        if  positionmm>self._openmm:
            raise GripperValidationError("The maximum opening is {} mm but the given value is {} mm".format(self._openmm,positionmm))
        if positionmm<self._closemm:
            raise GripperValidationError("The minimum closing is {} mm but the given value is {} mm".format(self._openmm,positionmm))
        
        position=int(self._mmToBit(positionmm))
        self.move(position,speed,force,wait,readStatus,refreshStatus)

    def realTimeMove(self,
                     requestedPosition,
                     minSpeedPosDelta=5,
                     maxSpeedPosDelta=100,
                     continuousGrip=True,
                     autoLock=True,
                     minimalMotion=2,
                     verbose=False):
        """Move the gripper in real time to the requested position.
        
        Parameters:
        -----------
        requestedPosition : int
            Requested position for the gripper in bits.\
            Integer between 0 and 255. 0 being the open position and 255 being the\
            close position.
        minSpeedPosDelta : int
            Minimum position delta to apply the\
            minimum speed. Default is 5.
        maxSpeedPosDelta : int
            Position delta over which the maximum\
            speed is applied. Default is 100.
        continuousGrip : bool
            If True, the gripper continuously try to\
            close on object even after object detection (force>0). Default is True.
        autoLock : bool
            If True, the gripper automatically perform a\
            full speed, full force grip after object detection. Default is True.
        minimalMotion : int
            Minimum motion in bit to perform when a\
            motion is requested. If the position delta between the current position and\
            the requested position is under this value, no motion is performed. Default\
            is 2.
        verbose : int
            Verbose level to print. 1 print all executed command. 2 print all commands.
        """
        #Check if the gripper is activated
        if not self.isActivated(refreshStatus=False):
            raise GripperNotActivatedError()
        if not self.isStarted(refreshStatus=False):
            raise GripperNotStartedError()
        if not self.is_speed_calibrated():
            raise GripperCalibrationError("The gripper need to be speed calibrated before using realtime move.")
        
        #2- Get time
        now=floor_to_ms(time.monotonic())
        
        
        #2 Build gripper command
        command=self._commandFilter(current_time=now,
                                    current_rPR=requestedPosition,
                                    minSpeedPosDelta=minSpeedPosDelta,
                                    maxSpeedPosDelta=maxSpeedPosDelta,
                                    continuousGrip=continuousGrip,
                                    autoLock=autoLock,
                                    minimalMotion=minimalMotion,
                                    verbose=verbose)
        
        if command["execution"]==WRITE_READ_COMMAND:
            if command["wait"]:
                self.move(command["rPR"],command["rSP"],command["rFR"],wait=True)
            else:
                self.move(command["rPR"],command["rSP"],command["rFR"],wait=False)

        elif command["execution"]==READ_COMMAND:
            self.readStatus()
        else:
            warnings.warn("No command executed",UserWarning, stacklevel=2)

    #STATUS
    def isActivated(self, refreshStatus=True):
        """Tells if the gripper is activated

        Returns:
        --------
        is_activated : bool
            True if the gripper is activated. False otherwise.
        """
        if refreshStatus:
            self.readStatus()
        gSTA = self._statusHistory[-1,GSTA]

        if gSTA == -1:
            warnings.warn("Status history is empty. Activation status unknown.",UserWarning, stacklevel=2)
            res=None

        res=False
        if gSTA == GSTA_ACTIVATED:
            res=True
        else:
            res=False
        
        return res

    def isStarted(self,refreshStatus=True):
        """Tells if the gripper is started

        Returns:
        --------
        is_started : bool
            True if the gripper is started. False otherwise.
        """
        if refreshStatus:
            self.readStatus()
        gGTO=self._statusHistory[-1,GGTO]

        if gGTO == -1:
            warnings.warn("Status history is empty. Action status unknown.",UserWarning, stacklevel=2)
            print(self.statusHistory())
            return None
        
        res=False
        if gGTO == GGTO_GO_TO_REQUESTED_POSITION:
            res=True
        
        return res
    
    def is_bit_calibrated(self):
        """Tells is the gripper is bit calibrated
        """
        return self._is_bit_calibrated

    def is_mm_calibrated(self):
        """Tells if the mm calibration is done.
        
        Returns:
        --------
        is_mm_calibrated : bool
            True if the gripper mm is calibrated. False otherwise."""
        return self._is_mm_calibrated

    def is_speed_calibrated(self):
        """TElls if the speed calibration is done.
        
        Returns:
        --------
        is_started : bool
            True if the gripper speed is calibrated. False otherwise."""
        return self._is_speed_calibrated

    def gripper_vmax_bits(self):
        """Return the maximum speed in bits per second

        Returns:
        vmax_bits : int
            Maximum gripper speed in bits per second.
        """
        if not self.is_speed_calibrated():
            raise GripperCalibrationError("The gripper is not speed calibrated")
        return self._gripper_vmax_bits
    
    def gripper_vmin_bits(self):
        """Return the minimum speed in bits per second

        Returns:
        --------
        vmin_bits : int
            Minimum gripper speed in bits per second.
        """

        if not self.is_speed_calibrated():
            raise GripperCalibrationError("The gripper is not speed calibrated")
        return self._gripper_vmin_bits
    
    def positionCommand(self):
        """Return the last commanded position value.

        Returns:
        --------
        int or None
            The last position command value (0-255), or None if history is empty.
        """
        value = self._statusHistory[-1,GPO]
        if value == -1:
            warnings.warn("Command history is empty. Last set position is unknown.",UserWarning, stacklevel=2)
            return None
        return value
    
    def position(self,refreshStatus=True):
        if refreshStatus:
            self.readStatus()
        res = self._statusHistory[-1,GPO]
        if res ==-1:
            warnings.warn("Status history is empty. Last position is unknown.",UserWarning, stacklevel=2)
            return None
        return int(res)

    def positionmm(self,refreshStatus=True):
        """Return the position of the gripper in mm.

        Returns:
        --------
        positionmm : float
            Current gripper position in mm
        
        .. note::
            Calibration is needed to use this function.\n
            Execute the function calibrate at least 1 time before using this function.
        """
        if not self.is_mm_calibrated():
            raise GripperNotCalibratedError()
        
        res=self._bitTomm(self.position(refreshStatus=refreshStatus))
        return res

    def speed(self):
        """Return the last set speed value.

        Returns:
        --------
        int or None
            The last speed value set (0-255), or None if history is empty.
        """
        value=self._commandHistory[-1, RSP]
        if value == -1:
            warnings.warn("Command history is empty. Last set speed is unknown.",UserWarning, stacklevel=2)
            return None
        return value
    
    def force(self):
        """Return the last set force value.

        Returns:
        --------
        int or None
            The last force value set (0-255), or None if history is empty.
        """
        value = self._commandHistory[-1, RFR]
        if value == -1:
            warnings.warn("Command history is empty. Last set force is unknown.",UserWarning, stacklevel=2)
            return None
        return value

    def objectDetection(self,mergedHistory=None, duration=0.2, tolerance=3):
        """Estimate object detection status from history data.

        Parameters:
        -----------
        mergedHistory : numpy.ndarray, optional
            Pre-merged history array. If None, merges internally.
        duration : float, optional
            Stability duration threshold. Default is 0.2.
        tolerance : int, optional
            Position tolerance. Default is 3.

        Returns:
        --------
        int
            Object detection status code.
        """
        #df_subset = df.loc[:, ["col1", "col2", "col3"]]
        #df_subset = df.loc[start_index:, ["time", "rPR", "gPO"]]

        if not self.is_bit_calibrated():
            raise GripperCalibrationError("The gripper need to be bit calibrated to be able to estimate object detection.")

        if mergedHistory is None:
            mergedHistory = self._mergeHistory()

        last_gOBJ = mergedHistory[-1, M_GOBJ]

        if last_gOBJ != -1:
            # If gOBJ is explicitly available from the gripper, use that value.
            return int(last_gOBJ)

        # Filter for valid status rows (gPO and rPR are required for estimation).
        valid_mask = (mergedHistory[:, M_GPO] != -1) & (mergedHistory[:, RPR] != -1)
        if not np.any(valid_mask):
            return GOBJ_IN_MOTION

        time = mergedHistory[valid_mask, TIME]
        rpr = mergedHistory[valid_mask, RPR]
        gpo = mergedHistory[valid_mask, M_GPO]

        last_gPO = gpo[-1]

        # If we have a recent request change, consider motion in progress first.
        rpr_diff_idx = np.flatnonzero(rpr != rpr[-1])
        if rpr_diff_idx.size > 0 and (time[-1] - time[rpr_diff_idx[-1]] <= duration):
            return GOBJ_IN_MOTION

        # Detect object only after the gripper position has been stable long enough.
        gpo_diff_idx = np.flatnonzero(gpo != last_gPO)
        if gpo_diff_idx.size == 0:
            # gPO constant on history window.
            expected = rpr[-1]
            expected = min(max(expected, self._openbit), self._closebit)

            if abs(expected - last_gPO) < tolerance:
                return GOBJ_AT_POSITION

            dt_constant = time[-1] - time[0]
            if dt_constant <= duration:
                return GOBJ_IN_MOTION

            if expected > last_gPO:
                return GOBJ_DETECTED_WHILE_CLOSING
            elif expected < last_gPO:
                return GOBJ_DETECTED_WHILE_OPENING
            else:
                return GOBJ_IN_MOTION

        idx = gpo_diff_idx[-1]
        dt = time[-1] - time[idx]
        if dt <= duration:
            return GOBJ_IN_MOTION

        expected = rpr[-1]
        expected = min(max(expected, self._openbit), self._closebit)

        if abs(expected - last_gPO) < tolerance:
            return GOBJ_AT_POSITION

        rpr_slice = rpr[idx:]
        gpo_slice = gpo[idx:]

        if np.all(rpr_slice > gpo_slice):
            return GOBJ_DETECTED_WHILE_CLOSING

        if np.all(rpr_slice < gpo_slice):
            return GOBJ_DETECTED_WHILE_OPENING

        return GOBJ_IN_MOTION
    
    def commandHistory(self):
        """Return the gripper command history as a pandas DataFrame.

        Returns:
        --------
        pd.DataFrame
            A DataFrame containing the command history with columns for time
            and various command registers (rARD, rATR, etc.).
        """
        pd = _get_pandas()
        columns = [COMMAND_HISTORY_COLUMNS_ID_2_NAME[i] for i in range(self._commandHistory.shape[1])]
        df = pd.DataFrame(self._commandHistory, columns=columns)
        return df

    def readStatus(self):
        """Retrieve gripper output register information and save it in the\
        parameter dictionary.
        """
        #Read 3 16bits registers starting from register 2000
        registers=self._client.read_input_registers(2000,count=3,device_id=self.device_id).registers
        t=floor_to_ms(time.monotonic())
        self._saveStatus(t,registers,readWrite=False)
    
    def status(self, refreshStatus=True):
        """Return the current gripper status as a dictionary.

        Parameters:
        -----------
        refreshStatus : bool, optional
            Whether to read fresh status from the gripper. Default is True.

        Returns:
        --------
        dict
            A dictionary containing current status values for all registers.
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

    def printStatus(self, refreshStatus=False):
        """Print gripper status info in the python terminal

        Examples
        --------
            >>> grip.move(100)
            >>> grip.printStatus()

            Output::

                ======================================================================
                                    GRIPPER STATUS
                ======================================================================

                gOBJ : 3
                └─ Fingers are at requested position. No object detected or object has been loss / dropped.

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
                └─ Echo of the requested position for the Gripper:100/255

                gPO  : 100
                └─ Actual position of the Gripper obtained via the encoders:100/255

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

    def statusHistory(self):
        """Return the gripper status history as a pandas DataFrame.

        Returns:
        --------
        pd.DataFrame
            A DataFrame containing the status history with columns for time
            and various status registers (gOBJ, gSTA, etc.).
        """
        pd = _get_pandas()
        columns = [STATUS_HISTORY_COLUMNS_ID_2_NAME[i] for i in range(self._statusHistory.shape[1])]
        df = pd.DataFrame(self._statusHistory, columns=columns)
        return df
    
    def history(self):
        """Return the merged command and status history as a pandas DataFrame.

        This method combines the command history and status history into a single
        DataFrame with all timestamps aligned.

        Returns:
        --------
        pd.DataFrame
            A DataFrame containing the merged history with columns for time,
            commands, and status values.
        """
        pd = _get_pandas()
        mergedHistory = self._mergeHistory()
        columns = [HISTORY_COLUMNS_ID_2_NAME[i] for i in range(mergedHistory.shape[1])]
        df = pd.DataFrame(mergedHistory, columns=columns)
        return df