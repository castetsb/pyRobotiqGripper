"""Core Robotiq Gripper class for controlling via Modbus RTU/TCP."""

#Iport libraries
from pymodbus.client import ModbusSerialClient, ModbusTcpClient
from pymodbus.framer import FramerType
import time
import serial
import serial.tools.list_ports
from .utils import *
from .constants import *
from .exceptions import (
    GripperConnectionError,
    GripperNotActivatedError,
    GripperNotCalibratedError,
    GripperTimeoutError,
    GripperPositionError,
    UnsupportedGripperTypeError,
)
import logging
import multiprocessing


class RobotiqGripper( ):
    """Class use to control Robotiq grippers (2F85, 2F140 or hande).

    This class provides methods to initialize, open, close, and monitor the gripper.

    Attributes
    ----------
    com_port : str
        COM port to which the gripper is connected. If AUTO_DETECTION,\
        the library will try to find the COM port to which the gripper is connected. On\
        Windows, COM ports are named COM1, COM2, etc. On Linux, COM ports are named\
        /dev/ttyUSB0, /dev/ttyUSB1, etc. Default is AUTO_DETECTION.
    device_id : int
        Address of the gripper (integer) usually 9.
    gripper_type : str
        Type of the gripper. Currently only "2F" is supported. Default is "
    connection_type : str
        Type of connection to the gripper. GRIPPER_MODE_RTU for direct Modbus RTU\
        connection (e.g. via USB/RS485 adapter). GRIPPER_MODE_RTU_VIA_TCP\
        for Modbus RTU connection via TCP (e.g. when using the UR RS485 URCAP).\
        Default is GRIPPER_MODE_RTU.
    tcp_host : str
        Host IP address for TCP connection. Default is "127.0.0.1".
    tcp_port : int
        Port for TCP connection. Default is 54321.
    debug : bool
        If True, enable debug logging for Modbus communication. Default is False.
    

    Examples
    --------
    Gripper connected at PC USB port:
        >>> gripper = RobotiqGripper()
        >>> gripper.connect()
        >>> gripper.resetActivate()
        >>> gripper.open()
        >>> gripper.close()
        >>> gripper.move(100) #Move at position 100 in bit
        >>> print(gripper.position) #Print gripper position in bit
        >>> gripper.calibrate(closemm=0,openmm=85) #Calibrate the gripper with 0mm when closed and 85mm when open
        >>> gripper.move_mm(50) #Move at position 50mm
        >>> gripper.printStatus() #Print gripper status information in the python terminal
        >>> print(gripper.positionmm) #Print gripper position in mm

    Gripper connected to UR robot via RS495 URCAP:
        >>> gripper = RobotiqGripper(connection_type=GRIPPER_MODE_RTU_VIA_TCP,tcp_host="192.168.1.100")
        >>> gripper.connect()
        >>> gripper.resetActivate()
        >>> gripper.open()

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
    def _probe_port_process(self, port, device_id, return_dict):
        try:
            client = ModbusSerialClient(
                port=port,
                baudrate=115200,
                parity='N',
                stopbits=1,
                bytesize=8,
                timeout=0.2
            )

            if not client.connect():
                return_dict["success"] = False
                return

            result = client.read_input_registers(
                address=2000,
                count=1,
                device_id=device_id
            )

            if result and not result.isError():
                return_dict["success"] = True
            else:
                return_dict["success"] = False

        except Exception:
            return_dict["success"] = False

        finally:
            try:
                client.close()
            except:
                pass
    def __init__(self,
                 com_port: str = AUTO_DETECTION,
                 device_id: int=9,
                 gripper_type: str = "2F",
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
            If AUTO_DETECTION, the library will try to find the COM port to which\
            the gripper is connected.\
            On Windows, COM ports are named COM1, COM2, etc. On Linux, COM ports are\
            named /dev/ttyUSB0, /dev/ttyUSB1, etc. Default is AUTO_DETECTION.
        device_id : int
            Address of the gripper (integer) usually 9.
        gripper_type : str
            Type of the gripper. Currently only "2F" is supported.\
            Default is "2F".
        connection_type : str
            Type of connection to the gripper.\
            GRIPPER_MODE_RTU for direct Modbus RTU connection (e.g. via USB/RS485\
            adapter). GRIPPER_MODE_RTU_VIA_TCP for Modbus RTU connection via TCP\
            (e.g. when using the UR RS485 URCAP). Default is GRIPPER_MODE_RTU.
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
            >>> gripper = RobotiqGripper()
            >>> gripper.connect()
            >>> gripper.resetActivate()
            >>> gripper.open()
            >>> gripper.close()
            >>> gripper.move(100) #Move at position 100 in bit
            >>> print(gripper.position) #Print gripper position in bit
            >>> gripper.calibrate(closemm=0,openmm=85) #Calibrate the gripper with 0mm when closed and 85mm when open
            >>> gripper.move_mm(50) #Move at position 50mm
            >>> gripper.printStatus() #Print gripper status information in the python terminal
            >>> print(gripper.positionmm) #Print gripper position in mm

        Gripper connected to UR robot via RS495 URCAP:
            >>> gripper = RobotiqGripper(connection_type=GRIPPER_MODE_RTU_VIA_TCP,tcp_host="192.168.1.100")
            >>> gripper.connect()
            >>> gripper.resetActivate()
            >>> gripper.open()
        """
        self.connection_type=connection_type
        self.device_id=device_id
        self.tcp_host=tcp_host
        self.tcp_port=tcp_port
        self.com_port=com_port
        self._client=self._create_modbus_client()

        
        #Attribute to monitore if the gripper is processing an action
        self.processing=False
        
        #Maximum allowed time to perform and action
        self.timeOut=10
        
        #Dictionnary where are stored description of each register state
        self.registerDic={}
        self._buildRegisterDic()
        
        #Dictionnary where are stored register values retrived from the gripper
        self.status={}
        self.readStatus()
        
        #Attributes to store open and close distance state information
        
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

        #Maximum and minimum speed of the gripper
        if gripper_type=="2F":
            self.gripper_vmax=GRIPPER_2F_VMAX
            self.gripper_vmin=GRIPPER_2F_VMIN
        else:
            raise UnsupportedGripperTypeError(gripper_type)

        self._commandHistory={}
        self._commandHistory["time"]=[time.monotonic()]*30
        time.sleep(1)
        updateList(self._commandHistory["time"],time.monotonic())
        
        self._commandHistory["positionCommand"]=[0]*30 #Command send to the gripper
        self._commandHistory["position"]=[0]*30 #Position read from the gripper
        self._commandHistory["positionRequest"]=[0]*30 #Requested position before command filtering
        self._commandHistory["speedCommand"]=[0]*30
        self._commandHistory["forceCommand"]=[0]*30
        self._commandHistory["detection"]=[0]*30
        self._commandHistory["gripCommand"]=[GRIP_NOT_REQUESTED]*30

        self.debug = debug
        self._configure_logging()

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
    
    def connect(self):
        """Connect to the gripper. If the connection is already established, do nothing.
        """
        if not self._client.connect():
            raise GripperConnectionError("Failed to connect to the gripper. Please check the connection and try again.")
    def disconnect(self):
        """Disconnect from the gripper. If the connection is already closed, do nothing.
        """
        self._client.close()

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

    def realTimeMove(self,requestedPosition,minSpeedPosDelta=5,maxSpeedPosDelta=100,continuousGrip=True,autoLock=True,minimalMotion=2):
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
        """
        #2- Get time
        now=time.monotonic()
        elapsedTime = now - self._commandHistory["time"][0]
        if elapsedTime<0:
            raise Exception("Negative elapsed time {} previous time {} , current time {}".format(elapsedTime,self._commandHistory["time"][0],now))
        
        
        #2 Build gripper command
        command=self._commandFilter(t0_RequestTime=now,
                                t0_RequestPosition=requestedPosition,
                                commandHistory=self._commandHistory,
                                status=self.status,
                                minSpeedPosDelta=minSpeedPosDelta,
                                maxSpeedPosDelta=maxSpeedPosDelta,
                                continuousGrip=continuousGrip,
                                autoLock=autoLock,
                                minimalMotion=minimalMotion)
        if command["execution"]==WRITE_READ_COMMAND:
            self._writePSFreadStatus(command["position"],command["speed"],command["force"])
            if command["wait"]>0:
                time.sleep(command["wait"])
            updateList(self._commandHistory["time"],now)
            updateList(self._commandHistory["positionCommand"],command["position"])
            updateList(self._commandHistory["position"],self.status["gPO"])
            updateList(self._commandHistory["positionRequest"],requestedPosition)
            updateList(self._commandHistory["speedCommand"],command["speed"])
            updateList(self._commandHistory["forceCommand"],command["force"])
            updateList(self._commandHistory["detection"],self.status["gOBJ"])
            updateList(self._commandHistory["gripCommand"],command["grip"])

        elif command["execution"]==READ_COMMAND:
            self.readStatus()
        else:
            print("No execution command")

    def _autoConnect(self):
        """Automatically detect the COM port to which the gripper is connected."""
        ports = serial.tools.list_ports.comports()

        for port in ports:
            print(f"Testing: {port.device}")

            manager = multiprocessing.Manager()
            return_dict = manager.dict()

            p = multiprocessing.Process(
                target=self._probe_port_process,
                args=(port.device, self.device_id, return_dict)
            )

            p.start()
            p.join(1.0)  # HARD TIMEOUT (1 second)

            if p.is_alive():
                print("Hard timeout — killing process")
                p.terminate()
                p.join()
                continue

            if return_dict.get("success", False):
                print(f"Gripper detected on {port.device}")
                return port.device

        raise GripperConnectionError("No gripper detected on any available port")

    def _buildRegisterDic(self):
        """Build a dictionnary with comment to explain each register variable.

        Dictionnary key are variable names. Dictionnary value are dictionnary\
        with comments about each statut of the variable (key=variable value,\
        value=comment)
        """
        ######################################################################
        #input register variable
        self.registerDic.update({"gOBJ":{},"gSTA":{},"gGTO":{},"gACT":{},
                                "kFLT":{},"gFLT":{},"gPR":{},"gPO":{},"gCU":{}})
        
        #gOBJ
        gOBJdic=self.registerDic["gOBJ"]
        
        gOBJdic[0]="Fingers are in motion towards requested position. No object detected."
        gOBJdic[1]="Fingers have stopped due to a contact while opening before requested position. Object detected opening."
        gOBJdic[2]="Fingers have stopped due to a contact while closing before requested position. Object detected closing."
        gOBJdic[3]="Fingers are at requested position. No object detected or object has been loss / dropped."
        
        #gSTA
        gSTAdic=self.registerDic["gSTA"]
        
        gSTAdic[0]="Gripper is in reset ( or automatic release ) state. See Fault Status if Gripper is activated."
        gSTAdic[1]="Activation in progress."
        gSTAdic[3]="Activation is completed."
        
        #gGTO
        gGTOdic=self.registerDic["gGTO"]
        
        gGTOdic[0]="Stopped (or performing activation / automatic release)."
        gGTOdic[1]="Go to Position Request."
        gGTOdic[2]="Unknown status"
        gGTOdic[3]="Unknown status"
        
        #gACT
        gACTdic=self.registerDic["gACT"]
        
        gACTdic[0]="Gripper reset."
        gACTdic[1]="Gripper activation."
        
        #kFLT
        kFLTdic=self.registerDic["kFLT"]
        i=0
        while i<256:
            kFLTdic[i]=i
            i+=1
        
        #See your optional Controller Manual (input registers & status).
        
        #gFLT
        gFLTdic=self.registerDic["gFLT"]
        i=0
        while i<256:
            gFLTdic[i]=i
            i+=1
        gFLTdic[0]="No fault (LED is blue)"
        gFLTdic[5]="Priority faults (LED is blue). Action delayed, activation (reactivation) must be completed prior to perfmoring the action."
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
        gPRdic=self.registerDic["gPR"]
        
        i=0
        while i<256:
            gPRdic[i]="Echo of the requested position for the Gripper:{}/255".format(i)
            i+=1
        
        #gPO
        gPOdic=self.registerDic["gPO"]
        i=0
        while i<256:
            gPOdic[i]="Actual position of the Gripper obtained via the encoders:{}/255".format(i)
            i+=1
        
        #gCU
        gCUdic=self.registerDic["gCU"]
        i=0
        while i<256:
            current=i*10
            gCUdic[i]="The current is read instantaneously from the motor drive, approximate current: {} mA".format(current)
            i+=1
    
        ######################################################################
        #output register variable
        self.registerDic.update({"rARD":{},
                                 "rATR":{},
                                 "rGTO":{},
                                 "rACT":{},
                                 "rPR":{},
                                 "rFR":{},
                                 "rSP":{}})
        
        ######################################################################
     
    def _saveStatus(self,registers):
        """Save the gripper status register values in the gripper status dictionary.

        The dictionary keys
        -------------------
        gOBJ:
            Object detection status. This built-in feature provides\
            information on possible object pick-up. Ignore if gGTO == 0.
        gSTA:
            Gripper status. Returns the current status and motion of the\
            gripper fingers.
        gGTO:
            Action status. Echo of the rGTO bit (go-to bit).
        gACT:
            Activation status. Echo of the rACT bit (activation bit).
        kFLT:
            See your optional controller manual for input registers and\
            status.
        gFLT:
            Fault status. Returns general error messages useful for\
            troubleshooting. A fault LED (red) is present on the gripper\
            chassis. The LED can be blue, red, or both, and can be solid\
            or blinking.
        gPR:
            Echo of the requested position for the gripper. Value between\
            0x00 and 0xFF.
        gPO:
            Actual position of the gripper obtained via the encoders.\
            Value between 0x00 and 0xFF.
        gCU:
            The current is read instantaneously from the motor drive. Value\
            between 0x00 and 0xFF. Approximate current equivalent is 10 times\
            the value read in mA.
        """
        now=time.monotonic()
        self.status["time"]=now
        #########################################
        #Register 2000
        #First Byte: gripperStatus
        #Second Byte: RESERVED
        
        #First Byte: gripperStatus
        gripperStatusReg0=(registers[0] >> 8) & 0b11111111 #xxxxxxxx00000000
        #########################################
        #Object detection
        self.status["gOBJ"]=(gripperStatusReg0 >> 6) & 0b11 #xx000000
        #Gripper status
        self.status["gSTA"]=(gripperStatusReg0 >> 4) & 0b11 #00xx0000
        #Action status. echo of rGTO (go to bit)
        self.status["gGTO"]=(gripperStatusReg0 >> 3) & 0b1 #0000x000
        #Activation status
        self.status["gACT"]=gripperStatusReg0 & 0b00000001 #0000000x
        
        #########################################
        #Register 2001
        #First Byte: Fault status
        #Second Byte: Pos request echo
        
        #First Byte: fault status
        faultStatusReg2= (registers[1] >>8)  & 0b11111111 #xxxxxxxx00000000
        #########################################
        #Universal controler
        self.status["kFLT"]=(faultStatusReg2 >> 4) & 0b1111 #xxxx0000
        #Fault
        self.status["gFLT"]=faultStatusReg2 & 0b00001111 #0000xxxx
        
        
        #########################################
        #Second Byte: Pos request echo
        posRequestEchoReg3=registers[1] & 0b11111111 #00000000xxxxxxxx
        #########################################
        #Echo of request position
        self.status["gPR"]=posRequestEchoReg3
        
        #########################################
        #Register 2002
        #First Byte: Position
        #Second Byte: Current
        
        #First Byte: Position
        positionReg4=(registers[2] >> 8) & 0b11111111 #xxxxxxxx00000000

        #########################################
        #Actual position of the gripper
        self.status["gPO"]=positionReg4
        
        #########################################
        #Second Byte: Current
        currentReg5=registers[2] & 0b0000000011111111 #00000000xxxxxxxx
        #########################################
        #Current
        self.status["gCU"]=currentReg5

    def readStatus(self):
        """Retrieve gripper output register information and save it in the\
        parameter dictionary.
        """
        #Read 3 16bits registers starting from register 2000
        registers=self._client.read_input_registers(2000,count=3,device_id=self.device_id).registers

        self._saveStatus(registers)
    
    def reset(self):
        """Reset the gripper (clear previous activation if any)
        """
        #Reset the gripper
        self._client.write_registers(1000,[0,0,0],device_id=self.device_id)
    
    def activate(self):
        """If not already activated, activate the gripper.

        .. warning::
            When you execute this function the gripper is going to fully open\
            and close. During this operation the gripper must be able to freely\
            move. Do not place object inside the gripper.
        """
        #Turn the variable which indicate that the gripper is processing
        #an action to True
        self.processing=True

        #Activate the gripper
        #rACT=1 Activate Gripper (must stay on after activation routine is
        #completed).
        self._client.write_registers(1000,[0b0000100100000000,0,0],device_id=self.device_id)

        #Waiting for activation to complete
        activationStartTime=time.time()
        activationCompleted=False
        activationTime=0
        
        while (not activationCompleted) and activationTime < self.timeOut:
            activationTime = time.time() - activationStartTime
            
            self.readStatus()
            gSTA=self.status["gSTA"]

            if gSTA==3:
                activationCompleted=True
                print("Activation completed. Activation time : "
                      , activationTime)
        if activationTime > self.timeOut:
            raise GripperTimeoutError("Activation", self.timeOut)

        self.processing=False
    
    def resetActivate(self):
        """Reset the gripper (clear previous activation if any) and activat\
        the gripper. During this operation the gripper will open and close.
        """
        #Reset the gripper
        self.reset()
        #Activate the gripper
        self.activate()
    
    def move(self,position,speed=255,force=255,wait=False):
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
        position=int(position)
        speed=int(speed)
        force=int(force)
        
        
        #Check if the grippre is activated
        if self.isActivated == False:
            raise GripperNotActivatedError()

        #Check input value
        if position>255 or position<0:
            raise GripperPositionError(position)
        
        self.processing=True
        

        #rARD(5) rATR(4) rGTO(3) rACT(0)
        #gACT=1 (Gripper activation.) and gGTO=1 (Go to Position Request.)
        self._client.write_registers(1000,[0b0000100100000000,
                                    position,
                                    speed * 0b100000000 + force],
                                    device_id=self.device_id)
        
        #Waiting for activation to complete
        motionStartTime=time.time()
        motionCompleted=False
        motionTime=0
        objectDetected=False

        if wait:
            while (not objectDetected) and (not motionCompleted)\
                and (motionTime<self.timeOut):

                motionTime= time.time()- motionStartTime
                self.readStatus()
                #Object detection status, is a built-in feature that provides
                #information on possible object pick-up. Ignore if gGTO == 0.
                gOBJ=self.status["gOBJ"]

                
                if gOBJ==1 or gOBJ==2: 
                    #Fingers have stopped due to a contact
                    objectDetected=True
                
                elif gOBJ==3:
                    #Fingers are at requested position.
                    objectDetected=False
                    motionCompleted=True
        
        if motionTime>self.timeOut:
            raise GripperTimeoutError("Motion", self.timeOut)
        
        position=self.status["gPO"]
        
    def close(self,speed=255,force=255):
        """Close the gripper.

        Parameters:
        -----------
        speed : int
            Gripper speed between 0 and 255. Default is 255.
        force : int
            Gripper force between 0 and 255. Default is 255.
        """
        self.move(255,speed,force)
    
    def open(self,speed=255,force=255):
        """Open the gripper
        
        Parameters:
        -----------
        speed : int
            Gripper speed between 0 and 255. Default is 255.
        force : int
            Gripper force between 0 and 255. Default is 255.
        """
        self.move(0,force,speed)
    
    def move_mm(self,positionmm,speed=255,force=255):
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
        if self.isCalibrated == False:
            raise GripperNotCalibratedError()

        if  positionmm>self._openmm:
            raise Exception("The maximum opening is {}".format(self._openmm))
        
        position=int(self._mmToBit(positionmm))
        self.move(position,speed,force)
        
    def getPosition(self):
        """Return the position of the gripper in bits

        Returns:
        --------
        position : int
            Position of the gripper in bits.
        """
        self.readStatus()

        position=self.status["gPO"]

        return position
    
    def _mmToBit(self,mm):
        """Convert a mm gripper opening in bit opening.

        .. note::
            Calibration is needed to use this function.\n
            Execute the function calibrate at least 1 time before using this function.
        """
        bit=(mm-self._bCoef)/self._aCoef
        
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
        mm=self._aCoef*bit+self._bCoef
        
        return mm
    
    def getPositionmm(self):
        """Return the position of the gripper in mm.

        Returns:
        --------
        positionmm : float
            Current gripper position in mm
        
        .. note::
            Calibration is needed to use this function.\n
            Execute the function calibrate at least 1 time before using this function.
        """
        position=self.getPosition()
        
        positionmm=self._bitTomm(position)
        return positionmm
    
    def calibrate(self,closemm,openmm):
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
        self._closemm=closemm
        self._openmm=openmm
        
        self.open()
        #get open bit
        self._openbit=self.getPosition()
        obit=self._openbit
        
        self.close()
        #get close bit
        self._closebit=self.getPosition()
        cbit=self._closebit
        
        self._aCoef=(closemm-openmm)/(cbit-obit)
        self._bCoef=(openmm*cbit-obit*closemm)/(cbit-obit)
    
    def printStatus(self):
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
        self.readStatus()
        
        # Print header
        print("\n" + "=" * 70)
        print(" " * 20 + "GRIPPER STATUS")
        print("=" * 70)
        
        # Find the longest key for alignment
        max_key_length = max(len(key) for key in self.status.keys() if key != "time")
        
        # Print each status item with description
        for key, value in self.status.items():
            if key != "time":
                # Print key and value with alignment
                print(f"\n{key:<{max_key_length}} : {value}")
                # Print description with indentation
                description = self.registerDic[key][value]
                print(f"  └─ {description}")
        
        print("\n" + "=" * 70 + "\n")

    def isActivated(self):
        """Tells if the gripper is activated
        
        Returns:
        --------
        is_activated : bool
            True if the gripper is activated. False otherwise.
        """
        
        self.readStatus()
        is_activated = (self.status["gSTA"]==3)

        return is_activated
    
    def isCalibrated(self):
        """Return if the gripper is qualibrated

        Returns:
        --------
        is_calibrated : bool
            True if the gripper is calibrated. False otherwise.
        """
        is_calibrated = False
        if (self._openmm is None) or (self._closemm is None):
            is_calibrated = False
        else:
            is_calibrated=True
        
        return is_calibrated
    
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

        #Calculate predicted position
        positionDelta = requestedPosition - startPosition
        
        direction = sign(positionDelta)

        motion = int(direction * float(self.gripper_vmin + (self.gripper_vmax - self.gripper_vmin) * speed / 255) * elapsedTime)
        
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
        bitPerSecond=GRIPPER_2F_VMIN + (float(self.gripper_vmax-self.gripper_vmin)/255)*speed
        return bitPerSecond
    
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
    
    def _estimatedObjectDetection(self):
        """Estimate if an object is detected based on the gripper command history.
        
        Returns:
        --------
        objectDetection : int
            0: No object detected
            1: Object detected while opening
            2: Object detected while closing
        """
        objectDetection=NO_OBJECT_DETECTED

        #If position is identical for more than 20x0.016s (time to move of 20 bits at\
        # slow speed)
        timeThresholdId=listIdValueUnderThreshold(self.commandHistory["time"],
                                                  self.commandHistory["time"][0]-COM_TIME*25)
        if timeThresholdId<2:
            timeThresholdId=2
        
        isImobile = areValueIdentical(self.commandHistory["position"][:timeThresholdId])
        if isImobile:

            if (min(self.commandHistory["positionCommand"][:timeThresholdId]) - self.commandHistory["position"][0]) > 0:
                objectDetection=OBJECT_DETECTED_WHILE_CLOSING
            elif (max(self.commandHistory["positionCommand"][:timeThresholdId]) - self.commandHistory["position"][0] )<0:
                objectDetection=OBJECT_DETECTED_WHILE_OPENING
            else:
                objectDetection=NO_OBJECT_DETECTED
        
        return objectDetection
    

    def _commandFilter(self,
                       t0_RequestTime,
                       t0_RequestPosition,
                       commandHistory,
                       status,
                       minSpeedPosDelta=5,
                       maxSpeedPosDelta=55,
                       continuousGrip=True,
                       autoLock=True,
                       minimalMotion=1):
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

        Returns:
        --------
        command : dict
            Filtered command.
        """

        #Object detection
        t1_CommandDetection =NO_OBJECT_DETECTED
        
        if (status is None) :
            t1_CommandDetection = self._estimatedObjectDetection(commandHistory)
        elif (status["time"]<commandHistory["time"][0]):
            t1_CommandDetection = self._estimatedObjectDetection(commandHistory)
        else:        
            #print("Detection from status")
            t1_CommandDetection = status["gOBJ"]

        elapsedTime = t0_RequestTime-commandHistory["time"][0]
        forceMin = continuousGrip*1
        command = {}

        command["execution"]=NO_COMMAND
        command["position"]=0
        command["speed"]=0
        command["force"]=forceMin
        command["grip"]=GRIP_NOT_REQUESTED
        command["wait"]=0
        
        t0_CalculatedPosition = self._positionEstimation(commandHistory["position"][0],commandHistory["positionCommand"][0],commandHistory["speedCommand"][0], elapsedTime)

        if (t1_CommandDetection == OBJECT_DETECTED_WHILE_OPENING) and autoLock:

            #An object have been detected during opening
            if commandHistory["gripCommand"][0]==GRIP_NOT_REQUESTED:
                #The gripper has not been request to grip
                if (t0_RequestPosition <= commandHistory["position"][0]):
                    #Secure the grip
                    command["execution"]=WRITE_READ_COMMAND
                    command["position"]=0
                    command["speed"]=255
                    command["force"]=255
                    command["grip"]=GRIP_REQUESTED
                    command["wait"]=self._travelTime(0,10,GRIPPER_VMAX)
                else:
                    #Release
                    command["execution"]=WRITE_READ_COMMAND
                    command["position"]=t0_RequestPosition
                    command["speed"]=255
                    command["force"]=255
                    command["grip"]=GRIP_NOT_REQUESTED
                    command["wait"]=self._travelTime(t0_CalculatedPosition,t0_RequestPosition,255)
            elif commandHistory["gripCommand"][0]==GRIP_REQUESTED:
                #The gripper has been requested to grip. Final grip position is unknown.
                if t0_RequestPosition <= commandHistory["position"][0]:
                    #The position is inside the grip
                    #Validate the grip
                    command["execution"]=READ_COMMAND
                    command["position"]=None
                    command["speed"]=None
                    command["force"]=None
                    command["grip"]=None
                    command["wait"]=None
                else:
                    #The position is ouside the grip
                    #Release
                    command["execution"]=WRITE_READ_COMMAND
                    command["position"]=t0_RequestPosition
                    command["speed"]=255
                    command["force"]=255
                    command["grip"]=GRIP_NOT_REQUESTED
                    command["wait"]=self._travelTime(t0_CalculatedPosition,t0_RequestPosition,255)
            else:
                #GRIP_VALIDATED

                if t0_RequestPosition <= commandHistory["position"][0]:
                    #The position is inside the grip
                    #Validate the grip
                    command["execution"]=READ_COMMAND
                    command["position"]=0
                    command["speed"]=0
                    command["force"]=0
                    command["grip"]=0
                    command["wait"]=0
                else:
                    #The position is ouside the grip
                    #Release
                    command["execution"]=WRITE_READ_COMMAND
                    command["position"]=t0_RequestPosition
                    command["speed"]=255
                    command["force"]=255
                    command["grip"]=0
                    command["wait"]=self._travelTime(commandHistory["position"][0],t0_RequestPosition,t0_Speed)

        elif t1_CommandDetection == OBJECT_DETECTED_WHILE_OPENING and autoLock:
            #print("Object detected while closing")
            #An object have been detected during closing
            if commandHistory["gripCommand"][0]==GRIP_NOT_REQUESTED:
                #The gripper has not been request to grip
                if t0_RequestPosition >= commandHistory["position"][0]:
                    #Secure the grip
                    command["execution"]=WRITE_READ_COMMAND
                    command["position"]=255
                    command["speed"]=255
                    command["force"]=255
                    command["grip"]=GRIP_REQUESTED
                    command["wait"]=self._travelTime(0,10,GRIPPER_VMAX)
                else:
                    #Release
                    command["execution"]=WRITE_READ_COMMAND
                    command["position"]=t0_RequestPosition
                    command["speed"]=255
                    command["force"]=255
                    command["grip"]=GRIP_NOT_REQUESTED
                    command["wait"]=self._travelTime(t0_CalculatedPosition,t0_RequestPosition,255)
            elif commandHistory["gripCommand"][0]==GRIP_REQUESTED:
                #The gripper has been requested to grip. Final grip position is unknown.
                if t0_RequestPosition >= commandHistory["position"][0]:
                    #The position is inside the grip
                    #Validate the grip
                    command["execution"]=READ_COMMAND
                    command["position"]=None
                    command["speed"]=None
                    command["force"]=None
                    command["grip"]=None
                    command["wait"]=None
                else:
                    #The position is ouside the grip
                    #Release
                    command["execution"]=WRITE_READ_COMMAND
                    command["position"]=t0_RequestPosition
                    command["speed"]=255
                    command["force"]=255
                    command["grip"]=0
                    command["wait"]=self._travelTime(t0_CalculatedPosition,t0_RequestPosition,255)
            else:
                #GRIP_VALIDATED

                if t0_RequestPosition >= commandHistory["position"][0]:
                    #The position is inside the grip
                    #Validate the grip
                    command["execution"]=READ_COMMAND
                    command["position"]=0
                    command["speed"]=0
                    command["force"]=0
                    command["grip"]=0
                    command["wait"]=0
                else:
                    #The position is ouside the grip
                    #Release
                    command["execution"]=WRITE_READ_COMMAND
                    command["position"]=t0_RequestPosition
                    command["speed"]=255
                    command["force"]=255
                    command["grip"]=0
                    command["wait"]=self._travelTime(t0_CalculatedPosition,t0_RequestPosition,255)

        else:
            if abs(commandHistory["positionCommand"][0]-t0_RequestPosition)>minimalMotion:
                if t0_RequestPosition==0 or t0_RequestPosition==255:
                    command["execution"]=WRITE_READ_COMMAND
                    command["position"]=t0_RequestPosition
                    command["speed"]=255
                    command["force"]=255
                    command["grip"]=GRIP_NOT_REQUESTED
                    command["wait"]=self._travelTime(t0_CalculatedPosition,t0_RequestPosition,255)
                else:
                    #Move only if request is distant of 2 bits to avoid having the gripper checking between 2 positions.

                    #t0_ speed calculation
                    t0_Speed = 0
                    posDelta = abs(t0_RequestPosition - t0_CalculatedPosition)
                    if posDelta <= minSpeedPosDelta:
                        #Requested position is close from current position. The speed is slow.
                        t0_Speed = 0
                    elif posDelta > maxSpeedPosDelta:
                        #Requested position is fare from the current position. The speed is fast.
                        t0_Speed = 255
                    else:
                        #Request is a bit distant. The speed increase with the distance between current position and requested position.
                        t0_Speed = int((float(posDelta - minSpeedPosDelta) /(maxSpeedPosDelta - minSpeedPosDelta)) * 255)
                    if (commandHistory["positionCommand"][0] == t0_RequestPosition) and (commandHistory["speedCommand"][0] == t0_Speed) and (commandHistory["forceCommand"][0] == t0_Force):
                        #t1_ command was identical as t0_ command. We do nothing.
                        command["execution"]=READ_COMMAND
                        command["position"]=None
                        command["speed"]=None
                        command["force"]=None
                        command["grip"]=None
                        command["wait"]=None
                    else:
                        command["execution"]=WRITE_READ_COMMAND
                        command["position"]=t0_RequestPosition
                        command["speed"]=t0_Speed
                        command["force"]=forceMin
                        command["grip"]=GRIP_NOT_REQUESTED
                        command["wait"]=0#travelTime(0,2,t0_Speed)
            else:
                command["execution"]=READ_COMMAND
                command["position"]=None
                command["speed"]=None
                command["force"]=None
                command["grip"]=None
                command["wait"]=None
        
        return command

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

        self._saveStatus(registers)
    
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
            print("Write failed")
    
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
            print("Write failed")
    
    def waitComplete(self, timeout=5.0):
        """Wait until the gripper has completed its motion or detect an object."""
        gOBJ=0b00
        startTime = time.time()
        while gOBJ == 0b00 and (time.time() - startTime) < self.timeOut:
            result=self.read_holding_registers(address=2000,
                                               count=1,
                                               device_id=self.device_id)
            registers=result.registers
            gripperStatusReg0=(registers[0] >> 8) & 0b11111111
            gOBJ=(gripperStatusReg0 >> 6) & 0b11
            
#Test
if False:
    grip=RobotiqGripper("COM8")
    grip.resetActivate()
    grip.readwrite_registers(read_address=2000,
                                        read_count=3,
                                        write_address=1001,
                                        values=[0, 255 * 0b100000000 + 255],
                                        device_id=grip.device_id)
    time.sleep(5)
    grip.readwrite_registers(read_address=2000,
                                        read_count=3,
                                        write_address=1001,
                                        values=[255, 255 * 0b100000000 + 255],
                                        device_id=grip.device_id)
    time.sleep(5)
    """
    grip.resetActivate()
    grip.reset()
    grip.goTo(0)
    grip.goTo(255)
    grip.printInfo()
    grip.goTo(255)
    time.sleep(5)
    grip.printInfo()
    grip.activate()
    grip.printInfo()
    
    grip.goTo(20)
    grip.goTo(230)
    grip.goTo(40)
    grip.goTo(80)
    
    grip.calibrate(0,36)
    grip.goTomm(10,255,255)
    grip.goTomm(40,1,255)
    """