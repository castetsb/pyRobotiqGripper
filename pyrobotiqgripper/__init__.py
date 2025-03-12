"""pyRobotiqGripper: Python Driver for Robotiq Grippers via Modbus RTU

pyRobotiqGripper is a Python library designed to facilitate control of Robotiq\
grippers using Modbus RTU communication via serial port.

This module provides documentation in two formats:

- Docstrings: Embedded within the code for easy access.
- Online Documentation: Extensive documentation available at\
    <https://pyrobotiqgripper.readthedocs.io/en/latest/>.
"""

#General information
__author__  = "Benoit CASTETS"
__email__   = "opensourceeng@robotiq.com"
__license__ = "Apache License, Version 2.0"
__url__ = "https://github.com/castetsb/pyRobotiqGripper"
__version__ = "1.0.0"

#Iport libraries
import minimalmodbus as mm
import time
import serial
import serial.tools.list_ports

#Constants
BAUDRATE=115200
BYTESIZE=8
PARITY="N"
STOPBITS=1
TIMEOUT=0.2
AUTO_DETECTION="auto"

class RobotiqGripper( mm.Instrument ):
    """Object control Robotiq grippers (2F85, 2F140 or hande).

    Suppose that the gripper is connected via the USB/RS485 adapter to the PC\
    executing this code.

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
        This object cannot be use to control epick, 3F or powerpick.
    """    
    
    def __init__(self, portname=AUTO_DETECTION,slaveAddress=9):
        """Create a RobotiqGripper object whic can be use to control Robotiq\
        grippers using modbus RTU protocol USB/RS485 connection.
        
        Args:
            - portname (str, optional): The serial port name, for example\
                /dev/ttyUSB0 (Linux), /dev/tty.usbserial (OS X) or COM4\
                (Windows). It is necesary to allowpermission to access this\
                connection using the bash comman sudo chmod 666 /dev/ttyUSB0.\
                By default the portname is set to "auto". In this case the\
                connection is done with the first gripper found as connected\
                to the PC.
            - slaveaddress (int, optional): Address of the gripper (integer)\
                usually 9.
        """
        #Gripper salve address
        self.slaveAddress=slaveAddress

        #Port on which is connected the gripper
        if portname == "auto":
            self.portname=self._autoConnect()
            if self.portname is None:
                raise Exception("No gripper detected")
        else:
            self.portname=portname
        
        #Create a pyserial object to connect to the gripper
        ser=serial.Serial(self.portname,
                          BAUDRATE,
                          BYTESIZE,
                          PARITY,
                          STOPBITS,
                          TIMEOUT)

        #Create the object using parent class contructor
        super().__init__(ser,
                         self.slaveAddress,
                         mm.MODE_RTU,
                         close_port_after_each_call=False,
                         debug=False)
        
        #Attribute to monitore if the gripper is processing an action
        self.processing=False
        
        #Maximum allowed time to perform and action
        self.timeOut=10
        
        #Dictionnary where are stored description of each register state
        self.registerDic={}
        self._buildRegisterDic()
        

        #Dictionnary where are stored register values retrived from the gripper
        self.paramDic={}
        self.readAll()
        
        #Attributes to store open and close distance state information
        
        #Distance between the fingers when gripper is closed
        self.closemm=None
        #Position in bit when gripper is closed
        self.closebit=None
        
        #Distance between the fingers when gripper is open
        self.openmm=None
        #Position in bit when gripper is open
        self.openbit=None
        
        self._aCoef=None
        self._bCoef=None
    
    def _autoConnect(self):
        """Return the name of the port on which is connected the gripper
        """
        ports=serial.tools.list_ports.comports()
        portName=None

        for port in ports:
            try:
                # Try opening the port
                ser = serial.Serial(port.device,BAUDRATE,BYTESIZE,PARITY,STOPBITS,TIMEOUT)

                device=mm.Instrument(ser,self.slaveAddress,mm.MODE_RTU,close_port_after_each_call=False,debug=False)

                #Try to write the position 100
                device.write_registers(1000,[0,100,0])

                #Try to read the position request eco
                registers=device.read_registers(2000,3,4)
                posRequestEchoReg3=registers[1] & 0b0000000011111111

                #Check if position request eco reflect the requested position
                if posRequestEchoReg3 != 100:
                    raise Exception("Not a gripper")
                portName=port.device
                del device

                ser.close()  # Close the port
            except:
                pass  # Skip if port cannot be opened
    
        # If no suitable port is found
        return portName

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
        
        gOBJdic[0]="Fingers are in motion towards requested position. No\
            object detected."
        gOBJdic[1]="Fingers have stopped due to a contact while opening before\
            requested position. Object detected opening."
        gOBJdic[2]="Fingers have stopped due to a contact while closing before\
            requested position. Object detected closing."
        gOBJdic[3]="Fingers are at requested position. No object detected or\
            object has been loss / dropped."
        
        #gSTA
        gSTAdic=self.registerDic["gSTA"]
        
        gSTAdic[0]="Gripper is in reset ( or automatic release ) state. See\
            Fault Status if Gripper is activated."
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
        gFLTdic[5]="Priority faults (LED is blue). Action delayed, activation\
            (reactivation) must be completed prior to perfmoring the action."
        gFLTdic[7]="Priority faults (LED is blue). The activation bit must be\
            set prior to action."
        gFLTdic[8]="Minor faults (LED continuous red). Maximum operating\
            temperature exceeded, wait for cool-down."
        gFLTdic[9]="Minor faults (LED continuous red). No communication during\
            at least 1 second."
        gFLTdic[10]="Major faults (LED blinking red/blue) - Reset is required\
            (rising edge on activation bit rACT needed). Under minimum\
            operating voltage."
        gFLTdic[11]="Major faults (LED blinking red/blue) - Reset is required\
            (rising edge on activation bit rACT needed). Automatic release in\
            progress."
        gFLTdic[12]="Major faults (LED blinking red/blue) - Reset is required\
            (rising edge on activation bit rACT needed). Internal fault;\
            contact support@robotiq.com."
        gFLTdic[13]="Major faults (LED blinking red/blue) - Reset is required\
            (rising edge on activation bit rACT needed). Activation fault,\
            verify that no interference or other error occurred."
        gFLTdic[14]="Major faults (LED blinking red/blue) - Reset is required\
            (rising edge on activation bit rACT needed). Overcurrent triggered."
        gFLTdic[15]="Major faults (LED blinking red/blue) - Reset is required\
            (rising edge on activation bit rACT needed). Automatic release\
            completed."
        
        #gPR
        gPRdic=self.registerDic["gPR"]
        
        i=0
        while i<256:
            gPRdic[i]="Echo of the requested position for the Gripper:\
                {}/255".format(i)
            i+=1
        
        #gPO
        gPOdic=self.registerDic["gPO"]
        i=0
        while i<256:
            gPOdic[i]="Actual position of the Gripper obtained via the encoders:\
                {}/255".format(i)
            i+=1
        
        #gCU
        gCUdic=self.registerDic["gCU"]
        i=0
        while i<256:
            current=i*10
            gCUdic[i]="The current is read instantaneously from the motor\
                drive, approximate current: {} mA".format(current)
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
     
    def readAll(self):
        """Retrieve gripper output register information and save it in the\
            parameter dictionary.

        The dictionary keys are as follows:

        - gOBJ: Object detection status. This built-in feature provides\
            information on possible object pick-up. Ignore if gGTO == 0.
        - gSTA: Gripper status. Returns the current status and motion of the\
            gripper fingers.
        - gGTO: Action status. Echo of the rGTO bit (go-to bit).
        - gACT: Activation status. Echo of the rACT bit (activation bit).
        - kFLT: See your optional controller manual for input registers and\
            status.
        - gFLT: Fault status. Returns general error messages useful for\
            troubleshooting. A fault LED (red) is present on the gripper\
            chassis. The LED can be blue, red, or both, and can be solid\
            or blinking.
        - gPR: Echo of the requested position for the gripper. Value between\
            0x00 and 0xFF.
        - gPO: Actual position of the gripper obtained via the encoders.\
            Value between 0x00 and 0xFF.
        - gCU: The current is read instantaneously from the motor drive. Value\
            between 0x00 and 0xFF. Approximate current equivalent is 10 times\
            the value read in mA.
        """
        #Clear parameter dictionnary data
        self.paramDic={}
        
        #Read 3 16bits registers starting from register 2000
        registers=self.read_registers(2000,3)
        
        #########################################
        #Register 2000
        #First Byte: gripperStatus
        #Second Byte: RESERVED
        
        #First Byte: gripperStatus
        gripperStatusReg0=(registers[0] >> 8) & 0b11111111 #xxxxxxxx00000000
        #########################################
        #Object detection
        self.paramDic["gOBJ"]=(gripperStatusReg0 >> 6) & 0b11 #xx000000
        #Gripper status
        self.paramDic["gSTA"]=(gripperStatusReg0 >> 4) & 0b11 #00xx0000
        #Action status. echo of rGTO (go to bit)
        self.paramDic["gGTO"]=(gripperStatusReg0 >> 3) & 0b1 #0000x000
        #Activation status
        self.paramDic["gACT"]=gripperStatusReg0 & 0b00000001 #0000000x
        
        #########################################
        #Register 2001
        #First Byte: Fault status
        #Second Byte: Pos request echo
        
        #First Byte: fault status
        faultStatusReg2= (registers[1] >>8)  & 0b11111111 #xxxxxxxx00000000
        #########################################
        #Universal controler
        self.paramDic["kFLT"]=(faultStatusReg2 >> 4) & 0b1111 #xxxx0000
        #Fault
        self.paramDic["gFLT"]=faultStatusReg2 & 0b00001111 #0000xxxx
        
        
        #########################################
        #Second Byte: Pos request echo
        posRequestEchoReg3=registers[1] & 0b11111111 #00000000xxxxxxxx
        #########################################
        #Echo of request position
        self.paramDic["gPR"]=posRequestEchoReg3
        
        #########################################
        #Register 2002
        #First Byte: Position
        #Second Byte: Current
        
        #First Byte: Position
        positionReg4=(registers[2] >> 8) & 0b11111111 #xxxxxxxx00000000

        #########################################
        #Actual position of the gripper
        self.paramDic["gPO"]=positionReg4
        
        #########################################
        #Second Byte: Current
        currentReg5=registers[2] & 0b0000000011111111 #00000000xxxxxxxx
        #########################################
        #Current
        self.paramDic["gCU"]=currentReg5
    
    def reset(self):
        """Reset the gripper (clear previous activation if any)
        """
        #Reset the gripper
        self.write_registers(1000,[0,0,0])
    
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
        self.write_registers(1000,[0b0000000100000000,0,0])

        #Waiting for activation to complete
        activationStartTime=time.time()
        activationCompleted=False
        activationTime=0
        
        while (not activationCompleted) and activationTime < self.timeOut:
            activationTime = time.time() - activationStartTime
            
            self.readAll()
            gSTA=self.paramDic["gSTA"]

            if gSTA==3:
                activationCompleted=True
                print("Activation completed. Activation time : "
                      , activationTime)
        if activationTime > self.timeOut:
            raise Exception("Activation did not complete without timeout.")

        self.processing=False
    
    def resetActivate(self):
        """Reset the gripper (clear previous activation if any) and activat\
        the gripper. During this operation the gripper will open and close.
        """
        #Reset the gripper
        self.reset()
        #Activate the gripper
        self.activate()
    
    def goTo(self,position,speed=255,force=255):
        """Go to the position with determined speed and force.
        
        Args:
            - position (int): Position of the gripper. Integer between 0 and 255.\
            0 being the open position and 255 being the close position.
            - speed (int): Gripper speed between 0 and 255
            - force (int): Gripper force between 0 and 255
        
        Returns:
            - objectDetected (bool): True if object detected
            - position (int): End position of the gripper in bits
        """
        position=int(position)
        speed=int(speed)
        force=int(force)
        
        
        #Check if the grippre is activated
        if self.isActivated == False:
            raise Exception ("Gripper must be activated before requesting\
                             an action.")

        #Check input value
        if position>255:
            raise Exception("Position value cannot exceed 255")
        elif position<0:
            raise Exception("Position value cannot be under 0")
        
        self.processing=True
        

        #rARD(5) rATR(4) rGTO(3) rACT(0)
        #gACT=1 (Gripper activation.) and gGTO=1 (Go to Position Request.)
        self.write_registers(1000,[0b0000100100000000,
                                    position,
                                    speed * 0b100000000 + force])
        
        #Waiting for activation to complete
        motionStartTime=time.time()
        motionCompleted=False
        motionTime=0
        objectDetected=False

        while (not objectDetected) and (not motionCompleted)\
            and (motionTime<self.timeOut):

            motionTime= time.time()- motionStartTime
            self.readAll()
            #Object detection status, is a built-in feature that provides
            #information on possible object pick-up. Ignore if gGTO == 0.
            gOBJ=self.paramDic["gOBJ"]

            
            if gOBJ==1 or gOBJ==2: 
                #Fingers have stopped due to a contact
                objectDetected=True
            
            elif gOBJ==3:
                #Fingers are at requested position.
                objectDetected=False
                motionCompleted=True
        
        if motionTime>self.timeOut:
            raise Exception("Gripper never reach its requested position and\
                            no object have been detected")
        
        position=self.paramDic["gPO"]

        return position, objectDetected
        
    def close(self,speed=255,force=255):
        """Close the gripper.

        Args:
            - speed (int, optional): Gripper speed between 0 and 255.\
            Default is 255.
            - force (int, optional): Gripper force between 0 and 255.\
            Default is 255.
        """
        self.goTo(255,speed,force)
    
    def open(self,speed=255,force=255):
        """Open the gripper
        
        Args:
            - speed (int, optional): Gripper speed between 0 and 255.\
            Default is 255.
            - force (int, optional): Gripper force between 0 and 255.\
            Default is 255.
        """
        self.goTo(0,force,speed)
    
    def goTomm(self,positionmm,speed=255,force=255):
        """Go to the requested opening expressed in mm

        Args:
            - positionmm (float): Gripper opening in mm.
            - speed (int, optional): Gripper speed between 0 and 255.\
            Default is 255.
            - force (int, optional): Gripper force between 0 and 255.\
            Default is 255.
        
        .. note::
            Calibration is needed to use this function.\n
            Execute the function calibrate at least 1 time before using this function.
        """
        if self.isCalibrated == False:
            raise Exception("The gripper must be calibrated before been requested to go to a position in mm")

        if  positionmm>self.openmm:
            raise Exception("The maximum opening is {}".format(self.openmm))
        
        position=int(self._mmToBit(positionmm))
        self.goTo(position,speed,force)
        
    def getPosition(self):
        """Return the position of the gripper in bits

        Returns:
            - int: Position of the gripper in bits.
        """
        self.readAll()

        position=self.paramDic["gPO"]

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
            float: Gripper position converted in mm
        
        .. note::
            Calibration is needed to use this function.\n
            Execute the function calibrate at least 1 time before using this function.
        """
        mm=self._aCoef*bit+self._bCoef
        
        return mm
    
    def getPositionmm(self):
        """Return the position of the gripper in mm.

        Returns:
            float: Current gripper position in mm
        
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

        Args:
            - closemm (float): Distance between the fingers when the gripper is\
            fully closed.
            - openmm (float): Distance between the fingers when the gripper is\
            fully open.
        """
        self.closemm=closemm
        self.openmm=openmm
        
        self.open()
        #get open bit
        self.openbit=self.getPosition()
        obit=self.openbit
        
        self.close()
        #get close bit
        self.closebit=self.getPosition()
        cbit=self.closebit
        
        self._aCoef=(closemm-openmm)/(cbit-obit)
        self._bCoef=(openmm*cbit-obit*closemm)/(cbit-obit)
    
    def printInfo(self):
        """Print gripper register info in the python terminal
        """
        self.readAll()
        for key,value in self.paramDic.items():
            print("{} : {}".format(key,value))
            print(self.registerDic[key][value])

    def isActivated(self):
        """Tells if the gripper is activated
        
        Returns:
            bool: True if the gripper is activated. False otherwise.
        """
        
        self.readAll()
        is_activated = (self.paramDic["gSTA"]==3)

        return is_activated
    
    def isCalibrated(self):
        """Return if the gripper is qualibrated

        Returns:
            bool: True if the gripper is calibrated. False otherwise.
        """
        is_calibrated = False
        if (self.openmm is None) or (self.closemm is None):
            is_calibrated = False
        else:
            is_calibrated=True
        
        return is_calibrated
            
#Test
if False:
    grip=RobotiqGripper()
    grip.resetActivate()
    #grip.reset()
    #grip.goTo(0)
    #grip.goTo(255)
    #grip.printInfo()
    #grip.goTo(255)
    #time.sleep(5)
    #grip.printInfo()
    #grip.activate()
    #grip.printInfo()
    
    #grip.goTo(20)
    #grip.goTo(230)
    #grip.goTo(40)
    #grip.goTo(80)
    
    #grip.calibrate(0,36)
    #grip.goTomm(10,255,255)
    #grip.goTomm(40,1,255)
