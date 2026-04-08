"""Integration tests for RobotiqGripper with physical hardware.

This test suite is designed to test the RobotiqGripper class with a physically
connected gripper. It tests all public methods and verifies the gripper's
behavior in real-world scenarios.

WARNING: Ensure the gripper is properly connected and can move freely before
running these tests. Do NOT place objects inside the gripper during calibration.

To run these tests:
    python test.py -v
    
Or with specific test:
    python -m pytest test.py::Hardware::test_connect_disconnect -v

Or
    python -m unittest test.Hardware.test_01_connect_disconnect
"""
import traceback
import unittest
import sys
import time
from functools import wraps
from pyrobotiqgripper import RobotiqGripper
import json


def detailed_errors(test_fn):
    @wraps(test_fn)
    def wrapper(self, *args, **kwargs):
        try:
            return test_fn(self, *args, **kwargs)
        except Exception:
            print("\n--- EXCEPTION in test: %s ---" % test_fn.__name__)
            traceback.print_exc()
            raise
    return wrapper
from pyrobotiqgripper.constants import *
from pyrobotiqgripper.exceptions import *
from pyrobotiqgripper.utils import *
import asciichartpy

class Hardware(unittest.TestCase):
    """Test suite for physical RobotiqGripper hardware testing."""

    def run(self, result=None):
        if result is None:
            result = self.defaultTestResult()

        result.startTest(self)
        try:
            try:
                self.setUp()
            except unittest.SkipTest as e:
                result.addSkip(self, str(e))
                return result
            except Exception:
                print("--- Exception in setUp ---")
                traceback.print_exc()
                result.addError(self, sys.exc_info())
                return result

            testMethod = getattr(self, self._testMethodName)
            try:
                testMethod()
            except Exception:
                print(f"--- Exception in test method {self._testMethodName} ---")
                traceback.print_exc()
                result.addError(self, sys.exc_info())

            try:
                self.tearDown()
            except Exception:
                print("--- Exception in tearDown ---")
                traceback.print_exc()
                result.addError(self, sys.exc_info())

        finally:
            result.stopTest(self)

        return result

    @classmethod
    def setUpClass(cls):
        """Set up the test suite with a gripper connection."""
        print("\n" + "="*70)
        print("HARDWARE TEST")
        print("="*70)
        print("Ensuring gripper is connected and powered on...")
        
        cls.gripper = RobotiqGripper(
            com_port=AUTO_DETECTION,  # Auto-detect COM port
            device_id=9,
            connection_type=GRIPPER_MODE_RTU,
            debug=False
        )
        
        try:
            cls.gripper.connect()
            print("Connected to gripper")
        except GripperConnectionError as e:
            raise unittest.SkipTest(f"Cannot connect to gripper: {e}")
        
        #cls.gripper.reset()
        cls.gripper.activate()

    @classmethod
    def tearDownClass(cls):
        """Clean up after all tests."""
        try:
            cls.gripper.disconnect()
            print("Disconnected from gripper")
        except:
            pass

    def setUp(self):
        """Reset gripper before each test."""
        try:
            self.gripper.connect()
            self.gripper.activate()
            self.gripper.start()
        except Exception as e:
            traceback.print_exc()
            raise unittest.SkipTest(f"Skipping test due to failed setup (activate/start): {e}")
            

    def test_01_connect_disconnect(self):
        """Test connection and disconnection to gripper."""
        print("\nTest: Connection & Disconnection")
        
        # Gripper should remain connected from setUpClass
        self.assertIsNotNone(self.gripper._client)
        
        # Test disconnect and reconnect
        self.gripper.disconnect()
        print("  - Disconnected from gripper")
        time.sleep(0.5)
        
        self.gripper.connect()
        print("  - Reconnected to gripper")
        self.assertIsNotNone(self.gripper._client)

    def test_02_read_status(self):
        """Test reading gripper status."""
        print("\nTest: Read Status")
        

        print(self.gripper.status(refreshStatus=True))
        
        # Verify key status registers exist
        expected_keys = ["gOBJ", "gSTA", "gGTO", "gACT", "kFLT", "gFLT", "gPR", "gPO", "gCU"]
        for key in expected_keys:
            self.assertIn(key, self.gripper.status(), f"Status key '{key}' missing")
        
        print(f"  - Status read successfully: {len(self.gripper.status())} registers")

    def test_03_reset(self):
        """Test gripper reset."""
        print("\nTest: Reset")
        
        self.gripper.reset()
        self.gripper.readStatus()

        self.assertEqual(self.gripper.status()["gACT"],GACT_RESET, "Gripper should be deactivated after reset")
        
        # After reset, gripper should not be activated
        print("  - Gripper reset successfully")

    def test_04_activate(self):
        """Test gripper activation."""
        print("\nTest: Activate")
        
        #self.gripper.reset()
        self.gripper.readStatus()
        gFLT=self.gripper.status()["gFLT"]
        print(REGISTER_DIC["gFLT"][gFLT])
        
        self.gripper.activate()

        print("last status")
        print(self.gripper.status(refreshStatus=True))

        self.assertEqual(self.gripper._commandHistory[-1,RACT],RACT_ACTIVATE,"The command registered in the command history does not show activation")

        self.assertEqual(self.gripper._statusHistory[-1,GACT],GACT_ACTIVATE, "Gripper should be activated after activate()")
        
        self.assertTrue(self.gripper.isActivated(), "Gripper should be activated after activate()")

        # Check if gripper is activated
        is_activated = self.gripper.isActivated()
        self.assertTrue(is_activated, "Gripper should be activated")
        print("  - Gripper activated successfully")

    def test_06_is_activated(self):
        """Test isActivated() method."""
        print("\nTest: Is Activated Check")
        
        self.gripper.activate()
        
        is_activated = self.gripper.isActivated()
        self.assertTrue(is_activated, "Gripper should be reported as activated")
        print(f"  - Gripper activation status: {is_activated}")

    def test_07_open(self):
        """Test opening the gripper."""
        print("\nTest: Open Gripper")
        
        self.gripper.activate()
        self.gripper.start()
        
        self.gripper.open(speed=255, force=255, wait=True)
        time.sleep(0.5)
        
        position = self.gripper.position()
        print(f"  - Gripper opened to position: {position} bits")
        # Fully open position should be close to 0
        self.assertLess(position, 50, "Gripper should be mostly open")

    def test_08_close(self):
        """Test closing the gripper."""
        print("\nTest: Close Gripper")
        
        self.gripper.activate()
        self.gripper.start()
        self.gripper.close(speed=255, force=255, wait=True)
        time.sleep(0.5)
        
        position = self.gripper.position()
        print(f"  - Gripper closed to position: {position} bits")
        # Fully closed position should be close to 255
        self.assertGreater(position, 200, "Gripper should be mostly closed")

    def test_09_move_to_position(self):
        """Test moving to specific positions."""
        print("\nTest: Move to Specific Position")
        
        self.gripper.activate()
        self.gripper.start()
        
        # Move to 50% position
        target_position = 127
        self.gripper.move(target_position, speed=100, force=100, wait=True)
        
        actual_position = self.gripper.position()
        print(f"  - Moved to position {target_position}, actual: {actual_position}")
        # Allow some tolerance due to sensor accuracy
        self.assertAlmostEqual(actual_position, target_position, delta=30)

    def test_10_move_various_positions(self):
        """Test moving to various positions."""
        print("\nTest: Move to Various Positions")
        
        self.gripper.activate()
        self.gripper.start()
        
        positions = [50, 100, 150, 200, 100, 50, 0, 255]
        
        for target_pos in positions:
            self.gripper.move(target_pos, speed=150, force=100, wait=True)
            actual_pos = self.gripper.position()
            print(f"  - Position {target_pos}: actual {actual_pos}")
            self.assertAlmostEqual(actual_pos, target_pos, delta=30)

    def test_11_position(self):
        """Test position() method."""
        print("\nTest: Get Position")
        
        self.gripper.activate()
        self.gripper.start()
        self.gripper.open(wait=True)
        
        position = self.gripper.position()
        self.assertIsInstance(position, int)
        self.assertGreaterEqual(position, 0)
        self.assertLessEqual(position, 255)
        
        print(f"  - Current gripper position: {position} bits")

    def test_12_calibrate_mm(self):
        """Test gripper calibration for mm positioning.
        
        WARNING: During calibration, the gripper will fully open and close.
        Ensure the gripper is free to move.
        """
        print("\nTest: Calibrate Gripper")
        print("  WARNING: Gripper will fully open and close")
        
        self.gripper.activate()
        self.gripper.start()
        
        # Calibrate: 0mm when closed, 85mm when open
        self.gripper.calibrate_bit()
        self.gripper.calibrate_mm(closemm=0, openmm=85)

        calibrated=self.gripper.is_mm_calibrated
        
        self.assertTrue(calibrated,f"Gripper should be calibrated after calibrate(). The value return be is_mm_calibrated is {calibrated}")
        print("  - Calibration completed successfully")

    def test_13_is_calibrated(self):
        """Test isCalibrated() method."""
        print("\nTest 13: Is Calibrated Check")
        
        self.gripper.activate()
        self.gripper.start()
        
        # Before calibration
        is_cal_before = self.gripper.is_mm_calibrated()
        print(f"  - Before calibration: {is_cal_before}")
        
        # Calibrate
        self.gripper.calibrate_bit()
        self.gripper.calibrate_mm(closemm=0, openmm=85)
        
        # After calibration
        is_cal_after = self.gripper.is_mm_calibrated()
        self.assertTrue(is_cal_after, "Gripper should be calibrated")
        print(f"  - After calibration: {is_cal_after}")

    def test_14_get_position_mm(self):
        """Test getPositionmm() method.
        
        Note: Gripper must be calibrated first.
        """
        print("\nTest: Get Position in mm")
        
        self.gripper.activate()
        self.gripper.start()
        
        # Calibrate first
        self.gripper.calibrate_bit()
        self.assertEqual(self.gripper.is_bit_calibrated(),True,"calibrate_bit() did not worked")
        self.gripper.calibrate_mm(closemm=0, openmm=85)

        self.assertEqual(self.gripper.is_mm_calibrated,True,"calibrate_mm() did not worked")
        
        # Get position in mm
        self.gripper.open(wait=True)
        position_mm = self.gripper.positionmm()
        print(f"  - Position in mm: {position_mm:.2f}")
        
        self.assertIsInstance(position_mm, (int, float))
        self.assertGreaterEqual(position_mm, -5)  # Allow small negative due to calibration
        self.assertLessEqual(position_mm, 90)  # Allow slight overshoot

    def test_15_move_mm(self):
        """Test moving to specific mm positions.
        
        Note: Gripper must be calibrated first.
        """
        print("\nTest: Move to Position in mm")
        
        self.gripper.activate()
        self.gripper.start()
        
        # Calibrate first
        self.gripper.calibrate_bit()
        self.assertEqual(self.gripper.is_bit_calibrated(),True,"calibrate_bit() did not worked")

        self.gripper.calibrate_mm(closemm=0, openmm=85)
        self.assertEqual(self.gripper.is_mm_calibrated(),True,"calibrate_mm() did not worked")
        
        # Move to specific mm position
        target_mm = 42.5  # 50% of range
        self.gripper.move_mm(target_mm, speed=100, force=100, wait=True)
        time.sleep(0.5)
        
        actual_mm = self.gripper.positionmm()
        print(f"  - Moved to {target_mm}mm, actual: {actual_mm:.2f}mm")
        # Allow tolerance
        self.assertAlmostEqual(actual_mm, target_mm, delta=5)

    def test_16_move_mm_various_positions(self):
        """Test moving to various mm positions."""
        print("\nTest: Move to Various mm Positions")
        
        self.gripper.activate()
        self.gripper.start()
        
        self.gripper.calibrate_bit()
        self.gripper.calibrate_mm(closemm=0, openmm=85)
        
        positions_mm = [0, 20, 42.5, 65, 85, 42.5, 20, 0]
        
        for target_mm in positions_mm:
            self.gripper.move_mm(target_mm, speed=150, force=100, wait=True)
            actual_mm = self.gripper.positionmm()
            print(f"  - Position {target_mm}mm: actual {actual_mm:.2f}mm")
            self.assertAlmostEqual(actual_mm, target_mm, delta=5)

    def test_17_real_time_move(self):
        """Test 17: realTimeMove() for smooth continuous motion.

        This test also captures a simple velocity curve by printing speed vs.
        delta-to-target for each request.
        """
        print("\nTest: Real-Time Move (Continuous Motion)")

        self.gripper.activate()
        self.gripper.start()
        self.gripper.calibrate_speed()

        # Move to a stable starting position
        self.gripper.move(10, speed=255, force=50, wait=True)

        target_positions = [50, 100, 150, 100, 50, 0, 255, 128]

        for target_pos in target_positions:
            self.gripper.realTimeMove(
                requestedPosition=target_pos,
                minSpeedPosDelta=5,
                maxSpeedPosDelta=100,
                continuousGrip=True,
                autoLock=True,
                minimalMotion=2,
                verbose=2)
            
            if self.gripper.objectDetection() in [GOBJ_DETECTED_WHILE_CLOSING,GOBJ_DETECTED_WHILE_OPENING]:
                print(self.gripper.history())

            self.assertFalse(self.gripper.objectDetection() in [GOBJ_DETECTED_WHILE_CLOSING,GOBJ_DETECTED_WHILE_OPENING],
                             "Object detected during realtime move while no object inside the gripper")
            time.sleep(0.2)  # Allow some time for movement
        print("  - Real-time move completed successfully")
    def test_18_print_status(self):
        """Test printStatus() method - outputs to console."""
        print("\nTest: Print Status")
        
        self.gripper.activate()
        self.gripper.start()
        
        # This method prints to console, we just verify it doesn't crash
        self.gripper.printStatus()
        print("  - Status printed successfully")

    def test_19_wait_complete(self):
        """Test waitComplete() method."""
        print("\nTest: Wait Complete")
        
        self.gripper.activate()
        self.gripper.start()
        self.gripper.move(0)
        startTime = time.time()
        # Start a move and then wait for it to complete
        self.gripper.move(255, speed=0, wait=True, readStatus=True)
        elapsedTime = time.time() - startTime
        self.assertGreater(elapsedTime, 0.5, "waitComplete should wait for movement to complete")       
        
        
        self.gripper.move(0)
        startTime = time.time()
        # Start a move and then wait for it to complete
        self.gripper.move(255, speed=0, wait=False, readStatus=True)
        # Wait for movement to complete
        self.gripper.waitComplete()
        
        print("  - Wait complete finished successfully")

    def test_20_timeout_configuration(self):
        """Test timeout configuration."""
        print("\nTest: Timeout Configuration")
        
        original_timeout = self.gripper.timeOut
        self.gripper.timeOut = 5
        
        self.assertEqual(self.gripper.timeOut, 5)
        
        self.gripper.timeOut = original_timeout
        print(f"  - Timeout set to: {self.gripper.timeOut}s")

    def test_21_status_dictionary(self):
        """Test that status dictionary is properly populated."""
        print("\nTest: Status Dictionary")
        
        self.gripper.readStatus()
        
        # Verify status dictionary structure
        self.assertIsInstance(self.gripper.status(), dict)
        self.assertGreater(len(self.gripper.status()), 0)
        
        # Common status keys
        expected_keys = ["gOBJ", "gSTA", "gGTO", "gACT", "kFLT", "gFLT", "gPR", "gPO", "gCU"]
        
        for key in expected_keys:
            self.assertIn(key, self.gripper.status())
        
        print(f"  - Status dictionary contains {len(self.gripper.status())} entries")
        print(f"  - Key registers: {list(self.gripper.status().keys())[:8]}")

    def test_22_quick_sequence(self):
        """Test a quick operational sequence."""
        print("\nTest: Quick Operational Sequence")
        
        # Reset and activate
        self.gripper.activate()
        self.gripper.start()
        print("  - Step 1: Reset & Activate")
        
        # Open fully
        self.gripper.open(speed=255, force=100, wait=True)
        print("  - Step 2: Open")
        
        # Move to 50% position
        self.gripper.move(127, speed=200, force=150, wait=True)
        print("  - Step 3: Move to 50%")
        
        # Close fully
        self.gripper.close(speed=255, force=255, wait=True)
        print("  - Step 4: Close")
        
        # Final status check
        self.gripper.readStatus()
        position = self.gripper.position()
        print(f"  - Final position: {position} bits")
        
    def test_23_speed_and_force_limits(self):
        """Test gripper speed and force limits."""
        print("\nTest: Speed & Force Limits")
        
        self.gripper.activate()
        self.gripper.start()
        
        # Calibrate first to access bit speed properties
        self.gripper.calibrate_bit()
        self.gripper.calibrate_mm(closemm=0, openmm=85)
        
        # Verify gripper limits are set
        self.assertIsNotNone(self.gripper.gripper_vmax_bits)
        self.assertIsNotNone(self.gripper.gripper_vmin_bits)
        
        print(f"  - Max speed: {self.gripper.gripper_vmax_bits} (bits/s)")
        print(f"  - Min speed: {self.gripper.gripper_vmin_bits} (bits/s)")
        
        # Test with various speeds
        position=0
        for speed in [0, 150, 255]:
            if position ==0:
                position=255
            else:
                position=0
            self.gripper.move(position, speed=speed, force=100, wait=True)
            time.sleep(0.2)
        
        print("  - Speed variations tested successfully")

    def test_24_command_history(self):
        """Test that command history is maintained."""
        print("\nTest: Command History")
        
        self.gripper.activate()
        self.gripper.start()
        
        # Verify command history exists
        self.assertIsNotNone(self.gripper._commandHistory)
        self.assertIn("time", self.gripper._commandHistory)
        self.assertIn("rPR", self.gripper._commandHistory)
        
        # Perform some movements
        for pos in [50, 100, 150, 100, 50]:
            self.gripper.move(pos, speed=100, force=100, wait=True)
        
        # Verify history is being updated
        self.assertGreater(len(self.gripper._commandHistory["time"]), 0)
        
        print(f"  - Command history maintained with {len(self.gripper._commandHistory['time'])} entries")

    def test_25_position_estimation(self):
        """Test _positionEstimation() by moving the gripper and comparing estimated vs actual position."""
        print("\nTest: Position Estimation")

        self.gripper.activate()
        self.gripper.start()
        self.gripper.calibrate_speed()

        # Move to start position
        self.gripper.move(self.gripper._openbit, speed=255, force=0, wait=True)

        # Start movement
        start_time = time.time()
        self.gripper.move(self.gripper._closebit, speed=0, force=0, wait=False)
        while time.time() - start_time < 1:
            time.sleep(0.01)
        self.gripper.stop()  # Stop movement to get a range of positions

        actual_pos = self.gripper.position()

        estimated_pos = self.gripper._positionEstimation(self.gripper._openbit, self.gripper._closebit, speed=0, elapsedTime=1)

        print(f"  - Start: {self.gripper._openbit}, Target: {self.gripper._closebit}, Speed: {0}")
        print(f"  - Elapsed: 1s, Actual pos: {actual_pos}, Estimated pos: {estimated_pos:.1f}")
        self.assertAlmostEqual(actual_pos, estimated_pos, delta=5, msg="Estimated position should match actual position within tolerance")

    def test_26_travel_time(self):
        """Test _travelTime() by measuring actual movement time vs estimated time."""
        print("\nTest: Travel Time")

        self.gripper.activate()
        self.gripper.start()
        self.gripper.calibrate(closemm=0, openmm=85)

        start_pos = self.gripper._openbit
        end_pos = 200
        # Test with medium speed
        speed = 128
        estimated_time = self.gripper._travelTime(start_pos, end_pos, speed)

        # Perform actual movement and measure time
        self.gripper.move(start_pos, speed=255, force=0, wait=True)  # Move to start position
        start_time = time.time()
        self.gripper.move(end_pos, speed=speed, force=0, wait=True)
        actual_time = time.time() - start_time

        print(f"  - Start: {start_pos}, End: {end_pos}, Speed: {speed}")
        print(f"  - Estimated time: {estimated_time:.3f}s, Actual time: {actual_time:.3f}s")
        self.assertAlmostEqual(actual_time, estimated_time, delta=0.5, msg="Actual travel time should match estimated time within tolerance")

        # Test with high speed
        speed_fast = 255
        estimated_time_fast = self.gripper._travelTime(start_pos, end_pos, speed_fast)

        self.gripper.move(start_pos, speed=255, force=0, wait=True)  # Move to start position

        start_time_fast = time.time()
        self.gripper.move(end_pos, speed=speed_fast, force=50, wait=True)  # Back to start
        actual_time_fast = time.time() - start_time_fast

        print(f"  - Return trip: Estimated time: {estimated_time_fast:.3f}s, Actual time: {actual_time_fast:.3f}s")
        self.assertAlmostEqual(actual_time_fast, estimated_time_fast, delta=0.5, msg="Actual return travel time should match estimated time within tolerance")
    def test_27_realTimeMove(self):
        """Test realTimeMove() by checking if gripper speed vary with distance to target."""
        print("\nTest: Real Time Move Speed")
        self.gripper.calibrate_speed()

        print("\nOpen the gripper")
        self.gripper.open(speed=255,wait=True)
        print("\nClose the gripper in realTimeMove")

        gripPositions=[]
        gripSpeeds=[]
        motionCompleted=False

        motionCompleted = False
        self.gripper.realTimeMove(requestedPosition=230, verbose=2)
        while not motionCompleted:
            self.gripper.realTimeMove(requestedPosition=230, verbose=2)
            gripPositions.append(self.gripper.position(refreshStatus=False))
            gripSpeeds.append(self.gripper.speed())
            if self.gripper.objectDetection()==GOBJ_AT_POSITION:
                motionCompleted = True
        print("command history")
        print(self.gripper._commandHistory)
        print("status History")
        print(self.gripper._statusHistory)
        print("Full history")
        print(self.gripper.history())

        
        series=[gripPositions,gripSpeeds]
        print(series)
        plot=asciichartpy.plot(series,{"height":40,"color":["red","green"]})
        print("Gripper position and speed over time\n")
        print(plot)

    def test_28_objectDetection(self):
        print("\nTest: estimated object detection")
        self.gripper.calibrate_speed()
        self.gripper.open(speed=255,wait=True)
        print("\nTest: Place and object between the finder of the gripper and press Enter")
        input()
        self.gripper.close(speed=0,wait=True)
        time.sleep(3)
        self.gripper.close(speed=0)
        time_out=5
        start_time=time.monotonic()
        duration =0
        self.gripper.readStatus()
        while (not (self.gripper.objectDetection() in [GOBJ_DETECTED_WHILE_CLOSING,
                                           GOBJ_DETECTED_WHILE_OPENING])
               and (duration<time_out)):
            
            self.gripper.close(speed=0)
            duration =time.monotonic()-start_time
        print("\nStatus")
        print(self.gripper.statusHistory())
        print("\nCommand")
        print(self.gripper.commandHistory())
        print("\nHistory")
        print(self.gripper.history())
        detection=self.gripper.objectDetection()
        print("Result of estimated object detection : ",REGISTER_DIC["gOBJ"][detection])
        self.assertEqual(detection in [GOBJ_DETECTED_WHILE_CLOSING,GOBJ_DETECTED_WHILE_OPENING],
                         True,
                         "No object has been detected")
        self.gripper.open()

    def test_29_objectDetection(self):
        print("\nTest: estimated object detection")
        self.gripper.calibrate_speed()
        self.gripper.open(speed=255,wait=True)

        print("\nPlace an objct inside the gripper")

        input()





        startTime = time.monotonic()
        t=time.monotonic()
        duration = 5
        while (t-startTime) < duration:
            self.gripper.close(speed=0,force=0,wait=False)



            current_time=self.gripper._commandHistory.loc[0,"time"]
            current_gPO=self.gripper._statusHistory.loc[0,"gPO"]
            prev_time=self.gripper._commandHistory.loc[1,"time"]
            prev_gPO=self.gripper._statusHistory.loc[1,"gPO"]
            prev_rPR=255
            prev_rSP=0
            dt=current_time-prev_time
            t=time.monotonic()

            detection=self.gripper._objectDetection(current_time,
                                                    current_gPO,
                                                    prev_time,
                                                    prev_gPO,
                                                    prev_rPR,
                                                    prev_rSP)
            print(f"dt : {int(dt*1000)} | current_gPO : {current_gPO} | prev_gPO : {prev_gPO} | prev_rPR : {prev_rPR}| prev_rSP : {prev_rSP}| Detection status : {detection} | closeBit {self.gripper._closebit} | openBit {self.gripper._openbit}")
        self.gripper.open()

    
    def test_30_history(self):
        print("\nTest: Check gripper history")
        print("\nBuilding some history")

        self.gripper.calibrate_speed()
        self.gripper.move(200,255,0,wait=True)
        self.gripper.move(100,0,wait=True)
        self.gripper.readStatus()
        self.gripper.move(180,0,wait=True)
        self.gripper.move(100,150,0,wait=True)
        self.gripper.readStatus()
        self.gripper.move(210,0,wait=True)
        self.gripper.move(20,100,0,wait=True)
        print("\nPlace an object in the gripper and press enter.")
        input()
        self.gripper.move(150,150,0,wait=True)
        self.gripper.readStatus()
        self.gripper.readStatus()
        self.gripper.move(0,255,0,wait=True)
        self.gripper.move(200,0,wait=True)
        self.gripper.move(100,wait=False,readStatus=False)
        time.sleep(0.3)
        self.gripper.move(200,wait=False,readStatus=False)
        time.sleep(0.2)
        self.gripper.readStatus()
        self.gripper.move(255,0,wait=True)
        self.gripper.move(200,0,wait=True)

        print("\nStatus history")
        print(self.gripper._statusHistory)
        print("\nCommand history")
        print(self.gripper._commandHistory)


        print("\nMerged history")
        history=merge_on_property(self.gripper._commandHistory,
                                  self.gripper._statusHistory,
                                  property="time")
        
        print("\nFilled command")
        columns_to_fill=["rPR","rSP","rFR"]
        history=forward_fill_columns(history,columns_to_fill,property="time")
        print(history)

        print("Close bit position",self.gripper._closebit)
        print("Open bit position",self.gripper._openbit)

        print("\nFill gPO")
        history=self.gripper._fill_gPO(history)
        print(history)

        print("\nFill gOBJ")
        history=self.gripper._fill_gOBJ(history)
        print(history)

        hist=self.gripper._mergeHistory()
        print("\nFinal history")
        print(hist)
    

    def test_31_history(self):
        print("\nTest : Check if position estiamtion is correct.")
        self.gripper.calibrate_speed()

        print(f"Max speed in bit/s: {self.gripper.gripper_vmax_bits}")
        print(f"Min speed in bit/s: {self.gripper.gripper_vmin_bits}")


        def closeTest(speed,duration=4):
            self.gripper.open(255,0,wait=True)
            startTime=time.monotonic()
            t=time.monotonic()
            print(f"\ngPO vs cPO at {speed} speed")
            print(f"Speed in bit/s : {int(self.gripper._bitPerSecond(speed))}")
            i=0
            while (t-startTime) < duration:
                self.gripper.close(speed)
                t=time.monotonic()
                if i==MAX_HISTORY:
                    hist=self.gripper._mergeHistory()
                    maxDelta=(hist["gPO"] - hist["cPO"]).abs().max()
                    idx = (hist["gPO"] - hist["cPO"]).abs().idxmax()
                    cPO=hist.loc[idx,"cPO"]
                    gPO=hist.loc[idx,"gPO"]
                    print(f"Max delta gPO cPO : {int(maxDelta)} | gPO {gPO} | cPO {int(cPO)}")
                    self.assertLess(maxDelta,40,"There is a difference of more than 40 bits between gPO and cPO")
                    i=0
                i+=1
        i=0
        while i<255:
            closeTest(i)
            i+=25
    
    def test_32_cOBJ(self):
        print("\nTest : Test calculated object detection.")
        self.gripper.calibrate_speed()

        def closeTest(speed,duration=4):
            self.gripper.open(255,0,wait=True)
            startTime=time.monotonic()
            t=time.monotonic()
            print(f"\ngPO vs cPO at {speed} speed")
            print(f"Speed in bit/s : {int(self.gripper._bitPerSecond(speed))}")
            i=0
            while (t-startTime) < duration:
                self.gripper.close(speed)
                t=time.monotonic()
                if i==MAX_HISTORY:
                    hist=self.gripper._mergeHistory()
                    hist=hist.sort_values("time").reset_index(drop=True)
                    print(hist)
                    #maxDelta=(hist["gPO"] - hist["cPO"]).abs().max()
                    #idx = (hist["gPO"] - hist["cPO"]).abs().idxmax()
                    #cPO=hist.loc[idx,"cPO"]
                    #gPO=hist.loc[idx,"gPO"]
                    #print(f"Max delta gPO cPO : {int(maxDelta)} | gPO {gPO} | cPO {int(cPO)}")
                    #self.assertLess(maxDelta,40,"There is a difference of more than 40 bits between gPO and cPO")
                    i=0
                i+=1
        closeTest(150,3)
        hist=self.gripper._mergeHistory()
        self.assertEqual(hist.loc[0,"cOBJ"])
    
    def test_33_complete_command(self):
        print("Test: The command complete function")

        command={"time": time.monotonic(),
                 "rPR": 100}
        print("Partial command :")
        print(command)
        self.gripper._complete_command(command)
        
        print("Completed command:")
        print(command)
        for key in COMMAND_HISTORY_COLUMNS_NAME_2_ID.keys():
            self.assertIn(key,command.keys(),"The command complete function did not add the key "+key+" to the command")
    
    def test_34_objectDetection_realtime(self):
        print("Test 34: Test that object detection does not make wrong object detection when the gripper is control in realtime.")
        #self.gripper.activate()
        self.gripper.calibrate_speed()
        with open('realTimeData.json', 'r') as f:
            data = json.load(f)
            i=0
            while i < len(data):
                t=time.monotonic()
                self.gripper.realTimeMove(data[i],verbose=0,continuousGrip=False)
                #cOBJ = self.gripper.objectDetection(self.gripper._mergeHistory(),duration=0.2)
                #self.assertNotIn(cOBJ,[GOBJ_DETECTED_WHILE_CLOSING,GOBJ_DETECTED_WHILE_OPENING],"Object detected while no object present")
                print(time.monotonic()-t)
                i+=1
    def test_35_writeFrequency(self):
        startTime=time.monotonic()

        maxWriteTime=0
        minWriteTime=10
        
        while time.monotonic()-startTime <10:
            start=time.monotonic()
            self.gripper.move(100,wait=False)
            duration = time.monotonic() - start
            if duration > maxWriteTime:
                maxWriteTime = duration
            if duration < minWriteTime:
                minWriteTime = duration
        
        print(f"Minimum write time: {minWriteTime}")
        print(f"Maximum write time: {maxWriteTime}")
            

        

    

        
class Software(unittest.TestCase):
    """Test utils functions."""
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def setUp(self):
        pass

    def test_1_forward_fill_columns(self):
        print("\nTest: Check the pandas table fill forward function")
        rawData = [
            {"time": 10, "rPR": np.nan},
            {"time": 9, "rPR": np.nan},
            {"time": 8, "rPR": 100},
            {"time": 7, "rPR": np.nan},
            {"time": 6, "rPR": np.nan},
            {"time": 5, "rPR": 200},
            {"time": 4, "rPR": np.nan},
            {"time": 3, "rPR": 50},
            {"time": 2, "rPR": np.nan},
            {"time": 1, "rPR": np.nan}]
        df_rawData = pd.DataFrame(rawData)
        filledData = [
            {"time": 10, "rPR": 100},
            {"time": 9, "rPR": 100},
            {"time": 8, "rPR": 100},
            {"time": 7, "rPR": 200},
            {"time": 6, "rPR": 200},
            {"time": 5, "rPR": 200},
            {"time": 4, "rPR": 50},
            {"time": 3, "rPR": 50},
            {"time": 2, "rPR": np.nan},
            {"time": 1, "rPR": np.nan}]
        df_filledData=pd.DataFrame(filledData)
        
        df_filled=forward_fill_columns(df_rawData,["rPR"],"time")
        print("\nTable to be filled forward")
        print(df_rawData)
        print("\nReference filled table")
        print(df_filledData)

        print("\nFilled table")
        print(df_filled)
        
        self.assertEqual(df_filled.equals(df_filledData),True,"The table has not been correctly filled forward.")

def print_test_instructions():
    """Print instructions for running hardware tests."""
    print("\n" + "="*70)
    print("ROBOTIQ GRIPPER HARDWARE TEST SUITE")
    print("="*70)
    print("\nIMPORTANT SAFETY INSTRUCTIONS:")
    print("1. Ensure the gripper is PHYSICALLY connected and powered on")
    print("2. Ensure the gripper can move FREELY (no obstructions)")
    print("3. Do NOT place objects inside during calibration tests")
    print("4. Monitor the gripper during the first test run")
    print("5. Stop immediately if anything seems wrong (Ctrl+C)")
    print("\nCOM Port Configuration:")
    print("- Gripper will auto-detect COM port (AUTO_DETECTION)")
    print("- Default device_id: 9")
    print("- Modify in test_gripper_hardware.py if needed")
    print("\nRunning all tests:")
    print("  python test_gripper_hardware.py -v")
    print("\nRunning specific test class:")
    print("  python -m pytest test_gripper_hardware.py::TestRobotiqGripperHardware -v")
    print("\nRunning specific test:")
    print("  python -m pytest test_gripper_hardware.py::TestRobotiqGripperHardware::test_07_open -v")
    print("\n" + "="*70 + "\n")


if __name__ == "__main__":
    print_test_instructions()
    unittest.main(verbosity=2)
