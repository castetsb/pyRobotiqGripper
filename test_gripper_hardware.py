"""Integration tests for RobotiqGripper with physical hardware.

This test suite is designed to test the RobotiqGripper class with a physically
connected gripper. It tests all public methods and verifies the gripper's
behavior in real-world scenarios.

WARNING: Ensure the gripper is properly connected and can move freely before
running these tests. Do NOT place objects inside the gripper during calibration.

To run these tests:
    python test_gripper_hardware.py
    
Or with specific test:
    python -m pytest test_gripper_hardware.py::TestRobotiqGripperHardware::test_connect_disconnect -v
"""

import unittest
import time
from pyrobotiqgripper import RobotiqGripper
from pyrobotiqgripper.constants import AUTO_DETECTION, GRIPPER_MODE_RTU
from pyrobotiqgripper.exceptions import (
    GripperConnectionError,
    GripperNotActivatedError,
    GripperNotCalibratedError,
)


class TestRobotiqGripperHardware(unittest.TestCase):
    """Test suite for physical RobotiqGripper hardware testing."""

    @classmethod
    def setUpClass(cls):
        """Set up the test suite with a gripper connection."""
        print("\n" + "="*70)
        print("HARDWARE TEST SETUP")
        print("="*70)
        print("Ensuring gripper is connected and powered on...")
        
        cls.gripper = RobotiqGripper(
            com_port=AUTO_DETECTION,  # Auto-detect COM port
            device_id=9,
            gripper_type="2F",
            connection_type=GRIPPER_MODE_RTU,
            debug=False
        )
        
        try:
            cls.gripper.connect()
            print("✓ Connected to gripper")
        except GripperConnectionError as e:
            raise unittest.SkipTest(f"Cannot connect to gripper: {e}")

    @classmethod
    def tearDownClass(cls):
        """Clean up after all tests."""
        try:
            cls.gripper.disconnect()
            print("✓ Disconnected from gripper")
        except:
            pass

    def setUp(self):
        """Reset gripper before each test."""
        try:
            self.gripper.readStatus()
            # Check if gripper needs activation
            if not self.gripper.isActivated():
                self.gripper.resetActivate()
                time.sleep(1)
        except Exception as e:
            print(f"Warning during setUp: {e}")

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
        
        self.gripper.readStatus()
        self.assertIsNotNone(self.gripper.status)
        self.assertGreater(len(self.gripper.status), 0)
        
        # Verify key status registers exist
        expected_keys = ["gOBJ", "gSTA", "gGTO", "gACT", "kFLT", "gFLT", "gPR", "gPO", "gCU"]
        for key in expected_keys:
            self.assertIn(key, self.gripper.status, f"Status key '{key}' missing")
        
        print(f"  - Status read successfully: {len(self.gripper.status)} registers")

    def test_03_reset(self):
        """Test gripper reset."""
        print("\nTest: Reset")
        
        self.gripper.reset()
        time.sleep(0.5)
        self.gripper.readStatus()
        
        # After reset, gripper should not be activated
        print("  - Gripper reset successfully")

    def test_04_activate(self):
        """Test gripper activation."""
        print("\nTest: Activate")
        
        self.gripper.reset()
        time.sleep(0.5)
        
        self.gripper.activate()
        time.sleep(1)
        self.gripper.readStatus()
        
        # Check if gripper is activated
        is_activated = self.gripper.isActivated()
        self.assertTrue(is_activated, "Gripper should be activated")
        print("  - Gripper activated successfully")

    def test_05_reset_activate(self):
        """Test combined reset and activate operation."""
        print("\nTest: Reset & Activate")
        
        self.gripper.resetActivate()
        time.sleep(1)
        
        self.assertTrue(self.gripper.isActivated(), 
                       "Gripper should be activated after resetActivate()")
        print("  - Reset and activate completed successfully")

    def test_06_is_activated(self):
        """Test isActivated() method."""
        print("\nTest: Is Activated Check")
        
        self.gripper.resetActivate()
        time.sleep(0.5)
        
        is_activated = self.gripper.isActivated()
        self.assertTrue(is_activated, "Gripper should be reported as activated")
        print(f"  - Gripper activation status: {is_activated}")

    def test_07_open(self):
        """Test opening the gripper."""
        print("\nTest: Open Gripper")
        
        self.gripper.resetActivate()
        time.sleep(0.5)
        
        self.gripper.open(speed=255, force=255, wait=True)
        time.sleep(0.5)
        
        position = self.gripper.getPosition()
        print(f"  - Gripper opened to position: {position} bits")
        # Fully open position should be close to 0
        self.assertLess(position, 50, "Gripper should be mostly open")

    def test_08_close(self):
        """Test closing the gripper."""
        print("\nTest: Close Gripper")
        
        self.gripper.resetActivate()
        time.sleep(0.5)
        
        self.gripper.close(speed=255, force=255, wait=True)
        time.sleep(0.5)
        
        position = self.gripper.getPosition()
        print(f"  - Gripper closed to position: {position} bits")
        # Fully closed position should be close to 255
        self.assertGreater(position, 200, "Gripper should be mostly closed")

    def test_09_move_to_position(self):
        """Test moving to specific positions."""
        print("\nTest: Move to Specific Position")
        
        self.gripper.resetActivate()
        time.sleep(0.5)
        
        # Move to 50% position
        target_position = 127
        self.gripper.move(target_position, speed=100, force=100, wait=True)
        time.sleep(0.5)
        
        actual_position = self.gripper.getPosition()
        print(f"  - Moved to position {target_position}, actual: {actual_position}")
        # Allow some tolerance due to sensor accuracy
        self.assertAlmostEqual(actual_position, target_position, delta=30)

    def test_10_move_various_positions(self):
        """Test moving to various positions."""
        print("\nTest: Move to Various Positions")
        
        self.gripper.resetActivate()
        time.sleep(0.5)
        
        positions = [50, 100, 150, 200, 100, 50, 0, 255]
        
        for target_pos in positions:
            self.gripper.move(target_pos, speed=150, force=100, wait=True)
            time.sleep(0.3)
            actual_pos = self.gripper.getPosition()
            print(f"  - Position {target_pos}: actual {actual_pos}")

    def test_11_get_position(self):
        """Test getPosition() method."""
        print("\nTest: Get Position")
        
        self.gripper.resetActivate()
        time.sleep(0.5)
        
        self.gripper.open(wait=True)
        time.sleep(0.3)
        
        position = self.gripper.getPosition()
        self.assertIsInstance(position, int)
        self.assertGreaterEqual(position, 0)
        self.assertLessEqual(position, 255)
        
        print(f"  - Current gripper position: {position} bits")

    def test_12_calibrate(self):
        """Test gripper calibration for mm positioning.
        
        WARNING: During calibration, the gripper will fully open and close.
        Ensure the gripper is free to move.
        """
        print("\nTest: Calibrate Gripper")
        print("  WARNING: Gripper will fully open and close")
        
        self.gripper.resetActivate()
        time.sleep(1)
        
        # Calibrate: 0mm when closed, 85mm when open
        self.gripper.calibrate(closemm=0, openmm=85)
        time.sleep(1)
        
        self.assertTrue(self.gripper.isCalibrated(), 
                       "Gripper should be calibrated after calibrate()")
        print("  - Calibration completed successfully")

    def test_13_is_calibrated(self):
        """Test isCalibrated() method."""
        print("\nTest: Is Calibrated Check")
        
        self.gripper.resetActivate()
        time.sleep(0.5)
        
        # Before calibration
        is_cal_before = self.gripper.isCalibrated()
        print(f"  - Before calibration: {is_cal_before}")
        
        # Calibrate
        self.gripper.calibrate(closemm=0, openmm=85)
        time.sleep(0.5)
        
        # After calibration
        is_cal_after = self.gripper.isCalibrated()
        self.assertTrue(is_cal_after, "Gripper should be calibrated")
        print(f"  - After calibration: {is_cal_after}")

    def test_14_get_position_mm(self):
        """Test getPositionmm() method.
        
        Note: Gripper must be calibrated first.
        """
        print("\nTest: Get Position in mm")
        
        self.gripper.resetActivate()
        time.sleep(0.5)
        
        # Calibrate first
        self.gripper.calibrate(closemm=0, openmm=85)
        time.sleep(0.5)
        
        # Get position in mm
        self.gripper.open(wait=True)
        time.sleep(0.3)
        position_mm = self.gripper.getPositionmm()
        print(f"  - Position in mm: {position_mm:.2f}")
        
        self.assertIsInstance(position_mm, (int, float))
        self.assertGreaterEqual(position_mm, -5)  # Allow small negative due to calibration
        self.assertLessEqual(position_mm, 90)  # Allow slight overshoot

    def test_15_move_mm(self):
        """Test moving to specific mm positions.
        
        Note: Gripper must be calibrated first.
        """
        print("\nTest: Move to Position in mm")
        
        self.gripper.resetActivate()
        time.sleep(0.5)
        
        # Calibrate first
        self.gripper.calibrate(closemm=0, openmm=85)
        time.sleep(0.5)
        
        # Move to specific mm position
        target_mm = 42.5  # 50% of range
        self.gripper.move_mm(target_mm, speed=100, force=100, wait=True)
        time.sleep(0.5)
        
        actual_mm = self.gripper.getPositionmm()
        print(f"  - Moved to {target_mm}mm, actual: {actual_mm:.2f}mm")
        # Allow tolerance
        self.assertAlmostEqual(actual_mm, target_mm, delta=5)

    def test_16_move_mm_various_positions(self):
        """Test moving to various mm positions."""
        print("\nTest: Move to Various mm Positions")
        
        self.gripper.resetActivate()
        time.sleep(0.5)
        
        self.gripper.calibrate(closemm=0, openmm=85)
        time.sleep(0.5)
        
        positions_mm = [0, 20, 42.5, 65, 85, 42.5, 20, 0]
        
        for target_mm in positions_mm:
            self.gripper.move_mm(target_mm, speed=150, force=100, wait=True)
            time.sleep(0.3)
            actual_mm = self.gripper.getPositionmm()
            print(f"  - Position {target_mm}mm: actual {actual_mm:.2f}mm")

    def test_17_real_time_move(self):
        """Test realTimeMove() for smooth continuous motion."""
        print("\nTest: Real-Time Move (Continuous Motion)")
        
        self.gripper.resetActivate()
        time.sleep(0.5)
        
        # Sequentially move gripper with realTimeMove
        target_positions = [50, 100, 150, 100, 50, 0, 255, 128]
        
        for target_pos in target_positions:
            self.gripper.realTimeMove(
                requestedPosition=target_pos,
                minSpeedPosDelta=5,
                maxSpeedPosDelta=100,
                continuousGrip=True,
                autoLock=True,
                minimalMotion=2
            )
            time.sleep(0.2)
        
        print("  - Real-time move completed successfully")

    def test_18_print_status(self):
        """Test printStatus() method - outputs to console."""
        print("\nTest: Print Status")
        
        self.gripper.resetActivate()
        time.sleep(0.5)
        
        # This method prints to console, we just verify it doesn't crash
        self.gripper.printStatus()
        print("  - Status printed successfully")

    def test_19_wait_complete(self):
        """Test waitComplete() method."""
        print("\nTest: Wait Complete")
        
        self.gripper.resetActivate()
        time.sleep(0.5)
        
        # Start a move and then wait for it to complete
        self.gripper.move(128, speed=100, force=100, wait=False, readStatus=False)
        
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
        self.assertIsInstance(self.gripper.status, dict)
        self.assertGreater(len(self.gripper.status), 0)
        
        # Common status keys
        expected_keys = ["gOBJ", "gSTA", "gGTO", "gACT", "kFLT", "gFLT", "gPR", "gPO", "gCU"]
        
        for key in expected_keys:
            self.assertIn(key, self.gripper.status)
        
        print(f"  - Status dictionary contains {len(self.gripper.status)} entries")
        print(f"  - Key registers: {list(self.gripper.status.keys())[:8]}")

    def test_22_quick_sequence(self):
        """Test a quick operational sequence."""
        print("\nTest: Quick Operational Sequence")
        
        # Reset and activate
        self.gripper.resetActivate()
        time.sleep(0.5)
        print("  - Step 1: Reset & Activate")
        
        # Open fully
        self.gripper.open(speed=255, force=100, wait=True)
        time.sleep(0.3)
        print("  - Step 2: Open")
        
        # Move to 50% position
        self.gripper.move(127, speed=200, force=150, wait=True)
        time.sleep(0.3)
        print("  - Step 3: Move to 50%")
        
        # Close fully
        self.gripper.close(speed=255, force=255, wait=True)
        time.sleep(0.3)
        print("  - Step 4: Close")
        
        # Final status check
        self.gripper.readStatus()
        position = self.gripper.getPosition()
        print(f"  - Final position: {position} bits")

    def test_23_speed_and_force_limits(self):
        """Test gripper speed and force limits."""
        print("\nTest: Speed & Force Limits")
        
        self.gripper.resetActivate()
        time.sleep(0.5)
        
        # Verify gripper limits are set
        self.assertIsNotNone(self.gripper.gripper_vmax)
        self.assertIsNotNone(self.gripper.gripper_vmin)
        
        print(f"  - Max speed: {self.gripper.gripper_vmax} (bits/s)")
        print(f"  - Min speed: {self.gripper.gripper_vmin} (bits/s)")
        
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
        
        self.gripper.resetActivate()
        time.sleep(0.5)
        
        # Verify command history exists
        self.assertIsNotNone(self.gripper._commandHistory)
        self.assertIn("time", self.gripper._commandHistory)
        self.assertIn("position", self.gripper._commandHistory)
        self.assertIn("positionCommand", self.gripper._commandHistory)
        
        # Perform some movements
        for pos in [50, 100, 150, 100, 50]:
            self.gripper.move(pos, speed=100, force=100, wait=True)
            time.sleep(0.2)
        
        # Verify history is being updated
        self.assertGreater(len(self.gripper._commandHistory["time"]), 0)
        
        print(f"  - Command history maintained with {len(self.gripper._commandHistory['time'])} entries")


class TestRobotiqGripperEdgeCases(unittest.TestCase):
    """Test edge cases and error handling."""

    @classmethod
    def setUpClass(cls):
        """Set up the test suite with a gripper connection."""
        cls.gripper = RobotiqGripper(
            com_port=AUTO_DETECTION,
            device_id=9,
            gripper_type="2F",
            connection_type=GRIPPER_MODE_RTU,
            debug=False
        )
        
        try:
            cls.gripper.connect()
        except GripperConnectionError:
            raise unittest.SkipTest("Cannot connect to gripper")

    @classmethod
    def tearDownClass(cls):
        """Clean up after all tests."""
        try:
            cls.gripper.disconnect()
        except:
            pass

    def setUp(self):
        """Reset before each test."""
        try:
            self.gripper.resetActivate()
            time.sleep(0.5)
        except:
            pass

    def test_boundary_position_0(self):
        """Test moving to position 0 (fully open)."""
        print("\nTest: Boundary - Position 0 (Fully Open)")
        self.gripper.move(0, speed=255, force=100, wait=True)
        time.sleep(0.2)
        position = self.gripper.getPosition()
        self.assertLess(position, 50)
        print(f"  - Position 0 achieved: {position}")

    def test_boundary_position_255(self):
        """Test moving to position 255 (fully closed)."""
        print("\nTest: Boundary - Position 255 (Fully Closed)")
        self.gripper.move(255, speed=255, force=255, wait=True)
        time.sleep(0.2)
        position = self.gripper.getPosition()
        self.assertGreater(position, 200)
        print(f"  - Position 255 achieved: {position}")

    def test_rapid_position_changes(self):
        """Test rapid position changes."""
        print("\nTest: Rapid Position Changes")
        for _ in range(5):
            self.gripper.move(0, speed=255, force=100, wait=False)
            time.sleep(0.1)
            self.gripper.move(255, speed=255, force=100, wait=False)
            time.sleep(0.1)
        print("  - Rapid changes completed")

    def test_low_speed_movement(self):
        """Test movement at minimum speed."""
        print("\nTest: Low Speed Movement")
        self.gripper.open(speed=self.gripper.gripper_vmin, force=50, wait=True)
        self.gripper.close(speed=self.gripper.gripper_vmin, force=50, wait=True)
        print("  - Low speed movement completed")

    def test_high_force_grip(self):
        """Test gripper with maximum force."""
        print("\nTest: High Force Grip")
        self.gripper.open(speed=255, force=0, wait=True)
        time.sleep(0.2)
        self.gripper.close(speed=100, force=255, wait=True)
        time.sleep(0.2)
        position = self.gripper.getPosition()
        print(f"  - High force grip achieved position: {position}")


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
