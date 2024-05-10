pyRobotiqGripper
=================

Python Driver for Robotiq Grippers via Modbus RTU

pyRobotiqGripper is a Python library designed to facilitate control of Robotiq grippers using Modbus RTU communication via serial port.
It is compatible with 2F85, 2F140, and Hande.

Documentation: `pyRobotiqGripper Documentation <https://pyrobotiqgripper.readthedocs.io/en/latest/>`_

Disclaimer
----------

This library can be seen as a starting point for a Robotiq integration project.
You are responsible for what you do with this library.
The author takes no responsibility for any malfunction.

Note: This library is not maintained by Robotiq.

How to Install
--------------

(To come soon) Install the pyRobotiqGripper python package using PIP.

```bash

    python -m pip install pyRobotiqGripper
```
Typical Usage
-------------

Import the pyRobotiqGripper module.

```python
    import pyRobotiqGripper
```
Create a Robotiq gripper object.

```python
    gripper = pyRobotiqGripper()
```
By default, the serial port on which the gripper is connected is automatically detected. However, you can manually specify the serial port name if you want to. Refer to the API documentation for more information.
You can now activate the gripper and eventually calibrate the gripper if you want to control the opening in mm instead of bit.

Note: During activation, the gripper is going to fully open and close. Do not disturb this process. Do not place an object inside the gripper.

Note: The gripper finger position varies from 0 to 255. It is coded on 8 bits.

```python
    gripper.activate()
    gripper.calibrate(0, 40)
```
You can now do whatever you want with the gripper: open, close, get position feedback, etc.

```python
    gripper.open()
    gripper.close()
    gripper.goTo(100)
    position_in_bit = gripper.getPosition()
    print(position_in_bit)
    gripper.goTomm(25)
    position_in_mm = gripper.getPositionmm()
    print(position_in_mm)
```

You can print the current status of gripper registers using printInfo.

```python
    gripper.printInfo()
```