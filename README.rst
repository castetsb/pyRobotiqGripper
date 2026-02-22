pyRobotiqGripper
================

Python Driver for Robotiq Grippers.

pyRobotiqGripper is a Python library designed to facilitate control of Robotiq grippers using Modbus RTU communication via serial port or over ethernet.
It is compatible with 2F85, 2F140, and Hande.

Documentation: `pyRobotiqGripper Documentation <https://pyrobotiqgripper.readthedocs.io/en/latest/>`_

1-Disclaimer
----------

This library can be seen as a starting point for a Robotiq integration project.
You are responsible for what you do with this library.
The author takes no responsibility for any malfunction.

Note: This library is not maintained by Robotiq.

2-How to Install
--------------

Install the pyRobotiqGripper python package using PIP.

.. code-block:: bash

    python3 -m pip install pyRobotiqGripper

3-Typical Usage:
-------------

3-1-Robotiq gripper connected at PC USB port via a USB to RS485 converter
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    from pyrobotiqgripper import RobotiqGripper

    #Create a Robotiq gripper object.
    gripper = pyRobotiqGripper.RobotiqGripper()

By default, the serial port on which the gripper is connected is automatically detected. However, you can manually specify the serial port name if you want to. Refer to the API documentation for more information.

3-2-Robotiq gripper connected to a UR robot with RS485 URCAP installed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Replace <UR_ROBOT_IP> with the actual IP address of your UR robot.

.. code-block:: python

    from pyrobotiqgripper import RobotiqGripper

    #Create a Robotiq gripper object.
    gripper = pyRobotiqGripper.RobotiqGripper(use_tcp=True, tcp_host=<UR_ROBOT_IP>)

3-3-Gripper control
~~~~~~~~~~~~~~~~~~~
Activate the gripper and do whatever you want with the gripper: open, close, get position feedback, etc.

.. code-block:: python
    gripper.activate()
    gripper.calibrate(closemm=0, openmm=40)
    gripper.open()
    gripper.close()
    gripper.move(100)
    position_in_bit = gripper.getPosition()
    print(position_in_bit)
    gripper.move_mm(25)
    position_in_mm = gripper.getPositionmm()
    print(position_in_mm)

Note: During activation, the gripper is going to fully open and close. Do not disturb this process. Do not place an object inside the gripper.

Note: Position, Speed and Force varie from 0 to 255. It is coded on 8 bits.

You can print the current status of gripper registers using printStatus(.

.. code-block:: python

    gripper.printStatus(()
