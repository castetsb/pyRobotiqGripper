Basic Usage
===========

Typical Usage
-------------

Robotiq gripper connected at PC USB port via a USB to RS485 converter
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    from pyrobotiqgripper import RobotiqGripper

    #Create a Robotiq gripper object.
    gripper = RobotiqGripper()

By default, the serial port on which the gripper is connected is automatically detected. However, you can manually specify the serial port name if you want to. Refer to the API documentation for more information.

Robotiq gripper connected to a UR robot with RS485 URCAP installed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Replace <UR_ROBOT_IP> with the actual IP address of your UR robot.

.. code-block:: python

    from pyrobotiqgripper import RobotiqGripper

    #Create a Robotiq gripper object.
    gripper = RobotiqGripper(connection_type="RTU_VIA_TCP", tcp_host=<UR_ROBOT_IP>)

Gripper control
~~~~~~~~~~~~~~~
Activate the gripper and do whatever you want with the gripper: open, close, get position feedback, etc.

.. code-block:: python

    gripper.activate()
    gripper.calibrate(closemm=0, openmm=40)
    gripper.open()
    gripper.close()
    gripper.move(100)
    position_in_bit = gripper.position()
    print(position_in_bit)
    gripper.move_mm(25)
    position_in_mm = gripper.positionmm()
    print(position_in_mm)

Note: During activation, the gripper is going to fully open and close. Do not disturb this process. Do not place an object inside the gripper.

Note: Position, Speed and Force varie from 0 to 255. It is coded on 8 bits.

You can print the current status of gripper registers using `printStatus()`.

.. code-block:: python

    gripper.printStatus()

For realtime control use the realTimeMove method.

.. code-block:: python

    gripper.realTimeMove(requestedPosition=100)

The realTimeMove method is designed to be called in a loop with a high frequency. It will move the gripper to the requested position with a speed that depends on the distance to the target position. This allows for a smooth and responsive control of the gripper.