Connection
==========

.. youtube:: rh-5ABvLBVI

Robotiq gripper connected at PC USB port via a USB to RS485 converter
---------------------------------------------------------------------

.. code-block:: python

    import pyrobotiqgripper as rq

    #Create a Robotiq gripper object.
    gripper = rq.RobotiqGripper()

By default, the serial port on which the gripper is connected is automatically detected.\
 However, you can manually specify the serial port name if you want to. Refer to the \
 API documentation for more information.

.. code-block:: python

    import pyrobotiqgripper as rq

    #Create a Robotiq gripper object and specify the serial port name.
    gripper = rq.RobotiqGripper(com_port="COM3")

Robotiq gripper connected to a UR robot with RS485 URCAP installed
------------------------------------------------------------------

Replace <UR_ROBOT_IP> with the actual IP address of your UR robot.

.. code-block:: python

    from pyrobotiqgripper import RobotiqGripper

    #Create a Robotiq gripper object.
    gripper = RobotiqGripper(connection_type="RTU_VIA_TCP", tcp_host=<UR_ROBOT_IP>)
