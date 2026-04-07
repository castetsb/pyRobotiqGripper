Advanced Usage
==============

Realtime control
----------------

.. youtube:: 55NyOWzFGxA

For realtime control use the realTimeMove method.

.. code-block:: python

    gripper.realTimeMove(requestedPosition=100)

The realTimeMove method is designed to be called in a loop with a high frequency. \
It will move the gripper to the requested position with a speed that depends on \
the distance to the target position. This allows for a smooth and responsive \
control of the gripper.

Joystick CLI Feature
--------------------

pyRobotiqGripper includes a command-line interface (CLI) tool for controlling the \
gripper using a joystick, gamepad or a mouse.

To use the Joystick CLI, run the following command:

.. code-block:: bash

    pyrobotiqgripper-joystick

.. note::
    By defaut the application will automatically detect the \
    the port on which is connected the gripper. It expects that the gripper is connected to \
    the PC via USB and that a joystick is also connected to the PC.

You can also check the available options:

.. code-block:: bash

    pyrobotiqgripper-joystick --help

Here below is an example where the application is launched with mouse control and the \
gripper communication is done via Modbus RTU over TCP.

.. code-block:: bash

    pyrobotiqgripper-joystick --connection-type "RTU_VIA_TCP" --tcp-host 10.0.0.153 --tcp-port 2000 --joystick-id -1 --verbose 1