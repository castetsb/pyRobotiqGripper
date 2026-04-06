Basic Usage
===========

.. youtube:: 0TauzknU79I

Activate the gripper and do whatever you want with the gripper: open, close, get \
position feedback, etc.

.. code-block:: python

    gripper.connect()
    gripper.activate()
    gripper.calibrate_mm(closemm=0, openmm=40)
    gripper.open()
    gripper.close()
    gripper.move(100)
    position_in_bit = gripper.position()
    print(position_in_bit)
    gripper.move_mm(25)
    position_in_mm = gripper.positionmm()
    print(position_in_mm)

.. note::
    During activation, the gripper is going to fully open and close. Do not disturb \
    this process. Do not place an object inside the gripper.

.. note::
    Position, Speed and Force varie from 0 to 255. It is coded on 8 bits.

You can print the current status of gripper registers using `printStatus()`.

.. code-block:: python

    gripper.printStatus()

The lastest gripper status can be retrieved as a dictionnary using `status()`.

.. code-block:: python

    gripper_status = gripper.status()
    print(gripper_status)

The name of the keys of the status dictionnary are the same as the names of the gripper registers.


