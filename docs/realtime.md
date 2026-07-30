# Realtime control

<div class="video-wrapper">
  <iframe src="https://www.youtube.com/embed/37321zn1-Vo" title="YouTube video player" allowfullscreen></iframe>
</div>

There are use cases where you may want to control the gripper in realtime,
like doing remote control of a robotic system (teleoperation, teaching
Physical AI, ...)

With its default baudrate of 115200 bps the gripper connected to a PC via USB
can receive control commands at a frequency close to 100Hz. Considering that
human vision is between 30 and 60 hz and that we are actually using our
vision to do remote control operation, the default control frequency is
sufficient.

!!! note
    Using robotiq user interface (RUI), it is possible to increase the gripper
    baudrate.

The gripper is typically controlled with a joystick.

The analog signal of such a joystick may be [-1,1] in the case of a game
console controller:

<p align="center">
  <img src="https://raw.githubusercontent.com/castetsb/pyRobotiqGripper/master/docs/_static/game_controller.jpg" alt="Game controller">
</p>

Or in the range [0,1] in the case of a VR controller trigger:

<p align="center">
  <img src="https://raw.githubusercontent.com/castetsb/pyRobotiqGripper/master/docs/_static/VR_controller.jpg" alt="VR controller">
</p>

!!! note
    The quality of joystick is important to take into account. You will not be
    able to control the gripper with precision if you don't have a good control
    signal.

    You have to also consider your ability to operate the joystick. Like for
    computer games, an experienced gamer will navigate with greater precision
    in the game than a beginner player.

2 functions designed to be called at high frequency for realtime control are
available in the pyrobotiqgripper package.

1) realTimePositionMove function

2) realTimeSpeedMove function

!!! note
    In case of a trigger joystick of a VR controller, a button could be used
    to reverse the signal so that the signal range goes from [0,1] to [-1,1]

!!! note
    While pyRobotiqPackage have quite good performance, it runs with python which
    is slower than C++. If you want to switch to C++ check Robotiq C++ driver:
    https://github.com/robotiq/grippers

## 1. Realtime position control

<div class="video-wrapper">
  <iframe src="https://www.youtube.com/embed/jxQrXifJz1g" title="YouTube video player" allowfullscreen></iframe>
</div>

Example of realtime control loop with realTimePositionMove function.

```python
################
# Initialisation
################

# 1- Connect a Joystick with pygame for example

import pygame
pygame.joystick.init()
js = pygame.joystick.Joystick(0) #Select joystick with ID 0
js.init()

# 2- Connect the gripper

gripper = RobotiqGripper() #Default parameters in this case

# 3- Gripper initialisation

gripper.connect()
gripper.activate()
gripper.start()
gripper.calibrate_speed()
gripper.open()

###############################
# Gripper realtime control loop
###############################

while true:
    # Get joystick axis position
    pygame.event.pump()
    positionSignal = js.get_axis(0)

    # Feed the realTimePositionMove function with joystick signal
    gripper.realTimePositionMove(positionSignal)
```

The realTimePositionMove method is designed to be called in a loop with a high frequency.
It will move the gripper to the requested position with a speed that depends on
the distance to the target position. This allows for a smooth and responsive
control of the gripper.

This function takes a position signal [0,1] as input
and sends a position, speed and force command to the gripper. It should work
well with the signal of a trigger controller.

The speed and force are adjusted with the distance between the actual gripper
position and the requested position. The more distant is the target, the faster
the gripper will move.

```python
realTimePositionMove(self,
                         controlSignal,
                         controlBuffer=0.05,
                         speedLowerControlThreshold=10,
                         speedUpperControlThreshold=30,
                         gripSpeed=None,
                         gripForce=None,
                         verbose=0)
```

The function offer the capability to manully set the force with the
controlSignal. Once an object is detected, the function switch from position
control to force control to let the operator set the force. The operator can
then exite the force mode to release the object and go back to the position
control mode.

If a gripSpeed and gripForce is set the manual force setting mode is disabled
and the gripper secure the object with those parmeters once an object is detected.

## 2. Realtime speed control

Example of realtime control loop with realTimeSpeedMove function.

```python
################
# Initialisation
################

# 1- Connect a Joystick with pygame for example

import pygame
pygame.joystick.init()
js = pygame.joystick.Joystick(0) #Select joystick with ID 0
js.init()

# 2- Connect the gripper

gripper = RobotiqGripper() #Default parameters in this case

# 3- Gripper initialisation

gripper.connect()
gripper.activate()
gripper.start()
gripper.calibrate_speed()
gripper.open()

###############################
# Gripper realtime control loop
###############################

while true:
    # Get joystick axis position
    pygame.event.pump()
    speedSignal = js.get_axis(0)
    forceSignal = js.get_axis(1)

    # Feed the realTimeSpeedMove function with joystick signal
    gripper.realTimeSpeedMove(speedSignal)
```

This function takes a speed signal [-1,1] and a force
signal [-1,1] as input and sends a position, speed and force command to the
gripper. It should work well with the signal of a gamePad like joystick.

The operator pushes the speed joystick out of the neutral position one way or
another to move the gripper, and when the joystick is released the gripper stays
in its current position.

```python
realTimeSpeedMove(self,
                 controlSignal,
                 controlBuffer=0.05,
                 gripSpeed=None,
                 gripForce=None,
                 verbose=0)
```

In a similar way as the realTimePositionMove function, the realTImeSpeedMove
function switch controlSignal from speed control to force control to let the
operator manually fixe the grip force.

If a gripSpeed and gripForce is set the manual force setting mode is disabled
and the gripper secure the object with those parmeters once an object is detected.

## 3. Joystick CLI Feature

The pyrobotiqgripper package includes a command-line interface CLI to
experiment with the realtime functions.

A joystick or a mouse can be used to control the gripper.

This CLI requires the `all` dependencies to be installed.

```bash
uv add "pyrobotiqgripper[all]"
```

To use the Joystick CLI, run the following command:

```bash
pyrobotiqgripper-joystick
```

!!! note
    By default the application will automatically detect the port on which
    the gripper is connected. It expects that the gripper is connected to
    the PC via USB and that a joystick is also connected to the PC.

You can check the help for details about available options:

```bash
pyrobotiqgripper-joystick --help
```

There are some nice option like:

- --bipper that provide an audio feedback of the grip force
- --visual-tool that provide a live visualisation of gripper command and status

Here below is an example where the application is launched with mouse control.
The gripper communication is done via Modbus RTU over TCP.
The communication control loop uses speed control (i.e. implements realTimeSpeedMove)

```bash
pyrobotiqgripper-joystick --connection-type "RTU_VIA_TCP" --tcp-host 10.0.0.153 --tcp-port 2000 --joystick-id -1 --verbose 1 --control-method speed
```

!!! note
    --connection-type "RTU" is recommanded to have a fast communication.
