# Connection

```{youtube} rh-5ABvLBVI
```

Before running the examples below, activate the virtual environment you used
to install the package, then launch a Python interpreter.

Activate the vitual environment.

If you are on Windows use the following command to activate the virtual environment

```bash
.venv\Scripts\activate
```

If you are on macOS/Linux use the following command to activate the virtual environment

```bash
source .venv/bin/activate
```

Once the virtual environment is activated, launch a Python interpreter:

```bash
python
```

````{note}
With uv you can directly run a python interpreter in the virtual environment
without having to activate it.

```bash
uv run python
```
````

## Robotiq gripper connected at PC USB port via a USB to RS485 converter

```python
import pyrobotiqgripper as rq

#Create a Robotiq gripper object.
gripper = rq.RobotiqGripper()
```

By default, the serial port on which the gripper is connected is automatically detected.
However, you can manually specify the serial port name if you want to. Refer to the
API documentation for more information.

```python
import pyrobotiqgripper as rq

#Create a Robotiq gripper object and specify the serial port name.
gripper = rq.RobotiqGripper(com_port="COM3")
```

## Robotiq gripper connected to a UR robot with RS485 URCAP installed

Replace <UR_ROBOT_IP> with the actual IP address of your UR robot.

```python
from pyrobotiqgripper import RobotiqGripper

#Create a Robotiq gripper object.
gripper = RobotiqGripper(connection_type="RTU_VIA_TCP", tcp_host=<UR_ROBOT_IP>)
```
