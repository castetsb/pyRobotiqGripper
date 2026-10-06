# Connection

<div class="video-wrapper">
  <iframe src="https://www.youtube.com/embed/rh-5ABvLBVI" title="YouTube video player" allowfullscreen></iframe>
</div>

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

!!! note
    With uv you can directly run a python interpreter in the virtual environment
    without having to activate it.

    ```bash
    uv run python
    ```

## 1. Via USB

Robotiq gripper connected at PC USB port via a USB to RS485 converter

```python
import pyrobotiqgripper as rq

#Create a Robotiq gripper object.
gripper = rq.RobotiqGripper()
```

By default, the serial port on which the gripper is connected is automatically detected.
However, you can manually specify the serial port name if you want to. Refer to the
API documentation for more information.

### How auto-detection works

Auto-detection works in three steps:

1. Only USB serial ports (ports with a USB vendor and product ID) are probed.
   Built-in and Bluetooth serial ports are skipped.
2. On each port, the Modbus IDs 9, 1, 2, ..., 8 are tried in turn (the
   `device_id` you pass is tried first). Each try reads the firmware version
   (holding register 500), with a 50 ms timeout and no retry.
3. The first 3 characters of the firmware version identify the product, e.g.
   `GC3-1.7.0` is a 2F gripper and `GD1-...` a Hand-E. Only these two products
   are accepted. Other devices answering on the port are skipped.

You can restrict the ports that are probed. This is useful when other serial
devices are connected, since probing writes a Modbus request to each port.

```python
import pyrobotiqgripper as rq

# Never probe the port of another device.
gripper = rq.RobotiqGripper(skip_ports=["/dev/ttyACM0"])

# Probe only these ports.
gripper = rq.RobotiqGripper(candidate_ports=["/dev/ttyUSB0", "/dev/ttyUSB1"])
```

Detection is also available on its own, e.g. to find the gripper before
opening your other serial devices, or to list the Robotiq devices connected:

```python
import pyrobotiqgripper as rq

found = rq.find_gripper()
print(found.port, found.device_id, found.product, found.firmware_version, found.serial_number)
gripper = rq.RobotiqGripper(com_port=found.port, device_id=found.device_id)

for device in rq.find_devices():
    print(device)
```

```python
import pyrobotiqgripper as rq

#Create a Robotiq gripper object and specify the serial port name.
gripper = rq.RobotiqGripper(com_port="COM3")
```

## 2. Via Ethernet 
It is possible to connect to a gripper using modbus RTU over ethernet. There is typical how you would communicate with a Robotiq gripper connected at the wirst of a UR robot with the RS485 URCAP installed.

Replace <UR_ROBOT_IP> with the actual IP address of your UR robot.

```python
from pyrobotiqgripper import RobotiqGripper

#Create a Robotiq gripper object.
gripper = RobotiqGripper(connection_type="RTU_VIA_TCP", tcp_host=<UR_ROBOT_IP>)
```
