"""Detection of Robotiq devices connected to serial ports.

Detection works in three steps:

1. List the USB serial ports (ports with a USB VID and PID).
2. On each port, try every Modbus ID of
   [`POTENTIAL_MODBUS_IDS`][pyrobotiqgripper.constants.POTENTIAL_MODBUS_IDS]
   and read the firmware version (holding register 500).
3. Identify the product from the first 3 characters of the firmware version
   (see [`DEVICE_DEFINITIONS`][pyrobotiqgripper.constants.DEVICE_DEFINITIONS]).

Probing runs in the calling process, with a short timeout and no retries, so
a silent port is given up quickly.
"""

import logging
import os
from dataclasses import dataclass
from typing import Iterable, Optional

import serial
import serial.tools.list_ports
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusIOException

from .constants import *
from .exceptions import GripperConnectionError

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class DetectedDevice:
    """A Robotiq device found on a serial port.

    Attributes:
        port (str): Serial port of the device (e.g. ``COM3`` or ``/dev/ttyUSB0``).
        device_id (int): Modbus ID the device answered on.
        firmware_version (str): Firmware version, e.g. ``GC3-1.7.0``.
        serial_number (str | None): Serial number, e.g. ``C-12345``, or None
            if it could not be read.
        product (str | None): Product name, ``2F`` or ``Hand-E``, or None for
            a product that RobotiqGripper cannot control.
    """
    port: str
    device_id: int
    firmware_version: str
    serial_number: Optional[str]
    product: Optional[str]

    @property
    def is_supported_gripper(self) -> bool:
        """True if [`RobotiqGripper`][pyrobotiqgripper.RobotiqGripper] can control this device."""
        return self.firmware_version[:3] in SUPPORTED_GRIPPER_FIRMWARES


def _same_port(a: str, b: str) -> bool:
    """Compare two port paths, resolving aliases such as udev symlinks."""
    return (os.path.normcase(os.path.realpath(a))
            == os.path.normcase(os.path.realpath(b)))


def list_candidate_ports(candidate_ports: Optional[Iterable[str]] = None,
                         skip_ports: Optional[Iterable[str]] = None,
                         usb_only: bool = True) -> list:
    """List the serial ports to probe for a Robotiq device.

    Args:
        candidate_ports: Ports to probe, in order. If None, the ports listed by
            the system are used.
        skip_ports: Ports never to probe. Paths are compared after resolving
            symlinks, so a udev alias matches its ``/dev/ttyUSB*`` port.
        usb_only: If True, ignore listed ports that have no USB VID/PID
            (built-in and Bluetooth serial ports). Not applied to
            ``candidate_ports``.

    Returns:
        list[str]: Port names to probe.
    """
    if candidate_ports is None:
        ports = [p.device for p in serial.tools.list_ports.comports()
                 if not usb_only or (p.vid is not None and p.pid is not None)]
    else:
        ports = list(candidate_ports)

    skip_ports = list(skip_ports or [])
    return [port for port in ports
            if not any(_same_port(port, skip) for skip in skip_ports)]


def _read_holding_bytes(client, address, count, device_id) -> Optional[bytes]:
    """Read ``count`` holding registers and return them as bytes, or None if
    the device did not answer."""
    try:
        result = client.read_holding_registers(address=address, count=count,
                                               device_id=device_id)
    except ModbusIOException:
        return None
    if result is None or result.isError() or len(result.registers) != count:
        return None
    return b"".join(r.to_bytes(2, "big") for r in result.registers)


def _decode_firmware_version(raw: bytes) -> str:
    """Decode register 500: 3 ASCII characters and 3 numbers."""
    prefix = raw[:3].decode("ascii", errors="replace")
    return f"{prefix}-{raw[3]}.{raw[4]}.{raw[5]}"


def _decode_serial_number(raw: bytes) -> str:
    """Decode register 510: up to 4 ASCII letters and a 32-bit number."""
    letters = raw[:4].rstrip(b"\0").decode("ascii", errors="replace")
    number = int.from_bytes(raw[4:8], "little")
    return f"{letters}-{number}"


def probe_port(port: str,
               device_ids: Iterable[int] = POTENTIAL_MODBUS_IDS,
               baudrate: int = BAUDRATE,
               timeout: float = DETECTION_TIMEOUT) -> Optional[DetectedDevice]:
    """Look for a Robotiq device on one serial port.

    Each Modbus ID is tried in turn by reading the firmware version; the first
    ID that answers is kept.

    Args:
        port: Serial port to probe.
        device_ids: Modbus IDs to try, in order.
        baudrate: Serial baudrate.
        timeout: Modbus timeout of each read, in seconds.

    Returns:
        DetectedDevice | None: The device found, or None.
    """
    client = ModbusSerialClient(port=port, baudrate=baudrate, parity=PARITY,
                                stopbits=STOPBITS, bytesize=BYTESIZE,
                                timeout=timeout, retries=0)
    # Unanswered reads are expected here: keep pymodbus from logging each one,
    # unless detection debugging is on.
    pymodbus_logger = logging.getLogger("pymodbus")
    pymodbus_level = pymodbus_logger.level
    if not logger.isEnabledFor(logging.DEBUG):
        pymodbus_logger.setLevel(logging.CRITICAL)
    try:
        if not client.connect():
            logger.debug("Cannot open %s", port)
            return None
        for device_id in device_ids:
            raw = _read_holding_bytes(client, FIRMWARE_VERSION_REGISTER, 3, device_id)
            if raw is None:
                logger.debug("No answer on %s from Modbus ID %d", port, device_id)
                continue
            firmware_version = _decode_firmware_version(raw)
            raw = _read_holding_bytes(client, SERIAL_NUMBER_REGISTER, 4, device_id)
            device = DetectedDevice(
                port=port,
                device_id=device_id,
                firmware_version=firmware_version,
                serial_number=_decode_serial_number(raw) if raw else None,
                product=DEVICE_DEFINITIONS.get(firmware_version[:3]),
            )
            logger.debug("Found %s", device)
            return device
        return None
    except (serial.SerialException, OSError) as e:
        # The port itself failed: trying other IDs on it would fail the same way.
        logger.debug("Error on %s: %s", port, e)
        return None
    finally:
        client.close()
        pymodbus_logger.setLevel(pymodbus_level)


def find_devices(candidate_ports: Optional[Iterable[str]] = None,
                 skip_ports: Optional[Iterable[str]] = None,
                 usb_only: bool = True,
                 device_ids: Iterable[int] = POTENTIAL_MODBUS_IDS,
                 baudrate: int = BAUDRATE,
                 timeout: float = DETECTION_TIMEOUT) -> list:
    """Find the Robotiq devices connected to serial ports.

    Any device answering a firmware version read is reported, including
    products that [`RobotiqGripper`][pyrobotiqgripper.RobotiqGripper] cannot
    control. See [`list_candidate_ports`][pyrobotiqgripper.detection.list_candidate_ports]
    for the port selection arguments, and [`probe_port`][pyrobotiqgripper.detection.probe_port]
    for the others.

    Returns:
        list[DetectedDevice]: One entry per port where a device answered.

    Examples:
        >>> import pyrobotiqgripper as rq
        >>> for device in rq.find_devices():
        ...     print(device.port, device.product, device.firmware_version)
        COM4 2F GC3-1.7.0
    """
    device_ids = list(device_ids)
    devices = []
    for port in list_candidate_ports(candidate_ports, skip_ports, usb_only):
        device = probe_port(port, device_ids, baudrate, timeout)
        if device is not None:
            devices.append(device)
    return devices


def find_gripper(candidate_ports: Optional[Iterable[str]] = None,
                 skip_ports: Optional[Iterable[str]] = None,
                 usb_only: bool = True,
                 device_ids: Iterable[int] = POTENTIAL_MODBUS_IDS,
                 baudrate: int = BAUDRATE,
                 timeout: float = DETECTION_TIMEOUT) -> DetectedDevice:
    """Find the first gripper that [`RobotiqGripper`][pyrobotiqgripper.RobotiqGripper]
    can control (2F or Hand-E).

    Ports are probed in order and the search stops at the first gripper.
    Arguments are those of [`find_devices`][pyrobotiqgripper.detection.find_devices].
    Applications can call it before opening their other serial devices, and
    pass the ports of those devices in ``skip_ports``.

    Returns:
        DetectedDevice: The gripper found. Pass ``port`` and ``device_id`` to
            [`RobotiqGripper`][pyrobotiqgripper.RobotiqGripper].

    Raises:
        GripperConnectionError: If no gripper is found.

    Examples:
        >>> import pyrobotiqgripper as rq
        >>> found = rq.find_gripper(skip_ports=["/dev/ttyACM0"])
        >>> gripper = rq.RobotiqGripper(com_port=found.port, device_id=found.device_id)
    """
    device_ids = list(device_ids)
    ports = list_candidate_ports(candidate_ports, skip_ports, usb_only)
    others = []
    for port in ports:
        device = probe_port(port, device_ids, baudrate, timeout)
        if device is None:
            continue
        if device.is_supported_gripper:
            return device
        others.append(device)

    message = (f"No 2F or Hand-E gripper found on ports: "
               f"{', '.join(ports) if ports else 'none'} "
               f"(Modbus IDs {', '.join(map(str, device_ids))}).")
    if others:
        message += " Other Robotiq devices found: " + ", ".join(
            f"{d.product or d.firmware_version} on {d.port}" for d in others) + "."
    message += (" Please check: 1) Gripper is powered, 2) USB cable connected, "
                "3) The port is not in use by another program.")
    raise GripperConnectionError(message)
