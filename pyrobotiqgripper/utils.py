"""Utility functions for pyRobotiqGripper package.

This module provides helper functions for common operations used by the gripper control system,
including list manipulation and mathematical utilities.
"""

import threading
import pandas as pd
from .constants import *
import numpy as np

# Numpy array manipulation utilities

def array_merge_on_first_column(arr1, arr2):
    """
    Merge two numpy arrays side by side based on a time column.
    
    Parameters:
        arr1, arr2: numpy arrays
        property: index of the time column (default: TIME)
    
    Returns:
        Merged array with all timestamps and NaN where data is missing.
    """
    # Ensure sorted (important for searchsorted)
    arr1 = arr1[arr1[:, 0].argsort()]
    arr2 = arr2[arr2[:, 0].argsort()]

    # All unique keys
    all_keys = np.union1d(arr1[:, 0], arr2[:, 0])

    # Prepare result filled with -1
    result = np.full(
        (len(all_keys), 1 + (arr1.shape[1]-1) + (arr2.shape[1]-1)),
        -1,
        dtype=arr1.dtype
    )

    # First column = keys
    result[:, 0] = all_keys

    # Match indices
    idx_A = np.searchsorted(all_keys, arr1[:, 0])
    idx_B = np.searchsorted(all_keys, arr2[:, 0])

    # Fill values from A and B
    result[idx_A, 1:arr1.shape[1]] = arr1[:, 1:]
    result[idx_B, arr1.shape[1]:] = arr2[:, 1:]

    return result

def array_forward_fill_columns(arr, columns, missing_value=-1):
    """
    In-place forward-fill selected columns in a NumPy array.

    Parameters:
        arr: 2D numpy array (modified in-place)
        columns: list of column indices to forward-fill
        missing_value: value used to represent missing data (default: -1)
    """
    for col in columns:
        col_data = arr[:, col]

        mask = col_data != missing_value

        idx = np.where(mask, np.arange(len(col_data)), 0)
        np.maximum.accumulate(idx, out=idx)

        missing_mask = ~mask
        col_data[missing_mask] = col_data[idx[missing_mask]]

def floor_to_ms(t):
    """Floor a time value to millisecond precision.

    Parameters:
    -----------
    t : float
        Time value in seconds.

    Returns:
    --------
    float
        Time floored to millisecond precision.
    """
    return np.floor(t * 1000) / 1000

def modbus_probe_with_timeout(port, device_id, timeout=1.0):
    """Probe a Modbus serial port with a timeout to check for device connectivity.

    Parameters:
    -----------
    port : str
        The serial port to probe.
    device_id : int
        The Modbus device ID.
    timeout : float, optional
        Timeout in seconds. Default is 1.0.

    Returns:
    --------
    bool
        True if device responds, False otherwise.
    """
    result_container = {"success": False}

    def worker():
        from pymodbus.client import ModbusSerialClient

        client = ModbusSerialClient(
            port=port,
            baudrate=115200,
            parity='N',
            stopbits=1,
            bytesize=8,
            timeout=0.2
        )

        try:
            if not client.connect():
                return

            result = client.read_input_registers(
                address=2000,
                count=1,
                device_id=device_id
            )

            if result and not result.isError():
                result_container["success"] = True

        except Exception:
            pass
        finally:
            try:
                client.close()
            except:
                pass

    thread = threading.Thread(target=worker)
    thread.daemon = True
    thread.start()
    thread.join(timeout)

    if thread.is_alive():
        print("Hard timeout reached")
        return False

    return result_container["success"]
