"""Utility functions for pyRobotiqGripper package.

This module provides helper functions for common operations used by the gripper control system,
including list manipulation and mathematical utilities.
"""

import threading
from .constants import *
import numpy as np

# Numpy array manipulation utilities

def get_bottom_continuous_rows(arr, col_idx):
    """Extracts the bottom continuous rows of a 2D array based on a target column.
    
    Checks the specified column from the bottom up and returns the slice of the 
    array where the values in that column are continuously equal to the very last 
    value in that column.

    Args:
        arr (numpy.ndarray): The input 2D NumPy array.
        col_idx (int): The index of the column to evaluate.

    Returns:
        numpy.ndarray: A sliced portion of the original array containing the bottom 
            rows where the specified column has the same continuous value.
    """
    # Extract the target column
    col = arr[:, col_idx]
    
    # Get the very last value in that column
    last_val = col[-1]
    
    # Create a boolean mask of where the column does NOT equal the last value
    not_equal_indices = np.where(col != last_val)[0]
    
    # If the array is empty, it means ALL values in the column are the same
    if len(not_equal_indices) == 0:
        return arr
        
    # Find the last index where the value changed, and add 1 to get the start of our slice
    start_idx = not_equal_indices[-1] + 1
    
    # Return the sliced array from that index to the end
    return arr[start_idx:]

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

def find_last_below_threshold(arr, col_idx, threshold):
    """
    Finds the first row index from the bottom where the value in a specified 
    column is less than a given threshold.

    Parameters
    ----------
    arr : numpy.ndarray
        A 2D NumPy array to search.
    col_idx : int
        The index of the column to evaluate.
    threshold : int or float
        The value that the column elements must be strictly less than.

    Returns
    -------
    int or None
        The integer row index of the first match starting from the bottom of 
        the array. Returns None if no rows meet the condition.
        
    Examples
    --------
    >>> import numpy as np
    >>> data = np.array([[1, 10], [2, 5], [3, 12], [4, 4], [5, 8]])
    >>> find_last_below_threshold(data, col_idx=1, threshold=6)
    3
    """
    # Find all row indices where the column value is less than the threshold
    matching_indices = np.where(arr[:, col_idx] < threshold)[0]
    
    # If we found any matches, return the last one (closest to the bottom)
    if matching_indices.size > 0:
        return matching_indices[-1]
    
    # Return None if the condition is never met
    return None

def calculate_final_position(start_position: float, time_speed_data: np.ndarray) -> float:
    """
    Calculates the final position given a starting position and a time-speed array.

    This function uses the trapezoidal rule to numerically integrate speed over 
    time, determining the total distance traveled, and adds it to the start position.

    Parameters
    ----------
    start_position : float
        The initial starting position.
    time_speed_data : np.ndarray
        A 2D NumPy array of shape (N, 2). The first column represents time (t), 
        and the second column represents speed (v).

    Returns
    -------
    float
        The calculated final position.

    Raises
    ------
    ValueError
        If the input array is not a 2D array with exactly 2 columns.
    """
    # Validate the input array shape
    if time_speed_data.ndim != 2 or time_speed_data.shape[1] != 2:
        raise ValueError("time_speed_data must be a 2D array with exactly 2 columns.")

    # Extract time and speed columns
    time = time_speed_data[:, 0]
    speed = time_speed_data[:, 1]

    # Integrate speed with respect to time using the updated NumPy 2.0 function
    distance_traveled = np.trapezoid(speed, x=time)

    # Calculate final position
    final_position = start_position + distance_traveled

    return float(final_position)

def get_line_function(x1, y1, x2, y2):
    """Returns a function f(x) representing the line passing through two points.

    This function calculates the slope (m) and y-intercept (b) for a line 
    defined by the coordinates (x1, y1) and (x2, y2). It then returns a 
    closure that can be used to evaluate the linear equation y = mx + b 
    for any given x.

    Args:
        x1 (int or float): The x-coordinate of the first point.
        y1 (int or float): The y-coordinate of the first point.
        x2 (int or float): The x-coordinate of the second point.
        y2 (int or float): The y-coordinate of the second point.

    Returns:
        function: A function that takes a single numeric argument `x` 
        and returns the corresponding `y` value on the calculated line.

    Raises:
        ValueError: If `x1` equals `x2`, meaning the points form a vertical 
            line where `y` cannot be expressed as a valid function of `x`.

    Examples:
        >>> my_line = get_line_function(1, 2, 3, 6)
        >>> my_line(5)
        10.0
    """
    if x1 == x2:
        raise ValueError("The points form a vertical line; y cannot be expressed as a function of x.")
    
    m = (y2 - y1) / (x2 - x1)
    b = y1 - m * x1
    
    def f(x):
        return m * x + b
        
    return f

import numpy as np


import numpy as np


def make_curve_function(origin, threshold, max_val=255.0):
    """Generates a callable image transformation function with an adjustable origin and smooth threshold ramp.

    The returned function maps input values (x) to output values (y). Values
    below or equal to the origin return 0.0. Values between the origin and the
    threshold follow a smooth cubic S-curve (smoothstep) blending seamlessly from
    0.0 to the maximum value. Any values above or equal to the threshold are
    flattened at `max_val`.

    Args:
        origin: A float representing the input value on the x-axis where the
            smooth curve begins to rise from zero.
        threshold: A float representing the input value where the curve
            reaches its maximum plateau. Must be greater than the origin.
        max_val: A float representing the maximum output limit. Defaults to 255.0.

    Returns:
        A function `f(x)` that accepts a scalar or a NumPy array and returns the
        transformed values following the smooth ramp profile.

    Raises:
        ValueError: If `threshold` is less than or equal to `origin`.

    Examples:
        >>> f = make_curve_function(origin=50.0, threshold=150.0, max_val=255.0)
        >>> f(30.0)
        0.0
        >>> f(50.0)
        0.0
        >>> f(100.0)
        127.5
        >>> f(150.0)
        255.0
        >>> f(180.0)
        255.0
    """
    if threshold <= origin:
        raise ValueError("Threshold must be strictly greater than the origin.")

    def curve_function(x):
        # Convert input to array for element-wise operation while supporting scalars
        x_arr = np.asarray(x, dtype=np.float64)

        # Shift the inputs by the origin, and normalize relative to the width of the ramp
        # Clamp between 0.0 and 1.0 to handle out-of-bounds inputs automatically
        t = np.clip((x_arr - origin) / (threshold - origin), 0.0, 1.0)

        # Apply smoothstep cubic equation: S(t) = 3t^2 - 2t^3
        result = (3 * t**2 - 2 * t**3) * max_val

        # Return scalar if input was scalar, otherwise return the array
        return float(result) if np.isscalar(x) else result

    return curve_function

def make_ramp_function(x0, x1, y0, y1):
    """
    Returns a piecewise function based on the provided parameters:
    - Below x0: value is y0
    - Between x0 and x1: linearly ramps up from y0 to y0 + y1
    - Above x1: value is y0 + y1
    """
    def ramp_function(x):
        if x <= x0:
            return y0
        elif x >= x1:
            return y0 + y1
        else:
            # Linear interpolation between (x0, y0) and (x1, y0 + y1)
            return y0 + (y1 / (x1 - x0)) * (x - x0)
            
    return ramp_function