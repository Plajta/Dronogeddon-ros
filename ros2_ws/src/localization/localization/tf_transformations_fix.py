#!/usr/bin/env python3
"""
Comprehensive fix for tf_transformations compatibility with numpy 1.21.5
This module provides patched tf_transformations functions and numpy compatibility
"""

import numpy as np
import math
import warnings

# Apply numpy 1.21.5 compatibility patch immediately
def apply_numpy_patch():
    """Apply comprehensive patch for numpy 1.21.5 compatibility"""
    warnings.filterwarnings('ignore', category=DeprecationWarning, module='numpy')
    warnings.filterwarnings('ignore', message='.*np.float.*', category=DeprecationWarning)
    
    # Restore deprecated numpy aliases
    if not hasattr(np, 'float'):
        np.float = np.float64
    if not hasattr(np, 'int'):
        np.int = np.int_
    if not hasattr(np, 'complex'):
        np.complex = np.complex128
    if not hasattr(np, 'bool'):
        np.bool = np.bool_

# Apply patch immediately when module is imported
apply_numpy_patch()

def quaternion_from_euler(roll, pitch, yaw, axes='sxyz'):
    """
    Return quaternion from Euler angles and axis sequence.
    Fixed version that works with numpy 1.21+
    """
    try:
        from tf_transformations import quaternion_from_euler as tf_quat_from_euler
        return tf_quat_from_euler(roll, pitch, yaw, axes)
    except (ImportError, AttributeError):
        # Fallback implementation
        return _quaternion_from_euler_fallback(roll, pitch, yaw)

def _quaternion_from_euler_fallback(roll, pitch, yaw):
    """
    Fallback implementation of quaternion_from_euler
    """
    # Convert to half angles
    roll_half = roll * 0.5
    pitch_half = pitch * 0.5
    yaw_half = yaw * 0.5
    
    # Compute trigonometric values
    cr = math.cos(roll_half)
    sr = math.sin(roll_half)
    cp = math.cos(pitch_half)
    sp = math.sin(pitch_half)
    cy = math.cos(yaw_half)
    sy = math.sin(yaw_half)
    
    # Compute quaternion components
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return [x, y, z, w]

def euler_from_quaternion(quaternion, axes='sxyz'):
    """
    Return Euler angles from quaternion for specified axis sequence.
    Fixed version that works with numpy 1.21+
    """
    try:
        from tf_transformations import euler_from_quaternion as tf_euler_from_quat
        return tf_euler_from_quat(quaternion, axes)
    except (ImportError, AttributeError):
        # Fallback implementation
        return _euler_from_quaternion_fallback(quaternion)

def _euler_from_quaternion_fallback(quaternion):
    """
    Fallback implementation of euler_from_quaternion
    """
    x, y, z, w = quaternion
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return [roll, pitch, yaw]

