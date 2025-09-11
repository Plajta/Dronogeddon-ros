#!/usr/bin/env python3
"""
Centralized tf_transformations fix for numpy 1.21.5 compatibility
Import this module to get working tf_transformations functions
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

# Try to import original tf_transformations first
try:
    import tf_transformations as _tf
    # If successful, use original functions
    quaternion_from_euler = _tf.quaternion_from_euler
    euler_from_quaternion = _tf.euler_from_quaternion
    
except (ImportError, AttributeError):
    # Define fallback implementations
    def quaternion_from_euler(roll, pitch, yaw, axes='sxyz'):
        """Fallback implementation of quaternion_from_euler"""
        roll_half = roll * 0.5
        pitch_half = pitch * 0.5
        yaw_half = yaw * 0.5
        
        cr = math.cos(roll_half)
        sr = math.sin(roll_half)
        cp = math.cos(pitch_half)
        sp = math.sin(pitch_half)
        cy = math.cos(yaw_half)
        sy = math.sin(yaw_half)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return [x, y, z, w]
    
    def euler_from_quaternion(quaternion, axes='sxyz'):
        """Fallback implementation of euler_from_quaternion"""
        x, y, z, w = quaternion
        
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return [roll, pitch, yaw]

# Export the functions for easy import
__all__ = ['quaternion_from_euler', 'euler_from_quaternion', 'apply_numpy_patch']

