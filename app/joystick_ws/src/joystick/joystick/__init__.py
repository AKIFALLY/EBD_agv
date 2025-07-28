"""
  DEPRECATION WARNING: Pygame-based joystick handling is deprecated.

This package contains deprecated Pygame-based joystick handling code:
- JoystickHandler: Deprecated Pygame-based direct joystick access
- JoystickTestNode: Deprecated test node using JoystickHandler

RECOMMENDED ALTERNATIVE:
Use standard ROS 2 joy_node for joystick input:

    ros2 run joy joy_node --ros-args --remap joy:=/agv/joy

Then use JoyHandler class for processing sensor_msgs/Joy messages.

The JoyHandler class in this package is still supported for ROS 2 Joy message processing.
"""

import warnings

# Issue deprecation warning for Pygame-based components
warnings.warn(
    "Pygame-based joystick handling (JoystickHandler, JoystickTestNode) is deprecated. "
    "Use standard ROS 2 joy_node instead. JoyHandler for ROS 2 Joy messages is still supported.",
    DeprecationWarning,
    stacklevel=2
)

# Export the still-supported JoyHandler
from .joy_handler import JoyHandler

# Export deprecated classes but with warnings
from .joystick_handler import JoystickHandler  # Deprecated
from .joystick_test_node import JoystickTestNode  # Deprecated

__all__ = [
    'JoyHandler',        #  Still supported for ROS 2 Joy
    'JoystickHandler',   #   Deprecated Pygame-based
    'JoystickTestNode',  #   Deprecated Pygame-based
]