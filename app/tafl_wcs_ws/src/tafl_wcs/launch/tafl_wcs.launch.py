#!/usr/bin/env python3
"""
TAFL WCS Launch File
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description"""
    
    # Get package share directory
    pkg_share = get_package_share_directory('tafl_wcs')
    
    # Default config file path
    default_config_file = os.path.join(pkg_share, 'config', 'default_config.yaml')
    
    # Declare launch arguments
    flows_dir_arg = DeclareLaunchArgument(
        'flows_dir',
        default_value='/app/config/tafl/flows',
        description='Directory containing TAFL flow files'
    )
    
    database_url_arg = DeclareLaunchArgument(
        'database_url',
        default_value='postgresql://agvc:password@192.168.100.254:5432/agvc',
        description='PostgreSQL database connection URL'
    )
    
    scan_interval_arg = DeclareLaunchArgument(
        'scan_interval',
        default_value='10.0',
        description='Interval for scanning flow files (seconds)'
    )
    
    auto_execute_arg = DeclareLaunchArgument(
        'auto_execute',
        default_value='True',
        description='Auto-execute enabled flows'
    )
    
    execution_interval_arg = DeclareLaunchArgument(
        'execution_interval',
        default_value='60.0',
        description='Execution interval for auto-execution (seconds)'
    )
    
    # TAFL WCS Node
    tafl_wcs_node = Node(
        package='tafl_wcs',
        executable='tafl_wcs_node',
        name='tafl_wcs_node',
        output='screen',
        parameters=[{
            'flows_dir': LaunchConfiguration('flows_dir'),
            'database_url': LaunchConfiguration('database_url'),
            'scan_interval': LaunchConfiguration('scan_interval'),
            'auto_execute': LaunchConfiguration('auto_execute'),
            'execution_interval': LaunchConfiguration('execution_interval'),
        }],
        respawn=True,
        respawn_delay=5,
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(flows_dir_arg)
    ld.add_action(database_url_arg)
    ld.add_action(scan_interval_arg)
    ld.add_action(auto_execute_arg)
    ld.add_action(execution_interval_arg)
    
    # Add node
    ld.add_action(tafl_wcs_node)
    
    return ld
