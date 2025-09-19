#!/usr/bin/env python3
"""
Launch file for Flow WCS system
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Flow WCS"""
    
    # Declare launch arguments
    flows_dir_arg = DeclareLaunchArgument(
        'flows_dir',
        default_value='/app/config/wcs/flows',
        description='Directory containing flow YAML files'
    )
    
    scan_interval_arg = DeclareLaunchArgument(
        'scan_interval',
        default_value='10.0',
        description='Interval for scanning flow files (seconds)'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )
    
    enable_monitor_arg = DeclareLaunchArgument(
        'enable_monitor',
        default_value='true',
        description='Enable flow monitor node'
    )
    
    # Get launch configurations
    flows_dir = LaunchConfiguration('flows_dir')
    scan_interval = LaunchConfiguration('scan_interval')
    log_level = LaunchConfiguration('log_level')
    enable_monitor = LaunchConfiguration('enable_monitor')
    
    # Set environment variables (commented out as rmw_zenohd might not be available)
    # set_rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_zenohd')
    
    # Flow WCS main node
    flow_wcs_node = Node(
        package='flow_wcs',
        executable='flow_wcs_node',
        name='flow_wcs',
        namespace='flow_wcs',
        output='screen',
        parameters=[{
            'flows_dir': flows_dir,
            'scan_interval': scan_interval,
            'log_level': log_level,
            'enable_parallel': True,
            'max_parallel_flows': 5,
            'retry_failed_flows': True,
            'retry_delay': 30.0,
            'checkpoint_enabled': True,
            'checkpoint_dir': '/app/data/flow_wcs/checkpoints'
        }],
        remappings=[
            # Remap topics if needed
            ('/flow_wcs/status', '/wcs/flow_status'),
            ('/flow_wcs/events', '/wcs/flow_events')
        ]
    )
    
    # Flow executor node (handles actual execution)
    flow_executor_node = Node(
        package='flow_wcs',
        executable='flow_executor',
        name='flow_executor',
        namespace='flow_wcs',
        output='screen',
        parameters=[{
            'log_level': log_level,
            'enable_async': True,
            'max_concurrent_steps': 10,
            'step_timeout': 60.0,
            'enable_caching': True,
            'cache_ttl': 300.0
        }]
    )
    
    # Flow monitor node (optional monitoring)
    flow_monitor_node = Node(
        package='flow_wcs',
        executable='flow_monitor',
        name='flow_monitor',
        namespace='flow_wcs',
        output='screen',
        parameters=[{
            'log_level': log_level,
            'monitor_interval': 5.0,
            'alert_on_failure': True,
            'metrics_enabled': True,
            'metrics_port': 9090
        }]
    )
    
    # Return launch description
    return LaunchDescription([
        # Arguments
        flows_dir_arg,
        scan_interval_arg,
        log_level_arg,
        enable_monitor_arg,
        
        # Environment
        # set_rmw,  # Commented out
        
        # Nodes - 只啟動主節點，它內部已包含 executor 和 monitor
        flow_wcs_node,
        # flow_executor_node,  # 已整合在 flow_wcs_node 內
        # flow_monitor_node    # 已整合在 flow_wcs_node 內
    ])