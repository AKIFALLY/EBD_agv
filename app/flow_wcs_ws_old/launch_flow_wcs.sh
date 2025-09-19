#!/bin/bash
# Launch script for Flow WCS system

echo "🚀 Starting Flow WCS Linear Flow System..."

# Source ROS 2 environment
source /app/setup.bash

# Source Flow WCS workspace
source /app/flow_wcs_ws/install/setup.bash

# Launch the Flow WCS system
echo "📋 Launching Flow WCS nodes..."
ros2 launch flow_wcs flow_wcs_launch.py

echo "✅ Flow WCS system launched successfully"