#!/bin/bash
# Wrapper script to run tests with proper environment

# Source ROS 2 and workspace environments
source /app/setup.bash
source /app/tafl_wcs_ws/install/setup.bash
source /app/db_proxy_ws/install/setup.bash
source /app/tafl_ws/install/setup.bash

# Run the test
python3 /app/tafl_wcs_ws/src/tafl_wcs/test/test_loader_port_rules.py
