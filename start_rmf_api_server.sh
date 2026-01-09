#!/bin/bash
# Script to start RMF API Server

# Source ROS environment
source /opt/ros/humble/setup.bash
source /home/robot1/rmf_ws/install/setup.bash

cd /home/robot1/rmf_ws/src/rmf-web/packages/api-server

# Create cache directory if it doesn't exist
mkdir -p /home/robot1/rmf_ws/run/cache

# Set configuration and start the API server
export RMF_API_SERVER_CONFIG=/home/robot1/rmf_ws/rmf_api_config.py
export PYTHONPATH=/home/robot1/rmf_ws/src/rmf-web/packages/api-server:$PYTHONPATH

echo "Starting RMF API Server on port 8000..."
echo "Configuration: $RMF_API_SERVER_CONFIG"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"

python3 -m api_server
