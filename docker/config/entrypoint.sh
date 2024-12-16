#!/bin/bash

set -e
source /opt/ros/humble/setup.bash

# Check if we have permissions to change ownership
if [ "$(id -u)" = "0" ]; then
    echo "Running as root, adjusting permissions..."
    chown -R ros2_ws:ros2_ws /home/ros2_ws || echo "Failed to change ownership, continuing..."
else
    echo "Not running as root, skipping ownership adjustment."
fi

exec "$@"
