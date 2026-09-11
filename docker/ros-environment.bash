# Shared by entrypoint, interactive terminals, login shells and bash -c tasks.
# ROS setup scripts are not safe under `set -u`.
source /opt/ros/jazzy/setup.bash
if [ -f /home/ubuntu/ros_ws/install/setup.bash ]; then
    source /home/ubuntu/ros_ws/install/setup.bash
fi
