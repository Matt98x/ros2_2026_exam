# Optional manual setup for an existing ROS container.
source /opt/ros/jazzy/setup.bash
if [ -f /home/ubuntu/ros_ws/install/setup.bash ]; then
    source /home/ubuntu/ros_ws/install/setup.bash
fi
