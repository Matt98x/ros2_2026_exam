#!/usr/bin/env bash
set -e
source /usr/local/share/robot-programming/ros-environment.bash
if ! ros2 pkg prefix ros_exam >/dev/null 2>&1; then
    echo 'Preparing the ROS workspace for this new container...'
    (cd /home/ubuntu/ros_ws && colcon build --packages-up-to ros_exam)
    source /home/ubuntu/ros_ws/install/setup.bash
fi
exec terminator --no-dbus --config=/usr/local/share/robot-programming/terminator.config --layout=robot
