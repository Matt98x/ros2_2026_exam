#!/usr/bin/env bash
source /usr/local/share/robot-programming/ros-environment.bash
cd /home/ubuntu/ros_ws/src || exit 1
case "${1:-}" in
    launch)
        exec ros2 launch ros_exam safe_drive.launch.py
        ;;
    drive)
        exec ros2 run ros_exam safe_drive.py --keyboard
        ;;
    obstacles)
        exec ros2 topic echo /closest_obstacle ros_exam/msg/ClosestObstacle
        ;;
    average)
        exec ros2 run ros_exam average_velocity.py
        ;;
    threshold)
        echo 'Enter a safety threshold in metres; Ctrl-C to stop.'
        while read -r -p 'threshold> ' threshold_value; do
            ros2 param set /safe_drive safety_threshold "$threshold_value"
        done
        ;;
    *) echo 'Expected launch, drive, obstacles, average or threshold' >&2; exit 1 ;;
esac
