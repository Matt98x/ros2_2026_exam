# ROS 2 2026 exam

ROS Jazzy robot programming exam with Gazebo simulation, keyboard driving,
obstacle-triggered command replay, and a C++ obstacle monitor.

## Clone the repository

Clone the project and its simulator submodule together:

```bash
git clone --recurse-submodules https://github.com/Matt98x/ros2_2026_exam.git
cd ros2_2026_exam
```

If you already cloned the repository without `--recurse-submodules`, download
the simulator from inside the project folder:

```bash
cd ros2_2026_exam
git submodule update --init --recursive
```

## Start with Docker and Terminator

Run from the project directory on a Linux host with Docker Compose and an active
X11 graphical session:

```bash
xhost +si:localuser:root

docker compose build robeng
docker compose up -d robeng
docker compose exec robeng robot-terminals
```

The image uses the ROS software in `carms84/robeng` and adds Terminator without
upgrading existing packages. ROS and the workspace overlay are sourced
automatically in Bash shells. The launcher builds the workspace if it is missing.

Terminator opens five panes:

- Simulation and safety controller.
- Keyboard driving.
- Closest obstacle topic.
- Average-velocity service results, refreshed every two seconds.
- A prompt to change the safety threshold in metres.

Click the keyboard pane before driving. Stop any previous simulation before
opening the layout again. Use `docker compose stop robeng` when finished.

## Manual build and launch

Open a shell in the running service:

```bash
docker compose exec robeng bash
```

Build and source the workspace inside the container:

```bash
cd /home/ubuntu/ros_ws
colcon build --packages-up-to ros_exam --cmake-force-configure
source install/setup.bash
ros2 launch ros_exam safe_drive.launch.py
```

The launch includes `bme_gazebo_sensors/launch/spawn_robot.launch.py` and starts
the Python safety controller and C++ monitor. For example, use
`safety_threshold:=0.7` to change the initial threshold or `rviz:=false` to omit RViz.

In another host terminal:

```bash
docker compose exec robeng bash
ros2 run ros_exam safe_drive.py --keyboard
```

For a disposable shell instead, use `docker compose run --rm robeng bash`.
Additional terminals must attach to that same container using `docker exec -it
<container-name> bash`. Build artifacts live outside the source bind mount and
must be rebuilt after container recreation. Rebuild after C++ or interface edits,
source the workspace, and restart the affected nodes.

## Driving and safety recovery

No Enter is needed for keyboard commands:

| Key | Action |
| --- | --- |
| `w` / `x` | Set linear velocity to +0.5 / -0.5 m/s |
| `a` / `d` | Set angular velocity to +0.3 / -0.3 rad/s |
| Space or `s` | Stop both velocities |
| `q` or Ctrl-C | Stop and quit |

Pressing the opposite direction stops that velocity first; pressing it again
reverses direction. Repeating a direction keeps the speed unchanged. Linear and
angular controls are independent.

To publish commands manually through the safety controller:

```bash
ros2 topic pub --rate 10 /cmd_vel_input geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'
```

The controller alone should publish on `/cmd_vel`. Input commands expire after
0.5 seconds without a new message.

Safe checkpoint timestamps are recorded every five seconds, together with the
history of published commands. When the closest laser reading falls below the
threshold, the controller selects the newest checkpoint at least five seconds
old and replays commands backwards, negating linear and angular velocities
independently while preserving their durations. For example, at t=12, with
checkpoints at t=0, 5, 10, it replays back to t=5.

Both velocities are replayed together, preserving arcs and pauses. On completion,
the robot stops and history is cleared. The controller prints `SAFETY STARTED`
and `SAFETY ENDED`. Recovery clears the keyboard command; enter a new command
afterwards. Stop external command publishers yourself to prevent automatic resumption.

Without an old enough checkpoint, the robot holds still until clear. Start in a
clear area and wait five seconds before driving. Stale laser data or an obstacle
within 0.25 m in the replay travel direction pause both replay components and
the replay timer. Recovery times out after 120 simulation seconds. Command replay
does not guarantee an exact physical return because acceleration and slip can
cause drift; there is no separate final orientation correction.

## Change the safety threshold

Use the standard ROS parameter service:

```bash
ros2 service call /safe_drive/set_parameters rcl_interfaces/srv/SetParameters "{parameters: [{name: safety_threshold, value: {type: 3, double_value: 0.7}}]}"
```

Equivalent shorthand:

```bash
ros2 param set /safe_drive safety_threshold 0.7
```

The threshold is in metres and must be finite and positive.

## C++ obstacle monitor and velocity average

`ros_exam/src/robot_monitor.cpp` starts with the launch. To run it separately:

```bash
ros2 run ros_exam robot_monitor
```

Inspect the custom obstacle message:

```bash
ros2 topic echo /closest_obstacle
ros2 interface show ros_exam/msg/ClosestObstacle
```

The message reports the closest obstacle's distance, angle and direction
(`front`, `left`, `right`, or `back`, relative to the laser frame), plus the current
safety threshold. The monitor starts at 0.5 m, synchronizes with `/safe_drive`
every 0.5 seconds, and retains its latest value between updates. No return is reported as `none`; invalid
scans as `unknown`.

Get the signed arithmetic mean of the latest five received `/cmd_vel_input`
messages:

```bash
ros2 service call /get_average_velocity ros_exam/srv/GetAverageVelocity '{}'
```

The response has `success`, `sample_count`, `average_linear` and `average_angular`.
With fewer than five valid inputs, it averages those available. With no inputs,
`success` is false. Repeated commands and stops count as inputs; this is not an
average of five distinct keypresses or measured odometry.

The average pane uses a persistent client to avoid repeated CLI discovery output:

```bash
ros2 run ros_exam average_velocity.py
```

This Python script only displays service results; the C++ node computes the
averages. The obstacle topic echo is independent of this client.

## Display forwarding

Compose preserves the host's `DISPLAY` (default `:0`) and mounts its X11 socket
and `${XAUTHORITY:-$HOME/.Xauthority}` authentication file. Run Compose from the
host graphical session; export `XAUTHORITY` if your session stores that file
elsewhere. The authentication file must exist.

If X11 denies access, allow the container's root user from the host:

```bash
xhost +si:localuser:root
```

Remove that allowance afterwards with `xhost -si:localuser:root`.
