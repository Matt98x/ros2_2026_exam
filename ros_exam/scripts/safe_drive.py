#!/usr/bin/env python3
import math
import os
import select
import sys
import termios
import tty
import time
from collections import deque
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class CommandSample:
    time: float
    linear: float
    angular: float


@dataclass
class ReplayStep:
    duration: float
    linear: float
    angular: float


def recovery_commands(checkpoints, history, now):
    """Reverse time order and both velocity signs, preserving the trajectory."""
    eligible = [stamp for stamp in checkpoints if now - stamp >= 5.0]
    if not eligible:
        return []
    start = eligible[-1]
    samples = list(history)
    steps = []
    for index, sample in enumerate(samples):
        end = samples[index + 1].time if index + 1 < len(samples) else now
        duration = min(end, now) - max(sample.time, start)
        if duration > 0:
            steps.append(ReplayStep(duration, -sample.linear, -sample.angular))
    return list(reversed(steps))


class SafeDrive(Node):
    def __init__(self):
        super().__init__('safe_drive')
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.declare_parameter('safety_threshold', 0.5)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.recovery_publisher = self.create_publisher(Bool, '/safe_drive/recovering', 10)
        self.create_subscription(Twist, '/cmd_vel_input', self.command_received, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_received, qos_profile_sensor_data)
        self.scan = None
        self.minimum = None
        self.scan_wall = self.command_wall = -math.inf
        self.command = Twist()
        self.checkpoints = deque(maxlen=4)
        self.history = deque()
        self.replay = deque()
        self.replay_last_time = None
        self.replay_running = False
        self.recovering = False
        self.recovery_started = 0.0
        self.last_time = None
        self.create_timer(0.05, self.update)
        self.get_logger().info('Ready: /cmd_vel_input; threshold service: /safe_drive/set_parameters')

    def validate_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == 'safety_threshold':
                if not math.isfinite(parameter.value) or parameter.value <= 0:
                    return SetParametersResult(successful=False, reason='Threshold must be finite and positive')
        return SetParametersResult(successful=True)

    def command_received(self, msg):
        # Commands during recovery are discarded, it requests a fresh command afterwards.
        if self.recovering:
            return
        if not (math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z)):
            self.command = Twist()
            return
        self.command = Twist()
        self.command.linear.x = max(-0.5, min(0.5, msg.linear.x))
        self.command.angular.z = max(-1.0, min(1.0, msg.angular.z))
        self.command_wall = time.monotonic()

    def scan_received(self, msg):
        ranges = [max(0.0, r) for r in msg.ranges if not math.isnan(r) and r != math.inf]
        self.minimum = min(ranges) if ranges else (math.inf if math.inf in msg.ranges else None)
        self.scan = msg
        self.scan_wall = time.monotonic()

    def reset_history(self):
        self.checkpoints.clear()
        self.history.clear()
        self.replay.clear()
        self.replay_last_time = None
        self.replay_running = False
        self.recovering = False
        self.command = Twist()
        self.command_wall = -math.inf

    def remember(self, now):
        if not self.checkpoints or now - self.checkpoints[-1] >= 5.0:
            self.checkpoints.append(now)
        # Keep the command spanning the oldest checkpoint as well as newer ones.
        while len(self.history) > 1 and self.history[1].time <= self.checkpoints[0]:
            self.history.popleft()

    def publish_recorded(self, command, now):
        if self.checkpoints:
            sample = CommandSample(now, command.linear.x, command.angular.z)
            if not self.history or (sample.linear, sample.angular) != (
                    self.history[-1].linear, self.history[-1].angular):
                self.history.append(sample)
        self.publisher.publish(command)

    def scan_ready(self):
        return self.minimum is not None and time.monotonic() - self.scan_wall <= 1.0

    def update(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_time is not None and now < self.last_time:
            if self.recovering:
                self.get_logger().warn('SAFETY ENDED: replay cancelled by simulation clock reset')
            self.reset_history()
        self.last_time = now
        if self.recovering:
            self.recover(now)
        elif not self.scan_ready():
            self.publish_recorded(Twist(), now)
            self.command = Twist()
        elif self.minimum < self.get_parameter('safety_threshold').value:
            self.command = Twist()
            self.command_wall = -math.inf
            self.replay = deque(recovery_commands(self.checkpoints, self.history, now))
            if self.replay:
                self.recovering = True
                self.recovery_started = now
                self.replay_last_time = now
                self.replay_running = False
                duration = sum(step.duration for step in self.replay)
                self.get_logger().warn(f'SAFETY STARTED: replaying {duration:.2f} s of commands in reverse')
                self.recover(now)
            else:
                # No sufficiently old checkpoint: hold still until clear.
                self.publish_recorded(Twist(), now)
        else:
            self.remember(now)
            self.publish_recorded(self.command if time.monotonic() - self.command_wall < 0.5
                                  else Twist(), now)
        self.recovery_publisher.publish(Bool(data=self.recovering))

    def travel_blocked(self, linear):
        if linear == 0.0:
            return False
        heading = 0.0 if linear > 0 else math.pi
        for i, distance in enumerate(self.scan.ranges):
            angle = self.scan.angle_min + i * self.scan.angle_increment
            if abs(wrap(angle - heading)) < 0.45 and not math.isnan(distance) and distance < 0.25:
                return True
        return False

    def recover(self, now):
        if now - self.recovery_started > 120.0:
            self.publisher.publish(Twist())
            self.reset_history()
            self.get_logger().error('SAFETY ENDED: replay timed out.')
            return
        # Advance only the time actually spent issuing replay commands. Pausing
        # stops BOTH components so an arc is never turned into a pure rotation.
        elapsed = max(0.0, now - self.replay_last_time) if self.replay_running else 0.0
        self.replay_last_time = now
        while self.replay and elapsed >= self.replay[0].duration - 1e-9:
            elapsed = max(0.0, elapsed - self.replay.popleft().duration)
        if not self.replay:
            self.publisher.publish(Twist())
            self.reset_history()
            self.get_logger().info('SAFETY ENDED: reverse command replay complete.')
            return
        self.replay[0].duration -= elapsed
        step = self.replay[0]
        self.replay_running = self.scan_ready() and not self.travel_blocked(step.linear)
        velocity = Twist()
        if self.replay_running:
            velocity.linear.x = step.linear
            velocity.angular.z = step.angular
        self.publisher.publish(velocity)


def keyboard():
    node = Node('safe_drive_keyboard')
    publisher = node.create_publisher(Twist, '/cmd_vel_input', 10)
    command = Twist()
    safety_active = False

    def recovery_received(msg):
        nonlocal command, safety_active
        if msg.data != safety_active:
            print('SAFETY STARTED: returning to safe position' if msg.data else
                  'SAFETY ENDED: recovery finished or cancelled.', flush=True)
        safety_active = msg.data
        if safety_active:
            command = Twist()

    node.create_subscription(Bool, '/safe_drive/recovering', recovery_received, 10)
    node.create_timer(0.1, lambda: publisher.publish(command))
    settings = termios.tcgetattr(sys.stdin)
    print('w/x: forward/reverse (0.5 m/s), a/d: turn (0.3 rad/s).\n'
          'Opposite direction stops that velocity; space/s stops both; q quits.')
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            if not select.select([sys.stdin], [], [], 0.05)[0]:
                continue
            key = os.read(sys.stdin.fileno(), 1).decode(errors='ignore').lower()
            if key in ('q', '\x03', '\x04', ''):
                break
            if safety_active:
                continue
            if key in ('w', 'x'):
                speed = 0.5 if key == 'w' else -0.5
                command.linear.x = 0.0 if command.linear.x * speed < 0 else speed
            elif key in ('a', 'd'):
                speed = 0.3 if key == 'a' else -0.3
                command.angular.z = 0.0 if command.angular.z * speed < 0 else speed
            elif key in (' ', 's'):
                command = Twist()
            else:
                continue
            publisher.publish(command)
            print(f'linear: {command.linear.x:+.2f} m/s  '
                  f'angular: {command.angular.z:+.2f} rad/s', flush=True)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if rclpy.ok():
            publisher.publish(Twist())
        node.destroy_node()


def main():
    interactive = '--keyboard' in sys.argv
    rclpy.init(args=[arg for arg in sys.argv[1:] if arg != '--keyboard'])
    node = None
    try:
        if interactive:
            keyboard()
        else:
            node = SafeDrive()
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.publisher.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
