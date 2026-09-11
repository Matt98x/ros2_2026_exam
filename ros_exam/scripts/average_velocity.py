#!/usr/bin/env python3
"""Print average velocities every two seconds using a persistent service client."""
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros_exam.srv import GetAverageVelocity


def main():
    rclpy.init()
    node = Node('average_velocity_display')
    client = node.create_client(GetAverageVelocity, '/get_average_velocity')

    try:
        while rclpy.ok():
            if not client.wait_for_service(timeout_sec=1.0):
                continue

            future = client.call_async(GetAverageVelocity.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            if not future.done():
                client.remove_pending_request(future)
                future.cancel()
                continue

            result = future.result()
            print(f'success={result.success}  samples={result.sample_count}  '
                  f'linear={result.average_linear:.3f} m/s  '
                  f'angular={result.average_angular:.3f} rad/s', flush=True)
            time.sleep(2.0)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
