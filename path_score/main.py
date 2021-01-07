#!/usr/bin/env python3
"""
Visualizes the CTE of some simple paths using rviz2
"""
import sys
import threading
from time import sleep
from typing import Optional, Iterable, Callable

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from path_score.generate_markers import get_marker
from path_score.generate_paths import generate_paths
from path_score.helpers import Env, state_t
from path_score.score_paths import score_paths


def main(args: Optional[Iterable[str]] = None):
    rclpy.init(args=args)
    env: Env = Env(Node('path_score'))
    info: Callable[[str], None] = env.nh.get_logger().info
    info("Starting up...")

    # For Rate objects to work
    thread = threading.Thread(target=rclpy.spin, args=(env.nh,), daemon=True)
    thread.start()

    # Initialize Publishers and Subscribers
    env.m_pub = env.nh.create_publisher(Marker, 'paths', 50)

    # Dummy Global Path
    from itertools import chain
    env.path = np.fromiter(chain.from_iterable(
        (2 * t, t ** 2 / 10) for t in np.arange(0.1, 10, 0.5)
    ), dtype=np.float).reshape((-1, 2))

    r = env.nh.create_rate(1)
    while rclpy.ok() and len(env.path) > 10:
        # Remove passed points
        # Note fails if robot and path have very different orientations, check for that

        update_global_path(env)

        # Publish Global Path and Current Position
        env.m_pub.publish(
            get_marker(env.nh.get_clock(), 50, [env.state[:2]], scale=0.5,
                       color=ColorRGBA(r=1.0, b=1.0, a=1.0)))
        sleep(0.01)

        env.m_pub.publish(get_marker(env.nh.get_clock(), 51, env.path, scale=0.2))
        sleep(0.01)

        paths = generate_paths(env)

        best_trajectory, cost = score_paths(env, paths)

        info(f"Lowest {cost=:.2f}")

        r.sleep()

        # Dummy state update
        #            [ x , y , theta, vel]
        env.state += [0.5, 0.1, 0.05, 0.1]

    env.nh.destroy_node()
    rclpy.shutdown()
    thread.join()


def update_global_path(env: Env):
    def line_behind_vehicle(x: float, y: float) -> float:
        p: state_t = env.state
        return (x - p[0]) * np.cos(p[2]) + (y - p[1]) * np.sin(p[2])

    def is_behind(x: float, y: float) -> bool:
        return line_behind_vehicle(x, y) < 0

    while is_behind(*env.path[0]):
        env.path = env.path[1:, :]


if __name__ == '__main__':
    main(sys.argv)
