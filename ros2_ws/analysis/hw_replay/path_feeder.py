#!/usr/bin/env python3
"""path_feeder.py - sends the robot bag's global plans to the replay controller (8 Oct 2026).

Subscribes /bag_plan (the bag's /turtlebot4/plan, remapped) and sends each new plan as a FollowPath goal on
/follow_path, at most every 0.9 s (sim time), as the behaviour tree does on the robot. Results are ignored;
the robot's motion comes from the bag's TF.
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath


class Feeder(Node):
    def __init__(self):
        super().__init__('replay_path_feeder', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.cli = ActionClient(self, FollowPath, 'follow_path')
        self.create_subscription(Path, 'bag_plan', self.on_plan, 10)
        self.last = None
        self.sent = 0

    def on_plan(self, msg):
        if not msg.poses:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last is not None and now - self.last < 0.9:
            return
        if not self.cli.server_is_ready():
            return
        g = FollowPath.Goal()
        g.path = msg
        g.controller_id = 'FollowPath'
        g.goal_checker_id = 'general_goal_checker'
        self.cli.send_goal_async(g)
        self.last = now
        self.sent += 1
        if self.sent == 1:
            self.get_logger().info('first path sent to the controller')


def main():
    rclpy.init()
    rclpy.spin(Feeder())


if __name__ == '__main__':
    main()
