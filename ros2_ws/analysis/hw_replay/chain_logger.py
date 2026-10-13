#!/usr/bin/env python3
"""chain_logger.py <out.csv> - logs the perception chain's outputs and the bag's originals (8 Oct 2026).
Rows: sim_time,topic,data  for /camera_ray_clusters /person_positions_fused /predicted_person_positions (replayed)
and /bag_ray /bag_fused /bag_pred (the robot's recorded ones, remapped by the player)."""
import sys, rclpy
from rclpy.node import Node
from std_msgs.msg import String
TOPICS = ['/camera_ray_clusters', '/person_positions_fused', '/predicted_person_positions', '/bag_ray', '/bag_fused', '/bag_pred']


class L(Node):
    def __init__(self, path):
        super().__init__('chain_logger', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.f = open(path, 'w')
        self.f.write('t,topic,data\n')
        for tp in TOPICS:
            self.create_subscription(String, tp, lambda m, tp=tp: self.on(tp, m), 50)

    def on(self, tp, m):
        t = self.get_clock().now().nanoseconds * 1e-9
        self.f.write(f'{t:.3f},{tp},"{m.data}"\n')
        self.f.flush()


rclpy.init()
rclpy.spin(L(sys.argv[1]))
