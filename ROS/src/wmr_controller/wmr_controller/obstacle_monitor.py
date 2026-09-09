#!/usr/bin/env python3
import copy
from concurrent.futures import ThreadPoolExecutor
from email import header

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, Vector3
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from wmr_controller.dbastar_controller_node import dbastarControllerNode
from wmr_interfaces.msg import AABB2DArray, AABB2D
from wmr_controller.aabb_utils import (
    aabb_map_from_messages,
    compare_aabb_maps,
    planner_obstacles,
)
import numpy as np
import sys
import os
import time
import glob
import yaml
BENCHMARK_FILE = "/home/lndw/wmr-ros/ROS/src/wmr_controller/external/realtime-dbastar/baselines/wmr-simulator/problems/benchmark/benchmark.yaml"
venv_pattern = os.path.expanduser('~/wmr-ros/ROS/.venv/lib/python3.*/site-packages')
venv_matches = glob.glob(venv_pattern)
if venv_matches:
    venv_path = venv_matches[0]
    if venv_path not in sys.path:
        sys.path.insert(0, venv_path)


class ObstacleMonitoringNode(Node):
    def __init__(self):
        super().__init__('obstacle_monitoring_node')
        
        # Parameters
        self.declare_parameter('obstacle_name', 'Obstacle01')
        self.declare_parameter('mocap_topic', '/poses')
        self.declare_parameter('obstacle_topic', '/obstacles_aabb')
        self.declare_parameter('obstacle_change_tolerance', 0.02)

        self.obstacle_name = str(self.get_parameter('obstacle_name').value)
        mocap_topic = str(self.get_parameter('mocap_topic').value)
        obstacle_topic = str(self.get_parameter('obstacle_topic').value)
        self.obstacle_change_tolerance = float(
            self.get_parameter('obstacle_change_tolerance').value
        )

        self.latest_pose = None
        self.initialized = False

        self.half_edge_length = 0.15
        self.z_previous = 100000
        self.replanning_flag = False

        if self.obstacle_change_tolerance < 0.0:
            raise ValueError("obstacle_change_tolerance must be non-negative")

        # QoS for mocap (BEST_EFFORT like controller_interface)
        mocap_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS for control commands (BEST_EFFORT, depth=1)
        obs_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to mocap poses
        self.mocap_sub = self.create_subscription(
            NamedPoseArray,
            mocap_topic,
            self.poses_callback,
            mocap_qos
        )
        
        #publish velocity commands (v, w) on /cmd_unicycle topic 
        self.obs_pub = self.create_publisher(AABB2DArray, obstacle_topic, obs_qos)
        
        
    def poses_callback(self, msg: NamedPoseArray):
        """Callback for motion capture poses"""

        #idea: mocap publishes at its own rate. The last pose that was incoming is stored for usage at the defined control loop execution rate.
        for pose in msg.poses:
            if pose.name == self.obstacle_name:
                self.latest_pose = pose.pose

                #init pose. Take zero if mocap doesn't have a pose yet.
                if not self.initialized:
                    x0 = pose.pose.position.x
                    y0 = pose.pose.position.y
                    z0 = pose.pose.position.z

                    if self.z_previous <= 0.5 and z0 > 0.5 and not self.replanning_flag:
                        self.get_logger().warn(f'obstacle is removed ')

                        x_min = pose.pose.position.x - self.half_edge_length
                        x_max = pose.pose.position.x + self.half_edge_length
                        y_min = pose.pose.position.y - self.half_edge_length
                        y_max = pose.pose.position.y + self.half_edge_length
                        box = AABB2D(
                            id=self.obstacle_name,
                            min_x=x_min,
                            min_y=y_min,
                            max_x=x_max,
                            max_y=y_max,
                        )
                        snapshot = AABB2DArray()
                        snapshot.header.stamp = self.get_clock().now().to_msg()
                        snapshot.header.frame_id = 'world'
                        snapshot.boxes = [] if box is None else [box]
                        self.obs_pub.publish(snapshot)
                        self.replanning_flag = True
                    elif self.z_previous > 0.5 and z0 <= 0.5 and not self.replanning_flag:
                        self.get_logger().warn(f'obstacle is added ')

                        x_min = pose.pose.position.x - self.half_edge_length
                        x_max = pose.pose.position.x + self.half_edge_length
                        y_min = pose.pose.position.y - self.half_edge_length
                        y_max = pose.pose.position.y + self.half_edge_length
                        box = AABB2D(
                            id=self.obstacle_name,
                            min_x=x_min,
                            min_y=y_min,
                            max_x=x_max,
                            max_y=y_max,
                        )
                        snapshot = AABB2DArray()
                        snapshot.header.stamp = self.get_clock().now().to_msg()
                        snapshot.header.frame_id = 'world'
                        snapshot.boxes = [] if box is None else [box]
                        self.obs_pub.publish(snapshot)
                        self.replanning_flag = True
                    
                    self.z_previous = z0
                    self.initialized = True
                    self.get_logger().info(f'Initialized at x={x0:.3f}, y={y0:.3f}, z={z0:.3f}')
                    continue

                if self.z_previous <= 0.5 and pose.pose.position.z > 0.5 and not self.replanning_flag:
                    self.get_logger().warn(f'obstacle is removed ')

                    x_min = pose.pose.position.x - self.half_edge_length
                    x_max = pose.pose.position.x + self.half_edge_length
                    y_min = pose.pose.position.y - self.half_edge_length
                    y_max = pose.pose.position.y + self.half_edge_length


                    box = AABB2D(
                        id=self.obstacle_name,
                        min_x=x_min,
                        min_y=y_min,
                        max_x=x_max,
                        max_y=y_max,
                    )
                    snapshot = AABB2DArray()
                    snapshot.header.stamp = self.get_clock().now().to_msg()
                    snapshot.header.frame_id = 'world'
                    snapshot.boxes = [] if box is None else [box]
                    self.obs_pub.publish(snapshot)
                    self.replanning_flag = True
                elif self.z_previous > 0.5 and pose.pose.position.z <= 0.5 and not self.replanning_flag:
                    self.get_logger().warn(f'obstacle is added ')
                        
                    x_min = pose.pose.position.x - self.half_edge_length
                    x_max = pose.pose.position.x + self.half_edge_length
                    y_min = pose.pose.position.y - self.half_edge_length
                    y_max = pose.pose.position.y + self.half_edge_length
                    box = AABB2D(
                            id=self.obstacle_name,
                            min_x=x_min,
                            min_y=y_min,
                            max_x=x_max,
                            max_y=y_max,
                        )
                    snapshot = AABB2DArray()
                    snapshot.header.stamp = self.get_clock().now().to_msg()
                    snapshot.header.frame_id = 'world'
                    snapshot.boxes = [] if box is None else [box]
                    self.obs_pub.publish(snapshot)
                    self.replanning_flag = True
                self.z_previous = pose.pose.position.z
                break


    
   
def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMonitoringNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()