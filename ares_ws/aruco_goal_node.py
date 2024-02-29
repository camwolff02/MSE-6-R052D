import time
from typing import NamedTuple

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers


class ArucoGoalNode(Node):
    def __init__(self):
        super().__init__("aruco_goal_node")

        # Declare and read parameters
        self.declare_parameter(
            name="activation_time",
            value=1.0,
            desciptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Time in seconds to have marker in frame before setting goal",
            ),
        )

        self.declare_parameter(
            name="stopping_distance",
            value=2.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Stopping distance for marker",
            ),
        )

        self.declare_parameter(
            name="delta_t",
            value=0.5,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Threshhold for time between sensor readings for two to be considered 'consecutive'",
            ),
        )

        self.activation_time = (
            self.get_parameter("activation_time").get_parameter_value().double_value
        )

        self.stopping_distance = (
           self.get_parameter("stopping_distance").get_parameter_value().double_value
        )

        self.delta_t = (
           self.get_parameter("delta_t").get_parameter_value().double_value
        )

        # Set up subscriptions
        self.poses_sub = self.create_subscription(
           PoseArray, "aruco_poses", self.pose_callback, 10
        )

        # Initialize local variables
        self.first_time_marker_seen = 0
        self.last_time_marker_seen = 0


    def pose_callback(self, pose_msg):
        curr_time = self.get_clock().now()

        # if more time has ellapsed than delta_t, then reset first time seen
        if curr_time - self.last_time_marker_seen > self.delta_t:
            self.first_time_marker_seen = curr_time
        self.last_time_marker_seen = curr_time

        # if more than the activation time has ellapsed, call goal
        if self.last_time_marker_seen - self.first_time_marker_seen >= self.activation_time:
            self.first_time_marker_seen = curr_time
            self.set_goal(pose_msg[0])


    def set_goal(self, pose):
        self.get_logger().info("Going to goal!")
     

def main():
    rclpy.init()
    node = ArucoGoalNode() 
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
