import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from geometry_msgs.msg import PoseArray, PoseStamped

from robot_navigator import BasicNavigator, NavigationResult

class ArucoGoalNode(Node):
    def __init__(self):
        super().__init__("aruco_goal_node")

        # Declare and read parameters
        self.declare_parameter(
            name="activation_time",
            value=2.0,
            descriptor=ParameterDescriptor(
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

        self.declare_parameter(
            name="timeout",
            value="600",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                decription="Time to seek goal before goal is abandoned"
            )

        )

        self.activation_time = Duration(
            seconds=self.get_parameter("activation_time").get_parameter_value().double_value
        )

        self.stopping_distance = (
           self.get_parameter("stopping_distance").get_parameter_value().double_value
        )

        self.delta_t = Duration(
           seconds=self.get_parameter("delta_t").get_parameter_value().double_value
        )

        self.timeout = Duration(
            seconds=self.get_parameter("timeout").get_parameter_value().double_value
        )

        # Set up subscriptions
        self.poses_sub = self.create_subscription(
           PoseArray, "aruco_poses", self.pose_callback, 10
        )

        # Initialize local variables
        self.first_time_marker_seen = None
        self.last_time_marker_seen = None
        self.seeking_goal = False  

        # Initialize nav2 navigator
        self.navigator = BasicNavigator()

        self.get_logger().info("Started, waiting for tags...")


    def pose_callback(self, pose_msg: PoseArray) -> None:
        curr_time = self.get_clock().now()

        if self.seeking_goal: return

        # initialize time if unset
        if self.first_time_marker_seen is None:
            self.first_time_marker_seen = curr_time
            self.last_time_marker_seen = curr_time

        # if more time has ellapsed than delta_t, then reset first time seen
        if curr_time - self.last_time_marker_seen > self.delta_t:
            self.first_time_marker_seen = curr_time
        self.last_time_marker_seen = curr_time

        # if more than the activation time has ellapsed, call goal
        if self.last_time_marker_seen - self.first_time_marker_seen >= self.activation_time:
            self.first_time_marker_seen = curr_time
            self.set_goal(pose_msg)


    def set_goal(self, pose_array: PoseArray):
        self.get_logger().info("Aruco detected! Setting goal")

        # create a stamped pose as a goal 
        goal_pose = PoseStamped()
        goal_pose.header = pose_array.header
        goal_pose.pose = pose_array.poses[0]

        # wait until Nav2 is ready, then send the goal 
        self.navigator.waitUntilNav2Active()
        self.seeking_goal = True
        self.navogator.goToPose(goal_pose)

        # Keep doing stuff as long as the robot is moving towards the goal
        i = 0
        while not self.navigator.isNavComplete():
            # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info('Distance remaining: ' + '{:.2f}'.format(
                feedback.distance_remaining) + ' meters.')
 
                # If the navigation is taking too long, cancel navigation
                if Duration.from_msg(feedback.navigation_time) > self.timeout:
                    self.navigator.cancelNav()

        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif result == NavigationResult.CANCELED:
            self.get_logger().error('Goal was canceled!')
        elif result == NavigationResult.FAILED:
            self.get_logger().error('Goal failed!')
        else:
            self.get_logger().fatal('Goal has an invalid return status!')


    def __del__(self):
        # Shut down the ROS 2 Navigation Stack
        self.navigator.lifecycleShutdown()

        
def main():
    rclpy.init()
    node = ArucoGoalNode() 
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
