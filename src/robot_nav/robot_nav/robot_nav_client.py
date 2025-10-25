#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from robot_interfaces.action import GoToPose
from geometry_msgs.msg import Pose
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class RobotNavClientNode(Node):
    def __init__(self):
        super().__init__(node_name="robot_nav_server")

        self.cg = ReentrantCallbackGroup()

        self.current_goal_handle: ClientGoalHandle|None = None
        
        self.nav_action_client = ActionClient(
            node=self,
            action_type=GoToPose,
            action_name="go_to_pose",
        )

        self.get_logger().info(f"Node {self.get_name()} is initialized")

    def send_goal(self, x: float, y: float, theta: float):
        while not self.nav_action_client.wait_for_server(2.0):
            self.get_logger().info(f"Waiting for the action server")

        goal = GoToPose.Goal()
        goal.goal_pose.position.x = x
        goal.goal_pose.position.y = y
        goal.goal_pose.orientation.z = theta

        future = self.nav_action_client.send_goal_async(goal=goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(callback=None)

    def goal_response_callback(self, futrue: rclpy.Future):
        self.current_goal_handle: ClientGoalHandle = futrue.result()
        if self.current_goal_handle.accepted:
            futrue = self.current_goal_handle.get_result_async()
            futrue.add_done_callback(callback=None)

    def goal_result_callback(self, future: rclpy.Future):
        result: GoToPose.Result = future.result().result
        self.get_logger().info(f"Reached final pose: x={result.final_pose.position.x :0.2f} y={result.final_pose.position.y :0.2f} theta={result.final_pose.orientation.z :0.2f}")

    def feedback_callback(self, feedback_msg):
        feedback:GoToPose.Feedback = feedback_msg.feedback
        self.get_logger().info(f"Current position: x={feedback.current_pose.position.x :0.2f} y={feedback.current_pose.position.y :0.2f} theta={feedback.current_pose.orientation.z :0.2f}")


def main(args=None):
    try:
        rclpy.init(args=args)
        node = RobotNavClientNode()
        rclpy.spin(node=node, executor=MultiThreadedExecutor())
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("KeyboardInterrupt")

    