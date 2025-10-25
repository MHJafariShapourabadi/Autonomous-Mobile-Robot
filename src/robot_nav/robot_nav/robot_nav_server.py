#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle, GoalResponse
from robot_interfaces.action import GoToPose
from geometry_msgs.msg import Pose
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from robot_interfaces.msg import BatteryState
from robot_interfaces.srv import ChargeBattery
import threading
import time
import copy

class RobotNavServerNode(Node):
    def __init__(self):
        super().__init__(node_name="robot_nav_server")

        self.cg = ReentrantCallbackGroup()
        self.current_goal_handle: ServerGoalHandle|None = None
        self.goal_lock = threading.Lock()
        self.robot_pose = Pose()
        self.robot_battery_state = BatteryState()

        self.nav_action_server = ActionServer(
            node=self,
            action_type=GoToPose,
            action_name="go_to_pose",
            goal_callback=self.goal_callback,
            execute_callback=self.execute_callback,
            callback_group= self.cg
        )

        self.battery_state_sub = self.create_subscription(
            msg_type=BatteryState,
            topic="battery_state",
            callback=self.battery_state_callback,
            qos_profile=10,
            callback_group=self.cg
        )

        self.charge_battery_client = self.create_client(
            srv_type=ChargeBattery,
            srv_name="charge_battery",
            callback_group=self.cg
        )   

        self.get_logger().info(f"Node {self.get_name()} is initialized")

    def battery_state_callback(self, msg: BatteryState):
            self.robot_battery_state.state = msg.state
            self.robot_battery_state.level = msg.level
            self.robot_battery_state.charging = msg.charging

    def goal_callback(self, goal: GoToPose.Goal):
        if self.robot_battery_state.state == BatteryState.LOW:
            self.get_logger().error("Battery is LOW, Rejecting the goal!")
            self.get_logger().warn("Requesting to charge the robot!")
            battery_charge_request = ChargeBattery.Request()
            battery_charge_request.charge = True
            self.charge_battery_client.call_async(request=battery_charge_request)
            return GoalResponse.REJECT
        
        with self.goal_lock:
            if self.current_goal_handle is not None and self.current_goal_handle.is_active:
                self.current_goal_handle.abort()

        self.get_logger().info("Acepting the goal")
        return GoalResponse.ACCEPT
         
    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Executing the goal")
        with self.goal_lock:
            self.current_goal_handle = goal_handle

        goal: GoToPose.Goal = goal_handle.request
        goal_pose = goal.goal_pose
        goal_x, goal_y, goal_theta = goal_pose.position.x, goal_pose.position.y, goal_pose.orientation.z

        result = GoToPose.Result()
        feedback = GoToPose.Feedback()

        while abs(goal_x - self.robot_pose.position.x) > 0.1 or abs(goal_y - self.robot_pose.position.y) > 0.1 or abs(goal_theta - self.robot_pose.orientation.z) > 0.1:
            if not goal_handle.is_active:
                result.final_pose = copy.deepcopy(self.robot_pose)
                return result
            self.robot_pose.position.x = self.robot_pose.position.x + 0.1 * (goal_x - self.robot_pose.position.x)
            self.robot_pose.position.y = self.robot_pose.position.y + 0.1 * (goal_y - self.robot_pose.position.y)
            self.robot_pose.orientation.z = self.robot_pose.orientation.z + 0.1 * (goal_theta - self.robot_pose.orientation.z)
            feedback.current_pose = copy.deepcopy(self.robot_pose)
            goal_handle.publish_feedback(feedback=feedback)
            self.get_logger().info(f"current position: x={self.robot_pose.position.x :0.2f} y={self.robot_pose.position.y :0.2f} theta={self.robot_pose.orientation.z: 0.2f}")
            time.sleep(0.3)

        goal_handle.succeed()
        result.final_pose = copy.deepcopy(self.robot_pose)
        return result


def main(args=None):
    try:
        rclpy.init(args=args)
        node = RobotNavServerNode()
        rclpy.spin(node=node, executor=MultiThreadedExecutor())
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("KeyboardInterrupt")

    