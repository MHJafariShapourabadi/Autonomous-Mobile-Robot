#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from robot_interfaces.msg import LedState
from robot_interfaces.srv import SetLed
from robot_interfaces.msg import BatteryState

class RobotLedNode(Node):
    def __init__(self):
        super().__init__(node_name="robot_led")

        self.led_state = LedState()
        self.led_state.led_state_arr = [False, False, True]
        self.led_state.battery_state = BatteryState.HIGH

        self.led_pub = self.create_publisher(
            msg_type=LedState,
            topic="led_state",
            qos_profile= QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )

        self.led_service = self.create_service(
            srv_type=SetLed,
            srv_name="set_led_state",
            callback=self.set_led_callback
        )

        self.get_logger().info(f"Node {self.get_name()} is initialized")

    def set_led_callback(self, request: SetLed.Request, response: SetLed.Response):
        self.led_state = request.led_state
        response.success = True
        self.led_pub.publish(self.led_state)
        return response
    
def main(args=None):
    try:
        rclpy.init(args=args)
        node = RobotLedNode()
        rclpy.spin(node=node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("KeyboardInterrupt")


