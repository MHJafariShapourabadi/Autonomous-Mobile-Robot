#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from robot_interfaces.msg import BatteryState
from robot_interfaces.srv import ConsumeBattery, ChargeBattery
from robot_interfaces.srv import SetLed
from sensor_msgs.msg import BatteryState as SensorBatteryState
from std_msgs.msg import Bool

class RobotBatteryNode(Node):
    def __init__(self):
        super().__init__(node_name="robot_battery")

        self.cg1 = MutuallyExclusiveCallbackGroup()
        self.cg2 = MutuallyExclusiveCallbackGroup()

        self.battery_level = 100.0
        self.battery_state = BatteryState.HIGH
        self.charging = False

        self.update_battery_sub = self.create_subscription(
            msg_type=SensorBatteryState,
            topic="/battery/state",
            callback=self.update_battery_state,
            qos_profile=10
        )

        self.battery_service = self.create_service(
            srv_type=ChargeBattery,
            srv_name="charge_battery",
            callback=self.charge_battery_callback,
            callback_group=self.cg1
        )

        self.led_client = self.create_client(
            srv_type=SetLed,
            srv_name="set_led_state",
            callback_group=self.cg1
        )

        self.battery_state_pub = self.create_publisher(
            msg_type=BatteryState,
            topic="battery_state",
            qos_profile= 10,
            callback_group=self.cg2
        )

        self.start_charge_battery_pub = self.create_publisher(
            msg_type=Bool,
            topic="/battery/recharge/start",
            qos_profile= 10,
            callback_group=self.cg2
        )

        self.stop_charge_battery_pub = self.create_publisher(
            msg_type=Bool,
            topic="/battery/recharge/stop",
            qos_profile= 10,
            callback_group=self.cg2
        )

        self.get_logger().info(f"Node {self.get_name()} is initialized")

    def update_battery_state(self, msg: SensorBatteryState):
        self.battery_level = msg.percentage
        if self.battery_level >= 80 and self.battery_state != BatteryState.HIGH:
            self.battery_state = BatteryState.HIGH
            request = SetLed.Request()
            request.led_state.led_state_arr = [False, False, True]
            request.led_state.battery_state = self.battery_state
            self.led_client.call_async(request=request)
        elif 30 <= self.battery_level < 80 and self.battery_state != BatteryState.MEDIUM:
            self.battery_state = BatteryState.MEDIUM
            request = SetLed.Request()
            request.led_state.led_state_arr = [False, True, False]
            request.led_state.battery_state = self.battery_state
            self.led_client.call_async(request=request)
        elif self.battery_level < 30 and self.battery_state != BatteryState.LOW:
            self.battery_state = BatteryState.LOW
            request = SetLed.Request()
            request.led_state.led_state_arr = [True, False, False]
            request.led_state.battery_state = self.battery_state
            self.led_client.call_async(request=request)
        if self.battery_level == 100:
            self.charging = False
            msg = Bool()
            msg.data = True
            self.stop_charge_battery_pub.publish(msg)

        battery_state_msg = BatteryState()
        battery_state_msg.state = self.battery_state
        battery_state_msg.level = float(self.battery_level)
        battery_state_msg.charging = self.charging
        self.battery_state_pub.publish(battery_state_msg)
        
    
    def charge_battery_callback(self, request: ChargeBattery.Request, response: ChargeBattery.Response):
        self.charging = request.charge
        msg = Bool()
        msg.data = True
        self.start_charge_battery_pub.publish(msg)
        response.success = True
        return response

    
def main(args=None):
    try:
        rclpy.init(args=args)
        node = RobotBatteryNode()
        rclpy.spin(node=node, executor=MultiThreadedExecutor())
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("KeyboardInterrupt")


