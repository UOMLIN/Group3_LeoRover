#!/usr/bin/env python3

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from time import sleep
from rclpy.node import Node
import rclpy

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool  # Import Bool message type
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS


class ParameterListener(Node):
    def __init__(self):
        super().__init__('parameter_listener')
        self.point_subscription = self.create_subscription(
            PoseStamped,
            'object_position_in_map',  # Topic for Point message
            self.point_update_callback,
            10
        )
        self.bool_subscription = self.create_subscription(
            Bool,
            'param_updates_bool',  # Topic for Bool message
            self.bool_update_callback,
            10
        )
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.new_data_available = False
        self.stand = False
        self.point_subscription  # prevent unused variable warning
        self.bool_subscription  # prevent unused variable warning

    def point_update_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.new_data_available = True
        self.get_logger().info(f"Updated parameters: x={self.x}, y={self.y}, z={self.z}")

    def bool_update_callback(self, msg):
        self.stand = msg.data
        self.get_logger().info(f"Received boolean value: {self.stand}")



def main():
    bot = InterbotixManipulatorXS(
        robot_model='px150',
        group_name='arm',
        gripper_name='gripper'
    )

    #   bot.gripper.left_finger_lower_limit=0.01

    listener = ParameterListener()

    try:
        while rclpy.ok():
            rclpy.spin_once(listener,timeout_sec=0.1)
            if listener.new_data_available:
                bot.arm.go_to_home_pose()
                bot.gripper.release()
                rclpy.spin_once(listener, timeout_sec=0.1)
                if listener.stand:
                    bot.arm.set_ee_pose_components(x=listener.x, z=listener.z,  moving_time=4)
                else:
                    bot.arm.set_ee_pose_components(x=listener.x, z=listener.z,pitch = 1.5, moving_time=4)
                print('x=', listener.x)
                print('stand=',listener.stand)
                bot.gripper.grasp(3)
                bot.arm.set_ee_pose_components(x=-0.25, z=0.15, moving_time=4)
                bot.gripper.release()
                bot.gripper.grasp()
                bot.arm.go_to_home_pose()
                bot.arm.go_to_sleep_pose()
                bot.shutdown()
    except:
        bot.shutdown()


if __name__ == '__main__':
    main()
