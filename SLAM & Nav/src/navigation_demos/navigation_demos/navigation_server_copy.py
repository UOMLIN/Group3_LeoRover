import subprocess
import rclpy
from rclpy.node import Node
from package_with_interfaces.srv import StartNavigationToTarget

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.publisher import Publisher

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
import time

class NavigationServer(Node):
    def __init__(self):
        super().__init__('navigation_server')
        self.srv = self.create_service(StartNavigationToTarget, 'navigate_to_pose', self.navigate_to_pose_callback)
        self.goal_pose = None  # This will store the goal pose
        # self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
    def navigate_to_pose_callback(self, request, response):
        self.get_logger().info('Received navigation request')

        # Store the goal pose
        self.goal_pose = request.pose

        # Now, regardless of the pose received, we respond with success
        response.success = True

        # # Publish the goal pose
        # goal_msg = PoseStamped()
        # goal_msg.pose = self.goal_pose
        # # The goal_msg containing the goal pose is published to the /goal_pose topic.
        # self.goal_publisher.publish(goal_msg)

        # Launch the nav_demo.launch.py file
        subprocess.run(['ros2', 'launch', 'navigation_demos', 'nav_demo.launch.py'])

        self.get_logger().info(f"Received Goal pose is: {self.goal_pose}")

        # Send the goal to the /navigate_to_pose action
        self.send_goal(self.goal_pose)

        return response
    
    # def send_goal(self, pose):
    #     goal_msg = NavigateToPose.Goal()
    #     goal_msg.pose.header = Header(stamp=Time(sec=0, nanosec=0), frame_id='map')
    #     goal_msg.pose.pose = pose
    #     goal_msg.behavior_tree = ''

    #     try:
    #         self.action_client.wait_for_server()
    #         future = self.action_client.send_goal_async(goal_msg)
    #         rclpy.spin_until_future_complete(self, future)
    #         goal_response = future.result()

    #         if not goal_response.accepted:
    #             self.get_logger().error('Goal rejected by action server')
    #         else:
    #             self.get_logger().info('Goal accepted by action server')
    #     except Exception as e:
    #         self.get_logger().error('Failed to send goal: ' + str(e))
    def send_goal(self, pose):
        self.get_logger().info("Waiting for 'NavigateToPose' action server")
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header = Header(stamp=Time(sec=0, nanosec=0), frame_id='map')
        goal_msg.pose.pose = pose
        goal_msg.behavior_tree = ''

        self.get_logger().info(
            'Navigating to goal: '
            + str(pose.position.x)
            + ' '
            + str(pose.position.y)
            + '...'
        )
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error(
                'Goal to '
                + str(pose.position.x)
                + ' '
                + str(pose.position.y)
                + ' was rejected!'
            )
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        start_navigation_to_target_node = NavigationServer()

        rclpy.spin(start_navigation_to_target_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()