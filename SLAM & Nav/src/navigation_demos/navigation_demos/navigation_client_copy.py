import rclpy
from rclpy.node import Node
from package_with_interfaces.srv import StartNavigationToTarget
from geometry_msgs.msg import Pose, Point, Quaternion

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self.cli = self.create_client(StartNavigationToTarget, 'navigate_to_pose')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = StartNavigationToTarget.Request()

    def send_request(self):
        self.req.pose = Pose()
        self.req.pose.position = Point(x=1.0, y=0.0, z=0.0)
        self.req.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.future = self.cli.call_async(self.req)
        self.get_logger().info(
        'Result of navigation: for the pose (%.1f, %.1f, %.1f):' %
        (self.req.pose.position.x, self.req.pose.position.y, self.req.pose.position.z))
        
def main(args=None):
    rclpy.init(args=args)

    navigation_client = NavigationClient()
    navigation_client.send_request()

    rclpy.spin(navigation_client)

    rclpy.shutdown()

if __name__ == '__main__':
    main()