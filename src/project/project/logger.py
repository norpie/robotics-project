import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
from .util import LidarData, OdomData


class Logger(Node):

    def __init__(self):
        super().__init__('logger')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        # create a Twist message
        self.cmd = Twist()
        self.lidar = None
        self.odom = None
        self.timer = self.create_timer(0.3, self.log)

    def laser_callback(self, msg):
        self.lidar = LidarData(msg)

    def odom_callback(self, msg):
        self.odom = OdomData(msg)

    def log(self):
        if self.lidar is None:
            return
        self.get_logger().info('--------------------------------------------')
        self.get_logger().info('front: %s' % str(self.lidar.at(0)))
        self.get_logger().info('left: %s' % str(self.lidar.min(0, 15)))
        self.get_logger().info('90°: %s' % str(self.lidar.at(90)))
        self.get_logger().info('behind: %s' % str(self.lidar.at(180)))
        self.get_logger().info('270°: %s' % str(self.lidar.at(270)))
        self.get_logger().info('right: %s' % str(self.lidar.min(345, 360)))
        self.get_logger().info('angle: %s' % str(self.odom.degrees))
        self.get_logger().info('x: %s' % str(self.odom.x))
        self.get_logger().info('y: %s' % str(self.odom.y))
        self.get_logger().info('x: %s' % str(self.odom.z))


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    logger = Logger()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(logger)
    # Explicity destroy the node
    logger.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
