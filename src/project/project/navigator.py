import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
import action_msgs.msg
from std_msgs.msg import String, Bool
from .models import Position, Orientation, Pose, Waypoint
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav2_simple_commander.robot_navigator import BasicNavigator


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.subscriber = self.create_subscription(String, 'navigator_goal', self.waypoint_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.map_subscriber = self.create_subscription(String, 'navigator_map', self.map_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.cancel_subscriber = self.create_subscription(String, 'navigator_goal_cancel', self.cancel_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.init_subscriber = self.create_subscription(String, 'navigator_init', self.init, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.complete_publisher = self.create_publisher(Bool, 'navigator_goal_complete', 10)
        self.status_publisher = self.create_publisher(String, 'navigator_goal_status', 10)
        self.logger = self.get_logger()
        self.navigator = None
        self.completed = False
        self.started = False
        self.logger.info("Navigator waiting for init pose...")

    # Callback for init pose
    def init(self, msg):
        if self.navigator is not None:
            self.navigator.cancelTask()
            self.navigator = None
        self.navigator = BasicNavigator()
        try:
            x, y, o_z, o_w = msg.data.split(',')
            x = float(x)
            y = float(y)
            o_z = float(o_z)
            o_w = float(o_w)
            self.logger.info(f"Received init pose: {x}, {y}, {o_z}, {o_w}")
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = o_z
            pose.pose.orientation.w = o_w
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            self.navigator.setInitialPose(pose)
            self.navigator.waitUntilNav2Active()
        except:
            self.logger.info("Error processing user input")
        self.status_publisher_timer = self.create_timer(1, self.publish_status)
        self.logger.info("Navigator ready")


    def map_callback(self, msg):
        self.logger.info("Received map, path: " + msg.data)
        self.started = False
        self.completed = False
        try:
            self.navigator.changeMap(msg.data)
            self.navigator.clearAllCostmaps()
        except Exception as e:
            self.logger.info(f"Error processing user input: {e}")


    def publish_status(self):
        msg = String()
        nav_status = self.navigator.status
        if nav_status is None:
            msg.data = str("No status")
        else:
            msg.data = str(nav_status)
        self.status_publisher.publish(msg)
        if self.navigator.isTaskComplete() and self.started and not self.completed:
            self.complete_publisher.publish(Bool(data=True))
            self.completed = True
            self.started = False
            self.logger.info("Goal complete")


    def waypoint_callback(self, msg):
        self.completed = False
        self.logger.info("Received data")
        try:
            x, y, o_z, o_w = msg.data.split(',')
            x = float(x)
            y = float(y)
            o_z = float(o_z)
            o_w = float(o_w)
            self.logger.info(f"Received waypoint: {x}, {y}, {o_z}, {o_w}")
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = o_z
            pose.pose.orientation.w = o_w
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            self.started = True
            self.logger.info("Before test")
            self.navigator.goToPose(pose)
        except Exception as e:
            self.logger.info(f"Error processing user input: {e}")


    def cancel_callback(self, _):
        if self.started:
            self.logger.info("Canceling goal")
            self.navigator.cancelTask()
            self.completed = False
            self.started = True


def main():
    rclpy.init()
    navigator = Navigator()

    try:
        rclpy.spin(navigator)
    finally:
        navigator.destroy_node()

    try:
        rclpy.shutdown()
    except:
        print('Already shutdown')


if __name__ == '__main__':
    main()
