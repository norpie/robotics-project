import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray

class Input(Node):
    def __init__(self):
        self.node = rclpy.create_node('user_input_node')
        self.goal_publisher = self.node.create_publisher(String, 'navigation_goals', 10)
        self.cancel_publisher = self.node.create_publisher(String, 'cancel_goal', 10)
        self.status_subscriber = self.node.create_subscription(GoalStatusArray, 'navigation_status', self.status_callback, 10)
        self.logger = self.node.get_logger()
        self.get_user_input()

    def status_callback(self, msg):
        for status in msg.status_list:
            if status.status == 3:  # 3 corresponds to STATUS_SUCCEEDED
                print("Navigation goal reached successfully")
            elif status.status == 4:  # 4 corresponds to STATUS_ABORTED
                print("Navigation goal aborted or failed")

    def get_user_input(self):
        # self.logger.info("Enter coordinates (x,y) or 'cancel' to cancel: ")
        # cancelled = False
        # while not cancelled:
        self.logger.info("Enter coordinates (x,y) or 'cancel' to cancel: ")
        user_input = input()
        # user_input = "1,2"
        self.logger.info("User input: " + user_input)
            # if user_input.lower() == 'cancel':
                # cancel_msg = String()
                # cancel_msg.data = 'cancel'
                # self.cancel_publisher.publish(cancel_msg)
            # else:
                # goal_msg = String()
                # goal_msg.data = user_input
                # self.goal_publisher.publish(goal_msg)
            # canelled = self.node.is_shutdown()

def main():
    rclpy.init()
    input = Input()

    try:
        rclpy.spin(input.node)
    except KeyboardInterrupt:
        pass

    input.node.destroy_node()

if __name__ == '__main__':
    main()
