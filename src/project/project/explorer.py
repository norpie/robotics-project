import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import String, Int32
from turtlebot3_interfaces.action import Mission


class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info('Explorer node started')
        self._action_client = ActionClient(
            self,
            Mission,
            'mission',
        )
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Action server not available, waiting...')
        self.explorer_goal_subscription = self.create_subscription(
            Int32, 'explorer_goal', self.explorer_goal_callback, 10)
        self.explorer_goal_cancel_subscription = self.create_subscription(
            String, 'explorer_goal_cancel', self.explorer_goal_cancel_callback, 10)
        self.explorer_goal_status_publisher = self.create_publisher(Int32, 'explorer_goal_status', 10)
        self.explorer_goal_end = self.create_publisher(String, 'explorer_goal_end', 10)

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

    def explorer_goal_callback(self, msg):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        frontiers = msg.data
        goal_msg = Mission.Goal()
        goal_msg.frontiers = frontiers
        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        msg = String()
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
            msg.data = 'success'
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))
            msg.data = 'failed'
        self.get_logger().info('Sending goal end...')
        self.explorer_goal_end.publish(msg)

    def explorer_goal_cancel_callback(self, _):
        self.get_logger().info('Canceling exploring goal...')
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

    def feedback_callback(self, feedback):
        frontiers = feedback.feedback.frontiers
        self.get_logger().info('frontiers: {0}'.format(frontiers))
        self.explorer_goal_status_publisher.publish(Int32(data=frontiers))


def main():
    rclpy.init()
    explorer = Explorer()

    try:
        rclpy.spin(explorer)
    finally:
        explorer.destroy_node()

    try:
        rclpy.shutdown()
    except:
        print('Already shutdown')


if __name__ == '__main__':
    main()
