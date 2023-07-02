import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Quaternion

import math

class TableExplorer(Node):
    def __init__(self):
        super().__init__('table_explorer')

        # Parameters
        self.table_links = self.declare_parameter('table_links', ['link1', 'link2', 'link3', 'link4']).value

        # Activation and deactivation services
        self.srv_activate = self.create_service(SetBool, 'activate', self.on_activate, callback_group=ReentrantCallbackGroup())
        self.srv_deactivate = self.create_service(SetBool, 'deactivate', self.on_deactivate, callback_group=ReentrantCallbackGroup())

        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action client
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Navigation goals
        self.goals = []


    def on_activate(self, request, response):
        for link in self.table_links:
            try:
                trans = self.tf_buffer.lookup_transform(self.get_name(), link, rclpy.time.Time())
                goal = PoseStamped()
                goal.header.frame_id = link
                goal.pose.position.x = trans.transform.translation.x
                goal.pose.position.y = trans.transform.translation.y
                goal.pose.position.z = trans.transform.translation.z
                goal.pose.orientation = self.turn_towards_table(trans.transform.translation)
                self.goals.append(goal)
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn('Failed to lookup transform: {0}'.format(e))

        self.send_goal(self.goals.pop(0))

        response.success = True
        response.message = 'Node activated.'
        return response

    def on_deactivate(self, request, response):
        self.client.cancel_all_goals()

        response.success = True
        response.message = 'Node deactivated.'
        return response

    def send_goal(self, goal):
        self.get_logger().info('Sending goal to {0}'.format(goal.header.frame_id))
        self.client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal
        self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback).add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Distance to goal: {0}'.format(feedback.distance_remaining))

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal to {0} rejected.'.format(goal_handle.pose.header.frame_id))
            return

        self.get_logger().info('Goal to {0} accepted.'.format(goal_handle.pose.header.frame_id))
        goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info('Reached goal with result {0}'.format(result.number_of_recoveries))

        if self.goals:
            self.send_goal(self.goals.pop(0))

    def turn_towards_table(self, table_position):
        angle = math.atan2(table_position.y, table_position.x)
        return Quaternion(0, 0, angle)

def main(args=None):
    rclpy.init(args=args)
    node = TableExplorer()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        executor.remove_node(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
