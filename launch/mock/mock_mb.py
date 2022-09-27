import argparse
import geometry_msgs.msg as geometry_msgs
import nav_msgs.msg as nav_msgs
import std_msgs.msg as std_msgs
import py_trees_ros.mock.actions
import py_trees_ros_interfaces.action as py_trees_actions
import rclpy
from rclpy.node import Node
import sys
import time

class MbMock(Node):

    def __init__(self):
        super().__init__('move_base_mock')
        self.publisher_ = self.create_publisher(std_msgs.String, '/turtlebot1/mb_feedback', 10)
        self.subscription = self.create_subscription(
            geometry_msgs.PoseStamped,
            '/turtlebot1/send_goal',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    def timer_callback(self):
        msg = std_msgs.String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def publish_msg(self, feedback):
        msg = std_msgs.String()
        msg.data = feedback
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        self.get_logger().info('Got new msg')
        # self.get_logger().info(msg)
        time.sleep(5)
        self.get_logger().info('Done')
        self.publish_msg('Done')

# class MinimalSubscriber(Node):

#     def __init__(self):
#         super().__init__('minimal_subscriber')
#         self.subscription = self.create_subscription(
#             std_msgs.String,
#             'topic',
#             self.listener_callback,
#             10)
#         self.subscription  # prevent unused variable warning

#     def listener_callback(self, msg):
#         self.get_logger().info('I heard: "%s"' % msg.data)

def main():
    """
    Entry point for the mock move base node.
    """
    parser = argparse.ArgumentParser(description='Mock a move_base controller')
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    parser.parse_args(command_line_args)

    rclpy.init()  # picks up sys.argv automagically internally
    mock_mb_node = MbMock()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(mock_mb_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        # mock_mb_node.abort()
        # caveat: often broken, with multiple spin_once or shutdown, error is the
        # mysterious:
        #   The following exception was never retrieved: PyCapsule_GetPointer
        #   called with invalid PyCapsule object
        executor.shutdown()  # finishes all remaining work and exits

    mock_mb_node.shutdown()
    rclpy.shutdown()