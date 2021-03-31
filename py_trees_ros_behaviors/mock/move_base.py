#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://github.com/splintered-reality/py_trees_ros_tutorials/raw/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Mocks a simple action server that rotates the robot 360 degrees.
"""


##############################################################################
# Imports
##############################################################################

import argparse
import geometry_msgs.msg as geometry_msgs
import nav_msgs.msg as nav_msgs
import std_msgs.msg as std_msgs
import py_trees_ros.mock.actions
import py_trees_ros_interfaces.action as py_trees_actions
import rclpy
from rclpy.node import Node
import sys

##############################################################################
# Class
##############################################################################

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(geometry_msgs.PoseStamped, '/turtlebot1/send_goal', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    def timer_callback(self):
        msg = std_msgs.String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def publish_pose(self, pose):
        self.publisher_.publish(pose)

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            std_msgs.String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

class MoveBase(py_trees_ros.mock.actions.GenericServer):
    """
    Simulates a move base style interface.

    Node Name:
        * **move_base_controller**

    Action Servers:
        * **/move_base** (:class:`py_trees_ros_interfaces.action.MoveBase`)

          * point to point move base action

    Args:
        duration: mocked duration of a successful action
    """
    def __init__(self, duration=None):
        super().__init__(
            node_name="move_base_controller",
            action_name="move_base",
            action_type=py_trees_actions.MoveBase,
            generate_feedback_message=self.generate_feedback_message,
            goal_received_callback=self.execute_goal,
            duration=duration
        )
        self.pose = geometry_msgs.PoseStamped()
        self.publisher_ = self.node.create_publisher(geometry_msgs.PoseStamped, '/turtlebot1/send_goal', 10)
        self.subscription = self.node.create_subscription(
            std_msgs.String,
            '/turtlebot1/mb_feedback',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.flag_goal_completed = False
        self.pose.pose.position = geometry_msgs.Point(x=0.0, y=0.0, z=0.0)
        self.subscriber_ = MinimalSubscriber()

    def listener_callback(self, msg):
        self.node.get_logger().info('I heard: "%s"' % msg.data)
        if msg.data == "Done":
            print("Goal achieved")
            self.flag_goal_completed = True

    def execute_goal(self, goal):
        self.pose = goal.target_pose
        self.flag_goal_completed = False
        self.publisher_.publish(self.pose)

    def generate_feedback_message(self) -> py_trees_actions.MoveBase.Feedback:
        """
        Do a fake pose incremenet and populate the feedback message.

        Returns:
            :class:`py_trees_actions.MoveBase.Feedback`: the populated feedback message
        """
        # actually doesn't go to the goal right now...
        # but we could take the feedback from the action
        # and increment this to that proportion
        # self.odometry.pose.pose.position.x += 0.01
        self.pose.pose.position.x += 0.01
        self.pose.header.frame_id = "odom"
        msg = py_trees_actions.MoveBase.Feedback()  # .Feedback() is more proper, but indexing can't find it
        msg.base_position = self.pose
        # print(self.percent_completed)
        # self.publisher_.publish(self.pose)
        if not self.flag_goal_completed:
            self.percent_completed = 13.33
        return msg


def main():
    """
    Entry point for the mock move base node.
    """
    parser = argparse.ArgumentParser(description='Mock a docking controller')
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    parser.parse_args(command_line_args)

    rclpy.init()  # picks up sys.argv automagically internally
    move_base = MoveBase()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(move_base.node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        move_base.abort()
        # caveat: often broken, with multiple spin_once or shutdown, error is the
        # mysterious:
        #   The following exception was never retrieved: PyCapsule_GetPointer
        #   called with invalid PyCapsule object
        executor.shutdown()  # finishes all remaining work and exits

    move_base.shutdown()
    rclpy.shutdown()
