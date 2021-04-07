#
# License: BSD
#   https://github.com/splintered-reality/py_trees_ros_behaviors/raw/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Behaviours for the tutorials.
"""

##############################################################################
# Imports
##############################################################################

import py_trees
import py_trees_ros
import rcl_interfaces.msg as rcl_msgs
import rcl_interfaces.srv as rcl_srvs
import rclpy
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import nav_msgs.msg as nav_msgs

from typing import Any, Callable
import random, math
##############################################################################
# Behaviours
##############################################################################


class FlashLedStrip(py_trees.behaviour.Behaviour):
    """
    This behaviour simply shoots a command off to the LEDStrip to flash
    a certain colour and returns :attr:`~py_trees.common.Status.RUNNING`.
    Note that this behaviour will never return with
    :attr:`~py_trees.common.Status.SUCCESS` but will send a clearing
    command to the LEDStrip if it is cancelled or interrupted by a higher
    priority behaviour.

    Publishers:
        * **/led_strip/command** (:class:`std_msgs.msg.String`)

          * colourised string command for the led strip ['red', 'green', 'blue']

    Args:
        name: name of the behaviour
        topic_name : name of the battery state topic
        colour: colour to flash ['red', 'green', blue']
    """
    def __init__(
            self,
            name: str,
            topic_name: str="/led_strip/command",
            colour: str="red"
    ):
        super(FlashLedStrip, self).__init__(name=name)
        self.topic_name = topic_name
        self.colour = colour

    def setup(self, **kwargs):
        """
        Setup the publisher which will stream commands to the mock robot.

        Args:
            **kwargs (:obj:`dict`): look for the 'node' object being passed down from the tree

        Raises:
            :class:`KeyError`: if a ros2 node isn't passed under the key 'node' in kwargs
        """
        self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.publisher = self.node.create_publisher(
            msg_type=std_msgs.String,
            topic=self.topic_name,
            qos_profile=py_trees_ros.utilities.qos_profile_latched()
        )
        self.feedback_message = "publisher created"

    def update(self) -> py_trees.common.Status:
        """
        Annoy the led strip to keep firing every time it ticks over (the led strip will clear itself
        if no command is forthcoming within a certain period of time).
        This behaviour will only finish if it is terminated or priority interrupted from above.

        Returns:
            Always returns :attr:`~py_trees.common.Status.RUNNING`
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        self.publisher.publish(std_msgs.String(data=self.colour))
        self.feedback_message = "flashing {0}".format(self.colour)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status):
        """
        Shoot off a clearing command to the led strip.

        Args:
            new_status: the behaviour is transitioning to this new status
        """
        self.logger.debug(
            "{}.terminate({})".format(
                self.qualified_name,
                "{}->{}".format(self.status, new_status) if self.status != new_status else "{}".format(new_status)
            )
        )
        self.publisher.publish(std_msgs.String(data=""))
        self.feedback_message = "cleared"


class ScanContext(py_trees.behaviour.Behaviour):
    """
    Alludes to switching the context of the runtime system for a scanning
    action. Technically, it reaches out to the mock robots safety sensor
    dynamic parameter, switches it off in :meth:`initialise()` and maintains
    that for the the duration of the context before returning it to
    it's original value in :meth:`terminate()`.

    Args:
        name (:obj:`str`): name of the behaviour
    """
    def __init__(self, name):
        super().__init__(name=name)

        self.cached_context = None

    def setup(self, **kwargs):
        """
        Setup the ros2 communications infrastructure.

        Args:
            **kwargs (:obj:`dict`): look for the 'node' object being passed down from the tree

        Raises:
            :class:`KeyError`: if a ros2 node isn't passed under the key 'node' in kwargs
        """
        self.logger.debug("%s.setup()" % self.__class__.__name__)

        # ros2 node
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # parameter service clients
        self.parameter_clients = {
            'get_safety_sensors': self.node.create_client(
                rcl_srvs.GetParameters,
                '/safety_sensors/get_parameters'
            ),
            'set_safety_sensors': self.node.create_client(
                rcl_srvs.SetParameters,
                '/safety_sensors/set_parameters'
            )
        }
        for name, client in self.parameter_clients.items():
            if not client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError("client timed out waiting for server [{}]".format(name))

    def initialise(self):
        """
        Reset the cached context and trigger the chain of get/set parameter
        calls involved in changing the context.

        .. note::

           Completing the chain of service calls here
           (with `rclpy.spin_until_future_complete(node, future)`)
           is not possible if this behaviour is encapsulated inside, e.g.
           a tree tick activated by a ros2 timer callback, since it is
           already part of a scheduled job in a spinning node. It will
           just deadlock.

           Prefer instead to chain a sequence of events that will be
           completed over a span of ticks instead of at best, blocking
           here and at worst, falling into deadlock.

        """
        self.logger.debug("%s.initialise()" % self.__class__.__name__)
        self.cached_context = None
        # kickstart get/set parameter chain
        self._send_get_parameter_request()

    def update(self) -> py_trees.common.Status:
        """
        Complete the chain of calls begun in :meth:`initialise()` and then
        maintain the context (i.e. :class:`py_trees.behaviour.Behaviour` and
        return :data:`~py_trees.common.Status.RUNNING`).
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        all_done = False

        # wait for get_parameter to return
        if self.cached_context is None:
            if self._process_get_parameter_response():
                self._send_set_parameter_request(value=True)
            return py_trees.common.Status.RUNNING

        # wait for set parameter to return
        if not all_done:
            if self._process_set_parameter_response():
                all_done = True
            return py_trees.common.Status.RUNNING

        # just spin around, wait for an interrupt to trigger terminate
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status):
        """
        Reset the parameters back to their original (cached) values.

        Args:
            new_status: the behaviour is transitioning to this new status
        """
        self.logger.debug("%s.terminate(%s)" % (self.__class__.__name__, "%s->%s" % (self.status, new_status) if self.status != new_status else "%s" % new_status))
        if (
            new_status == py_trees.common.Status.INVALID and
            self.cached_context is not None
           ):
            self._send_set_parameter_request(value=self.cached_context)
            # don't worry about the response, no chance to catch it anyway

    def _send_get_parameter_request(self):
        request = rcl_srvs.GetParameters.Request()  # noqa
        request.names.append("enabled")
        self.get_parameter_future = self.parameter_clients['get_safety_sensors'].call_async(request)

    def _process_get_parameter_response(self) -> bool:
        if not self.get_parameter_future.done():
            return False
        if self.get_parameter_future.result() is None:
            self.feedback_message = "failed to retrieve the safety sensors context"
            self.node.get_logger().error(self.feedback_message)
            # self.node.get_logger().info('Service call failed %r' % (future.exception(),))
            raise RuntimeError(self.feedback_message)
        if len(self.get_parameter_future.result().values) > 1:
            self.feedback_message = "expected one parameter value, got multiple [{}]".format("/safety_sensors/enabled")
            raise RuntimeError(self.feedback_message)
        value = self.get_parameter_future.result().values[0]
        if value.type != rcl_msgs.ParameterType.PARAMETER_BOOL:  # noqa
            self.feedback_message = "expected parameter type bool, got [{}]{}]".format(value.type, "/safety_sensors/enabled")
            self.node.get_logger().error(self.feedback_message)
            raise RuntimeError(self.feedback_message)
        self.cached_context = value.bool_value
        return True

    def _send_set_parameter_request(self, value: bool):
        request = rcl_srvs.SetParameters.Request()  # noqa
        parameter = rcl_msgs.Parameter()
        parameter.name = "enabled"
        parameter.value.type = rcl_msgs.ParameterType.PARAMETER_BOOL  # noqa
        parameter.value.bool_value = value
        request.parameters.append(parameter)
        self.set_parameter_future = self.parameter_clients['set_safety_sensors'].call_async(request)

    def _process_set_parameter_response(self) -> bool:
        if not self.get_parameter_future.done():
            return False
        if self.set_parameter_future.result() is not None:
            self.feedback_message = "reconfigured the safety sensors context"
        else:
            self.feedback_message = "failed to reconfigure the safety sensors context"
            self.node.get_logger().error(self.feedback_message)
            # self.node.get_logger().info('service call failed %r' % (future.exception(),))
        return True

class NavToWaypoint(py_trees.behaviour.Behaviour):
    def __init__(
                self,
                msg_type: Any,
                msg_goal: Any,
                name: str=py_trees.common.Name.AUTO_GENERATED,
                goal_topic_name: str="/led_strip/command",
                feddback_topic_name: str="/led_strip/command",
                odom_topic_name: str="amcl_pose",
                colour: str="red",
                waypoint_distance_tolerance=0.5,
                intermediate_pose=True
                ):
        super(NavToWaypoint, self).__init__(name=name)
        self.goal_topic_name = goal_topic_name
        self.feddback_topic_name = feddback_topic_name
        self.odom_topic_name = odom_topic_name
        self.msg_type = msg_type
        self.msg_goal = msg_goal
        self.node = None
        self.waypoint_distance_tolerance = waypoint_distance_tolerance
        self.intermediate_pose = intermediate_pose
        self.status = ["STATUS_UNKNOWN", "STATUS_EXECUTING", "STATUS_SUCCEEDED", "STATUS_ABORTED"]
        self.result_status = "STATUS_UNKNOWN"

    # def __init__(
    #             self,
    #             name: str,
    #             topic_name: str="/led_strip/command",
    #             colour: str="red"
    #     ):
    #         super(FlashLedStrip, self).__init__(name=name)
    #         self.topic_name = topic_name
    #         self.colour = colour

    def setup(self, **kwargs):
        """
        When is this called?
          This function should be either manually called by your program
          to setup this behaviour alone, or more commonly, via
          :meth:`~py_trees.behaviour.Behaviour.setup_with_descendants`
          or :meth:`~py_trees.trees.BehaviourTree.setup`, both of which
          will iterate over this behaviour, it's children (it's children's
          children ...) calling :meth:`~py_trees.behaviour.Behaviour.setup`
          on each in turn.

          If you have vital initialisation necessary to the success
          execution of your behaviour, put a guard in your
          :meth:`~py_trees.behaviour.Behaviour.initialise` method
          to protect against entry without having been setup.

        What to do here?
          Delayed one-time initialisation that would otherwise interfere
          with offline rendering of this behaviour in a tree to dot graph
          or validation of the behaviour's configuration.

          Good examples include:

          - Hardware or driver initialisation
          - Middleware initialisation (e.g. ROS pubs/subs/services)
          - A parallel checking for a valid policy configuration after
            children have been added or removed

        Args:
            **kwargs (:obj:`dict`): distribute arguments to this
               behaviour and in turn, all of it's children

        Raises:
            :class:`KeyError`: if a ros2 node isn't passed under the key 'node' in kwargs
            :class:`~py_trees_ros.exceptions.TimedOutError`: if the action server could not be found
        """
        self.logger.debug("  %s [NavToWaypoint::setup()]" % self.name)
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.publisher_ = self.node.create_publisher(self.msg_type, self.goal_topic_name, 10)
        self.publisher_param_ = self.node.create_publisher(std_msgs.Bool, self.goal_topic_name + '/set_intermediate_pose', 10)
    
        self.subscription = self.node.create_subscription(
            std_msgs.String,
            self.feddback_topic_name,
            self._listener_feedback,
            10)

        # self.subscription  # prevent unused variable warning

    def initialise(self):
        """
        When is this called?
          The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter.

        What to do here?
          Any initialisation you need before putting your behaviour
          to work.
        """
        self.logger.debug("  %s [NavToWaypoint::initialise()]" % self.name)
        self.sent_goal = self.msg_goal
        self.publisher_.publish(self.msg_goal)
        self.publisher_param_.publish(std_msgs.Bool(data=self.intermediate_pose))
        self.result_status = "STATUS_EXECUTING"

    def update(self) -> py_trees.common.Status:
        """
        When is this called?
          Every time your behaviour is ticked.

        What to do here?
          - Triggering, checking, monitoring. Anything...but do not block!
          - Set a feedback message
          - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        """
        self.logger.debug("  %s [NavToWaypoint::update()]" % self.name)
        ready_to_make_a_decision = random.choice([True, False])
        decision = random.choice([True, False])

        if self.result_status == "STATUS_EXECUTING":
            self.feedback_message = "running"
            return py_trees.common.Status.SUCCESS
        elif self.result_status == "STATUS_SUCCEEDED":
            self.feedback_message = "success"
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "failure"
            return py_trees.common.Status.SUCCESS
            
        # if not ready_to_make_a_decision:
        #     return py_trees.common.Status.RUNNING
        # elif decision:
        #     self.feedback_message = "We are not bar!"
        #     return py_trees.common.Status.SUCCESS
        # else:
        #     self.feedback_message = "Uh oh"
        #     return py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status):
        """
        When is this called?
           Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
        self.logger.debug("  %s [NavToWaypoint::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

    def _listener_feedback(self, msg):
        self.node.get_logger().info('I heard: "%s"' % msg.data)
        if msg.data == "Done":
            print("Goal achieved")
            self.result_status = "STATUS_SUCCEEDED"

    def _listener_odom(self, msg):
        current_pose = msg.pose.pose
        x_curr = msg.pose.pose.position.x
        y_curr = msg.pose.pose.position.y
        x_goal = self.sent_goal.pose.position.x
        y_goal = self.sent_goal.pose.position.y

        dist_to_goal = math.sqrt((x_curr-x_goal)**2 + (y_curr-y_goal)**2)
        self.node.get_logger().info("Current pose x=%.2f y=%.2f" % (x_curr,y_curr))
        self.node.get_logger().info("Current goal x=%.2f y=%.2f" % (x_goal,y_goal))
        self.node.get_logger().info("Distance to waypoint goal = %.2f" % dist_to_goal)

        if (dist_to_goal < self.waypoint_distance_tolerance):
            print("Goal achieved")
            self.result_status = "STATUS_SUCCEEDED"
