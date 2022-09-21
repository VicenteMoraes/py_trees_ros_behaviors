#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://github.com/splintered-reality/py_trees_ros_behaviors/raw/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
About
^^^^^

This tutorial adds additional complexity to the scanning application in order to
introduce a few patterns typical of most applications - cancellations, recovery
and result handling.

Specifically, there is now an undocking-move combination pre-scanning and
a move-docking combination post-scanning. When cancelling, the robot should
recover it's initial state so it is ready to accept future requests. In this
case, the robot must move home and dock, even when cancelled.

Additionally, the application should report out on it's result upon completion.

.. note::

    Preemption has been dropped from the application for simplicity. It could
    be reinserted, but care would be required to handle undocking and docking
    appropriately.

Tree
^^^^

.. code-block:: bash

   $ py-trees-render -b py_trees_ros_behaviors.seven_docking_cancelling_failing.tutorial_create_root

.. graphviz:: dot/tutorial-seven-docking-cancelling-failing.dot
   :align: center

.. literalinclude:: ../py_trees_ros_behaviors/seven_docking_cancelling_failing.py
   :language: python
   :linenos:
   :lines: 211-398
   :caption: seven_docking_cancelling_failing.py#tutorial_create_root

Succeeding
----------

.. graphviz:: dot/tutorial-seven-ere-we-go.dot
   :align: center

Assuming everything works perfectly, then the subtree will sequentially progress to completion
through undocking, move out, rotate, move home and docking actions as illustrated in the
dot graph above. However, nothing ever works perfectly, so ...

Failing
-------

.. image:: images/tutorial-seven-failure_paths.svg
   :align: center

If any step of the 'Ere we Go' sequence fails the mock robot robot will simply stop, drop
into the post-failure ('Die') subtree and commence post-failure actions. In this case
this consists of both an alarm signal (flashing red) and communication of failure to
the user (echoes to the screen, but could have been, for example, a middleware response
to the user's application).

These actions are merely post-failure notifications that would ostensibly result in
manual (human assisted) recovery of the situation. To attempt an automated recovery,
there are two options:

   1. Global Recovery - use the blackboard as a means of transferring information about the
      failure from the relevant behaviour (UnDock, Move Out, Move Home, Dock) to the
      post-failure subtree. Introspect the data and determine the right course of action in
      the post-failure subtree.

   2. Local Recovery - use a selector with each of the individual behaviours to immediately
      generate a recovery subtree specifically adapted to the behaviour that failed. This
      recovery subtree should also return :attr:`~py_trees.common.Status.FAILURE` so the
      parent sequence also returns :attr:`~py_trees.common.Status.FAILURE`. The
      'Die' subtree is then merely for common post-failure actions (e.g. notification and
      response).

The latter is technically preferable as the decision logic is entirely visible in the tree
connections, but it does cause an explosion in the scale of the tree and it's maintenance.

.. note::

   It is interesting to observe that although the application is considered to have
   failed, the 'Scan or Die' operation will return with :attr:`~py_trees.common.Status.SUCCESS`
   after which post-failure actions will kick in.
   Here, application failure is recorded in the 'Result2BB' behaviour which is later
   transmitted back to the user in the final stages of the application.

   Application failure is handled via the actions of behaviours,
   not the state of the tree.

.. tip::

   Decision logic in the tree is for routing decision making,
   not routing application failure/success, nor logical errors. Overloading
   tree decision logic with more than one purpose will constrain your
   application design to the point of non-usefulness.

Cancelling
----------

In this tutorial, the application listens continuously for cancellation requests and
will cancel the operation if it is currently between undocking and docking actions.

.. note::

   The approach demonstrated in this tutorial is simple, but sufficient as an example.
   Interactions are only one-way - from the user to the application.
   It neither prevents the user from requesting nor does it provide an informative
   response if the request is invalid (i.e. if the application is not running or already
   cancelling). It also falls short of caching and handling
   cancel requests across the entire application.
   These cases are easy to handle with additional logic in the tree - consider it
   a homework exercise :)

.. graphviz:: dot/tutorial-seven-cancel2bb.dot
   :align: center

Cancelling begins with catching incoming cancel requests:

.. image:: images/tutorial-seven-cancelling.svg
   :align: center

Cancelling is a high priority subtree, but here we make sure that the post-cancelling
workflow integrates with the non-cancelling workflow so that the robot returns to
it's initial location and state.


Results
-------

.. image:: images/tutorial-seven-result.svg
   :align: center

As noted earlier, it is typically important to keep application result logic
separate from the decision tree logic. To do so, the blackboard is used to
record the application result and an application result agnostic behaviour
is used to communicate the result back to the user in the final stage of the
application's lifecycle.


Running
^^^^^^^

.. code-block:: bash

    # Launch the tutorial
    $ ros2 launch py_trees_ros_behaviors skill_behavior.py
    # In another shell
    $ py-trees-tree-watcher -b
    # Trigger scan/cancel requests from the qt dashboard

.. image:: images/tutorial-seven-docking-cancelling-failing.png
"""

##############################################################################
# Imports
##############################################################################

import operator
import sys
import os
import time
import functools
import argparse
import json
import time

import launch
import launch_ros
import py_trees
import py_trees_ros.trees
import py_trees.console as console
import py_trees_ros_interfaces.action as py_trees_actions  # noqa
import rclpy

import geometry_msgs.msg as geometry_msgs
import std_msgs.msg as std_msgs

from . import behaviours
from . import skill_library
from . import mock
from mission_control.execution import ActiveSkillController, SequencingProcess, TaskStatus, LocalMissionController

##############################################################################
# Launcher
##############################################################################

# ros2 launch py_trees_ros_behaviors skill_behavior.py
def generate_launch_description():
    """
    Launcher for the tutorial.

    Returns:
        the launch description
    """
    return launch.LaunchDescription(
        mock.launch.generate_launch_nodes() +
        [
            launch_ros.actions.SetParameter(name='use_sim_time', value=True),
            launch_ros.actions.Node(
                package='py_trees_ros_behaviors',
                executable="skill-engine",
                output='screen',
                emulate_tty=True,
            )
        ]
    )

##############################################################################
# Tutorial
##############################################################################


skill_name = "0"
local_plan = [
    # ["navigation", ["PC Room 7", [[-19.0, 18.0, -1.57], [-19.0, 16.0], [-18.0, 16.0], [-13.5, 16.0], [-13.5, 18.0, -1.57]]]],
    # ["approach_person", ["nurse"]],
    # ["authenticate_person", ["nurse"]],
    # ["operate_drawer", ["open"]],
    # ["send_message", ["nurse"]],
    # ["wait_message", ["r1"]],
    # ["operate_drawer", ["close"]],
    # ["navigation", ["Laboratory", [[-13.5, 18.0, -1.57], [-13.5, 16.0], [-18.0, 16.0], [-19.0, 16.0], [-21.0, 16.0], [-26.0, 16.0], [-26.0, 13.0, 1.57]]]],
    # ["approach_robot", ["lab_arm"]],
    # ["operate_drawer", ["open"]],
    # ["send_message", ["lab_arm"]],
    # ["wait_message", ["r1"]],
    # ["operate_drawer", ["close"]]
]
logpub = None
available_skills = ["send_message", "wait_message"]

def get_plan():
    global skill_name
    # if skill_name == "SencondBT":
    #     skill_name = "NavToRoom"
    #     return ("NavToRoom", [])
    # else:
    #     skill_name = "SencondBT"
    #     return ("SencondBT", [])

    # if skill_name == "0":
    #     skill_name = "1"
    #     return ("ActionDrawer", ['open'])
    # elif skill_name == "1":
    #     skill_name = "2"
    #     return ("SendMessage", ['Open Drawer', "/nurse/comms"])
    # elif skill_name == "2":
    #     skill_name = "3"
    #     return ("WaitForMessage", ['deposit', "/nurse/action"])
    # elif skill_name == "3":
    #     skill_name = "0"
    #     return ("ActionDrawer", ['close'])

    if skill_name == "0":
        skill_name = "1"
        return ("navigation", [])
    elif skill_name == "1":
        skill_name = "2"
        return ("approach_person", ['nurse'])
    elif skill_name == "2":
        skill_name = "3"
        return ("authenticate_person", ['nurse'])
    # elif skill_name == "2":
    #     skill_name = "3"
    #     return ("ActionDrawer", ['open'])
    # elif skill_name == "3":
    #     skill_name = "4"
    #     return ("SendMessage", ['drawer opened', "/nurse/comms", 'deposit', "/nurse/action"])
    # elif skill_name == "4":
    #     skill_name = "5"
    #     return ("SendMessage", ['drawer opened', "/nurse/comms", 'deposit', "/nurse/action"])
    # elif skill_name == "5":
    #     skill_name = "6"
    #     return ("WaitForMessage", ['deposit', "/nurse/action"])

class task_type(Enum):
    NAV_TO = 'navigation'
    PICK_UP = 'pick_up'
    WAIT_MSG = 'wait_msg'
    SEND_MSG = 'send_msg'
    GOING_FAIL = 'going_to_fail'
    AUTH_PERSON = 'authenticate_person'
    APPR_PERSON = 'approach_person'
    APPR_ROBOT = 'approach_robot'
    ACT_DRAW = 'action_drawer'

def main():
    global local_plan, logpub, available_skills
    # init py ros node
    rclpy.init(args=None)
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    # get robot's available skills and local plan from global mission
    available_skills = available_skills + json.loads(os.environ['ROBOT_CONFIG'])["skills"]
    local_plan = json.loads(os.environ['ROBOT_CONFIG'])["local_plan"]

    collector_skill_library = SkillLibrary()
    collector_skill_library.add(task_type.PICK_UP.value, skill_library.OneTickSkill)
    collector_skill_library.add(task_type.NAV_TO.value, skill_library.Nav2RoomSkill)
    collector_skill_library.add(task_type.WAIT_MSG.value, skill_library.WaitMsgSkill)
    collector_skill_library.add(task_type.SEND_MSG.value, skill_library.SendMsgSkill)
    collector_skill_library.add(task_type.GOING_FAIL.value, skill_library.Going2FailSkill)
    collector_skill_library.add(task_type.AUTH_PERSON.value, skill_library.AuthPersonSkill)
    collector_skill_library.add(task_type.APPR_PERSON.value, skill_library.ApproachPersonSkill)
    collector_skill_library.add(task_type.APPR_ROBOT.value, skill_library.ApproachRobotSkill)
    collector_skill_library.add(task_type.ACT_DRAW.value, skill_library.ActionDrawerSkill)
    
    # console.loginfo(json.dumps(local_plan, indent=2, sort_keys=True))

    seq_proc = SequencingProcess(skill_library = collector_skill_library)
    task_status = TaskStatus()
    local_mission_ctrl  = LocalMissionController(ihtn_collect)
    active_skill_crl= ActiveSkillController()

    logpub = rclpy.create_node('pytrees_logger')
    publisher = logpub.create_publisher(std_msgs.String, '/log', 10)
    def timer_callback():
        pass
    timer_period = 0.5  # seconds
    timer = logpub.create_timer(timer_period, timer_callback)

    while rclpy.ok():
        seq_proc.run(local_mission_ctrl, active_skill_crl, task_status=task_status)
        active_skill_crl.active_skill.ros_spin(rclpy)
        
        # send log
        msg.data = '{}-skill-life-cycle-{}={}'.format(os.environ['ROBOT_NAME'],skill,'RUNNING')
        label = ''
        content = {
            'skill': list(collector_skill_library.skills_map.keys())[list(collector_skill_library.skills_map.values()).index(active_skill_crl.active_skill)],
            'skill-life-cycle': 'RUNNING',
            'label': label,
            'param_list': active_skill_crl.active_skill.task.attributes
        }
        logdata = {
            'level': 'debug',
            'entity': os.environ['ROBOT_NAME'],
            'content': content
        }
        msg.data = json.dumps(logdata)
        publisher.publish(msg)

    logpub.destroy_timer(timer)
    logpub.destroy_node()
    rclpy.shutdown()
