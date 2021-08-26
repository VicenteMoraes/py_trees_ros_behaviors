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
# skill_name = "SencondBT"

    # ic_corridor = Nodes("IC Corridor", [-37, 15])
    # ic_room_1 = Nodes("IC Room 1", [-39.44, 33.98, 0.00])
    # ic_room_2 = Nodes("IC Room 2", [-32.88, 33.95, 3.14])
    # ic_room_3 = Nodes("IC Room 3", [-40.23, 25.37, 0.00])
    # ic_room_4 = Nodes("IC Room 4", [-33.90, 18.93, 3.14])
    # ic_room_5 = Nodes("IC Room 5", [-38.00, 21.50, 0.00])
    # ic_room_6 = Nodes("IC Room 6", [-38.00, 10.00, 0.00])
    # pc_corridor = Nodes("PC Corridor", [-19, 16])
    # pc_room_1 = Nodes("PC Room 1", [-28.50, 18.00,-1.57])
    # pc_room_2 = Nodes("PC Room 2", [-27.23, 18.00,-1.57])
    # pc_room_3 = Nodes("PC Room 3", [-21.00, 18.00,-1.57])
    # pc_room_4 = Nodes("PC Room 4", [-19.00, 18.00,-1.57])
    # pc_room_5 = Nodes("PC Room 5", [-13.50, 18.00,-1.57])
    # pc_room_6 = Nodes("PC Room 6", [-11.50, 18,-1.57])
    # pc_room_7 = Nodes("PC Room 7", [-4, 18,-1.57])
    # pc_room_8 = Nodes("PC Room 8", [-27.23, 13.00, 1.57])
    # pc_room_9 = Nodes("PC Room 9", [-26.00, 13.00, 1.57])
    # pc_room_10 = Nodes("PC Room 10", [-18.00, 13.00, 1.57])
    # reception = Nodes("Reception", [-1, 20])
    # pharmacy_corridor = Nodes("Pharmacy Corridor", [0, 8])
    # pharmacy = Nodes("Pharmacy", [-2, 2.6])

# local_plan = [
#               # ['navigation', ['IC Room 5', [[-38, 23], [-37, 15], [-38, 21.5]]]],
#               ['navigation', ['IC Room 5', [[-38, 23], [-28, 16], [-31.4, 16], [-37, 15], [-38, 21.5]]]],
#               ['approach_person', ['nurse']], 
#               ['authenticate_person', ['nurse']], 
#               ['operate_drawer', ['open']], 
#               ['send_message', ['nurse']], 
#               ['wait_message', ['r1']], 
#               ['operate_drawer', ['close']], 
#               ['navigation', ['Pharmacy', [[-38, 21.5], [-37, 15], [-19, 16], [0, 8], [-2, 2.6]]]], 
#               ['approach_robot', ['lab_arm']], 
#               ['operate_drawer', ['open']], 
#               ['send_message', ['lab_arm']], 
#               ['wait_message', ['r1']], 
#               ['operate_drawer', ['close']]
#              ]
def formatlog(severity, who, loginfo, skill, params):
    return ('['+severity+'],'+
               who+','+
               loginfo+','+
               skill+','+
               params)
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
publisher = None
idx = 0
available_skills = ["send_message", "wait_message"]
def get_local_plan():
    global local_plan, idx, available_skills
    skill = local_plan[idx][0]
    params = None
    local_plan[idx][1]
    params = local_plan[idx][1]
    label = local_plan[idx][2]
    print(skill)
    print(params)
    print(label)
    print(f"{os.environ['ROBOT_NAME']} == {os.environ['CHOSE_ROBOT']}")
    if skill in available_skills:
        idx = idx + 1
        return (skill, params, label)
    elif os.environ['ROBOT_NAME'] == os.environ['CHOSE_ROBOT']:
        return ("fail", [skill]+params, 'UNAVAILABLE-SKILL')
    else:
        return (None, [], '')

def create_init_bt() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Sequence("SendMsg")
    # timer = py_trees.timers.Timer('Timer', duration=5.0)
    # suc = py_trees.behaviours.Success(name="Success")
    # root.add_children([suc])
    # print(param_list[3])
    lost_param = std_msgs.String(data="hello")
    param_to_bb1 = py_trees.behaviours.SetBlackboardVariable(
        name="param_to_bb ",
        variable_name='/lost_param',
        variable_value=lost_param
    )
    # hold = create_waypoints_sequence([0, 0])
    wait_for_lost_param = py_trees.behaviours.WaitForBlackboardVariable(
        name="WaitForParam",
        variable_name="/lost_param"
    )
    publisher1 = py_trees_ros.publishers.FromBlackboard(
        name="Publishlost_param",
        topic_name="/nurse/comms",
        topic_type=std_msgs.String,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variable="lost_param"
    )
    goal_pose = geometry_msgs.PoseStamped()
    goal_pose.header.frame_id = "base_link"
    goal_pose.pose.position = geometry_msgs.Point(x=0.0, y=0.0, z=0.0)
    param_to_bb2 = py_trees.behaviours.SetBlackboardVariable(
        name="param_to_bb ",
        variable_name='/goal_pose',
        variable_value=goal_pose
    )
    wait_for_goal_pose = py_trees.behaviours.WaitForBlackboardVariable(
        name="WaitForParam",
        variable_name="/goal_pose"
    )
    publisher2 = py_trees_ros.publishers.FromBlackboard(
        name="Publishgoal_pose",
        topic_name="/"+os.environ['ROBOT_NAME']+"/send_goal",
        topic_type=geometry_msgs.PoseStamped,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variable="goal_pose"
    )
    regulation = std_msgs.String(data="nurse")
    req_to_bb = py_trees.behaviours.SetBlackboardVariable(
        name="Send authentication request",
        variable_name='/regulation',
        variable_value=regulation
    )
    wait_for_regulation = py_trees.behaviours.WaitForBlackboardVariable(
        name="WaitForParam",
        variable_name="/regulation"
    )
    publisher3 = py_trees_ros.publishers.FromBlackboard(
        name="Publishregulation",
        topic_name="/led_strip/display",
        topic_type=std_msgs.String,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variable="regulation"
    )
    root.add_children([param_to_bb1, wait_for_lost_param, publisher1, param_to_bb2, wait_for_goal_pose, publisher2, req_to_bb, wait_for_regulation, publisher3])
    # root.add_children([resp_2bb, wait_for_res, is_ok, suc])
    return root

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

def send_report(status, skill, param_list):
    msg = std_msgs.String()
    # msg.data = formatlog('info',
    #     os.environ['ROBOT_NAME'],
    #     'skill-life-cycle',
    #     str(skill),
    #     '(status='+str(status)+', parameters='+str(param_list)+')')
    msg.data = '{}-skill-life-cycle-{}={}'.format(os.environ['ROBOT_NAME'],skill,param_list)
    msg.data = '{}-skill-life-cycle-{}={}'.format(os.environ['ROBOT_NAME'],skill,status)
    # publisher.publish(msg)
    content = {
        'skill': skill,
        'report-status': status,
        'param_list' : param_list
    }
    logdata = {
        'level': 'debug',
        'entity': os.environ['ROBOT_NAME'],
        'content': content
    }
    msg.data = json.dumps(logdata)
    publisher.publish(msg)
    print(status)


def load_skill(skill, param_list) -> py_trees.behaviour.Behaviour:
    if skill == "navigation":
        root = skills.create_nav_to_room_bt(param_list)
    elif skill == "operate_drawer":
        root = skills.create_action_drawer_bt(param_list)
    elif skill == "approach_robot":
        root = skills.create_approach_robot_bt(param_list)
    elif skill == "approach_person":
        root = skills.create_approach_nurse_bt(param_list)
    elif skill == "authenticate_person":
        root = skills.create_authenticate_nurse_bt(param_list)
    elif skill == "wait_message":
        root = skills.create_wait_message(param_list)
    elif skill == "send_message":
        root = skills.create_send_message(param_list)
    elif skill == "SencondBT":
        root = skills.create_second_bt()
    elif skill == "fail":
        root = skills.going_to_fail(param_list)
        send_report('UNAVAILABLE-SKILL', param_list[0], param_list)
        send_report('FAILURE', 'robot-without-skill', param_list)
    else:
        root = None
    return root

def logger(snapshot_visitor, behaviour_tree):
    """
    A post-tick handler that logs the tree (relevant parts thereof) to a yaml file.
    """
    print(console.cyan + "Logging.......................yes\n" + console.reset)
    if snapshot_visitor.changed:
        print(console.cyan + "Logging.......................yes\n" + console.reset)
        tree_serialisation = {
            'tick': behaviour_tree.count,
            'nodes': []
        }
        for node in behaviour_tree.root.iterate():
            node_type_str = "Behaviour"
            for behaviour_type in [py_trees.composites.Sequence,
                                   py_trees.composites.Selector,
                                   py_trees.composites.Parallel,
                                   py_trees.decorators.Decorator]:
                if isinstance(node, behaviour_type):
                    node_type_str = behaviour_type.__name__
            node_snapshot = {
                'name': node.name,
                'id': str(node.id),
                'parent_id': str(node.parent.id) if node.parent else "none",
                'child_ids': [str(child.id) for child in node.children],
                'tip_id': str(node.tip().id) if node.tip() else 'none',
                'class_name': str(node.__module__) + '.' + str(type(node).__name__),
                'type': node_type_str,
                'status': node.status.value,
                'message': node.feedback_message,
                'is_active': True if node.id in snapshot_visitor.visited else False
                }
            tree_serialisation['nodes'].append(node_snapshot)
        if behaviour_tree.count == 0:
            with open('dump.json', 'w+') as outfile:
                json.dump(tree_serialisation, outfile, indent=4)
                print(tree_serialisation)
        else:
            with open('dump.json', 'a') as outfile:
                json.dump(tree_serialisation, outfile, indent=4)
                print(tree_serialisation)
    else:
        print(console.yellow + "Logging.......................no\n" + console.reset)

def pre_tick_handler(behaviour_tree):
    print("\n--------- Run %s ---------\n" % behaviour_tree.count)


def post_tick_handler(snapshot_visitor, behaviour_tree):
    print(
        "\n" + py_trees.display.unicode_tree(
            root=behaviour_tree.root,
            visited=snapshot_visitor.visited,
            previously_visited=snapshot_visitor.previously_visited
        )
    )
    print(py_trees.display.unicode_blackboard())

def main():
    """
    Entry point for the demo script.
    """
    """
    get plan
    """
    global local_plan, logpub, publisher, available_skills
    console.logerror(json.dumps(os.environ['ROBOT_CONFIG'], indent=2, sort_keys=True))
    available_skills = available_skills + json.loads(os.environ['ROBOT_CONFIG'])["skills"]
    local_plan = json.loads(os.environ['ROBOT_CONFIG'])["local_plan"]
    console.loginfo(json.dumps(local_plan, indent=2, sort_keys=True))
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    print(os.environ['N_ROBOTS'])
    print(os.environ['ROBOT_NAME_'+str(1)])
    rclpy.init(args=None)
    # root = create_nav_to_room_bt()
    root = py_trees.composites.Sequence("Idle")
    timer = behaviours.MyTimer(name="timer", duration=10.0)
    suc = py_trees.behaviours.Success(name="Success")
    # root.add_child(suc)
    root.add_children([timer, suc])
    # root = create_init_bt()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    tree.setup(timeout=15)
    rclpy.spin_once(tree.node, timeout_sec=0)
    tree.root.tick_once()
    try:
        ####################
        # Tree Stewardship
        ####################
        behaviour_tree = py_trees.trees.BehaviourTree(root)
        behaviour_tree.add_pre_tick_handler(pre_tick_handler)
        behaviour_tree.visitors.append(py_trees_ros.visitors.SetupLogger(tree.node))
        snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        behaviour_tree.add_post_tick_handler(functools.partial(post_tick_handler, snapshot_visitor))
        behaviour_tree.visitors.append(snapshot_visitor)

        tree.setup(timeout=15)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)

    logpub = rclpy.create_node('pytrees_logger')
    publisher = logpub.create_publisher(std_msgs.String, '/log', 10)
    skill = "init_robobt"
    param_list = []
    label = "init_robobt"

    # blackboard = py_trees.blackboard.Client(name="Global")
    count = 0
    while rclpy.ok():
        try:
            print(tree.root.status)
            if tree.root.status == py_trees.common.Status.SUCCESS:
                msg = std_msgs.String()
                content = {
                    'skill': skill,
                    'skill-life-cycle': 'SUCCESS',
                    'label': label,
                    'param_list': param_list
                }
                logdata = {
                    'level': 'debug',
                    'entity': os.environ['ROBOT_NAME'],
                    'content': content
                }
                msg.data = json.dumps(logdata)
                publisher.publish(msg)
                (skill, param_list, label) = get_local_plan()
                root = load_skill(skill, param_list)
                msg = std_msgs.String()
                msg.data = formatlog('debug',
                        os.environ['ROBOT_NAME'],
                        'available-skills',
                        str(available_skills),
                        '')
                logdata = {
                    'level': 'debug',
                    'entity': os.environ['ROBOT_NAME'],
                    'content': 'available-skills={}'.format(available_skills)
                }
                msg.data = json.dumps(logdata)
                publisher.publish(msg)
                print(available_skills)
                # if root == None:
                #     print("Closing Robot "+os.environ['ROBOT_NAME']+" BT")
                    # break
                tree = py_trees_ros.trees.BehaviourTree(
                    root=root,
                    unicode_tree_debug=True
                )
                msg = std_msgs.String()
                # msg.data = formatlog('info',
                #     os.environ['ROBOT_NAME'],
                #     'skill-life-cycle',
                #     str(skill),
                #     '(status=STARTED'+', parameters='+str(param_list)+')')
                msg.data = '{}-skill-life-cycle-{}={}'.format(os.environ['ROBOT_NAME'],skill,'STARTED')
                content = {
                    'skill': skill,
                    'skill-life-cycle': 'STARTED',
                    'label': label,
                    'param_list': param_list
                }
                logdata = {
                    'level': 'debug',
                    'entity': os.environ['ROBOT_NAME'],
                    'content': content
                }
                msg.data = json.dumps(logdata)
                publisher.publish(msg)
                def timer_callback():
                    pass
                timer_period = 0.5  # seconds
                timer = logpub.create_timer(timer_period, timer_callback)
                # tree.root.setup(node=tree.node)
                tree.setup(timeout=15)
                console.logerror(skill)
                # console.logerror(param_list[0])
                print(py_trees.display.unicode_tree(root=tree.root))
                tree.root.tick_once()
            elif tree.root.status == py_trees.common.Status.RUNNING:
                rclpy.spin_once(tree.node, timeout_sec=0)
                rclpy.spin_once(logpub, timeout_sec=0)
                tree.root.tick_once()
                # print(blackboard)
                # send_report(tree.root.status, skill, param_list)
                if count%10 == 0:
                    print(py_trees.display.unicode_tree(root=tree.root))
                if count == 0:
                    msg = std_msgs.String()
                    # msg.data = formatlog('info',
                    #     os.environ['ROBOT_NAME'],
                    #     'skill-life-cycle',
                    #     str(skill),
                    #     '(status=RUNNING/parameters='+str(param_list)+')')
                    msg.data = '{}-skill-life-cycle-{}={}'.format(os.environ['ROBOT_NAME'],skill,'RUNNING')
                    content = {
                        'skill': skill,
                        'skill-life-cycle': 'RUNNING',
                        'label': label,
                        'param_list': param_list
                    }
                    logdata = {
                        'level': 'debug',
                        'entity': os.environ['ROBOT_NAME'],
                        'content': content
                    }
                    msg.data = json.dumps(logdata)
                    publisher.publish(msg)
                count = (count+1)%600
            else:
                send_report(tree.root.status, skill, param_list)
                # msg.data = "FAILURE"
                # msg.data = '{}-skill-life-cycle-{}={}'.format(os.environ['ROBOT_NAME'],skill,'FAILURE')
                # publisher.publish(msg)
                msg.data = '{}-skill-life-cycle-{}={}'.format(os.environ['ROBOT_NAME'],skill,'FAILURE')
                content = {
                    'skill': skill,
                    'skill-life-cycle': 'FAILURE',
                    'label': label,
                    'param_list': param_list
                }
                logdata = {
                    'level': 'debug',
                    'entity': os.environ['ROBOT_NAME'],
                    'content': content
                }
                msg.data = json.dumps(logdata)
                publisher.publish(msg)
                rclpy.spin_once(logpub, timeout_sec=0)

            # tree.root.tick_once()
            # rclpy.spin_once(tree.node)
            time.sleep(0.1)
            # console.logerror("tick")
            # rclpy.spin(tree.node)
        except KeyboardInterrupt:
            pass

    tree.shutdown()
    logpub.destroy_timer(timer)
    logpub.destroy_node()
    rclpy.shutdown()
