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

In this, the first of the tutorials, we start out with a behaviour that
collects battery data from a subscriber and stores the result on the
blackboard for other behaviours to utilise.

Data gathering up front via subscribers is a useful convention for
a number of reasons:

* Freeze incoming data for remaining behaviours in the tree tick so that decision making is consistent across the entire tree
* Avoid redundantly invoking multiple subscribers to the same topic when not necessary
* Python access to the blackboard is easier than ROS middleware handling

Typically data gatherers will be assembled underneath a parallel at or near
the very root of the tree so they may always trigger their update() method
and be processed before any decision making behaviours elsewhere in the tree.

Tree
^^^^

.. code-block:: bash

   $ py-trees-render -b py_trees_ros_behaviors.one_data_gathering.tutorial_create_root

.. graphviz:: dot/tutorial-one-data-gathering.dot
   :align: center

.. literalinclude:: ../py_trees_ros_behaviors/one_data_gathering.py
   :language: python
   :linenos:
   :lines: 121-153
   :caption: one_data_gathering.py#tutorial_create_root

Along with the data gathering side, you'll also notice the dummy branch for
priority jobs (complete with idle behaviour that is always
:attr:`~py_trees.common.Status.RUNNING`). This configuration is typical
of the :term:`data gathering` pattern.

Behaviours
^^^^^^^^^^

The tree makes use of the :class:`py_trees_ros.battery.ToBlackboard` behaviour.

This behaviour will cause the entire tree will tick over with
:attr:`~py_trees.common.Status.SUCCESS` so long as there is data incoming.
If there is no data incoming, it will simply
:term:`block` and prevent the rest of the tree from acting.


Running
^^^^^^^

.. code-block:: bash

    # Launch the tutorial
    $ ros2 launch py_trees_ros_behaviors tutorial_one_data_gathering_launch.py
    # In a different shell, introspect the entire blackboard
    $ py-trees-blackboard-watcher
    # Or selectively get the battery percentage
    $ py-trees-blackboard-watcher --list
    $ py-trees-blackboard-watcher /battery.percentage

.. image:: images/tutorial-one-data-gathering.gif
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
import rclpy
import sys

import argparse
import functools
import json
import time

from . import mock

##############################################################################
# Launcher
##############################################################################


def generate_launch_description():
    """
    Launcher for the tutorial.

    Returns:
        the launch description
    """
    return launch.LaunchDescription(
        mock.launch.generate_launch_nodes() +
        [
            launch_ros.actions.Node(
                package='py_trees_ros_behaviors',
                executable="tree-data-gathering",
                output='screen',
                emulate_tty=True,
            )
        ]
    )

##############################################################################
# Tutorial
##############################################################################


def tutorial_create_root() -> py_trees.behaviour.Behaviour:
    """
    Create a basic tree and start a 'Topics2BB' work sequence that
    will become responsible for data gathering behaviours.

    Returns:
        the root of the tree
    """
    root = py_trees.composites.Parallel(
        name="Tutorial One",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )

    topics2bb = py_trees.composites.Sequence("Topics2BB")
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2BB",
        topic_name="/battery/state",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        threshold=30.0
    )
    priorities = py_trees.composites.Selector("Tasks")
    idle = py_trees.behaviours.Running(name="Idle")
    flipper = py_trees.behaviours.Periodic(name="Flip Eggs", n=2)

    root.add_child(topics2bb)
    topics2bb.add_child(battery2bb)
    root.add_child(priorities)
    priorities.add_child(flipper)
    priorities.add_child(idle)

    return root


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

def send_report(status):
    print(status)


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

def tutorial_main():
    """
    Entry point for the demo script.
    """
    """
    get plan
    """
    global local_plan
    # console.logerror(json.dumps(os.environ['ROBOT_CONFIG'], indent=2, sort_keys=True))
    # local_plan = json.loads(os.environ['ROBOT_CONFIG'])["local_plan"]
    # console.loginfo(json.dumps(local_plan, indent=2, sort_keys=True))
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    print(os.environ['N_ROBOTS'])
    print(os.environ['ROBOT_NAME_'+str(1)])
    rclpy.init(args=None)
    # root = create_nav_to_room_bt()
    root = py_trees.composites.Sequence("Idle")
    suc = py_trees.behaviours.Success(name="Success")
    root.add_child(suc)
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



    # blackboard = py_trees.blackboard.Client(name="Global")
    while rclpy.ok():
        try:
            print(tree.root.status)
            if tree.root.status == py_trees.common.Status.SUCCESS:
                # (skill, param_list) = get_local_plan()
                # root = load_skill(skill, param_list)
                tree = py_trees_ros.trees.BehaviourTree(
                    root=root,
                    unicode_tree_debug=True
                )
                # tree.root.setup(node=tree.node)
                tree.setup(timeout=15)
                break
                # console.logerror(skill)
                # console.logerror(param_list[0])
                print(py_trees.display.unicode_tree(root=tree.root))
                tree.root.tick_once()
            elif tree.root.status == py_trees.common.Status.RUNNING:
                print(py_trees.display.unicode_tree(root=tree.root))
                rclpy.spin_once(tree.node, timeout_sec=0)
                tree.root.tick_once()
                # print(blackboard)
                send_report(tree.root.status)
            else:
                send_report(tree.root.status)

            # tree.root.tick_once()
            # rclpy.spin_once(tree.node)
            time.sleep(1)
            # console.logerror("tick")
            # rclpy.spin(tree.node)
        except KeyboardInterrupt:
            pass

    tree.shutdown()
    rclpy.shutdown()

