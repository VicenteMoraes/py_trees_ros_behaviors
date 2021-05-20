#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://github.com/splintered-reality/py_trees_ros_behaviors/raw/devel/LICENSE
#
import operator
import os
import sys
import time
import math
import functools

import launch
import launch_ros
import py_trees
import py_trees_ros.trees
import py_trees.console as console
import py_trees_ros_interfaces.action as py_trees_actions  # noqa
import rclpy

import geometry_msgs.msg as geometry_msgs
import std_msgs.msg as std_msgs
# import tf_conversions
# import tf2_ros.tf_conversions


from . import behaviours
from . import mock

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

##############################################################################
# Skills
##############################################################################

def create_wait_message(param_list) -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Sequence("WaitMsg")
    # ##################3 log
    os.environ['ROBOT_NAME']
    param = std_msgs.String(data=os.environ['ROBOT_NAME']+",time ros unavailable"+",create_wait_message triggered")
    param_to_bb = py_trees.behaviours.SetBlackboardVariable(
        name="param_to_bb ",
        variable_name='/param',
        variable_value=param
    )
    wait_for_req = py_trees.behaviours.WaitForBlackboardVariable(
        name="WaitForParam",
        variable_name="/param"
    )
    publisher1 = py_trees_ros.publishers.FromBlackboard(
        name="Publish",
        topic_name="/log",
        topic_type=std_msgs.String,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variable="param"
    )
    root.add_children([param_to_bb, wait_for_req, publisher1])

    # root = create_wacther_bt(sub_root)
    # timer = py_trees.timers.Timer('Timer', duration=5.0)
    timer = behaviours.MyTimer(name='Waiting', duration=1.0)
    resp_2bb = py_trees_ros.subscribers.ToBlackboard(
        name="response2BB_"+param_list[0],
        topic_name="/"+param_list[0]+"/comms",
        topic_type=std_msgs.String,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables = {'response_msg': 'data'}
    )
    wait_for_res = py_trees.behaviours.WaitForBlackboardVariable(
        name="WaitForResponse",
        variable_name="/response_msg"
    )
    is_ok = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Msg OK?",
        check=py_trees.common.ComparisonExpression(
            variable="response_msg",
            value='r1',
            operator=operator.eq
        )
    )
    suc = py_trees.behaviours.Success(name=param_list[0]+'_success')
    root.add_children([resp_2bb, wait_for_res, suc])
    return root

def create_send_message(param_list) -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Sequence("SendMsg")
    # timer = py_trees.timers.Timer('Timer', duration=5.0)
    # suc = py_trees.behaviours.Success(name="Success")
    # root.add_children([suc])
    # print(param_list[3])
    topic = "/"+param_list[0]+"/comms"
    timer1 = behaviours.MyTimer(name='Waiting', duration=1.0)
    timer2 = behaviours.MyTimer(name='Waiting', duration=1.0)
    timer3 = behaviours.MyTimer(name='Waiting', duration=1.0)
    timer4 = behaviours.MyTimer(name='Waiting', duration=1.0)
    param = std_msgs.String(data=os.environ['ROBOT_NAME']+",time ros unavailable"+",create_send_message")
    param_to_bb = py_trees.behaviours.SetBlackboardVariable(
        name="param_to_bb "+param_list[0],
        variable_name='/param',
        variable_value=param
    )
    wait_for_req = py_trees.behaviours.WaitForBlackboardVariable(
        name="WaitForParam",
        variable_name="/param"
    )
    publisher1 = py_trees_ros.publishers.FromBlackboard(
        name="Publish",
        topic_name=topic,
        topic_type=std_msgs.String,
        qos_profile=py_trees_ros.utilities.qos_profile_latched(),
        blackboard_variable="param"
    )
    publisher2 = py_trees_ros.publishers.FromBlackboard(
        name="Publish",
        topic_name=topic,
        topic_type=std_msgs.String,
        qos_profile=py_trees_ros.utilities.qos_profile_latched(),
        blackboard_variable="param"
    )
    publisher3 = py_trees_ros.publishers.FromBlackboard(
        name="Publish",
        topic_name=topic,
        topic_type=std_msgs.String,
        qos_profile=py_trees_ros.utilities.qos_profile_latched(),
        blackboard_variable="param"
    )
    publisher4 = py_trees_ros.publishers.FromBlackboard(
        name="Publish",
        topic_name=topic,
        topic_type=std_msgs.String,
        qos_profile=py_trees_ros.utilities.qos_profile_latched(),
        blackboard_variable="param"
    )
    publisher5 = py_trees_ros.publishers.FromBlackboard(
        name="Publish",
        topic_name=topic,
        topic_type=std_msgs.String,
        qos_profile=py_trees_ros.utilities.qos_profile_latched(),
        blackboard_variable="param"
    )
    # wait for response
    # timer = behaviours.MyTimer(name='Waiting', duration=1.0)
    # resp_2bb = py_trees_ros.subscribers.ToBlackboard(
    #     name="response2BB_",
    #     topic_name="/nurse/action",
    #     topic_type=std_msgs.String,
    #     qos_profile=py_trees_ros.utilities.qos_profile_latched(),
    #     blackboard_variables = {'response_msg': 'data'}
    # )
    # wait_for_res = py_trees.behaviours.WaitForBlackboardVariable(
    #     name="WaitForResponse",
    #     variable_name="/response_msg"
    # )
    # is_ok = py_trees.behaviours.CheckBlackboardVariableValue(
    #     name="Msg OK?",
    #     check=py_trees.common.ComparisonExpression(
    #         variable="response_msg",
    #         value=param_list[2],
    #         operator=operator.eq
    #     )
    # )
    # suc = py_trees.behaviours.Success(name=param_list[0]+'_success')
    root.add_children([param_to_bb, wait_for_req, publisher1, timer1, publisher2, timer2, publisher3, timer3, publisher4, timer4, publisher5])
    # root.add_children([resp_2bb, wait_for_res, is_ok, suc])
    return root

def create_nav_to_room_bt(ways) -> py_trees.behaviour.Behaviour:

    # Pseudo Waypoints Path
    # [[-28.5, 18.0, -1.57], [-19, 16], [-37, 15], [-39.44, 33.98, 0.0]]
    # ways = [[-38.0, 23.0, True ],
    #         [-37.0, 15.0, True ],
    #         [-38.0, 21.5, False]]
    # for i in range(0, len(ways)-1):
    #     ways[i].append(yaw)
    for i in range(1, len(ways)):
        yaw = math.atan2( (ways[i][1] - ways[i-1][1]) , (ways[i][0] - ways[i-1][0]))
        # aux = 0
        if len(ways[i]) > 2:
            # aux = ways[i].pop(2)
            # ways[i].append(aux)
            ways[i].append(True)
        else:
            ways[i].append(yaw)
            ways[i].append(True)
    if len(ways[-1]) > 2:
        ways[-1][3] = False
    else:
        ways[-1].append(False)

    # root = py_trees.composites.Sequence("NavTo")
    root = py_trees.composites.Parallel(
        name="NavTo",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )
    result_succeeded_to_bb = py_trees.behaviours.SetBlackboardVariable(
        name="reached_goal 'succeeded'",
        variable_name='reached_goal',
        variable_value=False
    )

    topics2bb = py_trees.composites.Sequence("Topics2BB")
    scan2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Scan2BB",
        topic_name='/'+os.environ['ROBOT_NAME']+"/scan",
        qos_profile=py_trees_ros.utilities.qos_profile_latched(),
        variable_name="event_scan_button"
    )
    cancel2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Cancel2BB",
        topic_name='/'+os.environ['ROBOT_NAME']+"/cancel",
        qos_profile=py_trees_ros.utilities.qos_profile_latched(),
        variable_name="event_cancel_button"
    )
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2BB",
        topic_name='/'+os.environ['ROBOT_NAME']+"/battery",
        qos_profile=py_trees_ros.utilities.qos_profile_latched(),
        threshold=30.0
    )
    tasks = py_trees.composites.Selector("Tasks")
    flash_red = behaviours.FlashLedStrip(
        name="Flash Red",
        colour="red"
    )
    # Emergency Tasks
    def check_battery_low_on_blackboard(blackboard: py_trees.blackboard.Blackboard) -> bool:
        return blackboard.battery_low_warning

    battery_emergency = py_trees.decorators.EternalGuard(
        name="Battery Low?",
        condition=check_battery_low_on_blackboard,
        blackboard_keys={"battery_low_warning"},
        child=flash_red
    )
    reach_goal = py_trees.composites.Selector(name="Goal Reached?")
    guard_room = py_trees.composites.Sequence("Guard Room")
    is_room_reached = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Room Reached?",
        check=py_trees.common.ComparisonExpression(
            variable="reached_goal",
            value=True,
            operator=operator.eq
        )
    )
    suc = py_trees.behaviours.Success(name='Success')
    sub_ways = create_waypoints_sequence(ways)
    

    # Build Tree
    root.add_child(topics2bb)
    topics2bb.add_children([scan2bb, cancel2bb, battery2bb])
    root.add_child(tasks)
    tasks.add_children([battery_emergency, reach_goal])
    reach_goal.add_children([guard_room, sub_ways])
    guard_room.add_children([is_room_reached, suc])

    return sub_ways

def create_waypoints_sequence(waypoints) -> py_trees.behaviour.Behaviour:
    waypoints.pop(0) # pop robot's initial position
    sub_root = py_trees.composites.Sequence("Waypoints Subtree")

    print(waypoints)
    print("Going to x="+str(waypoints[-1][0])+" y="+str(waypoints[-1][1]))
    param = std_msgs.String(data=os.environ['ROBOT_NAME']+",time ros unavailable"+",nav to ("+str(waypoints[-1][0])+";"+str(waypoints[-1][1])+')')
    param_to_bb = py_trees.behaviours.SetBlackboardVariable(
        name="param_to_bb ",
        variable_name='/param',
        variable_value=param
    )
    wait_for_req = py_trees.behaviours.WaitForBlackboardVariable(
        name="WaitForParam",
        variable_name="/param"
    )
    publisher1 = py_trees_ros.publishers.FromBlackboard(
        name="Publish",
        topic_name="/log",
        topic_type=std_msgs.String,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variable="param"
    )
    sub_root.add_children([param_to_bb, wait_for_req, publisher1])
    # move_actions = []
    for waypoint in waypoints:
        print("Adding: moving to x="+str(waypoint[0])+" y="+str(waypoint[1]))
        goal_pose = geometry_msgs.PoseStamped()
        goal_pose.header.frame_id = "map"
        timer = behaviours.MyTimer(
            name="Waiting to Move To x="+str(waypoint[0])+" y="+str(waypoint[1]),
            duration=2.0
        )
        q = quaternion_from_euler(0, 0, waypoint[2])
        goal_pose.pose.orientation.w = q[0]
        goal_pose.pose.orientation.x = q[1]
        goal_pose.pose.orientation.y = q[2]
        goal_pose.pose.orientation.z = q[3]
        goal_pose.pose.position = geometry_msgs.Point(
            x=float(waypoint[0]), 
            y=float(waypoint[1]), 
            z=0.0
        )
        move_to_somewhere = behaviours.NavToWaypoint(
                name="Move To x="+str(waypoint[0])+" y="+str(waypoint[1]),
                msg_type=geometry_msgs.PoseStamped,
                msg_goal=goal_pose,
                goal_topic_name="/"+os.environ['ROBOT_NAME']+"/send_goal",
                feddback_topic_name="/"+os.environ['ROBOT_NAME']+"/mb_feedback",
                odom_topic_name="/"+os.environ['ROBOT_NAME']+"/odom_aux",
                colour="red",
                intermediate_pose=waypoint[3]
        )
        sub_root.add_children([timer, move_to_somewhere])
        # move_actions.append(move_to_somewhere)

    result_succeeded_to_bb = py_trees.behaviours.SetBlackboardVariable(
        name="reached_goal 'succeeded'",
        variable_name='reached_goal',
        variable_value=True
    )
    sub_root.add_child(result_succeeded_to_bb)
    return sub_root

def create_authenticate_nurse_bt(param_list) -> py_trees.behaviour.Behaviour:
    # root = py_trees.behaviours.Success(name="Success")
    # Receive authentication
    # return succeeded
    root = py_trees.composites.Sequence("Authenticate Nurse")
    param = std_msgs.String(data=os.environ['ROBOT_NAME']+",time ros unavailable"+",Authenticate Nurse")
    param_to_bb = py_trees.behaviours.SetBlackboardVariable(
        name="param_to_bb "+param_list[0],
        variable_name='/param',
        variable_value=param
    )
    wait_for_req = py_trees.behaviours.WaitForBlackboardVariable(
        name="WaitForParam",
        variable_name="/param"
    )
    publisher1 = py_trees_ros.publishers.FromBlackboard(
        name="Publish",
        topic_name="/log",
        topic_type=std_msgs.String,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variable="param"
    )
    root.add_children([param_to_bb, wait_for_req, publisher1])
    # Send authentication request
    req_nurse = std_msgs.String(data="nurse")
    req_to_bb = py_trees.behaviours.SetBlackboardVariable(
        name="Send authentication request",
        variable_name='/req_nurse',
        variable_value=req_nurse
    )
    wait_for_req = py_trees.behaviours.WaitForBlackboardVariable(
        name="WaitForNurseReq",
        variable_name="/req_nurse"
    )
    publisher = py_trees_ros.publishers.FromBlackboard(
        topic_name="/led_strip/display",
        topic_type=std_msgs.String,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variable="req_nurse"
    )
    # wait for response
    nurse_resp_2bb = py_trees_ros.subscribers.ToBlackboard(
        name="NursRepos2BB",
        topic_name="/nurse/fauth",
        topic_type=std_msgs.String,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables = {'nurse_fauth': 'data'}
    )
    wait_for_res = py_trees.behaviours.WaitForBlackboardVariable(
        name="WaitForNurseAuth",
        variable_name="/nurse_fauth"
    )
    is_authenticated = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Auth OK?",
        check=py_trees.common.ComparisonExpression(
            variable="nurse_fauth",
            value='auth',
            operator=operator.eq
        )
    )
    # Request nurse position
    # Send position to robot
    # return succeeded
    root.add_children([req_to_bb, wait_for_req, publisher, nurse_resp_2bb, wait_for_res, is_authenticated])
    return root

def create_approach_nurse_bt(param_list) -> py_trees.behaviour.Behaviour:
    # root = py_trees.behaviours.Success(name="Success")
    root = py_trees.composites.Sequence("Approach Nurse")

    param = std_msgs.String(data=os.environ['ROBOT_NAME']+",time ros unavailable"+",Approach Nurse")
    param_to_bb = py_trees.behaviours.SetBlackboardVariable(
        name="param_to_bb "+param_list[0],
        variable_name='/param',
        variable_value=param
    )
    wait_for_req = py_trees.behaviours.WaitForBlackboardVariable(
        name="WaitForParam",
        variable_name="/param"
    )
    publisher1 = py_trees_ros.publishers.FromBlackboard(
        name="Publish",
        topic_name="/log",
        topic_type=std_msgs.String,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variable="param"
    )
    root.add_children([param_to_bb, wait_for_req, publisher1])
    # Request nurse position
        # my_message = std_msgs.String(data=param_list[0])
        # result_to_bb = py_trees.behaviours.SetBlackboardVariable(
        #     name="reached_goal 'succeeded'",
        #     variable_name='my_message',
        #     variable_value=my_message
        # )
        # wait_for_data = py_trees.behaviours.WaitForBlackboardVariable(
        #     name="WaitForData",
        #     variable_name="my_message"
        # )
        # publisher = py_trees_ros.publishers.FromBlackboard(
        #     topic_name="/drawer_"+param_list[0],
        #     topic_type=std_msgs.String,
        #     qos_profile=py_trees_ros.utilities.qos_profile_latched(),
        #     blackboard_variable="my_message"
        # )
        # suc = py_trees.behaviours.Success(name=param_list[0]+'_success')
        # root.add_children([result_to_bb, wait_for_data, publisher, suc])
        
    nurse_pos_2bb = py_trees_ros.subscribers.ToBlackboard(
        name="nursepos2BB",
        topic_name="/"+param_list[0]+"/pose",
        topic_type=geometry_msgs.PoseStamped,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables = {'nurse_pose': None}
    )
    wait_for_data = py_trees.behaviours.WaitForBlackboardVariable(
        name="WaitForNursePos",
        variable_name="/nurse_pose"
    )
    # got to nurse
    goal_pose = geometry_msgs.PoseStamped()
    goal_pose.pose.position = geometry_msgs.Point(x=15.0, y=15.0, z=0.0)
    move_to_nurse = behaviours.NavToFromBB(
        name="Move To Nurse",
        msg_type=geometry_msgs.PoseStamped,
        blackboard_variable="/nurse_pose",
        goal_topic_name='/'+os.environ['ROBOT_NAME']+"/send_goal",
        feddback_topic_name='/'+os.environ['ROBOT_NAME']+"/mb_feedback",
        odom_topic_name='/'+os.environ['ROBOT_NAME']+"/odom_aux",
        colour="red",
        intermediate_pose=False
    )

        # nursepos2bb = py_trees_ros.subscribers.EventToBlackboard(
        #     name="nursepos2BB",
        #     topic_name="/nurse/pose",
        #     qos_profile=py_trees_ros.utilities.qos_profile_latched(),
        #     variable_name="event_scan_button"
        # )
    # Request nurse position
    # Send position to robot
    # return succeeded
    root.add_children([nurse_pos_2bb, wait_for_data])
    return root

def create_approach_robot_bt(param_list) -> py_trees.behaviour.Behaviour:
    # root = py_trees.behaviours.Success(name="Success")
    root = py_trees.composites.Sequence("Approach robot")
    param = std_msgs.String(data=os.environ['ROBOT_NAME']+",time ros unavailable"+",Approach robot")
    param_to_bb = py_trees.behaviours.SetBlackboardVariable(
        name="param_to_bb "+param_list[0],
        variable_name='/param',
        variable_value=param
    )
    wait_for_req = py_trees.behaviours.WaitForBlackboardVariable(
        name="WaitForParam",
        variable_name="/param"
    )
    publisher1 = py_trees_ros.publishers.FromBlackboard(
        name="Publish",
        topic_name="/log",
        topic_type=std_msgs.String,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variable="param"
    )
    root.add_children([param_to_bb, wait_for_req, publisher1])
    # Request robot position
        # my_message = std_msgs.String(data=param_list[0])
        # result_to_bb = py_trees.behaviours.SetBlackboardVariable(
        #     name="reached_goal 'succeeded'",
        #     variable_name='my_message',
        #     variable_value=my_message
        # )
        # wait_for_data = py_trees.behaviours.WaitForBlackboardVariable(
        #     name="WaitForData",
        #     variable_name="my_message"
        # )
        # publisher = py_trees_ros.publishers.FromBlackboard(
        #     topic_name="/drawer_"+param_list[0],
        #     topic_type=std_msgs.String,
        #     qos_profile=py_trees_ros.utilities.qos_profile_latched(),
        #     blackboard_variable="my_message"
        # )
        # suc = py_trees.behaviours.Success(name=param_list[0]+'_success')
        # root.add_children([result_to_bb, wait_for_data, publisher, suc])
        
    robot_pos_2bb = py_trees_ros.subscribers.ToBlackboard(
        name="robotpos2BB",
        topic_name="/"+param_list[0]+"/pose",
        topic_type=geometry_msgs.PoseStamped,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables = {'robot_pose': None}
    )
    wait_for_data = py_trees.behaviours.WaitForBlackboardVariable(
        name="WaitForrobotPos",
        variable_name="/robot_pose"
    )
    # got to robot
    goal_pose = geometry_msgs.PoseStamped()
    goal_pose.pose.position = geometry_msgs.Point(x=15.0, y=15.0, z=0.0)
    move_to_robot = behaviours.NavToFromBB(
        name="Move To robot",
        msg_type=geometry_msgs.PoseStamped,
        blackboard_variable="/robot_pose",
        goal_topic_name='/'+os.environ['ROBOT_NAME']+"/send_goal",
        feddback_topic_name='/'+os.environ['ROBOT_NAME']+"/mb_feedback",
        odom_topic_name='/'+os.environ['ROBOT_NAME']+"/odom_aux",
        colour="red",
        intermediate_pose=False
    )

        # robotpos2bb = py_trees_ros.subscribers.EventToBlackboard(
        #     name="robotpos2BB",
        #     topic_name="/robot/pose",
        #     qos_profile=py_trees_ros.utilities.qos_profile_latched(),
        #     variable_name="event_scan_button"
        # )
    # Request nurse position
    # Send position to robot
    # return succeeded
    root.add_children([robot_pos_2bb, wait_for_data])
    return root

def create_action_drawer_bt(param_list) -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Sequence("ActionDrawer")

    param = std_msgs.String(data=os.environ['ROBOT_NAME']+",time ros unavailable"+",ActionDrawer "+param_list[0])
    param_to_bb = py_trees.behaviours.SetBlackboardVariable(
        name="param_to_bb "+param_list[0],
        variable_name='/param',
        variable_value=param
    )
    wait_for_req = py_trees.behaviours.WaitForBlackboardVariable(
        name="WaitForParam",
        variable_name="/param"
    )
    publisher1 = py_trees_ros.publishers.FromBlackboard(
        name="Publish",
        topic_name="/log",
        topic_type=std_msgs.String,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variable="param"
    )
    root.add_children([param_to_bb, wait_for_req, publisher1])

    # root = py_trees.composites.Sequence("SendMsg")
    timer = behaviours.MyTimer(name=param_list[0]+'ing', duration=1.0)
    my_message = std_msgs.String(data=param_list[0])
    result_to_bb = py_trees.behaviours.SetBlackboardVariable(
        name="reached_goal 'succeeded'",
        variable_name='my_message',
        variable_value=my_message
    )
    wait_for_data = py_trees.behaviours.WaitForBlackboardVariable(
        name="WaitForData",
        variable_name="my_message"
    )
    publisher = py_trees_ros.publishers.FromBlackboard(
        topic_name="/drawer_"+param_list[0],
        topic_type=std_msgs.String,
        qos_profile=py_trees_ros.utilities.qos_profile_latched(),
        blackboard_variable="my_message"
    )
    suc = py_trees.behaviours.Success(name=param_list[0]+'_success')
    root.add_children([result_to_bb, wait_for_data, publisher, timer, suc])
    # action_node = None
    # if param_list[0] == "open":
    #     action_node = behaviours.ActionDrawer(action="open")
    # else:
    #     action_node = behaviours.ActionDrawer(action="close")

   # result_succeeded_to_bb = py_trees.behaviours.SetBlackboardVariable(
   #      name="reached_goal 'succeeded'",
   #      variable_name='reached_goal',
   #      variable_value=True
   #  )
   #  sub_root.add_children([result_succeeded_to_bb])

    # root = create_wacther_bt(sub_root)
    return root

def create_wacther_bt(subtree_to_insert) -> py_trees.behaviour.Behaviour:
    # root = py_trees.composites.Sequence("NavTo")
    root = py_trees.composites.Parallel(
        name="WatchBT",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )

    topics2bb = py_trees.composites.Sequence("Topics2BB")
    scan2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Scan2BB",
        topic_name="/dashboard/scan",
        qos_profile=py_trees_ros.utilities.qos_profile_latched(),
        variable_name="event_scan_button"
    )
    cancel2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Cancel2BB",
        topic_name="/dashboard/cancel",
        qos_profile=py_trees_ros.utilities.qos_profile_latched(),
        variable_name="event_cancel_button"
    )
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2BB",
        topic_name="/turtlebot1/battery",
        qos_profile=py_trees_ros.utilities.qos_profile_latched(),
        threshold=30.0
    )
    tasks = py_trees.composites.Selector("Tasks")
    flash_red = behaviours.FlashLedStrip(
        name="Flash Red",
        colour="red"
    )
    # Emergency Tasks
    def check_battery_low_on_blackboard(blackboard: py_trees.blackboard.Blackboard) -> bool:
        return blackboard.battery_low_warning

    battery_emergency = py_trees.decorators.EternalGuard(
        name="Battery Low?",
        condition=check_battery_low_on_blackboard,
        blackboard_keys={"battery_low_warning"},
        child=flash_red
    )
    reach_goal = py_trees.composites.Selector(name="Goal Reached?")
    guard_room = py_trees.composites.Sequence("Guard Room")
    is_room_reached = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Room Reached?",
        check=py_trees.common.ComparisonExpression(
            variable="reached_goal",
            value=True,
            operator=operator.eq
        )
    )
    suc = py_trees.behaviours.Success(name='Success')

    # Build Tree
    root.add_child(topics2bb)
    topics2bb.add_children([scan2bb, cancel2bb, battery2bb])
    root.add_child(tasks)
    tasks.add_children([battery_emergency, subtree_to_insert])

    return root


##############################################################################
# Tutorial
##############################################################################

def create_second_bt() -> py_trees.behaviour.Behaviour:
    """
    Insert a task between battery emergency and idle behaviours that
    controls a rotation action controller and notifications simultaenously
    to scan a room.

    Returns:
        the root of the tree
    """
    root = py_trees.composites.Parallel(
        name="Tutorial Seven",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )

    topics2bb = py_trees.composites.Sequence("Topics2BB")
    scan2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Scan2BB",
        topic_name="/dashboard/scan",
        qos_profile=py_trees_ros.utilities.qos_profile_latched(),
        variable_name="event_scan_button"
    )
    cancel2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Cancel2BB",
        topic_name="/dashboard/cancel",
        qos_profile=py_trees_ros.utilities.qos_profile_latched(),
        variable_name="event_cancel_button"
    )
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2BB",
        topic_name="/turtlebot1/battery",
        qos_profile=py_trees_ros.utilities.qos_profile_latched(),
        threshold=30.0
    )
    tasks = py_trees.composites.Selector("Tasks")
    flash_red = behaviours.FlashLedStrip(
        name="Flash Red",
        colour="red"
    )

    # Emergency Tasks
    def check_battery_low_on_blackboard(blackboard: py_trees.blackboard.Blackboard) -> bool:
        return blackboard.battery_low_warning

    battery_emergency = py_trees.decorators.EternalGuard(
        name="Battery Low?",
        condition=check_battery_low_on_blackboard,
        blackboard_keys={"battery_low_warning"},
        child=flash_red
    )
    # Worker Tasks
    scan = py_trees.composites.Sequence(name="Scan")
    is_scan_requested = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Scan?",
        check=py_trees.common.ComparisonExpression(
            variable="event_scan_button",
            value=True,
            operator=operator.eq
        )
    )
    scan_or_die = py_trees.composites.Selector(name="Scan or Die")
    die = py_trees.composites.Sequence(name="Die")
    failed_notification = py_trees.composites.Parallel(
        name="Notification",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    failed_flash_green = behaviours.FlashLedStrip(name="Flash Red", colour="red")
    failed_pause = py_trees.timers.Timer("Pause", duration=3.0)
    result_failed_to_bb = py_trees.behaviours.SetBlackboardVariable(
        name="Result2BB\n'failed'",
        variable_name='scan_result',
        variable_value='failed'
    )
    ere_we_go = py_trees.composites.Sequence(name="Ere we Go")
    undock = py_trees_ros.actions.ActionClient(
        name="UnDock",
        action_type=py_trees_actions.Dock,
        action_name="dock",
        action_goal=py_trees_actions.Dock.Goal(dock=False),
        generate_feedback_message=lambda msg: "undocking"
    )
    scan_or_be_cancelled = py_trees.composites.Selector("Scan or Be Cancelled")
    cancelling = py_trees.composites.Sequence("Cancelling?")
    is_cancel_requested = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Cancel?",
        check=py_trees.common.ComparisonExpression(
            variable="event_cancel_button",
            value=True,
            operator=operator.eq
        )
    )
    move_home_after_cancel = py_trees_ros.actions.ActionClient(
        name="Move Home",
        action_type=py_trees_actions.MoveBase,
        action_name="move_base",
        action_goal=py_trees_actions.MoveBase.Goal(),
        generate_feedback_message=lambda msg: "moving home"
    )
    result_cancelled_to_bb = py_trees.behaviours.SetBlackboardVariable(
        name="Result2BB\n'cancelled'",
        variable_name='scan_result',
        variable_value='cancelled'
    )
    move_out_and_scan = py_trees.composites.Sequence("Move Out and Scan")
    move_base = py_trees_ros.actions.ActionClient(
        name="Move Out",
        action_type=py_trees_actions.MoveBase,
        action_name="move_base",
        action_goal=py_trees_actions.MoveBase.Goal(),
        generate_feedback_message=lambda msg: "moving out"
    )
    scanning = py_trees.composites.Parallel(
        name="Scanning",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    scan_context_switch = behaviours.ScanContext("Context Switch")
    scan_rotate = py_trees_ros.actions.ActionClient(
        name="Rotate",
        action_type=py_trees_actions.Rotate,
        action_name="rotate",
        action_goal=py_trees_actions.Rotate.Goal(),
        generate_feedback_message=lambda msg: "{:.2f}%%".format(msg.feedback.percentage_completed)
    )
    scan_flash_blue = behaviours.FlashLedStrip(name="Flash Blue", colour="blue")
    move_home_after_scan = py_trees_ros.actions.ActionClient(
        name="Move Home",
        action_type=py_trees_actions.MoveBase,
        action_name="move_base",
        action_goal=py_trees_actions.MoveBase.Goal(),
        generate_feedback_message=lambda msg: "moving home"
    )
    result_succeeded_to_bb = py_trees.behaviours.SetBlackboardVariable(
        name="Result2BB\n'succeeded'",
        variable_name='scan_result',
        variable_value='succeeded'
    )
    celebrate = py_trees.composites.Parallel(
        name="Celebrate",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    celebrate_flash_green = behaviours.FlashLedStrip(name="Flash Green", colour="green")
    celebrate_pause = py_trees.timers.Timer("Pause", duration=3.0)
    dock = py_trees_ros.actions.ActionClient(
        name="Dock",
        action_type=py_trees_actions.Dock,
        action_name="dock",
        action_goal=py_trees_actions.Dock.Goal(dock=True),  # noqa
        generate_feedback_message=lambda msg: "docking"
    )

    class SendResult(py_trees.behaviour.Behaviour):

        def __init__(self, name: str):
            super().__init__(name="Send Result")
            self.blackboard = self.attach_blackboard_client(name=self.name)
            self.blackboard.register_key(
                key="scan_result",
                access=py_trees.common.Access.READ
            )

        def update(self):
            print(console.green +
                  "********** Result: {} **********".format(self.blackboard.scan_result) +
                  console.reset
                  )
            return py_trees.common.Status.SUCCESS

    send_result = SendResult(name="Send Result")

    # Fallback task
    idle = py_trees.behaviours.Running(name="Idle")

    root.add_child(topics2bb)
    topics2bb.add_children([scan2bb, cancel2bb, battery2bb])
    root.add_child(tasks)
    tasks.add_children([battery_emergency, scan, idle])
    scan.add_children([is_scan_requested, scan_or_die, send_result])
    scan_or_die.add_children([ere_we_go, die])
    die.add_children([failed_notification, result_failed_to_bb])
    failed_notification.add_children([failed_flash_green, failed_pause])
    ere_we_go.add_children([undock, scan_or_be_cancelled, dock, celebrate])
    scan_or_be_cancelled.add_children([cancelling, move_out_and_scan])
    cancelling.add_children([is_cancel_requested, move_home_after_cancel, result_cancelled_to_bb])
    move_out_and_scan.add_children([move_base, scanning, move_home_after_scan, result_succeeded_to_bb])
    scanning.add_children([scan_context_switch, scan_rotate, scan_flash_blue])
    celebrate.add_children([celebrate_flash_green, celebrate_pause])
    return root
