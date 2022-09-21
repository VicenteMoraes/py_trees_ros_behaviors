import py_trees_ros.trees
import py_trees_ros_interfaces.action as py_trees_actions  # noqa

import geometry_msgs.msg as geometry_msgs
import std_msgs.msg as std_msgs

from src import behaviours
from src import mock

from mission_control.execution import SkillLibrary, TickStatus, SkillImplementation

# func utils
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

class WaitMsgSkill(SkillImplementation):
    def __del__(self):
        self.tree.shutdown()

    def ros_spin(rclpy):
        rclpy.spin_once(self.tree.node, timeout_sec=0)

    def on_load(self, task):
        self.root = self.create_wait_message(task.attributes)
        self.tree = py_trees_ros.trees.BehaviourTree(
            root=self.root,
            unicode_tree_debug=True
        )
        self.tree.setup(timeout=15)
        self.task = task

    def on_tick(self):
        self.tree.root.tick_once()
        if self.tree.status == py_trees.common.Status.RUNNING:
            return TickStatus(status=TickStatus.Type.IN_PROGRESS, task=self.task)
        elif self.tree.status == py_trees.common.Status.SUCCESS:
            return TickStatus(status=TickStatus.Type.COMPLETED_WITH_SUC, task=self.task)
        else:
            return TickStatus(status=TickStatus.Type.FATAL_FAILURE, task=self.task)

    def on_complete(self):
        print('task completed')

    def create_wait_message(param_list) -> py_trees.behaviour.Behaviour:
        root = py_trees.composites.Sequence("WaitMsg")
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
        content = {
            'skill': 'wait-message',
            'status': 'message-received'
        }
        logdata = {
            'level': 'info',
            'entity': os.environ['ROBOT_NAME'],
            'content': content
        }
        param = std_msgs.String(data=json.dumps(logdata))
        log_param = py_trees.behaviours.SetBlackboardVariable(
            name="ParamToBbLog",
            variable_name='/param',
            variable_value=param
        )
        log_wait = py_trees.behaviours.WaitForBlackboardVariable(
            name="WaitForParamLog",
            variable_name="/param"
        )
        log_pub = py_trees_ros.publishers.FromBlackboard(
            name="PublishLog",
            topic_name="/log",
            topic_type=std_msgs.String,
            qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
            blackboard_variable="param"
        )
        root.add_children([log_param, log_wait, log_pub])
        return root

class SendMsgSkill(SkillImplementation):
    def __del__(self):
        self.tree.shutdown()

    def ros_spin(rclpy):
        rclpy.spin_once(self.tree.node, timeout_sec=0)

    def on_load(self, task):
        self.root = self.create_send_message(task.attributes)
        self.tree = py_trees_ros.trees.BehaviourTree(
            root=self.root,
            unicode_tree_debug=True
        )
        self.tree.setup(timeout=15)
        self.task = task

    def on_tick(self):
        self.tree.root.tick_once()
        if self.tree.status == py_trees.common.Status.RUNNING:
            return TickStatus(status=TickStatus.Type.IN_PROGRESS, task=self.task)
        elif self.tree.status == py_trees.common.Status.SUCCESS:
            return TickStatus(status=TickStatus.Type.COMPLETED_WITH_SUC, task=self.task)
        else:
            return TickStatus(status=TickStatus.Type.FATAL_FAILURE, task=self.task)

    def on_complete(self):
        print('task completed')

    def create_send_message(param_list) -> py_trees.behaviour.Behaviour:
        root = py_trees.composites.Sequence("SendMsg")
        content = {
            'skill': 'send-message',
            'to': param_list[0]
        }
        logdata = {
            'level': 'info',
            'entity': os.environ['ROBOT_NAME'],
            'content': content
        }
        param = std_msgs.String(data=json.dumps(logdata))
        log_param = py_trees.behaviours.SetBlackboardVariable(
            name="ParamToBbLog",
            variable_name='/param',
            variable_value=param
        )
        log_wait = py_trees.behaviours.WaitForBlackboardVariable(
            name="WaitForParamLog",
            variable_name="/param"
        )
        log_pub = py_trees_ros.publishers.FromBlackboard(
            name="PublishLog",
            topic_name="/log",
            topic_type=std_msgs.String,
            qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
            blackboard_variable="param"
        )
        root.add_children([log_param, log_wait, log_pub])


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
        root.add_children([param_to_bb, wait_for_req, publisher1, timer1, publisher2, timer2, publisher3, timer3, publisher4, timer4, publisher5])
        # root.add_children([resp_2bb, wait_for_res, is_ok, suc])
        return root

class Going2FailSkill(SkillImplementation):
    def __del__(self):
        self.tree.shutdown()

    def ros_spin(rclpy):
        rclpy.spin_once(self.tree.node, timeout_sec=0)

    def on_load(self, task):
        self.root = self.going_to_fail(task.attributes)
        self.tree = py_trees_ros.trees.BehaviourTree(
            root=self.root,
            unicode_tree_debug=True
        )
        self.tree.setup(timeout=15)
        self.task = task

    def on_tick(self):
        self.tree.root.tick_once()
        if self.tree.status == py_trees.common.Status.RUNNING:
            return TickStatus(status=TickStatus.Type.IN_PROGRESS, task=self.task)
        elif self.tree.status == py_trees.common.Status.SUCCESS:
            return TickStatus(status=TickStatus.Type.COMPLETED_WITH_SUC, task=self.task)
        else:
            return TickStatus(status=TickStatus.Type.FATAL_FAILURE, task=self.task)

    def on_complete(self):
        print('task completed')

    def going_to_fail(self, params) -> py_trees.behaviour.Behaviour:
        root = py_trees.composites.Sequence("Idle")
        timer = behaviours.MyTimer(name="timer", duration=2.0)
        fail = py_trees.behaviours.Failure(name='Failure')
        root.add_children([timer, fail])
        return root

class Nav2RoomSkill(SkillImplementation):
    def __del__(self):
        self.tree.shutdown()

    def ros_spin(rclpy):
        rclpy.spin_once(self.tree.node, timeout_sec=0)

    def on_load(self, task):
        self.root = self.create_nav_to_room_bt(task.attributes)
        self.tree = py_trees_ros.trees.BehaviourTree(
            root=self.root,
            unicode_tree_debug=True
        )
        self.tree.setup(timeout=15)
        self.task = task

    def on_tick(self):
        self.tree.root.tick_once()
        if self.tree.status == py_trees.common.Status.RUNNING:
            return TickStatus(status=TickStatus.Type.IN_PROGRESS, task=self.task)
        elif self.tree.status == py_trees.common.Status.SUCCESS:
            return TickStatus(status=TickStatus.Type.COMPLETED_WITH_SUC, task=self.task)
        else:
            return TickStatus(status=TickStatus.Type.FATAL_FAILURE, task=self.task)

    def on_complete(self):
        print('task completed')

    def create_nav_to_room_bt(self, path) -> py_trees.behaviour.Behaviour:
        destiny = path[0]
        ways = path[1]

        # Pseudo Waypoints Path
        # [[-28.5, 18.0, -1.57], [-19, 16], [-37, 15], [-39.44, 33.98, 0.0]]
        # ways = [[-38.0, 23.0, True ],
        #         [-37.0, 15.0, True ],
        #         [-38.0, 21.5, False]]
        # for i in range(0, len(ways)-1):
        #     ways[i].append(yaw)
        for i in range(0, len(ways)-1):
            yaw = math.atan2( (ways[i+1][1] - ways[i][1]) , (ways[i+1][0] - ways[i][0]))
            # aux = 0
            if len(ways[i]) > 2:
                # aux = ways[i].pop(2)
                # ways[i].append(aux)
                ways[i].append(True)
            else:
                ways[i].append(yaw)
                ways[i].append(True)
        if len(ways[-1]) > 3:
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
        sub_ways = self.create_waypoints_sequence(destiny, ways)
        

        # Build Tree
        root.add_child(topics2bb)
        topics2bb.add_children([scan2bb, cancel2bb, battery2bb])
        root.add_child(tasks)
        tasks.add_children([battery_emergency, reach_goal])
        reach_goal.add_children([guard_room, sub_ways])
        guard_room.add_children([is_room_reached, suc])

        return sub_ways

    def create_waypoints_sequence(self, destiny, waypoints) -> py_trees.behaviour.Behaviour:
        waypoints.pop(0) # pop robot's initial position
        sub_root = py_trees.composites.Sequence("Waypoints Subtree")

        print(waypoints)
        print("Going to x="+str(waypoints[-1][0])+" y="+str(waypoints[-1][1]))
        
        # move_actions = []
        for waypoint in waypoints:
            print("Adding: moving to x="+str(waypoint[0])+" y="+str(waypoint[1]))
            # TASK
            goal_pose = geometry_msgs.PoseStamped()
            goal_pose.header.frame_id = "map"
            timer = behaviours.MyTimer(
                name="Waiting to Move To x="+str(waypoint[0])+" y="+str(waypoint[1]),
                duration=5.0
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
            # TASK UPDATE [LOG]
            percentage = float(waypoints.index(waypoint)+1)/len(waypoints)
            content = {
                'skill': 'navigation',
                'status': 'way-point-reached',
                'goingto': destiny,
                'percentage': '{:02.2f}%'.format(percentage*100)
            }
            logdata = {
                'level': 'debug',
                'entity': os.environ['ROBOT_NAME'],
                'content': content
            }
            param = std_msgs.String(data=json.dumps(logdata))
            log_param = py_trees.behaviours.SetBlackboardVariable(
                name="ParamToBbLog",
                variable_name='/param',
                variable_value=param
            )
            log_wait = py_trees.behaviours.WaitForBlackboardVariable(
                name="WaitForParamLog",
                variable_name="/param"
            )
            log_pub = py_trees_ros.publishers.FromBlackboard(
                name="PublishLog",
                topic_name="/log",
                topic_type=std_msgs.String,
                qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
                blackboard_variable="param"
            )
            sub_root.add_children([log_param, log_wait, log_pub])

        result_succeeded_to_bb = py_trees.behaviours.SetBlackboardVariable(
            name="reached_goal 'succeeded'",
            variable_name='reached_goal',
            variable_value=True
        )
        sub_root.add_child(result_succeeded_to_bb)
        return sub_root

class AuthPersonSkill(SkillImplementation):
    def __del__(self):
        self.tree.shutdown()

    def ros_spin(rclpy):
        rclpy.spin_once(self.tree.node, timeout_sec=0)

    def on_load(self, task):
        self.root = self.create_authenticate_nurse_bt(task.attributes)
        self.tree = py_trees_ros.trees.BehaviourTree(
            root=self.root,
            unicode_tree_debug=True
        )
        self.tree.setup(timeout=15)
        self.task = task

    def on_tick(self):
        self.tree.root.tick_once()
        if self.tree.status == py_trees.common.Status.RUNNING:
            return TickStatus(status=TickStatus.Type.IN_PROGRESS, task=self.task)
        elif self.tree.status == py_trees.common.Status.SUCCESS:
            return TickStatus(status=TickStatus.Type.COMPLETED_WITH_SUC, task=self.task)
        else:
            return TickStatus(status=TickStatus.Type.FATAL_FAILURE, task=self.task)

    def on_complete(self):
        print('task completed')

    def create_authenticate_nurse_bt(self, param_list) -> py_trees.behaviour.Behaviour:
        root = py_trees.composites.Sequence("Authenticate Nurse")
        content = {
            'skill': 'authenticate-person',
            'status': 'request-authentication',
            'to': 'nurse'
        }
        logdata = {
            'level': 'debug',
            'entity': os.environ['ROBOT_NAME'],
            'content': content
        }
        param = std_msgs.String(data=json.dumps(logdata))
        log_param = py_trees.behaviours.SetBlackboardVariable(
            name="ParamToBbLog",
            variable_name='/param',
            variable_value=param
        )
        log_wait = py_trees.behaviours.WaitForBlackboardVariable(
            name="WaitForParamLog",
            variable_name="/param"
        )
        log_pub = py_trees_ros.publishers.FromBlackboard(
            name="PublishLog",
            topic_name="/log",
            topic_type=std_msgs.String,
            qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
            blackboard_variable="param"
        )
        root.add_children([log_param, log_wait, log_pub])
        # Request nurse position
        # log authentication request
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
        # Send position to robot
        root.add_children([req_to_bb, wait_for_req, publisher, nurse_resp_2bb, wait_for_res, is_authenticated])
        content = {
            'skill': 'authenticate-person',
            'status': 'received-authentication',
            'from': 'nurse'
        }
        logdata = {
            'level': 'debug',
            'entity': os.environ['ROBOT_NAME'],
            'content': content
        }
        param = std_msgs.String(data=json.dumps(logdata))
        log_param = py_trees.behaviours.SetBlackboardVariable(
            name="ParamToBbLog",
            variable_name='/param',
            variable_value=param
        )
        log_wait = py_trees.behaviours.WaitForBlackboardVariable(
            name="WaitForParamLog",
            variable_name="/param"
        )
        log_pub = py_trees_ros.publishers.FromBlackboard(
            name="PublishLog",
            topic_name="/log",
            topic_type=std_msgs.String,
            qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
            blackboard_variable="param"
        )
        root.add_children([log_param, log_wait, log_pub])
        # return succeeded
        return root

class ApproachPersonSkill(SkillImplementation):
    def __del__(self):
        self.tree.shutdown()

    def ros_spin(rclpy):
        rclpy.spin_once(self.tree.node, timeout_sec=0)

    def on_load(self, task):
        self.root = self.create_approach_nurse_bt(task.attributes)
        self.tree = py_trees_ros.trees.BehaviourTree(
            root=self.root,
            unicode_tree_debug=True
        )
        self.tree.setup(timeout=15)
        self.task = task

    def on_tick(self):
        self.tree.root.tick_once()
        if self.tree.status == py_trees.common.Status.RUNNING:
            return TickStatus(status=TickStatus.Type.IN_PROGRESS, task=self.task)
        elif self.tree.status == py_trees.common.Status.SUCCESS:
            return TickStatus(status=TickStatus.Type.COMPLETED_WITH_SUC, task=self.task)
        else:
            return TickStatus(status=TickStatus.Type.FATAL_FAILURE, task=self.task)

    def on_complete(self):
        print('task completed')

    def create_approach_nurse_bt(self, param_list) -> py_trees.behaviour.Behaviour:
            root = py_trees.composites.Sequence("Approach Nurse")
    
            content = {
                'skill': 'approach-person',
                'status': 'localize-person',
                'who': 'nurse'
            }
            logdata = {
                'level': 'debug',
                'entity': os.environ['ROBOT_NAME'],
                'content': content
            }
            param = std_msgs.String(data=json.dumps(logdata))
            log_param = py_trees.behaviours.SetBlackboardVariable(
                name="ParamToBbLog",
                variable_name='/param',
                variable_value=param
            )
            log_wait = py_trees.behaviours.WaitForBlackboardVariable(
                name="WaitForParamLog",
                variable_name="/param"
            )
            log_pub = py_trees_ros.publishers.FromBlackboard(
                name="PublishLog",
                topic_name="/log",
                topic_type=std_msgs.String,
                qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
                blackboard_variable="param"
            )
            root.add_children([log_param, log_wait, log_pub])
                
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
    
            # Request nurse position
            # Send position to robot
            # return succeeded
            root.add_children([nurse_pos_2bb, wait_for_data])
            content = {
                'skill': 'approach-person',
                'status': 'goto-person',
                'who': 'nurse'
            }
            logdata = {
                'level': 'debug',
                'entity': os.environ['ROBOT_NAME'],
                'content': content
            }
            param = std_msgs.String(data=json.dumps(logdata))
            log_param = py_trees.behaviours.SetBlackboardVariable(
                name="ParamToBbLog",
                variable_name='/param',
                variable_value=param
            )
            log_wait = py_trees.behaviours.WaitForBlackboardVariable(
                name="WaitForParamLog",
                variable_name="/param"
            )
            log_pub = py_trees_ros.publishers.FromBlackboard(
                name="PublishLog",
                topic_name="/log",
                topic_type=std_msgs.String,
                qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
                blackboard_variable="param"
            )
            root.add_children([log_param, log_wait, log_pub])
            return root

class ApproachRobotSkill(SkillImplementation):
    def __del__(self):
        self.tree.shutdown()

    def ros_spin(rclpy):
        rclpy.spin_once(self.tree.node, timeout_sec=0)

    def on_load(self, task):
        self.root = self.create_approach_robot_bt(task.attributes)
        self.tree = py_trees_ros.trees.BehaviourTree(
            root=self.root,
            unicode_tree_debug=True
        )
        self.tree.setup(timeout=15)
        self.task = task

    def on_tick(self):
        self.tree.root.tick_once()
        if self.tree.status == py_trees.common.Status.RUNNING:
            return TickStatus(status=TickStatus.Type.IN_PROGRESS, task=self.task)
        elif self.tree.status == py_trees.common.Status.SUCCESS:
            return TickStatus(status=TickStatus.Type.COMPLETED_WITH_SUC, task=self.task)
        else:
            return TickStatus(status=TickStatus.Type.FATAL_FAILURE, task=self.task)

    def on_complete(self):
        print('task completed')

    def create_approach_robot_bt(self, param_list) -> py_trees.behaviour.Behaviour:
        root = py_trees.composites.Sequence("Approach robot")
        content = {
            'skill': '=approach-robot',
            'status': 'localize-robot',
            'who': '{}'.format(param_list[0])
        }
        logdata = {
            'level': 'debug',
            'entity': os.environ['ROBOT_NAME'],
            'content': content
        }
        param = std_msgs.String(data=json.dumps(logdata))
        log_param = py_trees.behaviours.SetBlackboardVariable(
            name="ParamToBbLog",
            variable_name='/param',
            variable_value=param
        )
        log_wait = py_trees.behaviours.WaitForBlackboardVariable(
            name="WaitForParamLog",
            variable_name="/param"
        )
        log_pub = py_trees_ros.publishers.FromBlackboard(
            name="PublishLog",
            topic_name="/log",
            topic_type=std_msgs.String,
            qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
            blackboard_variable="param"
        )
        root.add_children([log_param, log_wait, log_pub])
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

        # Request nurse position
        # Send position to robot
        # return succeeded
        root.add_children([robot_pos_2bb, wait_for_data])
        content = {
            'skill': 'approach-robot',
            'status': 'goto-robot',
            'who': '{}'.format(param_list[0])
        }
        logdata = {
            'level': 'debug',
            'entity': os.environ['ROBOT_NAME'],
            'content': content
        }
        param = std_msgs.String(data=json.dumps(logdata))
        log_param = py_trees.behaviours.SetBlackboardVariable(
            name="ParamToBbLog",
            variable_name='/param',
            variable_value=param
        )
        log_wait = py_trees.behaviours.WaitForBlackboardVariable(
            name="WaitForParamLog",
            variable_name="/param"
        )
        log_pub = py_trees_ros.publishers.FromBlackboard(
            name="PublishLog",
            topic_name="/log",
            topic_type=std_msgs.String,
            qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
            blackboard_variable="param"
        )
        root.add_children([log_param, log_wait, log_pub])
        return root

class ActionDrawerSkill(SkillImplementation):
    def __del__(self):
        self.tree.shutdown()

    def ros_spin(rclpy):
        rclpy.spin_once(self.tree.node, timeout_sec=0)

    def on_load(self, task):
        self.root = self.create_action_drawer_bt(task.attributes)
        self.tree = py_trees_ros.trees.BehaviourTree(
            root=self.root,
            unicode_tree_debug=True
        )
        self.tree.setup(timeout=15)
        self.task = task

    def on_tick(self):
        self.tree.root.tick_once()
        if self.tree.status == py_trees.common.Status.RUNNING:
            return TickStatus(status=TickStatus.Type.IN_PROGRESS, task=self.task)
        elif self.tree.status == py_trees.common.Status.SUCCESS:
            return TickStatus(status=TickStatus.Type.COMPLETED_WITH_SUC, task=self.task)
        else:
            return TickStatus(status=TickStatus.Type.FATAL_FAILURE, task=self.task)

    def on_complete(self):
        print('task completed')

    def create_action_drawer_bt(self, param_list) -> py_trees.behaviour.Behaviour:
        root = py_trees.composites.Sequence("ActionDrawer")
        content = {
            'skill': 'operate-drawer',
            'status': 'task-update',
            'action': '{}'.format(param_list[0])
        }
        logdata = {
            'level': 'debug',
            'entity': os.environ['ROBOT_NAME'],
            'content': content
        }
        param = std_msgs.String(data=json.dumps(logdata))
        log_param = py_trees.behaviours.SetBlackboardVariable(
            name="ParamToBbLog",
            variable_name='/param',
            variable_value=param
        )
        log_wait = py_trees.behaviours.WaitForBlackboardVariable(
            name="WaitForParamLog",
            variable_name="/param"
        )
        log_pub = py_trees_ros.publishers.FromBlackboard(
            name="PublishLog",
            topic_name="/log",
            topic_type=std_msgs.String,
            qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
            blackboard_variable="param"
        )
        root.add_children([log_param, log_wait, log_pub])

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

        return root
