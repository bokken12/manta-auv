#!/usr/bin/env python
from __future__ import print_function
import rospy
import smach
import smach_ros
import smach_viewer
import roslaunch
import actionlib
from vortex_msgs.msg import PropulsionCommand, Manipulator
from sensor_msgs.msg import Joy
import actionlib_tutorials.msg
from std_msgs.msg import Bool, String, Float64

import actionlib
from actionlib_msgs.msg import GoalStatus
from depth_hold_action_server.msg import DepthHoldAction, DepthHoldGoal, DepthHoldActionFeedback
from pitch_hold_action_server.msg import PitchHoldAction, PitchHoldGoal, PitchHoldActionFeedback
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point, Quaternion, Wrench, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
from math import radians, pi
from vortex_msgs.msg import CameraObjectInfo
from robot_localization.srv import SetPose


class AC_handler():
    def __init__(self):
        self.depth_hold_ac = self.action_client('depth_hold_action_server', DepthHoldAction)
        #self.pitch_hold_ac = self.action_client('pitch_hold_action_server', PitchHoldAction)
        self.dp_controller_ac = self.action_client('move_base', MoveBaseAction)
    def action_client(self, name, message_type):
        client = actionlib.SimpleActionClient(name, message_type)
        client.wait_for_server()
        return client

    def cancel_all_goals(self):
        self.depth_hold_ac.cancel_all_goals()
        #self.pitch_hold_ac.cancel_all_goals()
        self.dp_controller_ac.cancel_all_goals()


def mission_trigger_callback(trigger_signal):
    print('Received signal')
    global mission_in_progress
    print(mission_in_progress)
    if(trigger_signal.data == True):
        mission_in_progress = not mission_in_progress
        print(mission_in_progress)

def depth_hold_feedback_callback(msg):
    global depth_hold_ready
    depth_hold_ready = msg.feedback.ready


def request_preempt():
    global mission_in_progress
    if mission_in_progress:
        return False
    else:
        return True

class Idle(smach.State):
    def __init__(self, arm_pub_, mode_pub_):
        smach.State.__init__(self, outcomes=['doing','waiting'])
        # subscribe to signal
        print('Init')
        self.rate = rospy.Rate(10)
        self.arm_pub_ = arm_pub_
        self.mode_pub_ = mode_pub_

    def execute(self, userdata):
        #print('Executing')
        global mission_in_progress
        if mission_in_progress == False:
            #print('Waiting')
            self.rate.sleep()
            return 'waiting'
        else:
            rospy.wait_for_service("/set_pose")
            try:
                reset_odom_call = rospy.ServiceProxy("/set_pose",SetPose)
                pose_msg = PoseWithCovarianceStamped()
                #pose_msg.pose.pose.covariance = 
                reset_odom_call(pose_msg)
            except rospy.ServiceException, e:
                print("Service failed: %s",e)

            self.arm_pub_.publish('ARM')
            print('Jumping')
            return 'doing'


class Cancel(smach.State):
    def __init__(self, ac_handler, waypoint_client, arm_pub_, mode_pub_, cam_disarm_pub_):
        smach.State.__init__(self, outcomes=['canceld'])
        self.ac_handler = ac_handler
        self.waypoint_client = waypoint_client
        self.arm_pub_ = arm_pub_
        self.mode_pub_ = mode_pub_
        self.cam_disarm_pub_ = cam_disarm_pub_
        self.reset_merger_pub = rospy.Publisher('/reset_merger', Bool, queue_size=1)
        self.stop_yaw_pub = rospy.Publisher('/yaw_arm', Bool, queue_size=1)

    def execute(self, userdata):
        #Kill all nodes
        self.ac_handler.cancel_all_goals()
        self.waypoint_client.goal_cnt = 0
        self.arm_pub_.publish('stop')
        mode_msg = PropulsionCommand()
        mode_msg.control_mode = [
            (True),
            (False),
            (False),
            (False),
            (False),
            (False)
        ]

        self.mode_pub_.publish(mode_msg)
        global mission_in_progress
        mission_in_progress = False
        self.cam_disarm_pub_.publish('Stop')
        self.stop_yaw_pub.publish(False)
        self.reset_merger_pub.publish(True)

        return 'canceld'


class Dive(smach.State):
    def __init__(self, ac_handler, cam_arm_pub_):
        smach.State.__init__(self, outcomes=['submerged','continue','preempted'])
        print('Diving')
        self.ac_handler = ac_handler
        self.cam_arm_pub_ = cam_arm_pub_
        self.rate = rospy.Rate(10)
        self.counter = 0
        self.yaw_arm_pub = rospy.Publisher('/yaw_arm', Bool, queue_size=1)


    def execute(self, userdata):
        # turn on diving stuff and do that shit
        if request_preempt():
            return 'preempted'

        if self.ac_handler.depth_hold_ac.get_state() != 1:
            goal = DepthHoldGoal(depth = -0.5)
            self.ac_handler.depth_hold_ac.send_goal(goal)
            self.cam_arm_pub_.publish('Start')
            self.yaw_arm_pub.publish(True)
            while(self.ac_handler.depth_hold_ac.get_state()!=1 and request_preempt() != True): 
                self.rate.sleep()

        #if self.ac_handler.pitch_hold_ac.get_state() != 1:
            #goal_pitch = PitchHoldGoal(pitch = 0)
            #self.ac_handler.pitch_hold_ac.send_goal(goal_pitch)
            #while(self.ac_handler.pitch_hold_ac.get_state()!=1):
            #   self.rate.sleep()

        #self.rate.sleep()
        global depth_hold_ready

        if depth_hold_ready:#self.client.get_state[1]:
            return 'submerged'
        else:
            return 'continue'


class SearchGate(smach.State):
    def __init__(self, ac_handler)
         smach.State.__init__(self, outcomes=['found','search','cancel'])
         self.ac_handler = ac_handler
         self.rate = rospy.Rate(10)


    def 


def main():
    #rospy.init_node('action_client_py')
    rospy.init_node('Qualification_run')

    global mission_in_progress
    mission_in_progress = False
    global depth_hold_ready
    depth_hold_ready = False
    global gate_conf
    gate_conf = 0

    rospy.Subscriber("mission_trigger", Bool, mission_trigger_callback)

    rospy.Subscriber("depth_hold_action_server/feedback", DepthHoldActionFeedback, depth_hold_feedback_callback)

    rospy.Subscriber('/gate_midpoint', CameraObjectInfo, gate_conf_callback, queue_size=1)

    arm_pub_ = rospy.Publisher('/mcu_arm', String, queue_size=1)

    cam_arm_pub_ = rospy.Publisher('/start_front_camera', String, queue_size=1)
    cam_disarm_pub_ = rospy.Publisher('/stop_front_camera', String, queue_size=1)
    mode_pub_ = rospy.Publisher('/manta/mode', PropulsionCommand, queue_size=1)


    ac_handler = AC_handler()
    waypoint_client = WaypointClient()
    sm = smach.StateMachine(outcomes = ['Done'])

    sis = smach_ros.IntrospectionServer('Qualification_run_server', sm, '/SM_ROOT')
    sis.start()


    with sm:
        smach.StateMachine.add('Idle', Idle(arm_pub_, mode_pub_),
                                transitions={'doing':'Dive', 'waiting':'Idle'})
        smach.StateMachine.add('Cancel', Cancel(ac_handler, waypoint_client, arm_pub_, mode_pub_, cam_disarm_pub_),
                                transitions={'canceld':'Idle'})
        smach.StateMachine.add('Dive', Dive(ac_handler, cam_arm_pub_),
                                transitions={'submerged':'Search', 'continue':'Dive','preempted':'Cancel'})
 #       smach.StateMachine.add('Search', Search(ac_handler, waypoint_client, mode_pub_),
 #                              transitions={'found':'Move', 'continue':'Search', 'preempted':'Cancel'})
 #       smach.StateMachine.add('Move', Move(),
 #                               transitions={'stop_state_machine':'Done', 'move_finished':'Cancel', 'gate_lost':'Search', 'continue':'Move','preempted':'Cancel'})


    outcome = sm.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
