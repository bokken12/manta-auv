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
from std_msgs.msg import Bool, String

import actionlib
from actionlib_msgs.msg import GoalStatus
from depth_hold_action_server.msg import DepthHoldAction, DepthHoldGoal, DepthHoldActionFeedback
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from math import radians, pi
from vortex_msgs.msg import CameraObjectInfo
#from vortex_msgs import Bouy_camera

#fiks at cameranode er tilgjengelig for state machine
#subscribe paa riktig topics saa man kan se om gate er funnet
'''
class subscribe_to_camera():
    #gir ut bredde, lengde og midpoints. confidence: 0=ser ikke, 1=ser, 0.5=forbi
    def __init_(self):
        rospy.Subscriber('/gate_midpoint', CameraObjectInfo, self.callback, queue_size=1)
        self.conf = 0
        self.midpoints = (0,0)
        self.width = 0

    def get_conf(self):
        return self.conf

    def callback(self, msg):
        self.conf = msg.confidence
        #self.midpoints = (msg.sub_gate.pos_x, msg.sub_gate.pos_y)
        #self.width = msg.sub_gate.frame_width
'''

def gate_conf_callback(msg):
    global gate_conf
    gate_conf = msg.confidence
    #print(gate_conf)

class WaypointClient():

    def __init__(self):
        #rospy.init_node('waypoint_client')

        # waypoint goal count
        self.goal_cnt = 0

        # Create a list to hold the target quaternions (orientations)
        quaternions = list()

        # First define the corner orientations as Euler angles
        euler_angles = (pi/2, pi, 3*pi/2, 0)

        # Then convert the angles to quaterions
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes = 'sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)

        # Create a list to hold the waypoint poses
        self.waypoints = list()

        # Append each of the four waypoints to the list. Each waypoint
        # is a pose consisting of a position and orientation in the map frame
        self.waypoints.append(Pose(Point(0.0, 0.0, -0.3), quaternions[0]))
        self.waypoints.append(Pose(Point(2.0, 0.0, -0.3), quaternions[1]))
        self.waypoints.append(Pose(Point(2.0, 2.0, -0.3), quaternions[2]))
        self.waypoints.append(Pose(Point(0.0, 0.0, -0.3), quaternions[3]))

        #Create action client
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(15.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")

    def start(self):
        self.movebase_client()


    # Callback triggered if the controller recieves the goal for the client and processes it
    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")


    # Callback triggered by feedback from the controller action server
    def feedback_cb(self, feedback):
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")


    # Checks the status message from the server
    def status_cb(self, status, result):

    #uint8 PENDING=0
    #uint8 ACTIVE=1
    #uint8 PREEMPTED=2
    #uint8 SUCCEEDED=3
    #uint8 ABORTED=4
    #uint8 REJECTED=5
    #uint8 PREEMPTING=6
    #uint8 RECALLING=7
    #uint8 RECALLED=8
    #uint8 LOST=9

        #add a new goal count when goal reached
        self.goal_cnt += 1

        # status PREEMPTED=2
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        # status SUCCEDED=3
        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached")

            if self.goal_cnt<len(self.waypoints):
                # produce the next goal
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.waypoints[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.waypoints[self.goal_cnt]))
                self.client.send_goal(next_goal, self.status_cb, self.active_cb, self.feedback_cb) 
            else:
                rospy.loginfo("Final waypoint have been reached")
                rospy.signal_shutdown("Mission success, shutdown client")
                return

        # status ABORTED
        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        # status REJECTED
        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        # status RECALLED - The goal received a cancel request before it started executing
        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.waypoints[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.waypoints[self.goal_cnt]))

        # Send goal
        self.client.send_goal(goal, self.status_cb, self.active_cb, self.feedback_cb)
        #rospy.spin()

class AC_handler():
    def __init__(self):
        self.depth_hold_ac = self.action_client('depth_hold_action_server', DepthHoldAction)
        self.dp_controller_ac = self.action_client('move_base', MoveBaseAction)

    def action_client(self, name, message_type):
        client = actionlib.SimpleActionClient(name, message_type)
        client.wait_for_server()
        return client

    def cancel_all_goals(self):
        self.depth_hold_ac.cancel_all_goals()
        self.dp_controller_ac.cancel_all_goals()


def mission_trigger_callback(trigger_signal):
    print('Received signal')
    global mission_in_progress
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
            self.arm_pub_.publish('ARM')
            print('Jumping')
            return 'doing'

class Cancel(smach.State):
    def __init__(self, ac_handler, waypoint_client, arm_pub_, mode_pub_):
        smach.State.__init__(self, outcomes=['canceld'])
        self.ac_handler = ac_handler
        self.waypoint_client = waypoint_client
        self.arm_pub_ = arm_pub_
        self.mode_pub_ = mode_pub_

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
        return 'canceld'


class Dive(smach.State):
    def __init__(self, ac_handler):
        smach.State.__init__(self, outcomes=['submerged','continue','preempted'])
        print('Diving')
        self.ac_handler = ac_handler
        self.rate = rospy.Rate(10)
        self.counter = 0


    def execute(self, userdata):
        # turn on diving stuff and do that shit
        if request_preempt():
            return 'preempted'

        if self.ac_handler.depth_hold_ac.get_state() != 1:
            goal = DepthHoldGoal(depth = -0.5)
            self.ac_handler.depth_hold_ac.send_goal(goal)
            while(self.ac_handler.depth_hold_ac.get_state()!=1):
                self.rate.sleep()

        #self.rate.sleep()
        global depth_hold_ready

        if depth_hold_ready:#self.client.get_state[1]:
            return 'submerged'
        else:
            return 'continue'





class Search(smach.State):
    def __init__(self, ac_handler, waypoint_client, mode_pub_):
        smach.State.__init__(self, outcomes=['found', 'continue','preempted'])
        #initialize stuff here
        self.counter = 0
        self.ac_handler = ac_handler
        self.waypoint_client = waypoint_client
        self.mode_pub_ = mode_pub_
        self.first = True
        '''
        self.camera_sub = camera_sub
        self.gate_found = self.camera_sub.get_conf()
        '''

    def execute(self, userdata):
        if request_preempt():
            return 'preempted'
        #Do findGateStuff here
        self.gate_found = False
        '''
        if self.ac_handler.dp_controller_ac.get_state() != 1:
            goal = MoveBaseGoal()
            goal.target_pose.pose.position.x = 5.0
            goal.target_pose.pose.position.y = 0.0
            goal.target_pose.pose.position.z = 0.0
            print(goal.target_pose.pose.position)
            self.ac_handler.dp_controller_ac.send_goal(goal)
        '''
        if self.first:
            self.first = False

        mode_msg = PropulsionCommand()
        mode_msg.control_mode = [
            (False),
            (True),
            (False),
            (False),
            (False),
            (False)
        ]

        self.mode_pub_.publish(mode_msg)
        self.waypoint_client.start()
        global gate_conf
        #print(gate_conf)
        if gate_conf == 1:
            return 'found'
        elif gate_conf == 0:
            return 'continue'


class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop_state_machine' ,'move_finished','gate_lost', 'continue','preempted'])
        self.counter = 0
        #self.camera = subscribe_to_camera()
        self.gate_found = 1#self.camera.conf


    def execute(self, userdata):
        if request_preempt():
            return 'preempted'
        #Do driving motion here
        gate_passed = False
        gate_lost = False
        self.counter += 1
        if(self.counter > 100):
            gate_passed = True

        elif self.gate_found == 0.5:
            return 'move_finished'
        elif self.gate_found == 0:
            return 'gate_lost'
        else:
            return 'continue'



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


    mode_pub_ = rospy.Publisher('/manta/mode', PropulsionCommand, queue_size=1)
    '''
    mode_msg = PropulsionCommand()
    mode_msg.control_mode = [
            (False),
            (True),
            (False),
            (False),
            (False),
            (False)
        ]

    rate = rospy.Rate(1)
    rate.sleep()
    mode_pub_.publish(mode_msg)
    '''
    ac_handler = AC_handler()
    waypoint_client = WaypointClient()
    sm = smach.StateMachine(outcomes = ['Done'])

    sis = smach_ros.IntrospectionServer('Qualification_run_server', sm, '/SM_ROOT')
    sis.start()


    with sm:
        smach.StateMachine.add('Idle', Idle(arm_pub_, mode_pub_),
                                transitions={'doing':'Dive', 'waiting':'Idle'})
        smach.StateMachine.add('Cancel', Cancel(ac_handler, waypoint_client, arm_pub_, mode_pub_),
                                transitions={'canceld':'Idle'})
        smach.StateMachine.add('Dive', Dive(ac_handler),
                                transitions={'submerged':'Search', 'continue':'Dive','preempted':'Cancel'})
        smach.StateMachine.add('Search', Search(ac_handler, waypoint_client, mode_pub_),
                                transitions={'found':'Move', 'continue':'Search', 'preempted':'Cancel'})
        smach.StateMachine.add('Move', Move(),
                                transitions={'stop_state_machine':'Done', 'move_finished':'Cancel', 'gate_lost':'Search', 'continue':'Move','preempted':'Cancel'})


    outcome = sm.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
