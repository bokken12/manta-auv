#!/usr/bin/env python
from __future__ import print_function
import rospy
import smach
import smach_ros
import smach_viewer
import roslaunch
import actionlib
import tf2_ros
from vortex_msgs.msg import PropulsionCommand, Manipulator
from sensor_msgs.msg import Joy
import actionlib_tutorials.msg
from std_msgs.msg import Bool, String, Float64
from nav_msgs.msg import Odometry

import actionlib
from actionlib_msgs.msg import GoalStatus
from depth_hold_action_server.msg import DepthHoldAction, DepthHoldGoal, DepthHoldActionFeedback
from pitch_hold_action_server.msg import PitchHoldAction, PitchHoldGoal, PitchHoldActionFeedback
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point, Quaternion, Wrench, PoseWithCovarianceStamped, TransformStamped, Vector3Stamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import radians, pi
from vortex_msgs.msg import CameraObjectInfo
from robot_localization.srv import SetPose
import math
#from vortex_msgs import Bouy_camera

def gate_callback(msg):
    global gate_conf
    gate_conf = msg.confidence
    global image_width
    image_width = msg.frame_width
    global gate_midpoint
    gate_midpoint = (msg.pos_x, msg.pos_y)

def buoy_callback(msg):
    global buoy_conf
    buoy_conf = msg.confidence
    global buoy_image_width
    buoy_image_width = msg.frame_width
    global gate_midpoint
    buoy_midpoint = (msg.pos_x, msg.pos_y)

class AC_handler():
    def __init__(self):
        self.depth_hold_ac = self.action_client('depth_hold_action_server', DepthHoldAction)
        #self.pitch_hold_ac = self.action_client('pitch_hold_action_server', PitchHoldAction)
        self.dp_controller_ac = self.action_client('move_base', MoveBaseAction)
        self.surge_arm_pub_ = rospy.Publisher('/surge_arm', Bool, queue_size=1)
        self.surge_pub_ = rospy.Publisher('/surge_goal', Float64, queue_size=1)
        self.sway_arm_pub_ = rospy.Publisher('/sway_arm', Bool, queue_size=1)
        self.surge_pub = rospy.Publisher('/surge_input', Wrench, queue_size=1)
        self.sway_pub = rospy.Publisher('/sway_input', Wrench, queue_size=1)
        self.sway_pub_ = rospy.Publisher('/sway_goal', Float64, queue_size=1)

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

def odom_callback(msg):
    global current_surge
    global current_sway
    global current_heave
    global current_roll
    global current_pitch
    global current_yaw
    curreny_heave = msg.pose.pose.position.z
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (current_roll, current_pitch, current_yaw) = euler_from_quaternion(orientation_list)
    current_surge = msg.pose.pose.position.x*math.cos(current_yaw) + msg.pose.pose.position.y*math.sin(current_yaw)
    current_sway = -msg.pose.pose.position.x*math.sin(current_yaw) + msg.pose.pose.position.y*math.cos(current_yaw)

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
        self.first = True
    def execute(self, userdata):
        #print('Executing')'
        if self.first:
            self.first = False
            #self.arm_pub_.publish('ARM')

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
                reset_odom_call(pose_msg)
            except rospy.ServiceException, e:
                print("Service failed: %s",e)

            self.arm_pub_.publish('ARM')

            print('Jumping')
            return 'doing'

class Cancel(smach.State):
    def __init__(self, ac_handler, arm_pub_, mode_pub_, cam_disarm_pub_):
        smach.State.__init__(self, outcomes=['canceld'])
        self.ac_handler = ac_handler
        self.arm_pub_ = arm_pub_
        self.mode_pub_ = mode_pub_
        self.cam_disarm_pub_ = cam_disarm_pub_
        self.reset_merger_pub = rospy.Publisher('/reset_merger', Bool, queue_size=1)
        self.stop_yaw_pub = rospy.Publisher('/yaw_arm', Bool, queue_size=1)


    def execute(self, userdata):
        #Kill all nodes
        self.ac_handler.cancel_all_goals()
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
        self.ac_handler.surge_arm_pub_.publish(False)
        self.ac_handler.sway_arm_pub_.publish(False)
        self.mode_pub_.publish(mode_msg)
        global mission_in_progress
        mission_in_progress = False
        self.cam_disarm_pub_.publish('Stop')
        self.stop_yaw_pub.publish(False)
        self.reset_merger_pub.publish(True)

        return 'canceld'
class start_point(smach.State):
    def __init__(self, ac_handler):
        smach.State.__init__(self, outcomes=['reached','continue','preempted'])
        self.yaw_goal_pub_ = rospy.Publisher('/yaw_goal', Float64, queue_size=1)
        self.yaw_goal_msg = Float64()
        self.ac_handler = ac_handler
        self.surge_msg = Float64()
        self.sway_msg = Float64()

        self.surge_ready_sub = rospy.Subscriber('/surge_ready', Bool, self.surge_ready_callback)

        self.surge_ready = False

        self.first = True

        self.rate = rospy.Rate(10)

    def surge_ready_callback(self, msg):
        self.surge_ready = msg.data

    def execute(self,userdata):
        if request_preempt():
            return 'preempted'
        global current_surge
        global current_sway
        global current_yaw

        if(self.first):
            #Set point
            global current_surge
            global current_sway
            self.surge_msg.data = 11.0 + current_surge
            self.sway_msg.data = 0.0 + current_sway
            self.yaw_goal_msg.data = -65
            self.ac_handler.surge_arm_pub_.publish(True)
            self.ac_handler.sway_arm_pub_.publish(True)
            self.yaw_goal_pub_.publish(self.yaw_goal_msg)
            self.ac_handler.surge_pub_.publish(self.surge_msg)
            self.ac_handler.sway_pub_.publish(self.sway_msg)
            self.first = False
            self.surge_ready = False

        if((self.first == False) and (self.surge_ready)):
            return 'reached'
        self.rate.sleep()
        return 'continue'

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
            goal = DepthHoldGoal(depth = -0.8)
            self.ac_handler.depth_hold_ac.send_goal(goal)
            self.cam_arm_pub_.publish('Start')
            self.yaw_arm_pub.publish(True)
            while(self.ac_handler.depth_hold_ac.get_state()!=1 and request_preempt() != True):
                self.rate.sleep()

        global depth_hold_ready

        if depth_hold_ready:
            return 'submerged'
        else:
            return 'continue'

class SearchGate(smach.State):
    def __init__(self, ac_handler):
        smach.State.__init__(self, outcomes=['found', 'continue','preempted'])
        #initialize stuff here
        self.gate_counter = 0
        self.ac_handler = ac_handler
        self.yaw_goal_pub_ = rospy.Publisher('/yaw_goal', Float64, queue_size=1)
        self.yaw_goal_msg = Float64()
        self.rate = rospy.Rate(10)
        self.surge_msg = Float64()
        self.sway_msg = Float64()
        self.first = True


    def execute(self, userdata):
        if request_preempt():
            return 'preempted'
        if self.first:
            global current_yaw
            self.yaw_goal_msg.data = current_yaw*180/math.pi
            self.ac_handler.surge_arm_pub_.publish(False)
            self.ac_handler.sway_arm_pub_.publish(False)
            surge_input = Wrench()
            sway_input = Wrench()
            self.rate.sleep()
            self.ac_handler.sway_pub.publish(sway_input)
            self.ac_handler.surge_pub.publish(surge_input)
            self.first = False
        global gate_conf
        if gate_conf == 1:
            self.gate_counter += 1
            if self.gate_counter > 5:
                self.gate_counter = 0
                return 'found'
        self.yaw_goal_msg.data += 1
        self.yaw_goal_pub_.publish(self.yaw_goal_msg)
        self.rate.sleep()
        return 'continue'

class CenterGate(smach.State):
    def __init__(self, ac_handler):
        smach.State.__init__(self, outcomes=['lost','centred','continue', 'preempted'])
        self.ac_handler = ac_handler
        self.yaw_goal_pub_ = rospy.Publisher('/yaw_goal', Float64, queue_size=1)
        self.yaw_goal_msg = Float64()
        self.rate = rospy.Rate(10)
        self.rate_long = rospy.Rate(0.5)
        self.K_p = 10
        self.K_d = 3
        self.dt = 0.1
        self.max = 40
        self.min = -40
        self.first = True
        self.pre_error = 0

        self.surge_msg = Float64()
        self.sway_msg = Float64()
        self.ac_handler.sway_pub_ = rospy.Publisher('/sway_goal', Float64, queue_size=1)
        self.surge_ready_sub = rospy.Subscriber('/surge_ready', Bool, self.surge_ready_callback)
        self.surge_ready = True

        self.first_surge = True


    def get_center_input(self, error):
        P = self.K_p*error

        derivative = (error - self.pre_error)/self.dt
        D = self.K_d*derivative

        sum = P+D
        if (sum > self.max):
            sum = self.max
        elif(sum < self.min):
            sum = self.min
        self.pre_error = error

        return sum

    def surge_ready_callback(self, msg):
        self.surge_ready = msg.data

    def execute(self,userdata):
        if request_preempt():
            return 'preempted'
        global gate_conf
        global gate_midpoint
        global image_width

        if self.first:
            global current_yaw
            self.first = False
            self.yaw_goal_msg.data = current_yaw*180/math.pi
        if gate_conf > 0.4:
            error = gate_midpoint[0] - image_width/2
            if error > 50:
                self.yaw_goal_msg.data += 0.1
            elif error < -50:
                self.yaw_goal_msg.data -= 0.1
            else:
                if(self.first_surge):
                    step_length = 1
                    global current_yaw
                    global current_surge
                    self.ac_handler.surge_arm_pub_.publish(True)
                    self.surge_msg = step_length  - current_surge
                    self.ac_handler.surge_pub_.publish(self.surge_msg)
                    self.first_surge = False
                    self.surge_ready = False
                    self.rate_long.sleep()
                elif((self.first_surge == False) and (self.surge_ready)):
                    return 'centred'
                return "continue"
            self.yaw_goal_pub_.publish(self.yaw_goal_msg)
            self.rate.sleep()
            return "continue"
        else:
            if((self.first_surge == False) and (self.surge_ready)):
                return 'centred'
            return "continue"

class gate_trick(smach.State):
    def __init__(self, ac_handler):
        smach.State.__init__(self, outcomes=['trick_done', 'continue', 'preempted'])
        self.yaw_goal_pub_ = rospy.Publisher('/yaw_goal', Float64, queue_size=1)
        self.yaw_goal_msg = Float64()
        self.ac_handler = ac_handler
        self.surge_msg = Float64()
        self.sway_msg = Float64()
        self.first = False

        self.first_sent = False
        self.second_sent = False
        self.third_sent = False
        self.fourth_sent = False
        self.fifth_sent = False
        self.sixth_sent = False
        self.seventh_sent = False
        self.eight_sent = False
        self.start_yaw = 0
        self.total = 0
        self.goal = 0
        self.rate = rospy.Rate(10)
        self.rate_long = rospy.Rate(0.1)
    def execute(self,userdata):
        if request_preempt():
            self.first_sent = False
            self.second_sent = False
            self.third_sent = False
            self.fourth_sent = False
            self.fifth_sent = False
            self.sixth_sent = False
            self.seventh_sent = False
            self.eight_sent = False
            return 'preempted'

        global current_surge
        global current_sway
        global curreny_yaw
        current_sway = 0.0
        if not(self.first):
            self.start_yaw = current_yaw*180/math.pi
            self.first = True
            self.goal = self.start_yaw + 720
            self.yaw_goal_msg.data = self.start_yaw

            self.ac_handler.surge_arm_pub_.publish(False)
            self.ac_handler.sway_arm_pub_.publish(False)
            surge_input = Wrench()
            sway_input = Wrench()
            self.rate.sleep()
            self.ac_handler.sway_pub.publish(sway_input)
            self.ac_handler.surge_pub.publish(surge_input)
            self.surge_msg.data = 0 + current_surge
            self.ac_handler.surge_pub_.publish(self.surge_msg)
            self.sway_msg.data = 0 + current_sway
            self.ac_handler.sway_pub_.publish(self.sway_msg)

        self.yaw_goal_msg.data += 3
        if self.yaw_goal_msg.data > 360:
            self.yaw_goal_msg.data -= 360
        self.yaw_goal_pub_.publish(self.yaw_goal_msg)
        self.rate.sleep()
        if self.total+self.start_yaw > self.goal:
            self.ac_handler.surge_arm_pub_.publish(True)
            self.ac_handler.sway_arm_pub_.publish(False)
            self.surge_msg.data = 5 - current_surge
            self.rate.sleep()
            #self.ac_handler.surge_pub_.publish(self.surge_msg)
            #self.rate_long.sleep()
            return 'trick_done'
        self.total += 3
        return 'continue'

class Dive2(smach.State):
    def __init__(self, ac_handler, cam_arm_pub_):
        smach.State.__init__(self, outcomes=['submerged','continue','preempted'])
        print('Diving')
        self.ac_handler = ac_handler
        self.cam_arm_pub_ = cam_arm_pub_
        self.rate = rospy.Rate(10)
        self.counter = 0
        self.yaw_arm_pub = rospy.Publisher('/yaw_arm', Bool, queue_size=1)
        self.first = True

    def execute(self, userdata):
        # turn on diving stuff and do that shit
        if request_preempt():
            return 'preempted'

        if self.first:
            self.ac_handler.surge_arm_pub_.publish(False)
            self.ac_handler.sway_arm_pub_.publish(False)
            surge_input = Wrench()
            sway_input = Wrench()
            self.rate.sleep()
            self.ac_handler.sway_pub.publish(sway_input)
            self.ac_handler.surge_pub.publish(surge_input)
            self.first = False
            goal = DepthHoldGoal(depth = -2.0)
            self.ac_handler.depth_hold_ac.send_goal(goal)
        global depth_hold_ready

        if depth_hold_ready:
            return 'submerged'
        else:
            return 'continue'

class buoy_point(smach.State):
    def __init__(self, ac_handler):
        smach.State.__init__(self, outcomes=['reached','continue','preempted'])
        self.yaw_goal_pub_ = rospy.Publisher('/yaw_goal', Float64, queue_size=1)
        self.yaw_goal_msg = Float64()
        self.ac_handler = ac_handler
        self.surge_msg = Float64()
        self.sway_msg = Float64()

        self.surge_ready_sub = rospy.Subscriber('/surge_ready', Bool, self.surge_ready_callback)

        self.surge_ready = False

        self.first = True

        self.rate = rospy.Rate(10)

    def surge_ready_callback(self, msg):
        self.surge_ready = msg.data

    def execute(self,userdata):
        if request_preempt():
            return 'preempted'
        global current_surge
        global current_sway
        global current_yaw

        if(self.first):
            #Set point
            global current_surge
            global current_sway
            global current_yaw
            self.surge_msg.data = 13.0 - current_surge
            self.sway_msg.data = 0.0 + current_sway
            self.yaw_goal_msg.data = 10 + current_yaw*180/math.pi
            self.ac_handler.surge_arm_pub_.publish(True)
            self.ac_handler.sway_arm_pub_.publish(True)
            self.yaw_goal_pub_.publish(self.yaw_goal_msg)
            self.ac_handler.surge_pub_.publish(self.surge_msg)
            self.ac_handler.sway_pub_.publish(self.sway_msg)
            self.first = False
            self.surge_ready = False

        if((self.first == False) and (self.surge_ready)):
            return 'reached'
        self.rate.sleep()
        return 'continue'

class SearchBuoy(smach.State):
    def __init__(self, ac_handler):
        smach.State.__init__(self, outcomes=['give_up','found', 'continue','preempted'])
        #initialize stuff here
        self.gate_counter = 0
        self.ac_handler = ac_handler
        self.yaw_goal_pub_ = rospy.Publisher('/yaw_goal', Float64, queue_size=1)
        self.yaw_goal_msg = Float64()
        self.rate = rospy.Rate(10)
        self.surge_msg = Float64()
        self.sway_msg = Float64()
        self.first = True
        self.total = 0


    def execute(self, userdata):
        if request_preempt():
            return 'preempted'
        if self.first:
            global current_yaw
            self.yaw_goal_msg.data = current_yaw*180/math.pi
            self.ac_handler.surge_arm_pub_.publish(False)
            self.ac_handler.sway_arm_pub_.publish(False)
            surge_input = Wrench()
            sway_input = Wrench()
            self.rate.sleep()
            self.ac_handler.sway_pub.publish(sway_input)
            self.ac_handler.surge_pub.publish(surge_input)
            self.first = False
        global buoy_conf
        if buoy_conf == 1:
            self.buoy_counter += 1
            if self.buoy_counter > 5:
                self.buoy_counter = 0
                return 'found'
        self.total += 1
        self.yaw_goal_msg.data += 1
        self.yaw_goal_pub_.publish(self.yaw_goal_msg)
        if self.total > 359:
            return 'give_up'
        self.rate.sleep()
        return 'continue'

class CenterBuoy(smach.State):
    def __init__(self, ac_handler):
        smach.State.__init__(self, outcomes=['lost','centred','continue', 'preempted'])
        self.ac_handler = ac_handler
        self.yaw_goal_pub_ = rospy.Publisher('/yaw_goal', Float64, queue_size=1)
        self.yaw_goal_msg = Float64()
        self.rate = rospy.Rate(10)
        self.rate_long = rospy.Rate(0.5)
        self.K_p = 10
        self.K_d = 3
        self.dt = 0.1
        self.max = 40
        self.min = -40
        self.first = True
        self.pre_error = 0

        self.surge_msg = Float64()
        self.sway_msg = Float64()
        self.ac_handler.sway_pub_ = rospy.Publisher('/sway_goal', Float64, queue_size=1)
        self.surge_ready_sub = rospy.Subscriber('/surge_ready', Bool, self.surge_ready_callback)
        self.surge_ready = True

        self.first_surge = True


    def get_center_input(self, error):
        P = self.K_p*error

        derivative = (error - self.pre_error)/self.dt
        D = self.K_d*derivative

        sum = P+D
        if (sum > self.max):
            sum = self.max
        elif(sum < self.min):
            sum = self.min
        self.pre_error = error

        return sum

    def surge_ready_callback(self, msg):
        self.surge_ready = msg.data

    def execute(self,userdata):
        if request_preempt():
            return 'preempted'
        global buoy_conf
        global buoy_midpoint
        global buoy_image_width

        if self.first:
            global current_yaw
            self.first = False
            self.yaw_goal_msg.data = current_yaw*180/math.pi
        if gate_conf > 0.4:
            error = buoy_midpoint[0] - buoy_image_width/2
            if error > 50:
                self.yaw_goal_msg.data += 0.1
            elif error < -50:
                self.yaw_goal_msg.data -= 0.1
            else:
                if(self.first_surge):
                    step_length = 1
                    global current_yaw
                    global current_surge
                    self.ac_handler.surge_arm_pub_.publish(True)
                    self.surge_msg = step_length  - current_surge
                    self.ac_handler.surge_pub_.publish(self.surge_msg)
                    self.first_surge = False
                    self.surge_ready = False
                    self.rate_long.sleep()
                elif((self.first_surge == False) and (self.surge_ready)):
                    return 'centred'
                return "continue"
            self.yaw_goal_pub_.publish(self.yaw_goal_msg)
            self.rate.sleep()
            return "continue"
        else:
            if((self.first_surge == False) and (self.surge_ready)):
                return 'centred'
            return "continue"

class forward(smach.State):
    def __init__(self, ac_handler):
        smach.State.__init__(self, outcomes=['reached','continue','preempted'])
        self.yaw_goal_pub_ = rospy.Publisher('/yaw_goal', Float64, queue_size=1)
        self.yaw_goal_msg = Float64()
        self.ac_handler = ac_handler
        self.surge_msg = Float64()
        self.sway_msg = Float64()

        self.surge_ready_sub = rospy.Subscriber('/surge_ready', Bool, self.surge_ready_callback)

        self.surge_ready = False

        self.first = True

        self.rate = rospy.Rate(10)

    def surge_ready_callback(self, msg):
        self.surge_ready = msg.data

    def execute(self,userdata):
        if request_preempt():
            return 'preempted'
        global current_surge
        global current_sway
        global current_yaw

        if(self.first):
            #Set point
            global current_surge
            global current_sway
            global current_yaw
            self.surge_msg.data = 0.0 - current_surge
            self.sway_msg.data = 0.0 + current_sway
            self.yaw_goal_msg.data = 30 + current_yaw*180/math.pi
            self.ac_handler.surge_arm_pub_.publish(True)
            self.ac_handler.sway_arm_pub_.publish(True)
            self.yaw_goal_pub_.publish(self.yaw_goal_msg)
            self.ac_handler.surge_pub_.publish(self.surge_msg)
            self.ac_handler.sway_pub_.publish(self.sway_msg)
            self.first = False
            self.surge_ready = False

        if((self.first == False) and (self.surge_ready)):
            return 'reached'
        self.rate.sleep()
        return 'continue'


def main():
    rospy.init_node('Qualification_run')

    global mission_in_progress
    mission_in_progress = False
    global depth_hold_ready
    depth_hold_ready = False
    global gate_conf
    gate_conf = 0
    global buoy_conf
    buoy_conf = 0
    rospy.Subscriber("mission_trigger", Bool, mission_trigger_callback)

    rospy.Subscriber("depth_hold_action_server/feedback", DepthHoldActionFeedback, depth_hold_feedback_callback)

    rospy.Subscriber('/gate_midpoint', CameraObjectInfo, gate_callback, queue_size=1)

    rospy.Subscriber('/pole_midpoint', CameraObjectInfo, buoy_callback, queue_size=1)

    rospy.Subscriber('/odometry/filtered', Odometry, odom_callback)

    arm_pub_ = rospy.Publisher('/mcu_arm', String, queue_size=1)

    cam_arm_pub_ = rospy.Publisher('/start_front_camera', String, queue_size=1)
    cam_disarm_pub_ = rospy.Publisher('/stop_front_camera', String, queue_size=1)
    mode_pub_ = rospy.Publisher('/manta/mode', PropulsionCommand, queue_size=1)

    ac_handler = AC_handler()

    sm = smach.StateMachine(outcomes = ['Done'])

    sis = smach_ros.IntrospectionServer('Qualification_run_server', sm, '/SM_ROOT')
    sis.start()

    with sm:
        smach.StateMachine.add('Idle', Idle(arm_pub_, mode_pub_),
                                transitions={'doing':'Dive', 'waiting':'Idle'})
        smach.StateMachine.add('Cancel', Cancel(ac_handler, arm_pub_, mode_pub_, cam_disarm_pub_),
                                transitions={'canceld':'Idle'})
        smach.StateMachine.add('Dive', Dive(ac_handler, cam_arm_pub_),
                                transitions={'submerged':'start_point', 'continue':'Dive','preempted':'Cancel'})
        smach.StateMachine.add('start_point', start_point(ac_handler),
                        transitions={'reached':'SearchGate', 'continue':'start_point', 'preempted':'Cancel'})
        smach.StateMachine.add('SearchGate', SearchGate(ac_handler),
                                transitions={'found':'CenterGate', 'continue':'SearchGate', 'preempted':'Cancel'})
        smach.StateMachine.add('CenterGate', CenterGate(ac_handler),
                                transitions={'lost':'SearchGate','centred':'Trick', 'continue':'CenterGate', 'preempted':'Cancel'})
        smach.StateMachine.add('Trick', gate_trick(ac_handler),
			transitions={'trick_done':'Dive2', 'continue':'Trick', 'preempted':'Cancel'})
        smach.StateMachine.add('Dive2', Dive2(ac_handler, cam_arm_pub_),
                                transitions={'submerged':'buoy_point', 'continue':'Dive2','preempted':'Cancel'})
        smach.StateMachine.add('buoy_point', buoy_point(ac_handler),
                        transitions={'reached':'SearchBuoy', 'continue':'buoy_point', 'preempted':'Cancel'})
        smach.StateMachine.add('SearchBuoy', SearchBuoy(ac_handler),
                                transitions={'give_up':'forward','found':'CenterBuoy', 'continue':'SearchBuoy', 'preempted':'Cancel'})
        smach.StateMachine.add('CenterBuoy', CenterBuoy(ac_handler),
                                transitions={'lost':'SearchBuoy','centred':'Cancel', 'continue':'CenterBuoy', 'preempted':'Cancel'})
        smach.StateMachine.add('forward', forward(ac_handler),
                        transitions={'reached':'Cancel', 'continue':'forward', 'preempted':'Cancel'})

    outcome = sm.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
