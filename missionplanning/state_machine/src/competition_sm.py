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

#fiks at cameranode er tilgjengelig for state machine
#subscribe paa riktig topics saa man kan se om gate er funnet

class subscribe_to_camera():
    #gir ut bredde, lengde og midpoints. confidence: 0=ser ikke, 1=ser, 0.5=forbi
    def __init_(self):
        rospy.Subscriber('/gate_midpoint', CameraObjectInfo, self.callback, queue_size=1)
        self.conf = 0
        self.midpoints = (0,0)
        self.width = 0

    def get_conf(self):
        conf = self.conf
        return conf

    def get_midpoint(self):
        return self.midpoints

    def get_width(self):
        return self.width

    def callback(self, msg):
        self.conf = msg.confidence
        self.midpoints = (msg.sub_gate.pos_x, msg.sub_gate.pos_y)
        self.width = msg.sub_gate.frame_width


def gate_callback(msg):
    global gate_conf
    gate_conf = msg.confidence
    global image_width
    image_width = msg.frame_width
    global gate_midpoint
    gate_midpoint = (msg.pos_x, msg.pos_y)


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
        self.waypoints.append(Pose(Point(0.0, 0.0, -0.5), quaternions[0]))
        self.waypoints.append(Pose(Point(3.0, 0.0, -0.5), quaternions[1]))
        self.waypoints.append(Pose(Point(0.0, 0.0, -0.5), quaternions[2]))
        self.waypoints.append(Pose(Point(0.0, 0.0, -0.5), quaternions[3]))
        self.waypoints.append(Pose(Point(0.0, 0.0, -0.5), quaternions[3]))
        self.waypoints.append(Pose(Point(0.0, 0.0, -0.5), quaternions[3]))
        self.waypoints.append(Pose(Point(0.0, 0.0, -0.5), quaternions[3]))
        self.waypoints.append(Pose(Point(0.0, 0.0, -2), quaternions[3]))

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

def odom_callback(msg):
    global current_surge
    global current_sway
    global current_heave
    global current_roll
    global current_pitch
    global current_yaw
    #current_surge = msg.pose.pose.position.x
    #curreny_sway = msg.pose.pose.position.y
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
        self.surge_arm_pub_ = rospy.Publisher('/surge_arm', Bool, queue_size=1)
        self.sway_arm_pub_ = rospy.Publisher('/sway_arm', Bool, queue_size=1)


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
        self.surge_arm_pub_.publish(False)
        self.sway_arm_pub_.publish(False)
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
            goal = DepthHoldGoal(depth = -0.8)
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
    def __init__(self, ac_handler, cam_class):
        smach.State.__init__(self, outcomes=['found', 'continue','preempted'])
        #initialize stuff here
        self.gate_counter = 0
        self.ac_handler = ac_handler
        self.yaw_goal_pub_ = rospy.Publisher('/yaw_goal', Float64, queue_size=1)
        self.yaw_goal_msg = Float64()
        self.rate = rospy.Rate(10)
        self.cam_class = cam_class
        '''
        self.camera_sub = camera_sub
        self.gate_found = self.camera_sub.get_conf()
        '''
        self.surge_msg = Float64()
        self.surge_arm_pub_ = rospy.Publisher('/surge_arm', Bool, queue_size=1)
        self.surge_pub_ = rospy.Publisher('/surge_goal', Float64, queue_size=1)
        self.sway_arm_pub_ = rospy.Publisher('/sway_arm', Bool, queue_size=1)
        self.sway_msg = Float64()
        self.sway_pub_ = rospy.Publisher('/sway_goal', Float64, queue_size=1)
        self.first = True
        self.surge_pub = rospy.Publisher('/surge_input', Wrench, queue_size=1)
        self.sway_pub = rospy.Publisher('/sway_input', Wrench, queue_size=1)


    def execute(self, userdata):
        if request_preempt():
            return 'preempted'
        if self.first:
            global current_yaw
            self.yaw_goal_msg.data = current_yaw*180/math.pi
            self.surge_arm_pub_.publish(False)
            self.sway_arm_pub_.publish(False)
            surge_input = Wrench()
            sway_input = Wrench()
            self.rate.sleep()
            self.sway_pub.publish(sway_input)
            self.surge_pub.publish(surge_input)

            #self.surge_msg.data = 0
            #self.surge_pub_.publish(self.surge_msg)
            #self.sway_msg.data = 0
            #self.sway_pub_.publish(self.sway_msg)
            self.first = False
        global gate_conf
        #gate_conf = self.cam_class.get_conf() 
        if gate_conf == 1:
            self.gate_counter += 1
            if self.gate_counter > 5:
                self.gate_counter = 0
                return 'found'
        self.yaw_goal_msg.data += 1
       # if self.yaw_goal_msg > 359:
       #     self.yaw_goal_msg.data = 0
        self.yaw_goal_pub_.publish(self.yaw_goal_msg)
        self.rate.sleep()
        return 'continue'
        '''
        SKAL DETTE BRUKES?
        if self.ac_handler.dp_controller_ac.get_state() != 1:
            goal = MoveBaseGoal()
            goal.target_pose.pose.position.x = 5.0
            goal.target_pose.pose.position.y = 0.0
            goal.target_pose.pose.position.z = 0.0
            print(goal.target_pose.pose.position)
            self.ac_handler.dp_controller_ac.send_goal(goal)
        
        if self.first:
            self.first = False
        '''
class CenterGate(smach.State):
    def __init__(self, ac_handler, cam_class):
        smach.State.__init__(self, outcomes=['lost','centred','continue', 'preempted'])
        self.ac_handler = ac_handler
        self.yaw_goal_pub_ = rospy.Publisher('/yaw_goal', Float64, queue_size=1)
        self.yaw_goal_msg = Float64()
        self.rate = rospy.Rate(10)
        self.rate_long = rospy.Rate(0.5)
        self.cam_class = cam_class
        self.K_p = 10
        self.K_d = 3
        self.dt = 0.1
        self.max = 40
        self.min = -40
        self.first = True
        self.pre_error = 0

        self.surge_msg = Float64()
        self.sway_msg = Float64()
        self.surge_arm_pub_ = rospy.Publisher('/surge_arm', Bool, queue_size=1)
        self.surge_pub_ = rospy.Publisher('/surge_goal', Float64, queue_size=1)
        self.sway_arm_pub_ = rospy.Publisher('/sway_arm', Bool, queue_size=1)
        self.sway_pub_ = rospy.Publisher('/sway_goal', Float64, queue_size=1)
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
        #gate_conf = self.cam_class.get_conf()
        global gate_midpoint
        #gate_midpoint = self.cam_class.get_midpoint()
        global image_width
        #width = self.cam_class.get_width()
        if self.first:
            global current_yaw
            self.first = False
            self.yaw_goal_msg.data = current_yaw*180/math.pi
        if gate_conf > 0.4:
            #global current_yaw
            error = gate_midpoint[0] - image_width/2 
            #  Gaa mot riktig side
            #if error > -50 and error < 50:# Maa velge fornuftige verdier
	#	self.yaw_goal_msg.data = (current_yaw*180)/math.pi
             #   self.yaw_goal_pub_.publish(self.yaw_goal_msg)
             #   return 'continue'
            #yaw_update = self.get_center_input(error)
            #self.yaw_goal_msg.data = (current_yaw*180)/math.pi + yaw_update
            #self.yaw_goal_pub_.publish(self.yaw_goal_msg)
            #self.rate.sleep()
            if error > 50:
                self.yaw_goal_msg.data += 0.1
            elif error < -50:
                self.yaw_goal_msg.data -= 0.1
            else:
                if(self.first_surge):
                    step_length = 4
                    global current_yaw
                    global current_surge
                    self.surge_arm_pub_.publish(True)
                    self.surge_msg = step_length  - current_surge
                    self.surge_pub_.publish(self.surge_msg)
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

class MovetoGate(smach.State):
    def __init__(self, ac_handler, cam_class):
        smach.State.__init__(self, outcomes=['gate_reached','continue', 'preempted'])
	self.surge_msg = Float64()
        self.sway_msg = Float64()
        self.surge_arm_pub_ = rospy.Publisher('/surge_arm', Bool, queue_size=1)
        self.surge_pub_ = rospy.Publisher('/surge_goal', Float64, queue_size=1)
        self.sway_arm_pub_ = rospy.Publisher('/sway_arm', Bool, queue_size=1)

        self.sway_pub_ = rospy.Publisher('/sway_goal', Float64, queue_size=1)
        self.yaw_goal_pub_ = rospy.Publisher('/yaw_goal', Float64, queue_size=1)
        self.yaw_goal_msg = Float64()

	self.counter = 0
	self.surge_ready_sub = rospy.Subscriber('/surge_ready', Bool, self.surge_ready_callback)
        self.surge_ready = True
	#self.gate_found = self.camera.conf
        self.rate = rospy.Rate(10)
        self.cam_class = cam_class
        self.first = True
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.surge_pub_input = rospy.Publisher('/surge_input', Wrench, queue_size=1)


    def surge_ready_callback(self, msg):
        self.surge_ready = msg.data
	
    def execute(self,userdata):
        if request_preempt():
            return 'preempted'
        if self.first:
            global current_yaw
            yaw_goal_msg = Float64()
            yaw_goal_msg = current_yaw
            self.yaw_goal_pub_.publish(yaw_goal_msg)
            self.surge_arm_pub_.publish(True)
            self.sway_arm_pub_.publish(True)
            self.first = False

        if(self.surge_ready):
            '''
            global current_surge
	    try:
                trans = self.tfBuffer.lookup_transform('odom', 'base_link', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                return 'continue'
            v = Vector3Stamped()
            v.vector.x = 3
            v.vector.y = 0
            v.vector.z = 0

            vt = tf2_geometry_msgs.do_transform_vector3(v, trans)
            self.surge_msg.data = vt.vector.x
            self.sway_msg.data = vt.vector.y
            self.sway_pup_.publish(self.sway_msg)
            '''
            step_length = 2 
            global current_yaw
            self.surge_msg = step_length*math.cos(current_yaw)
            self.surge_pub_.publish(self.surge_msg)
            #self.sway_msg = -step_length*math.sin(current_yaw)
            #self.sway_pub_.publish(self.sway_msg)

            if gate_conf != 1:
                return 'gate_reached'
	'''
            self.counter += 1
        if self.counter > 5:
            surge_input = Wrench()
            surge_input.force.x = 0
            self.surge_pub_input.publish(surge_input)
            return 'gate_reached'	
        '''
        self.rate.sleep()
        return 'continue'
	
class gate_trick(smach.State):
    def __init__(self, ac_handler):
        smach.State.__init__(self, outcomes=['trick_done', 'continue', 'preempted'])
        self.yaw_goal_pub_ = rospy.Publisher('/yaw_goal', Float64, queue_size=1)
        self.yaw_goal_msg = Float64()

        self.surge_msg = Float64()
        self.sway_msg = Float64()
        self.surge_arm_pub_ = rospy.Publisher('/surge_arm', Bool, queue_size=1)
        self.surge_pub_ = rospy.Publisher('/surge_goal', Float64, queue_size=1)
        self.sway_arm_pub_ = rospy.Publisher('/sway_arm', Bool, queue_size=1)
        self.surge_pub = rospy.Publisher('/surge_input', Wrench, queue_size=1)
        self.sway_pub = rospy.Publisher('/sway_input', Wrench, queue_size=1)
        self.sway_pub_ = rospy.Publisher('/sway_goal', Float64, queue_size=1)
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
            self.goal = self.start_yaw + 720-90
            self.yaw_goal_msg.data = self.start_yaw

            self.surge_arm_pub_.publish(False)
            self.sway_arm_pub_.publish(False)
            surge_input = Wrench()
            sway_input = Wrench()
            self.rate.sleep()
            self.sway_pub.publish(sway_input)
            self.surge_pub.publish(surge_input)
            self.surge_msg.data = 0 + current_surge
            self.surge_pub_.publish(self.surge_msg)
            self.sway_msg.data = 0 + current_sway
            self.sway_pub_.publish(self.sway_msg)
            

        self.yaw_goal_msg.data += 3
       # if self.yaw_goal_msg > 359:
       #     self.yaw_goal_msg.data = 0
        if self.yaw_goal_msg.data > 360:
            self.yaw_goal_msg.data -= 360
        self.yaw_goal_pub_.publish(self.yaw_goal_msg)
        self.rate.sleep()
        if self.total > self.goal:
            self.surge_arm_pub_.publish(True)
            self.sway_arm_pub_.publish(False)
            self.surge_msg.data = 5 - current_surge
            self.rate.sleep()
            self.surge_pub_.publish(self.surge_msg)
            self.rate_long.sleep()
            return 'trick_done'
        self.total += 3
        return 'continue'



        

        '''
        if not (self.first_sent):
            self.start_yaw = current_yaw*180/math.pi
            #self.surge_msg.data = current_surge
            #self.surge_pub_.publish(self.surge_msg)
            #self.sway_msg.data = current_sway
            #self.sway_pub_.publish(self.sway_msg)
            self.yaw_goal_msg.data = 270  + self.start_yaw
            self.yaw_goal_pub_.publish(self.yaw_goal_msg)
            #self.which_point += 1
            self.first_sent = True
            self.rate.sleep()
            #self.surge_ready = False
            #self.sway_ready = False
            return 'continue'
        if not(self.second_sent) and current_yaw*180/math.pi > self.yaw_goal_msg.data - 10.0:
            #self.surge_msg.data = current_surge
            #self.surge_pub_.publish(self.surge_msg)
            #self.sway_msg.data = current_sway
            #self.sway_pub_.publish(self.sway_msg)
            self.yaw_goal_msg.data =  90 + self.start_yaw
            self.yaw_goal_pub_.publish(self.yaw_goal_msg)
            #self.which_point += 1
            self.second_sent = True
            self.rate.sleep()
            #self.surge_ready = False
            #self.sway_ready = False
            return 'continue'
        if self.second_sent and current_yaw*180/math.pi > self.yaw_goal_msg.data - 10.0: 
            return 'trick_done'
        return 'continue'
'''
'''
        if not(self.third_sent):
            #self.surge_msg.data = current_surge
            #self.surge_pub_.publish(self.surge_msg)
            #self.sway_msg.data = current_sway
            self.sway_pub_.publish(self.sway_msg)
            self.yaw_goal_msg.data = -90 + self.start_yaw
            self.yaw_goal_pub_.publish(self.yaw_goal_msg.data)
            #self.which_point += 1
            self.third_sent = True
            self.rate.sleep()
            #self.surge_ready = False
            #self.sway_ready = False
            return 'continue'

        if not(self.fourth_sent):
            #self.surge_msg.data = current_surge
            #self.surge_pub_.publish(self.surge_msg)
            #self.sway_msg.data = current_sway
            #self.sway_pub_.publish(self.sway_msg)
            self.yaw_goal_msg.data = self.start_yaw
            self.yaw_goal_pub_.publish(self.yaw_goal_msg.data)
            #self.which_point += 1
            self.fourth_sent = True
            self.rate.sleep
            #self.surge_ready = False
            #self.sway_ready = False
            return 'trick_done'
        return 'continue'
       
        if not(self.fifth_sent):
            self.surge_msg.data = current_surge
            self.surge_pub_.publish(self.surge_msg)
            self.sway_msg.data = current_sway
            self.sway_pub_.publish(self.sway_msg)
            self.yaw_goal_msg.data = self.start_yaw
            self.yaw_goal_pub_.publish(self.yaw_goal_msg.data)
            #self.which_point += 1
            self.fifth_sent = True
            self.rate.sleep()
            self.surge_ready = False
            self.sway_ready = False
            return 'continue'

        if not(self.sixth_sent):
            self.surge_msg.data = current_surge
            self.surge_pub_.publish(self.surge_msg)
            self.sway_msg.data = current_sway
            self.sway_pub_.publish(self.sway_msg)
            self.yaw_goal_msg.data = self.start_yaw
            self.yaw_goal_pub_.publish(self.yaw_goal_msg.data)
            #self.which_point += 1
            self.sixth_sent = True
            self.rate.sleep()
            self.surge_ready = False
            self.sway_ready = False
            return 'continue'

        if not(self.seventh_sent):
            self.surge_msg.data = current_surge
            self.surge_pub_.publish(self.surge_msg)
            self.sway_msg.data = current_sway
            self.sway_pub_.publish(self.sway_msg)
            self.yaw_goal_msg.data = self.start_yaw
            self.yaw_goal_pub_.publish(self.yaw_goal_msg.data)
            #self.which_point += 1
            self.seventh_sent = True
            self.rate.sleep()
            self.surge_ready = False
            self.sway_ready = False
            return 'continue'

        if not(self.eigth_sent):
            self.surge_msg.data = current_surge
            self.surge_pub_.publish(self.surge_msg)
            self.sway_msg.data = current_sway
            self.sway_pub_.publish(self.sway_msg)
            self.yaw_goal_msg.data = self.start_yaw
            self.yaw_goal_pub_.publish(self.yaw_goal_msg.data)
            #self.which_point += 1
            self.eigth_sent = True
            self.rate.sleep()
            self.surge_ready = False
            self.sway_ready = False
            self.first_sent = False
            self.second_sent = False
            self.third_sent = False
            self.fourth_sent = False
            self.fifth_sent = False
            self.sixth_sent = False
            self.seventh_sent = False
            self.eight_sent = False
            return 'trick_done'
        '''
'''
        self.total += 0.5
        self.yaw_goal_msg.data += 5
        self.yaw_goal_pub_.publish(self.yaw_goal_msg)
        self.rate.sleep()
        if self.total > 359.5:
            return 'trick_done'
        return 'continue'
'''



class start_point(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached','continue','preempted'])
        self.yaw_goal_pub_ = rospy.Publisher('/yaw_goal', Float64, queue_size=1)
        self.yaw_goal_msg = Float64()

        self.surge_msg = Float64()
        self.sway_msg = Float64()
        self.surge_arm_pub_ = rospy.Publisher('/surge_arm', Bool, queue_size=1)
        self.surge_pub_ = rospy.Publisher('/surge_goal', Float64, queue_size=1)
        self.sway_arm_pub_ = rospy.Publisher('/sway_arm', Bool, queue_size=1)

        self.sway_pub_ = rospy.Publisher('/sway_goal', Float64, queue_size=1)
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
            self.surge_msg.data = 10.0 + current_surge
            self.sway_msg.data = 0.0 + current_sway
            self.yaw_goal_msg.data = 65
            self.surge_arm_pub_.publish(True)
            self.sway_arm_pub_.publish(True)
            self.yaw_goal_pub_.publish(self.yaw_goal_msg)
            self.surge_pub_.publish(self.surge_msg)
            self.sway_pub_.publish(self.sway_msg) 
            self.first = False
            self.surge_ready = False

        if((self.first == False) and (self.surge_ready)):
            return 'reached'
        self.rate.sleep()
        return 'continue'

'''

class through_gate(smach.State):
    def __init__(self, ac_handler):
        smach.State.__init__(self, outcomes=['gate_passed','continue', 'preempted'])
        self.surge_msg = Float64()
        self.surge_arm_pub_ = rospy.Publisher('/surge_arm', Bool, queue_size=1)
        self.surge_pub_ = rospy.Publisher('/surge_goal', Float64, queue_size=1)
        self.surge_ready_sub = rospy.Subscriber('/surge_ready', Bool, self.surge_ready)
        self.surge_ready = True
        self.first = True
	self.start_surge = 0
        self.surge_pub_input = rospy.Publisher('/surge_input', Wrench, queue_size=1)
      
    def execute(self,userdata):
	if request_preempt():
            return 'preempted'
        global current_surge
	if first: 
            self.start_surge = current_surge
            self.first = False
        self.surge_goal_msg.data = current_surge + 0.5
        self.surge_goal_pub_.publish(self.surge_goal_msg)
        surge_input = Wrench()
        surge_input.force.x = 10
        self.surge_pub_input.publish(surge_input)

        if(False): #(current_surge >= 3 + self.start_surge):
            return 'gate_passed'
        return 'continue' 
        
	
        


class searchPathmarker(smach.State):
    def __init__(self, ac_handler, waypoint_client, arm_pub_, mode_pub_, cam_disarm_pub_):
        smach.State.__init__(self, outcomes=['found','continiue', 'preempted'])

    def execute:
        pass

class centerPathmarker(smach.State):
    def __init__(self, ac_handler, waypoint_client, arm_pub_, mode_pub_, cam_disarm_pub_):
        smach.State.__init__(self, outcomes=['centred','continiue', 'preempted'])
    def execute:
        pass

class pathmarkerHeading(smach.State):
    def __init__(self, ac_handler, waypoint_client, arm_pub_, mode_pub_, cam_disarm_pub_):
        smach.State.__init__(self, outcomes=['good_heading','continiue', 'preempted'])
    def execute:
        pass

class moveSearchDragur(smach.State):
    def __init__(self, ac_handler, waypoint_client, arm_pub_, mode_pub_, cam_disarm_pub_):
        smach.State.__init__(self, outcomes=['found','continiue', 'preempted'])
    def execute:
        pass

class centerDragur(smach.State):
    def __init__(self, ac_handler, waypoint_client, arm_pub_, mode_pub_, cam_disarm_pub_):
        smach.State.__init__(self, outcomes=['centred','continiue', 'preempted'])
    def execute:
        pass

class hitDragur(smach.State):
    def __init__(self, ac_handler, waypoint_client, arm_pub_, mode_pub_, cam_disarm_pub_):
        smach.State.__init__(self, outcomes=['hit','continiue', 'preempted'])
    def execute:
        pass

class reverse(smach.State):
    def __init__(self, ac_handler, waypoint_client, arm_pub_, mode_pub_, cam_disarm_pub_):
        smach.State.__init__(self, outcomes=['reversed','continiue', 'preempted'])

'''

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

    rospy.Subscriber('/gate_midpoint', CameraObjectInfo, gate_callback, queue_size=1)

    rospy.Subscriber('/odometry/filtered', Odometry, odom_callback)

    arm_pub_ = rospy.Publisher('/mcu_arm', String, queue_size=1)

    cam_arm_pub_ = rospy.Publisher('/start_front_camera', String, queue_size=1)
    cam_disarm_pub_ = rospy.Publisher('/stop_front_camera', String, queue_size=1)
    mode_pub_ = rospy.Publisher('/manta/mode', PropulsionCommand, queue_size=1)

    cam_class = subscribe_to_camera()

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
        smach.StateMachine.add('Cancel', Cancel(ac_handler, waypoint_client, arm_pub_, mode_pub_, cam_disarm_pub_),
                                transitions={'canceld':'Idle'})
        smach.StateMachine.add('Dive', Dive(ac_handler, cam_arm_pub_),
                                transitions={'submerged':'start_point', 'continue':'Dive','preempted':'Cancel'})
        smach.StateMachine.add('SearchGate', SearchGate(ac_handler, cam_class),
                                transitions={'found':'CenterGate', 'continue':'SearchGate', 'preempted':'Cancel'})
        smach.StateMachine.add('CenterGate', CenterGate(ac_handler, cam_class),
                                transitions={'lost':'SearchGate','centred':'Trick', 'continue':'CenterGate', 'preempted':'Cancel'})
        smach.StateMachine.add('MoveGate', MovetoGate(ac_handler, cam_class),
                                transitions={'gate_reached':'Trick','continue':'MoveGate','preempted':'Cancel'})
	smach.StateMachine.add('Trick', gate_trick(ac_handler),
			transitions={'trick_done':'Cancel', 'continue':'Trick', 'preempted':'Cancel'})

        smach.StateMachine.add('start_point', start_point(),
                        transitions={'reached':'SearchGate', 'continue':'start_point', 'preempted':'Cancel'})

    outcome = sm.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
