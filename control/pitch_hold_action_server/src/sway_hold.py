#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64, Bool
import math


class Sway_pd:
    def __init__(self):
        self.K_p = 15
        self.K_d = 8
        self.dt = 0.1
        self.max = 20
        self.min = -20

        self.pre_error = 0
        self.sway_goal = 0.0
        self.armed = False
        self.pi = math.pi

        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.get_rotation)#('/odometry/filtered', Odometry, get_rotation)
        self.sway_pub = rospy.Publisher('/sway_input', Wrench, queue_size=1)
        self.sway_goal_sub = rospy.Subscriber('/sway_goal', Float64, self.set_sway_goal)
        self.sway_arm_sub = rospy.Subscriber('/sway_arm', Bool, self.sway_arm)
        self.sway_ready_pub = rospy.Publisher('/sway_ready', Bool, queue_size=1)
        self.ready = False
        self.start_sway = 0
        self.first_after_goal = False

    def sway_arm(self, msg):
        if msg.data:
            self.armed = True
        else:
            self.armed = False

    def set_sway_goal(self, msg):
        #print(msg)
        self.sway_goal = msg.data
        self.ready = False
        self.first_after_goal = True

    def get_sway_input(self, error):
        P = self.K_p*error

        derivative = (error - self.pre_error)/self.dt
        D = self.K_d*derivative

        sum = P + D
        if (sum > self.max):
            sum = self.max
        elif(sum < self.min):
            sum = self.min
        self.pre_error = error

        return sum



    def get_rotation (self, msg):
        #global roll, pitch, sway
        #orientation_q = msg.pose.pose.orientation
        #orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        #(roll, pitch, sway) = euler_from_quaternion(orientation_list)
        #print('roll: ', roll)
        #print('pitch: ', pitch)
        #print('sway: ', sway)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        surge = msg.pose.pose.position.x*math.cos(yaw) + msg.pose.pose.position.y*math.sin(yaw)
        sway = -msg.pose.pose.position.x*math.sin(yaw) + msg.pose.pose.position.y*math.cos(yaw)

        if self.first_after_goal:
            self.start_sway = sway
            self.first_after_goal = False
        if self.armed:
            sway_input = Wrench()
            #collected = math.sqrt(sway*sway + sway*sway)
            error = self.sway_goal-sway

            sway_input.force.y = self.get_sway_input(error)
            self.sway_pub.publish(sway_input)
            if abs(error) < 0.3:
                self.ready = True
            else:
                self.ready = False

    def main(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.sway_ready_pub.publish(self.ready)
            r.sleep()



if __name__ == '__main__':
    rospy.init_node('sway_controller')
    sway_pd = Sway_pd()
    sway_pd.main()

