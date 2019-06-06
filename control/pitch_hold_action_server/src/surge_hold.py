#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64, Bool
import math


class Surge_pd:
    def __init__(self):
        self.K_p = 10
        self.K_d = 3
        self.dt = 0.1
        self.max = 20
        self.min = -20

        self.pre_error = 0
        self.surge_goal = 0.0
        self.armed = False
        self.pi = math.pi

        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.get_rotation)#('/odometry/filtered', Odometry, get_rotation)
        self.surge_pub = rospy.Publisher('/surge_input', Wrench, queue_size=1)
        self.surge_goal_sub = rospy.Subscriber('/surge_goal', Float64, self.set_surge_goal)
        self.surge_arm_sub = rospy.Subscriber('/surge_arm', Bool, self.surge_arm)

    def surge_arm(self, msg):
        if msg.data:
            self.armed = True
        else:
            self.armed = False

    def set_surge_goal(self, msg):
        #print(msg)
        self.surge_goal = msg.data

    def get_surge_input(self, error):
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
        #global roll, pitch, surge
        #orientation_q = msg.pose.pose.orientation
        #orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        #(roll, pitch, surge) = euler_from_quaternion(orientation_list)
        #print('roll: ', roll)
        #print('pitch: ', pitch)
        #print('surge: ', surge)
        surge = msg.pose.pose.position.x
        if self.armed:
            surge_input = Wrench()
            surge_input.force.x = self.get_surge_input(surge-self.surge_goal)
            self.surge_pub.publish(surge_input)

    def main(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            r.sleep()



if __name__ == '__main__':
    rospy.init_node('surge_controller')
    surge_pd = Surge_pd()
    surge_pd.main()
