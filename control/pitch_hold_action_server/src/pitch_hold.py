#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64
import math


class Pitch_pd:
    def __init__(self):
        self.K_p = 10
        self.K_d = 3
        self.dt = 0.1
        self.max = 40
        self.min = -40

        self.pre_error = 0
        self.pitch_goal = 0.0

        self.pi = math.pi

        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.get_rotation)#('/odometry/filtered', Odometry, get_rotation)
        self.pitch_pub = rospy.Publisher('/pitch_input', Wrench, queue_size=1)
        self.pitch_goal_sub = rospy.Subscriber('/pitch_goal', Float64, self.set_pitch_goal)

    def set_pitch_goal(self, msg):
        #print(msg)
        self.pitch_goal = self.pi*(msg.data)/180

    def get_pitch_input(self, error):
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
        global roll, pitch, pitch
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, pitch) = euler_from_quaternion (orientation_list)
        print('roll: ', roll)
        print('pitch: ', pitch)
        print('pitch: ', pitch)

        pitch_input = Wrench()
        pitch_input.torque.y = self.get_pitch_input(pitch-self.pitch_goal)
        self.pitch_pub.publish(pitch_input)

    def main(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            r.sleep()



if __name__ == '__main__':
    rospy.init_node('pitch_controller')
    pitch_pd = Pitch_pd()
    pitch_pd.main()
