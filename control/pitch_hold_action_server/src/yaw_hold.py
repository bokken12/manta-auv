#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64, Bool
import math


class Yaw_pd:
    def __init__(self):
        self.K_p = 10
        self.K_d = 3
        self.dt = 0.1
        self.max = 40
        self.min = -40

        self.pre_error = 0
        self.yaw_goal = 0.0

        self.pi = math.pi
        self.armed = False

        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.get_rotation)#('/odometry/filtered', Odometry, get_rotation)
        self.yaw_pub = rospy.Publisher('/yaw_input', Wrench, queue_size=1)
        self.yaw_goal_sub = rospy.Subscriber('/yaw_goal', Float64, self.set_yaw_goal)

        self.yaw_arm = rospy.Subscriber('/yaw_arm', Bool, self.arm_callback)

    def arm_callback(self,msg):
        if msg.data:
            self.armed = True
        else:
            self.armed = False

    def set_yaw_goal(self, msg):
        #print(msg)
        '''
        if(msg.data > 179 ):
            msg.data -= 360
        elif(msg.data < -179):
            msg.data += 360
        '''
        self.yaw_goal = self.pi*(msg.data)/180

    def get_yaw_input(self, error):
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
        global roll, pitch, yaw
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        #print('roll: ', roll)
        #print('pitch: ', pitch)
        #print('yaw: ', yaw)

        yaw_input = Wrench()
        error = yaw-self.yaw_goal
        if(error > 2*self.pi):
            error -= 2*self.pi
        elif(error < -2*self.pi):
            error += 2*self.pi
        yaw_input.torque.z = self.get_yaw_input(error)
        if self.armed:
            self.yaw_pub.publish(yaw_input)
        else:
            yaw_input.torque.z = 0
            self.yaw_pub.publish(yaw_input)

    def main(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            r.sleep()



if __name__ == '__main__':
    rospy.init_node('yaw_controller')
    yaw_pd = Yaw_pd()
    yaw_pd.main()
