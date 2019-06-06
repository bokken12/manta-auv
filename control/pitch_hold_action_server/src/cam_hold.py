#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64, Bool
import math
from vortex_msgs.msg import CameraObjectInfo


class Cam_pd:
    def __init__(self):
        self.K_p = 10
        self.K_d = 3
        self.dt = 0.1
        self.max = 40
        self.min = -40

        self.pre_error = 0
        self.cam_goal = 0.0

        self.pi = math.pi

        self.heading_goal = 0

        self.cam_armed = False
        self.heading_msg = Float64()

        self.odom_sub = rospy.Subscriber('gate_midpoint', CameraObjectInfo, self.cam_center)#('/odometry/filtered', Odometry, get_rotation)
        self.cam_pub = rospy.Publisher('/yaw_goal', Float64, queue_size=1)
        self.cam_goal_sub = rospy.Subscriber('/cam_center', Bool, self.set_cam_goal)

        self.r = rospy.Rate(10)


    def set_cam_goal(self, msg):
        self.cam_armed = msg.data

    def get_cam_input(self, error):
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


    def cam_center (self, msg):
        width = msg.frame_width
        #height = msg.frame_height
        pix_x = msg.pos_x
        if msg.confidence > 0.4:
            error = width/2 - pix_x
            print(error)
            if error > 40:
                self.heading_goal += 0.5
            elif error < -40:
                self.heading_goal -= 0.5
            else:
                pass
            self.heading_msg.data = self.heading_goal
            self.cam_pub.publish(self.heading_msg)
            #self.r.sleep()



        #cam_input = Wrench()
        #cam_input.torque.z = self.get_cam_input(cam-self.cam_goal)
        #self.cam_pub.publish(cam_input)

    def main(self):
        while not rospy.is_shutdown():
            self.r.sleep()



if __name__ == '__main__':
    rospy.init_node('cam_controller')
    cam_pd = Cam_pd()
    cam_pd.main()
