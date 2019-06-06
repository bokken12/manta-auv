'''
#!/usr/bin/env python
from __future__ import print_function
import rospy
import smach
import smach_ros
import smach_viewer
import roslaunch
import actionlib
import actionlib_tutorials.msg
from std_msgs.msg import Bool, String

#from vortex_msgs import Bouy_camera

#fiks at cameranode er tilgjengelig for state machine
#subscribe paa riktig topics saa man kan se om gate er funnet

def mission_trigger_callback(trigger_signal):
    print('Received signal')
    global mission_in_progress
    if(trigger_signal.data == True):
        mission_in_progress = not mission_in_progress
        print(mission_in_progress)

def request_preempt():
    global mission_in_progress
    if mission_in_progress:
        return False
    else:
        return True

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['doing','waiting'])
        # subscribe to signal
        print('Init')
        self.rate = rospy.Rate(10)

    def execute(self, userdata):
        #print('Executing')
        global mission_in_progress
        if mission_in_progress == False:
            #print('Waiting')
            self.rate.sleep()
            return 'waiting'
        else:
            print('Jumping')
            return 'doing'

class Cancel(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['canceld'])

    def execute(self, userdata):
        #Kill all nodes
        print('Received mission trigger')
        global mission_in_progress
        mission_in_progress = False
        return 'canceld'


def main():
    #rospy.init_node('action_client_py')
    rospy.init_node('Mission_trigger_test')

    global mission_in_progress
    mission_in_progress = False

    rospy.Subscriber("mission_trigger", Bool, mission_trigger_callback)


    sm = smach.StateMachine(outcomes = ['Done'])

    sis = smach_ros.IntrospectionServer('Qualification_run_server', sm, '/SM_ROOT')
    sis.start()


    with sm:
        smach.StateMachine.add('Idle', Idle(),
                                transitions={'doing':'Cancel', 'waiting':'Idle'})
        smach.StateMachine.add('Cancel', Cancel(),
                                transitions={'canceld':'Idle'})

    outcome = sm.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
'''
