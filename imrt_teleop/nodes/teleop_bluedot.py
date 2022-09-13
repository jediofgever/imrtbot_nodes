#!/usr/bin/env python

# Example code for teleoperating the IMRT100 robot
# using an app called Blue Dot
#
# This is an example of an event-driven program
# The program will wait for something to happen in the
# Blue Dot app on the paired device. When the user presses the
# blue dot in the Blue Dot app, it will trigger a function call
# in this program.


# Import some modules that we will need
import rospy
import bluedot
import time
from geometry_msgs.msg import Twist


class TeleopBluedot:
    # Robot dimentions
    # ROBOT_WIDTH = 0.40 # m

    def __init__(self):
        
        self._vx_gain = 0.4
        self._wz_gain = 0.8
        self._cmd_pub = rospy.Publisher("teleop/cmd_vel", Twist, queue_size=1)
        self._bd = bluedot.BlueDot()

    def publish_cmd(self):
        vx = 0
        wz = 0
        
        if self._bd.is_pressed:

            # use pos.x, pos.y and pos.distance to determin vx and wz
            vx = self._vx_gain * self._bd.position.distance * (-1,1)[self._bd.position.y > 0]
            wz = self._wz_gain * self._bd.position.x * (1,-1)[self._bd.position.y > 0]


        cmd_msg = Twist()
        cmd_msg.linear.x = vx
        cmd_msg.linear.y = 0
        cmd_msg.angular.z = wz

        self._cmd_pub.publish(cmd_msg)




def main():

    rospy.init_node("teleop_bluedot", anonymous=True)
    rospy.loginfo("Teleop Bluedot starting..")
    teleop_bluedot = TeleopBluedot()

    rospy.loginfo("Running..")
    try:
        while not rospy.is_shutdown():
            teleop_bluedot.publish_cmd()
            rospy.sleep(0.1)

    finally:
        print("Goodbye")


if __name__ == '__main__':
    main()
      
    
    




