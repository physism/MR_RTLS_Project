#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np
import controller

class Control_center:
    def __init__(self, Ctrl):
        self.x_hat = np.zeros(3)
        self.x_ref = np.array([0.0, 0.0, 0.0])
        self.Ctrl = Ctrl
        self.control_input = np.zeros(2)
    def pose_callback(self, msg):
        # Store x_hat
        received = msg.data.split()
        self.x_hat[0] = float(received[0])
        self.x_hat[1] = float(received[1])
        self.x_hat[2] = float(received[2])
    def control(self):
        self.control_input = self.Ctrl.control(self.x_hat, self.x_ref)

def rtls_control():
    # Controller parameters
    gamma1 = 0.3
    gamma2 = 0.1
    h = 0.5
    Ctrl = controller.Kinematic(gamma1, gamma2, h)

    rospy.init_node('rtls_control', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    center = Control_center(Ctrl)
    rospy.Subscriber('/tb3a/x_hat', String, center.pose_callback)
    pub = rospy.Publisher('/tb3a/cmd_vel', Twist, queue_size=10)

    while not rospy.is_shutdown():
        center.control()
        rospy.loginfo(str(center.control_input))
        output_msg = Twist()
        output_msg.linear.x = center.control_input[0]
        output_msg.angular.z = center.control_input[1]
        pub.publish(output_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        rtls_control()
    except rospy.ROSInterruptException:
        pass
