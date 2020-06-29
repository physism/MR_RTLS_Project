#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import numpy as np
import sympy as sym
import estimator

class Estimation_center:
    def __init__(self, Est):
        self.rtls_distances = [0.0, 0.0, 0.0, 0.0]
        self.heading_angle = 0.0
        self.heading_angle_init = 0.0
        self.angle_init_flag = False
        self.num_turn = 0
        self.control_input = [0.0, 0.0, 0.1]
        self.Est = Est
        self.x_hat = np.zeros(3)
    def rtls_callback(self, msg):
        # Store RTLS distances
        received = msg.data.split()
        try:
            index = received.index('1')
            self.rtls_distances[0] = float(received[index + 1]) * 0.001
            self.rtls_distances[1] = float(received[index + 2]) * 0.001
            self.rtls_distances[2] = float(received[index + 3]) * 0.001
            self.rtls_distances[3] = float(received[index + 4]) * 0.001
        except:
            pass
    def imu_callback(self, msg):
        # Store heading angle
        if not self.angle_init_flag:
            self.heading_angle_init = np.deg2rad(msg.orientation.z)
            self.angle_init_flag = True
        else:
            angle = np.deg2rad(msg.orientation.z) - self.heading_angle_init + 2 * np.pi * self.num_turn
            angle_diff = angle - self.heading_angle
            if angle_diff < -np.pi:
                self.num_turn = self.num_turn + 1
                angle = angle + 2 * np.pi
            elif angle_diff > np.pi:
                self.num_turn = self.num_turn - 1
                angle = angle - 2 * np.pi
            self.heading_angle = angle
    def input_callback(self, msg):
        self.control_input[0] = msg.linear.x
        self.control_input[1] = msg.angular.z
#        self.control_input[2] = 0.1
    def localization(self):
        elements = [0.0, 0.0, 0.0, 0.0, 0.0]
        elements[0:4] = self.rtls_distances
        elements[4] = self.heading_angle
        msr = np.array(elements)
#        ctrl = np.array([0.0, 0.0, 0.1])
        ctrl = np.array(self.control_input)
        alpha = not (0 in msr)
        self.x_hat = self.Est.estimate(msr, ctrl, alpha)

def rtls_localization():
    # State variables, Control inputs, Time-varying parameters
    x, y, theta = sym.symbols('x y theta')
    v_l, v_theta, dt = sym.symbols('v_l v_theta dt')
    state_set = (x, y, theta)
    input_set = (v_l, v_theta, dt)
    # Position values of fixed anchors #1 ~ #4
    anchor_pos = np.array([[-0.6, -0.6], [-0.6, 0.6], [0.6, 0.6], [0.6, -0.6]])
    # State and measurement equations
    f = sym.Matrix(1, 3, [0, 0, 0])
    f[0] = x + v_l * dt * sym.cos(theta + 0.5 * v_theta * dt)
    f[1] = y + v_l * dt * sym.sin(theta + 0.5 * v_theta * dt)
    f[2] = theta + v_theta * dt
    h = sym.Matrix(1, 5, [0, 0, 0, 0, 0])
    h[0] = sym.sqrt((x - anchor_pos[0][0]) ** 2 + (y - anchor_pos[0][1]) ** 2)
    h[1] = sym.sqrt((x - anchor_pos[1][0]) ** 2 + (y - anchor_pos[1][1]) ** 2)
    h[2] = sym.sqrt((x - anchor_pos[2][0]) ** 2 + (y - anchor_pos[2][1]) ** 2)
    h[3] = sym.sqrt((x - anchor_pos[3][0]) ** 2 + (y - anchor_pos[3][1]) ** 2)
    h[4] = theta
    # Initialize estimator
    x_init = np.array([0.0, 0.0, 0.0])
    z_init = np.ones(5)
#    u_init = np.concatenate((ctrl_data[0], np.array(sampling_time)))
    P = np.zeros((3, 3))
    Q = np.diag([0.01, 0.01, 0.01])
    R = np.diag([0.02, 0.02, 0.02, 0.02, 0.1])
#    Est = estimator.EKF(f, h, state_set, input_set, P, Q, R, x_init)
#    Est = estimator.PF_Gaussian(f, h, state_set, input_set, 500, P, Q, R, x_init)
    Est = estimator.FIR(f, h, state_set, input_set, 15, x_init, z_init)

    rospy.init_node('rtls_localization', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    center = Estimation_center(Est)
    rospy.Subscriber('/tb3a/read', String, center.rtls_callback)
    rospy.Subscriber('/tb3a/mw_ahrsv1/imu', Imu, center.imu_callback)
    rospy.Subscriber('/tb3a/cmd_vel', Twist, center.input_callback)
    pub = rospy.Publisher('x_hat', String, queue_size=10)

    while not rospy.is_shutdown():
        center.localization()
        rospy.loginfo(str(center.x_hat))
        output_str = str(center.x_hat[0]) + ' ' + str(center.x_hat[1]) + ' ' + str(center.x_hat[2])
        pub.publish(output_str)

        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

if __name__ == '__main__':
    try:
        rtls_localization()
    except rospy.ROSInterruptException:
        pass
