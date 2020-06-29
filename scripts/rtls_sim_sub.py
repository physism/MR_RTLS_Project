#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import numpy as np
import sympy as sym
import estimator

def callback(data, Est):
#    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    received = data.data.split()

    msr = np.zeros(5)
    msr[0] = 0.001 * float(received[0])
    msr[1] = 0.001 * float(received[1])
    msr[2] = 0.001 * float(received[2])
    msr[3] = 0.001 * float(received[3])
    msr[4] = float(received[4])
    ctrl = np.array([0.1, 0.5, 0.1])
    alpha = not (0 in msr)

    x_hat = Est.estimate(msr, ctrl, alpha)
#    print(x_hat)
    rospy.loginfo(str(x_hat))

def rtls_sim_sub():
    # State variables, Control inputs, Time-varying parameters
    x, y, theta = sym.symbols('x y theta')
    v_l, v_theta, dt = sym.symbols('v_l v_theta dt')
    state_set = (x, y, theta)
    input_set = (v_l, v_theta, dt)
    # Position values of fixed anchors #1 ~ #4
    anchor_pos = np.array([[-3.0, -3.0], [-3.0, 3.0], [3.0, 3.0], [3.0, -3.0]])
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
    x_init = np.array([2.5, 0.5, 0])
    z_init = np.ones(5)
#    u_init = np.concatenate((ctrl_data[0], np.array(sampling_time)))
    P = np.zeros((3, 3))
    Q = np.diag([0.01, 0.01, 0.01])
    R = np.diag([0.02, 0.02, 0.02, 0.02, 0.1])
#    Est = estimator.EKF(f, h, state_set, input_set, P, Q, R, x_init)
#    Est = estimator.PF_Gaussian(f, h, state_set, input_set, 500, P, Q, R, x_init)
    Est = estimator.FIR(f, h, state_set, input_set, 15, x_init, z_init)

    rospy.init_node('rtls_sim_sub', anonymous=True)
    rospy.Subscriber('sim_data', String, callback, Est)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    rtls_sim_sub()
