#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import numpy as np
import sympy as sym

def rtls_sim_pub():
    pub = rospy.Publisher('sim_data', String, queue_size=10)
    rospy.init_node('rtls_sim_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # State variables, Control inputs, Time-varying parameters
    x, y, theta = sym.symbols('x y theta')
    v_l, v_theta, dt = sym.symbols('v_l v_theta dt')
    state_set = (x, y, theta)
    input_set = (v_l, v_theta, dt)
    anchor_pos = np.array([[-3.0, 3.0], [-3.0, 3.0], [3.0, 3.0], [3.0, -3.0]])
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
    f_func = sym.lambdify(state_set + input_set, f)
    h_func = sym.lambdify(state_set + input_set, h)
    # Initialization
#    time_init = rospy.Time.now()
    state = np.array([2.0, 2.0, 0.0])
    msr = np.zeros(5)
    sampling_time = 0.1
    # Scenario: Constant input
    ctrl = np.array([0.1, 0.5, sampling_time])
    args1 = np.zeros(5)
    args2 = np.zeros(5)

    while not rospy.is_shutdown():
        args1 = np.concatenate((state, ctrl))
        state = f_func(*args1)[0]
        args2 = np.concatenate((state, ctrl))
        msr = h_func(*args2)[0]
        msr_mm = msr[0:4] * 1000
        msr_angle = msr[4]

        output_str = "%d %d %d %d %f" % (msr_mm[0], msr_mm[1], msr_mm[2], msr_mm[3], msr_angle)

        rospy.loginfo(output_str)
        pub.publish(output_str)

        rate.sleep()

if __name__ == '__main__':
    try:
        rtls_sim_pub()
    except rospy.ROSInterruptException:
        pass
