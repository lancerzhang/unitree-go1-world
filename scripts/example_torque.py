#!/usr/bin/python

import sys
import time
import math
import numpy as np
import os
import rospy
from unitree_legged_msgs.msg import MotorCmd, MotorState

if __name__ == '__main__':
    rospy.init_node('unitree_motor_controller', anonymous=True)
    pub = rospy.Publisher('/go1_gazebo/FR_thigh_controller/command', MotorCmd, queue_size=10)
    rate = rospy.Rate(500)  # 500 Hz

    d = {'FR_0': 0, 'FR_1': 1, 'FR_2': 2,
         'FL_0': 3, 'FL_1': 4, 'FL_2': 5,
         'RR_0': 6, 'RR_1': 7, 'RR_2': 8,
         'RL_0': 9, 'RL_1': 10, 'RL_2': 11}

    PosStopF = math.pow(10, 9)
    VelStopF = 16000.0
    HIGHLEVEL = 0xee
    LOWLEVEL = 0xff
    sin_count = 0
    motiontime = 0

    motor_state = MotorState()

    def motor_state_callback(msg):
        global motor_state
        motor_state = msg

    rospy.Subscriber('/go1_gazebo/FR_thigh_controller/state', MotorState, motor_state_callback)

    while not rospy.is_shutdown():
        motiontime += 1

        freq_Hz = 2
        freq_rad = freq_Hz * 2 * math.pi

        if motiontime >= 500:
            sin_count += 1
            torque = (0 - motor_state.q) * 10.0 + (0 - motor_state.dq) * 1.0
            torque = np.fmin(np.fmax(torque, -5.0), 5.0)

            motor_cmd = MotorCmd()
            motor_cmd.mode = LOWLEVEL
            motor_cmd.q = PosStopF
            motor_cmd.dq = VelStopF
            motor_cmd.Kp = 0
            motor_cmd.Kd = 0
            motor_cmd.tau = torque

            pub.publish(motor_cmd)

        rate.sleep()
