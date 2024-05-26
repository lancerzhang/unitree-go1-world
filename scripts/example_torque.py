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
    
    joint_names = ['FR_hip', 'FR_thigh', 'FR_calf', 
                   'FL_hip', 'FL_thigh', 'FL_calf', 
                   'RR_hip', 'RR_thigh', 'RR_calf', 
                   'RL_hip', 'RL_thigh', 'RL_calf']
    
    publishers = {name: rospy.Publisher(f'/go1_gazebo/{name}_controller/command', MotorCmd, queue_size=10) for name in joint_names}
    motor_states = {name: MotorState() for name in joint_names}

    def create_callback(name):
        def callback(msg):
            global motor_states
            motor_states[name] = msg
        return callback

    subscribers = [rospy.Subscriber(f'/go1_gazebo/{name}_controller/state', MotorState, create_callback(name)) for name in joint_names]

    rate = rospy.Rate(500)  # 500 Hz

    PosStopF = math.pow(10, 9)
    VelStopF = 16000.0
    sin_count = 0
    motiontime = 0

    while not rospy.is_shutdown():
        motiontime += 1

        if motiontime >= 500:
            sin_count += 1
            for name in joint_names:
                state = motor_states[name]
                torque = (0 - state.q) * 10.0 + (0 - state.dq) * 1.0
                torque = np.fmin(np.fmax(torque, -5.0), 5.0)

                motor_cmd = MotorCmd()
                motor_cmd.mode = 0xff
                motor_cmd.q = PosStopF
                motor_cmd.dq = VelStopF
                motor_cmd.Kp = 0
                motor_cmd.Kd = 0
                motor_cmd.tau = torque

                publishers[name].publish(motor_cmd)

        rate.sleep()
