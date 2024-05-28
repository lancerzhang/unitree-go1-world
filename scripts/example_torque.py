#!/usr/bin/python

import sys
import time
import math
import numpy as np
import os
import rospy
from unitree_legged_msgs.msg import MotorCmd, MotorState

robot_name='go1'

def create_motor_cmd(mode, q, dq, tau, Kp, Kd):
    motor_cmd = MotorCmd()
    motor_cmd.mode = mode
    motor_cmd.q = q
    motor_cmd.dq = dq
    motor_cmd.tau = tau
    motor_cmd.Kp = Kp
    motor_cmd.Kd = Kd
    return motor_cmd

if __name__ == '__main__':
    rospy.init_node('unitree_motor_controller', anonymous=True)
    
    joint_groups = {
        'calf': ['FR_calf', 'FL_calf', 'RR_calf', 'RL_calf'],
        'hip': ['FR_hip', 'FL_hip', 'RR_hip', 'RL_hip'],
        'thigh': ['FR_thigh', 'FL_thigh', 'RR_thigh', 'RL_thigh']
    }
    
    publishers = {name: rospy.Publisher(f'/{robot_name}_gazebo/{name}_controller/command', MotorCmd, queue_size=10) for group in joint_groups.values() for name in group}
    motor_states = {name: MotorState() for group in joint_groups.values() for name in group}

    def create_callback(name):
        def callback(msg):
            global motor_states
            motor_states[name] = msg
        return callback

    subscribers = [rospy.Subscriber(f'/{robot_name}_gazebo/{name}_controller/state', MotorState, create_callback(name)) for group in joint_groups.values() for name in group]

    rate = rospy.Rate(500)  # 500 Hz

    calf_cmd = create_motor_cmd(10, -1.2999999523162842, 0.0, 0.0, 300.0, 15.0)
    hip_cmd = create_motor_cmd(10, 0.0, 0.0, 0.0, 180.0, 8.0)
    thigh_cmd = create_motor_cmd(10, 0.6700000166893005, 0.0, 0.0, 180.0, 8.0)

    motiontime = 0

    while not rospy.is_shutdown():
        motiontime += 1

        if motiontime >= 500:
            for name in joint_groups['calf']:
                publishers[name].publish(calf_cmd)
            for name in joint_groups['hip']:
                publishers[name].publish(hip_cmd)
            for name in joint_groups['thigh']:
                publishers[name].publish(thigh_cmd)

        rate.sleep()
