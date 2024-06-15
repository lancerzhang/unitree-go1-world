import rospy
from transforms3d.euler import quat2euler
from unitree_legged_msgs.msg import MotorCmd


def create_motor_cmd(mode, q, dq, tau, Kp, Kd):
    motor_cmd = MotorCmd()
    motor_cmd.mode = mode
    motor_cmd.q = q
    motor_cmd.dq = dq
    motor_cmd.tau = tau
    motor_cmd.Kp = Kp
    motor_cmd.Kd = Kd
    return motor_cmd


def check_messages(env, event):
    current_time = rospy.Time.now()
    motor_state_timeout = rospy.Duration(2)  # 2 seconds timeout
    imu_timeout = rospy.Duration(2)  # 2 seconds timeout

    for name, last_time in env.last_motor_state_time.items():
        if current_time - last_time > motor_state_timeout:
            rospy.logerr(f"Motor state {name} not received for 2 seconds, stopping the program.")
            rospy.signal_shutdown("Motor state timeout")

    if current_time - env.last_imu_time > imu_timeout:
        rospy.logerr("IMU state not received for 2 seconds, stopping the program.")
        rospy.signal_shutdown("IMU state timeout")


def is_flipped(env):
    if env.current_imu:
        flipped_time_threshold = 10.0  # 10 second threshold
        rpy = quat2euler(
            [env.current_imu.orientation.w, env.current_imu.orientation.x, env.current_imu.orientation.y,
             env.current_imu.orientation.z])
        roll, pitch, yaw = rpy
        current_time = rospy.Time.now()

        if abs(roll) > env.flipped_threshold or abs(pitch) > env.flipped_threshold:
            if not hasattr(env, 'flip_start_time'):
                env.flip_start_time = current_time
            elif (current_time - env.flip_start_time).to_sec() > flipped_time_threshold:
                rospy.loginfo(f'Robot is flipped for more than {flipped_time_threshold} seconds.')
                return True
        else:
            if hasattr(env, 'flip_start_time'):
                del env.flip_start_time

    return False


def reset_env(env):
    env.reset_world_service()
    rospy.sleep(1)  # Wait for the reset to complete

    # Reset joint positions
    hip_positions = [0.0] * 4
    thigh_positions = [0.67] * 4
    calf_positions = [-1.3] * 4

    # Publish the reset positions
    for i, group in enumerate(env.joint_groups.values()):
        positions = hip_positions if 'hip' in group[0] else thigh_positions if 'thigh' in group[0] else calf_positions
        for j, name in enumerate(group):
            env.publishers[name].publish(create_motor_cmd(10, positions[j], 0.0, 0.0, 300.0, 15.0))

    rospy.sleep(1)  # Wait for the positions to be set

    obs = env.get_obs()
    info = {}
    return obs, info
