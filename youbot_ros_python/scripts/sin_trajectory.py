import time

import rospy
from math import *
import random
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

NUM_JOINTS = 5
RATE = 30

SIN_BASE_A = pi / 2
SIN_JOINTS_A = pi / 8

# y = b + A*sin(w*t + q)
b = [pi, pi / 2 - pi / 8, pi + pi / 6, pi / 2 + pi / 12, pi]
A = [SIN_BASE_A, SIN_JOINTS_A, SIN_JOINTS_A, SIN_JOINTS_A, SIN_JOINTS_A]
w = [2.0, 2.7, 3.2, 3.6, 4.1]
q = [random.random() * pi for i in range(NUM_JOINTS)]
# A = [0, 0, 0, 0, 0]

assert len(A) == NUM_JOINTS
assert len(w) == NUM_JOINTS
assert len(q) == NUM_JOINTS


def sin_trajectory(arm=1):
    pub = rospy.Publisher(f'/arm_{arm}/arm_controller/command', JointTrajectory, queue_size=1)
    rospy.init_node(f'arm_{arm}_controller_sin', anonymous=True)
    rate = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        traj_point = JointTrajectory()
        traj_point.header.stamp = rospy.Time.now()
        traj_point.joint_names = [f"arm_joint_{i}" for i in range(1, NUM_JOINTS + 1)]
        point = JointTrajectoryPoint()
        for i in range(NUM_JOINTS):
            point.positions.append(b[i] + A[i] * sin(w[i] * time.process_time() + q[i]))
            # point.velocities.append(A[i] * w[i] * cos(w[i] * time.process_time() + q[i]))
            # point.accelerations.append(- A[i] * w[i] * w[i] * sin(w[i] * time.process_time() + q[i]))
        point.time_from_start = rospy.Duration.from_sec(1 / RATE)  # secs
        traj_point.points.append(point)
        # rospy.loginfo(f"sending point {[point.positions for point in traj_point.points]}")
        pub.publish(traj_point)
        rate.sleep()


if __name__ == '__main__':
    try:
        sin_trajectory()
    except rospy.ROSInterruptException:
        pass
