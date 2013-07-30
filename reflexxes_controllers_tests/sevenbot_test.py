#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def main():

    pub = rospy.Publisher('sevenbot/joint_traj_controller/trajectory_command',JointTrajectory)
    rospy.init_node('traj_test')

    p1 = JointTrajectoryPoint()
    p1.positions = [0,0,0,0,0,0,0]
    p1.velocities = [0,0,0,0,0,0,0]
    p1.accelerations = [0,0,0,0,0,0,0]

    p2 = JointTrajectoryPoint()
    p2.positions = [0,0,1.0,0,-1.0,0,0]
    p2.velocities = [0,0,0,0,0,0,0]
    p2.accelerations = [0,0,0,0,0,0,0]
    p2.time_from_start = rospy.Time(4.0)

    p3 = JointTrajectoryPoint()
    p3.positions = [0,0,-1.0,0,1.0,0,0]
    p3.velocities = [0,0,0,0,0,0,0]
    p3.accelerations = [0,0,0,0,0,0,0]
    p3.time_from_start = rospy.Time(20.0)


    traj = JointTrajectory()
    traj.joint_names = ['j1','j2','j3','j4','j5','j6','j7']
    traj.points = [p1,p2,p3]


    rospy.loginfo(traj)

    r = rospy.Rate(1) # 10hz
    pub.publish(traj)
    r.sleep()
    pub.publish(traj)



if __name__ == '__main__':
    main()
