#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.position)
    pub = rospy.Publisher('arm_controller/command', JointTrajectory, queue_size=10)

    joints_str = JointTrajectory()
    joints_str.header = Header()
    joints_str.header.stamp = rospy.Time.now()
    joints_str.joint_names = ['base_joint', 'shoulder', 'elbow', 'wrist']
    point=JointTrajectoryPoint()
    point.positions = [data.position[0], data.position[1], data.position[2], data.position[3]]
    point.time_from_start = rospy.Duration(2)
    joints_str.points.append(point)

    pub.publish(joints_str)


    pub = rospy.Publisher('gripper_controller/command', JointTrajectory, queue_size=10)

    joints_str = JointTrajectory()
    joints_str.header = Header()
    joints_str.header.stamp = rospy.Time.now()
    joints_str.joint_names = ['gripper']
    point=JointTrajectoryPoint()
    point.positions = [data.position[4]]
    point.time_from_start = rospy.Duration(2)
    joints_str.points.append(point)

    pub.publish(joints_str)
    rospy.loginfo("position updated")



def listener():
    rospy.init_node('states', anonymous=True)
    rospy.Subscriber("joint_states", JointState, callback)

    rospy.spin()


def talker():
    pub = rospy.Publisher('arm_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('trajectory')
    rate = rospy.Rate(10) # 10hz
    joints_str = JointTrajectory()
    joints_str.header = Header()
    joints_str.header.stamp = rospy.Time.now()
    joints_str.joint_names = ['base_joint', 'shoulder', 'elbow', 'wrist']
    point=JointTrajectoryPoint()
    point.positions = [0.5, 0.5, 0.5, 0.5]
    point.time_from_start = rospy.Duration(2)
    joints_str.points.append(point)
    while not rospy.is_shutdown():
      joints_str.header.stamp = rospy.Time.now()
      pub.publish(joints_str)
      rospy.loginfo("position updated")
      rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
