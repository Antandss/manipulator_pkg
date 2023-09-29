#!/usr/bin/env python

from manipulator_pkg.scripts.controller_manager import ControllerManager
from manipulator_pkg.scripts.manipulator import Manipulator
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion

robot_description= "heron_robot"
planning_group = "manipulator"
end_effector_link = "gripper_link"

joint_state_home = [1.57, 0,00, -2.83, -0.32, 1.57, 1.57]
joint_state_lookout_right = [3.14, -1.57, -1.57, -1,57, 1.57, 2.35]
joint_state_lookout_left = [0.0, -1.57, -1.57, -1.57, 1.57, 2.09]

# define coordinates of cans and add "differences"

def moveit_arm_control_client():
    rospy.init_node('moveit_arm_control_client')
    cm = ControllerManager()
    m = Manipulator(robot_description, planning_group, end_effector_link)

    cm.switch("compliant")

    pub = rospy.Publisher('/cartesian_trajectory_generator/new_goal', Pose, queue_size=10)
    pose_msg = Pose()
    pose_msg.position = Point()
    pose_msg.orientation = Quaternion()
    pose_msg.position.x = 14.0
    pose_msg.position.y = -3.0
    pose_msg.position.z = 0.78
    pub.publish(pose_msg) #this doesn't appear to be working when launched

    # unfold arm
    m.goto_joint_state(joint_state_lookout_left)
    # pick left can (probably a call to goto_pose for some pose)

    # fold arm
    m.goto_joint_state(joint_state_home)
    # unfold arm
    m.goto_joint_state(joint_state_lookout_right)
    # place can(same here as for the above)

    # fold arm
    m.goto_joint_state(joint_state_home)

if __name__ == 'main':
    moveit_arm_control_client()