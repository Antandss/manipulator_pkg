#! /usr/bin/env python

import rospy
import sys
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetMotionPlanRequest
import moveit_commander
import actionlib
from manipulator_pkg.msg import GoToAction, GoToActionFeedback
from geometry_msgs.msg import Pose, PoseStamped
import tf


class Manipulator():
    _feedback = GoToActionFeedback()

    def __init__(self, robot_description, planning_group, end_effector_link) -> None:
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot_commander = moveit_commander.RobotCommander(robot_description)

        self.scene = moveit_commander.PlanningSceneInterface() 

        self.group = moveit_commander.MoveGroupCommander(planning_group) #arm
        self.group.set_end_effector_link(end_effector_link)
        self.group.set_max_velocity_scaling_factor(0.3)
        self.group.set_max_acceleration_scaling_factor(0.3) 

        self.base_frame = "ur5e_base_link"

        self.tf = tf.TransformListener()

    def goto_pose_base(self, pose, planning_time=5):
        # pose here is in base frame coordinates
        self.group.set_pose_target(pose)
        self.group.set_planning_time
        return self.group.go(wait=True)        

    def goto_pose(self, pose, planning_time=5):
        # pose here is in arbitrary frame and will be transformed to base_frame 
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.base_frame 
        pose_stamped.pose = pose
        pose_base_frame = self.tf.transformPose(self.base_frame, pose_stamped)
        return self.goto_pose_base(pose_base_frame, planning_time)

    def goto_joint_state(self, joint_states, planning_time=5):
        # setting joint target in group and executing
        self.group.set_joint_value_target(joint_states)
        self.group.set_planning_time = planning_time
        return self.group.go(wait=True)
        

if __name__ == "__main__":
    rospy.init_node("manipulator")
    robot_description = "heron_robot"
    planning_group = "manipulator"
    end_effector_link = "gripper_link" # finger_link_1 fringer_link_2 ?

    m = Manipulator(robot_description, planning_group, end_effector_link)
    rospy.spin()



