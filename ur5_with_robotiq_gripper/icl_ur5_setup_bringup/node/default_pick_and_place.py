#!/usr/bin/env python3

import rospy
import moveit_commander
import geometry_msgs.msg
import sys

class PickAndPlace:
    def __init__(self):
        rospy.init_node('pick_and_place', anonymous=False)

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group_name = rospy.get_param('~arm_group', 'manipulator')
        self.gripper_group_name = rospy.get_param('~gripper_group', 'gripper')
        self.arm_move_group = moveit_commander.MoveGroupCommander(self.arm_group_name)
        self.gripper_move_group = moveit_commander.MoveGroupCommander(self.gripper_group_name)

        self.gripper_joint_name = rospy.get_param('~gripper_joint', 'finger_joint')
        self.gripper_open_position = rospy.get_param('~gripper_open_position', 0.0)  # Open position for finger_joint
        self.gripper_closed_position = rospy.get_param('~gripper_closed_position', 0.78)  # Closed position for finger_joint
        self.approach_height = rospy.get_param('~approach_height', 0.2)

        self.pick_pose = geometry_msgs.msg.Pose()
        self.pick_pose.position.x = 0.5
        self.pick_pose.position.y = 0.2
        self.pick_pose.position.z = 0.5
        self.pick_pose.orientation.x = 0.0
        self.pick_pose.orientation.y = 1.0
        self.pick_pose.orientation.z = 0.0
        self.pick_pose.orientation.w = 0.0  # Downward-facing quaternion

        self.place_pose = geometry_msgs.msg.Pose()
        self.place_pose.position.x = 0.5
        self.place_pose.position.y = -0.2
        self.place_pose.position.z = 0.5
        self.place_pose.orientation.x = 0.0
        self.place_pose.orientation.y = 1.0
        self.place_pose.orientation.z = 0.0
        self.place_pose.orientation.w = 0.0  # Downward-facing quaternion

        self.home_joint_positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]  # Example home position

        self.execute_pick_and_place()

    def execute_pick_and_place(self):
        # Move to home position before starting
        self.move_to_home_position()

        # Approach pick location
        self.move_to_pose(self.offset_pose(self.pick_pose, self.approach_height), "approach pick")
        # Move to pick location
        self.move_to_pose(self.pick_pose, "pick")
        # Close gripper
        self.control_gripper(close=True)
        # Retreat from pick location
        self.move_to_pose(self.offset_pose(self.pick_pose, self.approach_height), "retreat pick")

        # Move to home position before placing
        self.move_to_home_position()

        # Approach place location
        self.move_to_pose(self.offset_pose(self.place_pose, self.approach_height), "approach place")
        # Move to place location
        self.move_to_pose(self.place_pose, "place")
        # Open gripper
        self.control_gripper(close=False)
        # Retreat from place location
        self.move_to_pose(self.offset_pose(self.place_pose, self.approach_height), "retreat place")

        # Move to home position after completing the task
        self.move_to_home_position()

    def move_to_home_position(self):
        rospy.loginfo("Moving to home position...")
        self.arm_move_group.set_joint_value_target(self.home_joint_positions)
        success = self.arm_move_group.go(wait=True)
        self.arm_move_group.stop()
        if not success:
            rospy.logwarn("Failed to move to home position.")

    def move_to_pose(self, pose, description):
        rospy.loginfo(f"Moving to {description}...")
        self.arm_move_group.set_pose_target(pose)
        success = self.arm_move_group.go(wait=True)
        self.arm_move_group.stop()
        self.arm_move_group.clear_pose_targets()
        if not success:
            rospy.logwarn(f"Failed to move to {description}.")

    def offset_pose(self, pose, offset_z):
        offset_pose = geometry_msgs.msg.Pose()
        offset_pose.position.x = pose.position.x
        offset_pose.position.y = pose.position.y
        offset_pose.position.z = pose.position.z + offset_z
        offset_pose.orientation = pose.orientation
        return offset_pose

    def control_gripper(self, close):
        # Determine the target position based on the close parameter
        target_position = self.gripper_closed_position if close else self.gripper_open_position

        # Log the action being performed
        rospy.loginfo(f"Setting gripper joint '{self.gripper_joint_name}' to {'close' if close else 'open'} position ({target_position}).")

        # Set the joint value target for the gripper
        self.gripper_move_group.set_joint_value_target({self.gripper_joint_name: target_position})

        # Execute the motion
        success = self.gripper_move_group.go(wait=True)
        self.gripper_move_group.stop()

        # Log the result of the operation
        if success:
            rospy.loginfo(f"Gripper successfully moved to {'close' if close else 'open'} position.")
        else:
            rospy.logwarn("Failed to control gripper.")

if __name__ == '__main__':
    try:
        PickAndPlace()
    except rospy.ROSInterruptException:
        pass
