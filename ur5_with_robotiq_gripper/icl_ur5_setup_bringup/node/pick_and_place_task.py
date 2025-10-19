#!/usr/bin/env python

import rospy
import moveit_commander
import tf2_ros
import geometry_msgs.msg
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output

class PickAndPlace:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('pick_and_place_task', anonymous=True)

        # Initialize MoveIt!
        moveit_commander.roscpp_initialize([])
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # Initialize TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialize Gripper Publisher
        self.gripper_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', Robotiq2FGripper_robot_output, queue_size=10)


        self.move_to_home_position()

        # Wait for the transform to be available
        tag_transform = None
        while not tag_transform:
            tag_transform = self.wait_for_transform()
            rospy.sleep(1)

        self.plan_pick_and_place_moves(tag_transform)

    def plan_pick_and_place_moves(self, tag_transform):
        # Calculate the approach, grasp, and retreat poses
        approach_pose = geometry_msgs.msg.Pose()
        approach_pose.position.x = tag_transform.transform.translation.x
        approach_pose.position.y = tag_transform.transform.translation.y
        approach_pose.position.z = tag_transform.transform.translation.z + 0.2  # 10cm above the tag
        approach_pose.orientation = tag_transform.transform.rotation

        grasp_pose = geometry_msgs.msg.Pose()
        grasp_pose.position.x = tag_transform.transform.translation.x
        grasp_pose.position.y = tag_transform.transform.translation.y
        grasp_pose.position.z = tag_transform.transform.translation.z  # At the tag
        grasp_pose.orientation = tag_transform.transform.rotation

        retreat_pose = geometry_msgs.msg.Pose()
        retreat_pose.position.x = tag_transform.transform.translation.x
        retreat_pose.position.y = tag_transform.transform.translation.y
        retreat_pose.position.z = tag_transform.transform.translation.z + 0.2 # 10cm above the tag
        retreat_pose.orientation = tag_transform.transform.rotation

        # Move to approach pose
        self.move_group.set_pose_target(approach_pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Move to grasp pose
        self.move_group.set_pose_target(grasp_pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Close gripper
        self.close_gripper()

        # Move to retreat pose
        self.move_group.set_pose_target(retreat_pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Open gripper
        self.open_gripper()

    def open_gripper(self):
        cmd = Robotiq2FGripper_robot_output()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rPR = 0
        cmd.rSP = 255
        cmd.rFR = 150
        self.gripper_pub.publish(cmd)
        rospy.sleep(1)

    def close_gripper(self):
        cmd = Robotiq2FGripper_robot_output()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rPR = 255
        cmd.rSP = 255
        cmd.rFR = 150
        self.gripper_pub.publish(cmd)
        rospy.sleep(1)


    def move_to_home_position(self):
        # Move the arm to a known 'scan' pose
        joint_goal = self.move_group.get_current_joint_values()
        # TODO: Set the joint values for the home position
        joint_goal[0] = 0
        joint_goal[1] = -1.57
        joint_goal[2] = 1.57
        joint_goal[3] = -1.57
        joint_goal[4] = -1.57
        joint_goal[5] = 0

        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

    def wait_for_transform(self):
        # Look up the transform from base_link to tag_0
        try:
            trans = self.tf_buffer.lookup_transform('base_link', 'tag_0', rospy.Time(0), rospy.Duration(1.0))
            return trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform not found")
            return None

if __name__ == '__main__':
    try:
        PickAndPlace()
    except rospy.ROSInterruptException:
        pass
