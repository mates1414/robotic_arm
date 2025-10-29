#!/usr/bin/env python

import rospy
import moveit_commander
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
import sys
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output

class PickAndPlace:
    """
    Pick-and-place node that:
    - waits for an AprilTag frame (e.g. 'tag_0'),
    - transforms the tag pose into the robot base frame using TF2,
    - computes approach / grasp / retreat poses for a 5cm cube under the tag,
    - closes the Robotiq gripper to pick and opens to release,
    - moves the object to a configured place pose.

    Assumptions and tunables are available via ROS params (defaults set below).
    """

    def __init__(self):
        rospy.init_node('pick_and_place_task', anonymous=False)

        # MoveIt and robot
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group_name = rospy.get_param('~arm_group', 'manipulator')
        self.arm = moveit_commander.MoveGroupCommander(self.arm_group_name)

        # Gripper publisher (Robotiq 2F)
        self.gripper_topic = rospy.get_param('~gripper_topic', 'Robotiq2FGripperRobotOutput')
        self.gripper_pub = rospy.Publisher(self.gripper_topic, Robotiq2FGripper_robot_output, queue_size=10)
        rospy.sleep(0.5)  # give publisher time to register

        # Try to initialize a MoveIt gripper group (optional). If it doesn't exist, we'll fall back to topic-based control.
        self.gripper_group_name = rospy.get_param('~gripper_group', 'gripper')
        self.gripper_move_group = None
        try:
            self.gripper_move_group = moveit_commander.MoveGroupCommander(self.gripper_group_name)
            rospy.loginfo(f"Initialized MoveIt gripper group '{self.gripper_group_name}' for gripper control")
        except Exception:
            rospy.logwarn(f"MoveIt gripper group '{self.gripper_group_name}' not available; will use Robotiq topic commands")

        # Gripper joint name and positions for MoveIt control (if using a joint-based gripper)
        self.gripper_joint_name = rospy.get_param('~gripper_joint', 'finger_joint')
        self.gripper_open_position = rospy.get_param('~gripper_open_position', 0.0)
        self.gripper_closed_position = rospy.get_param('~gripper_closed_position', 0.78)

        # TF2 for transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Frames and tag name
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.tag_frame = rospy.get_param('~tag_frame', 'tag_0')
        # Use optical frame because Gazebo plugin and apriltag_ros publish in optical convention
        self.camera_frame = rospy.get_param('~camera_frame', 'realsense_color_optical_frame')

        # Object and grasp parameters
        self.cube_size = rospy.get_param('~cube_size', 0.05)  # 5 cm cube
        self.grasp_z_offset = rospy.get_param('~grasp_z_offset', 0.1)  # relative to tag z (tune per setup)
        self.approach_height = rospy.get_param('~approach_height', 0.18)  # how high above grasp to approach
        self.gripper_close_wait = rospy.get_param('~gripper_close_wait', 1.0)
        self.gripper_open_wait = rospy.get_param('~gripper_open_wait', 1.0)

        # Place pose (in base frame) - some sensible default, override with params
        self.place_pose = geometry_msgs.msg.Pose()
        self.place_pose.position.x = rospy.get_param('~place_x', 0.5)
        self.place_pose.position.y = rospy.get_param('~place_y', -0.2)
        self.place_pose.position.z = rospy.get_param('~place_z', 0.5)
        # Default: gripper facing downward
        self.place_pose.orientation.x = rospy.get_param('~place_ori_x', 0.0)
        self.place_pose.orientation.y = rospy.get_param('~place_ori_y', 1.0)
        self.place_pose.orientation.z = rospy.get_param('~place_ori_z', 0.0)
        self.place_pose.orientation.w = rospy.get_param('~place_ori_w', 0.0)

        # Home joint configuration
        self.home_joint_positions = rospy.get_param('~home_joint_positions', [0.0, -1.57, 1.57, -1.57, -1.57, 0.0])

        rospy.loginfo("PickAndPlace node initialized. Waiting for AprilTag...")

        # Run the high-level task
        self.run()

    def run(self):
        # Move robot to home
        self.move_to_home()

        # Wait for the tag to appear and get its pose in the base frame
        tag_pose_in_base = self.wait_for_tag_pose(timeout=30.0)
        if tag_pose_in_base is None:
            rospy.logerr("AprilTag not found within timeout. Aborting.")
            return

        rospy.loginfo(f"AprilTag pose in {self.base_frame}: {tag_pose_in_base}")

        # Compute pick poses (approach, grasp, retreat)
        pick_pose = geometry_msgs.msg.Pose()
        pick_pose.position.x = tag_pose_in_base.position.x
        pick_pose.position.y = tag_pose_in_base.position.y
        # Compute grasp Z using cube size and user offset. We assume the tag origin is at/near the top of the cube.
        pick_pose.position.z = tag_pose_in_base.position.z + self.grasp_z_offset
        pick_pose.orientation = self.downward_orientation_from_tag(tag_pose_in_base.orientation)

        approach_pose = self.offset_pose(pick_pose, self.approach_height)
        retreat_pose = self.offset_pose(pick_pose, self.approach_height)

        # Move to approach
        self.move_to_pose(approach_pose, 'approach')

        # Move down to grasp
        self.move_to_pose(pick_pose, 'grasp')

        # Close gripper (Robotiq)
        self.command_gripper(close=True)
        rospy.sleep(self.gripper_close_wait)

        # Retreat
        self.move_to_pose(retreat_pose, 'retreat')

        # Move to place approach
        place_approach = self.offset_pose(self.place_pose, self.approach_height)
        self.move_to_pose(place_approach, 'place approach')

        # Move to place
        self.move_to_pose(self.place_pose, 'place')

        # Open gripper to release
        self.command_gripper(close=False)
        rospy.sleep(self.gripper_open_wait)

        # Retreat from place
        self.move_to_pose(place_approach, 'place retreat')

        # Return home
        self.move_to_home()

        rospy.loginfo('Pick-and-place task completed.')

    def wait_for_tag_pose(self, timeout=30.0):
        """Wait up to timeout seconds for the tag frame to be present and return a Pose in the base frame.

        Strategy:
        1) Try direct transform base_frame <- tag_frame (most direct).
        2) If not available, try tag expressed in camera_frame then transform that PoseStamped to base_frame.
        """
        start = rospy.Time.now()
        rate = rospy.Rate(5)
        while (rospy.Time.now() - start).to_sec() < timeout and not rospy.is_shutdown():
            try:
                # 1) Direct base <- tag (preferred)
                if self.tf_buffer.can_transform(self.base_frame, self.tag_frame, rospy.Time(0)):
                    trans = self.tf_buffer.lookup_transform(self.base_frame, self.tag_frame, rospy.Time(0), rospy.Duration(1.0))
                    ps = geometry_msgs.msg.PoseStamped()
                    ps.header = trans.header
                    ps.pose.position.x = trans.transform.translation.x
                    ps.pose.position.y = trans.transform.translation.y
                    ps.pose.position.z = trans.transform.translation.z
                    ps.pose.orientation = trans.transform.rotation
                    rospy.loginfo_once(f"Found tag '{self.tag_frame}' directly in {self.base_frame}")
                    return ps.pose

                # 2) Try tag in camera frame then transform pose to base_frame
                if self.tf_buffer.can_transform(self.camera_frame, self.tag_frame, rospy.Time(0)):
                    trans_cam = self.tf_buffer.lookup_transform(self.camera_frame, self.tag_frame, rospy.Time(0), rospy.Duration(1.0))
                    tag_in_cam = geometry_msgs.msg.PoseStamped()
                    tag_in_cam.header.stamp = trans_cam.header.stamp
                    tag_in_cam.header.frame_id = self.camera_frame
                    tag_in_cam.pose.position.x = trans_cam.transform.translation.x
                    tag_in_cam.pose.position.y = trans_cam.transform.translation.y
                    tag_in_cam.pose.position.z = trans_cam.transform.translation.z
                    tag_in_cam.pose.orientation = trans_cam.transform.rotation

                    # ensure we can transform camera->base (most setups will)
                    if self.tf_buffer.can_transform(self.base_frame, self.camera_frame, rospy.Time(0)):
                        try:
                            tag_in_base = self.tf_buffer.transform(tag_in_cam, self.base_frame, rospy.Duration(1.0))
                            rospy.loginfo_once(f"Transformed tag from {self.camera_frame} to {self.base_frame}")
                            return tag_in_base.pose
                        except Exception as e:
                            rospy.logwarn_once(f"Failed to transform tag pose from {self.camera_frame} to {self.base_frame}: {e}")
                    else:
                        rospy.logwarn_once(f"Cannot transform from {self.camera_frame} to {self.base_frame} yet; check static transform / URDF")

                # If reached here, no usable transform yet
                rospy.logdebug("Tag not transformable to base yet; waiting...")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logdebug(f"TF exception while waiting for tag: {e}")
            rate.sleep()
        return None

    def downward_orientation_from_tag(self, tag_orientation):
        """
        Compute an orientation that keeps the gripper pointing downwards while respecting tag rotation around z.
        For simplicity we return a quaternion that orients the end-effector to face down (pointing -z) and keep yaw from tag.
        """
        # If the tag provides a rotation, we prefer to keep its yaw but point the tool straight down.
        # We'll extract yaw from tag_orientation and build a quaternion with roll=pi, pitch=0, yaw=tag_yaw
        import math
        from tf.transformations import euler_from_quaternion, quaternion_from_euler

        q = [tag_orientation.x, tag_orientation.y, tag_orientation.z, tag_orientation.w]
        try:
            roll, pitch, yaw = euler_from_quaternion(q)
        except Exception:
            roll, pitch, yaw = 0.0, 0.0, 0.0

        # roll = pi (180°) flips the gripper to face down along -Z in many UR tool frames
        qd = quaternion_from_euler(math.pi, 0.0, yaw)
        out = geometry_msgs.msg.Quaternion()
        out.x, out.y, out.z, out.w = qd[0], qd[1], qd[2], qd[3]
        return out

    def move_to_home(self):
        rospy.loginfo('Moving to home joint configuration')
        self.arm.set_joint_value_target(self.home_joint_positions)
        ok = self.arm.go(wait=True)
        self.arm.stop()
        if not ok:
            rospy.logwarn('Failed to reach home position')

    def move_to_pose(self, pose, label='pose'):
        rospy.loginfo(f"Moving to {label}: x={pose.position.x:.3f} y={pose.position.y:.3f} z={pose.position.z:.3f}")
        self.arm.set_pose_target(pose)
        ok = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        if not ok:
            rospy.logwarn(f"Move to {label} failed")

    def offset_pose(self, pose, dz):
        p = geometry_msgs.msg.Pose()
        p.position.x = pose.position.x
        p.position.y = pose.position.y
        p.position.z = pose.position.z + dz
        p.orientation = pose.orientation
        return p

    def command_gripper(self, close=True):
        """Control the gripper. Prefer MoveIt gripper group if available, otherwise publish Robotiq commands.

        close: True -> close gripper, False -> open gripper
        """
        # If a MoveIt gripper move group exists, use joint control
        if hasattr(self, 'gripper_move_group') and self.gripper_move_group is not None:
            try:
                target_position = self.gripper_closed_position if close else self.gripper_open_position
                rospy.loginfo(f"Moving gripper (MoveIt) to {'closed' if close else 'open'} position: {target_position}")
                self.gripper_move_group.set_joint_value_target({self.gripper_joint_name: target_position})
                ok = self.gripper_move_group.go(wait=True)
                self.gripper_move_group.stop()
                if not ok:
                    rospy.logwarn('MoveIt gripper movement failed; falling back to topic command')
                else:
                    rospy.loginfo('MoveIt gripper action succeeded')
                    return
            except Exception as e:
                rospy.logwarn(f"Exception while commanding MoveIt gripper: {e}; falling back to topic command")

        # Fallback: publish Robotiq topic commands
        try:
            cmd = Robotiq2FGripper_robot_output()
            cmd.rACT = 1
            cmd.rGTO = 1
            if close:
                cmd.rPR = 255
                cmd.rSP = 255
                cmd.rFR = 150
                rospy.loginfo('Publishing Robotiq command: close')
            else:
                cmd.rPR = 0
                cmd.rSP = 255
                cmd.rFR = 150
                rospy.loginfo('Publishing Robotiq command: open')
            self.gripper_pub.publish(cmd)
        except Exception as e:
            rospy.logerr(f"Failed to publish Robotiq gripper command: {e}")


if __name__ == '__main__':
    try:
        PickAndPlace()
    except rospy.ROSInterruptException:
        pass
