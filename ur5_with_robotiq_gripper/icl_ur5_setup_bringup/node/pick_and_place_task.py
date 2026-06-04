#!/usr/bin/env python3

import rospy
import moveit_commander
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
import sys
import copy
import actionlib
from sensor_msgs.msg import JointState
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
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
        # max_effort sent in the GripperCommand goal. The gripper is POSITION-
        # controlled (effort control was tried and reverted — see README), so this
        # bounds the holding torque rather than the squeeze force. Tune per gripper.
        self.gripper_max_effort = rospy.get_param('~gripper_max_effort', 20.0)

        # Direct GripperCommand action client (primary gripper control). Sending
        # the goal ourselves lets us set max_effort, which MoveIt's gripper group
        # leaves at 0 (= unbounded).
        self.gripper_action_ns = rospy.get_param('~gripper_action_ns', '/gripper/gripper_cmd')
        self.gripper_action = actionlib.SimpleActionClient(self.gripper_action_ns, GripperCommandAction)
        if self.gripper_action.wait_for_server(rospy.Duration(5.0)):
            rospy.loginfo(f"Connected to GripperCommand action server '{self.gripper_action_ns}'")
        else:
            rospy.logwarn(f"GripperCommand action server '{self.gripper_action_ns}' not available; "
                          f"will fall back to MoveIt group / Robotiq topic")
            self.gripper_action = None

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
        # Gripper TCP offset: distance from tool0 (end-effector frame) to gripper finger tips
        # For Robotiq 85: base_link to finger tips is approximately 16-18 cm when closed
        # This prevents the gripper from trying to position tool0 inside the object
        self.gripper_tcp_offset = rospy.get_param('~gripper_tcp_offset', 0.20)  # meters, tune based on your gripper
        self.grasp_z_offset = rospy.get_param('~grasp_z_offset', 0.0)  # additional offset relative to object top (tune per setup)
        self.approach_height = rospy.get_param('~approach_height', 0.15)  # how high above grasp to approach
        self.gripper_close_wait = rospy.get_param('~gripper_close_wait', 1.0)
        self.gripper_open_wait = rospy.get_param('~gripper_open_wait', 1.0)
        # Seconds to let the robot/controllers settle after spawn before the
        # first trajectory (prevents a one-off CONTROL_FAILED at startup).
        self.startup_settle_time = rospy.get_param('~startup_settle_time', 5.0)

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
        # Let the simulation/controllers settle after spawn before the first
        # trajectory. The robot drops into place at startup, and sending a goal
        # while it is still moving causes a one-off CONTROL_FAILED.
        rospy.loginfo('Waiting for controllers/robot to settle before first move...')
        try:
            rospy.wait_for_message('/joint_states', JointState, timeout=30.0)
        except Exception:
            rospy.logwarn('No /joint_states received yet; continuing anyway')
        rospy.sleep(self.startup_settle_time)

        # Give the gripper a definite open/hold goal so it settles to a known
        # position before we command the first arm trajectory.
        self.command_gripper(close=False)
        rospy.sleep(1.0)

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
        
        # Compute grasp Z:
        # 1. Tag is at the top of the cube
        # 2. We want to grasp at the center/slightly below center of the cube
        # 3. But tool0 needs to be higher because gripper fingers extend down by gripper_tcp_offset
        # Formula: tool0_z = tag_z - (cube_size/2) + gripper_tcp_offset + grasp_z_offset
        pick_pose.position.z = (tag_pose_in_base.position.z 
                                - (self.cube_size / 2.0)  # go to cube center
                                + self.gripper_tcp_offset  # offset tool0 up so fingers reach object
                                + self.grasp_z_offset)     # user fine-tuning
        
        rospy.loginfo(f"Pick pose calculation: tag_z={tag_pose_in_base.position.z:.3f}, "
                     f"cube_size={self.cube_size:.3f}, tcp_offset={self.gripper_tcp_offset:.3f}, "
                     f"grasp_offset={self.grasp_z_offset:.3f} -> tool0_z={pick_pose.position.z:.3f}")
        
        pick_pose.orientation = self.downward_orientation_from_tag(tag_pose_in_base.orientation)

        approach_pose = self.offset_pose(pick_pose, self.approach_height)
        retreat_pose = self.offset_pose(pick_pose, self.approach_height)

        # Tell MoveIt about the support shelf and the cube so it plans the
        # approach from straight above, instead of sweeping the arm sideways
        # through the cube and knocking it off the shelf.
        self.add_collision_scene(tag_pose_in_base)

        # Move to approach (collision-aware: comes in from above the cube)
        self.move_to_pose(approach_pose, 'approach')

        # Remove the cube collision object so the gripper may contact it, then
        # descend straight down (Cartesian) so we don't sweep into it.
        self.scene.remove_world_object('target_object')
        rospy.sleep(0.5)
        self.cartesian_move(pick_pose, 'grasp descend')

        # Close gripper (Robotiq) - the gazebo_grasp_fix plugin attaches the cube
        self.command_gripper(close=True)
        rospy.sleep(self.gripper_close_wait)

        # Retreat straight up (Cartesian), then drop the shelf from the scene
        self.cartesian_move(retreat_pose, 'retreat')
        self.scene.remove_world_object('support_shelf')

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
        logged_frames = False
        
        while (rospy.Time.now() - start).to_sec() < timeout and not rospy.is_shutdown():
            try:
                # Debug: Log available frames once
                if not logged_frames:
                    all_frames = self.tf_buffer.all_frames_as_string()
                    rospy.loginfo(f"Available TF frames:\n{all_frames}")
                    logged_frames = True
                
                # 1) Direct base <- tag (preferred)
                if self.tf_buffer.can_transform(self.base_frame, self.tag_frame, rospy.Time(0)):
                    trans = self.tf_buffer.lookup_transform(self.base_frame, self.tag_frame, rospy.Time(0), rospy.Duration(1.0))
                    ps = geometry_msgs.msg.PoseStamped()
                    ps.header = trans.header
                    ps.pose.position.x = trans.transform.translation.x
                    ps.pose.position.y = trans.transform.translation.y
                    ps.pose.position.z = trans.transform.translation.z
                    ps.pose.orientation = trans.transform.rotation
                    rospy.loginfo(f"✓ Found tag '{self.tag_frame}' directly in {self.base_frame}: "
                                f"x={ps.pose.position.x:.3f}, y={ps.pose.position.y:.3f}, z={ps.pose.position.z:.3f}")
                    return ps.pose

                # 2) Try tag in camera frame then transform pose to base_frame
                if self.tf_buffer.can_transform(self.camera_frame, self.tag_frame, rospy.Time(0)):
                    trans_cam = self.tf_buffer.lookup_transform(self.camera_frame, self.tag_frame, rospy.Time(0), rospy.Duration(1.0))
                    tag_in_cam = geometry_msgs.msg.PoseStamped()
                    tag_in_cam.header.stamp = rospy.Time.now()  # Use current time for transform
                    tag_in_cam.header.frame_id = self.camera_frame
                    tag_in_cam.pose.position.x = trans_cam.transform.translation.x
                    tag_in_cam.pose.position.y = trans_cam.transform.translation.y
                    tag_in_cam.pose.position.z = trans_cam.transform.translation.z
                    tag_in_cam.pose.orientation = trans_cam.transform.rotation
                    
                    rospy.loginfo(f"Tag in {self.camera_frame}: x={tag_in_cam.pose.position.x:.3f}, "
                                f"y={tag_in_cam.pose.position.y:.3f}, z={tag_in_cam.pose.position.z:.3f}")

                    # ensure we can transform camera->base (most setups will)
                    if self.tf_buffer.can_transform(self.base_frame, self.camera_frame, rospy.Time(0)):
                        try:
                            tag_in_base = self.tf_buffer.transform(tag_in_cam, self.base_frame, rospy.Duration(1.0))
                            rospy.loginfo(f"✓ Transformed tag from {self.camera_frame} to {self.base_frame}: "
                                        f"x={tag_in_base.pose.position.x:.3f}, y={tag_in_base.pose.position.y:.3f}, "
                                        f"z={tag_in_base.pose.position.z:.3f}")
                            return tag_in_base.pose
                        except Exception as e:
                            rospy.logwarn(f"Failed to transform tag pose from {self.camera_frame} to {self.base_frame}: {e}")
                    else:
                        rospy.logwarn_once(f"Cannot transform from {self.camera_frame} to {self.base_frame} yet; check URDF/TF tree")

                # If reached here, no usable transform yet
                rospy.logdebug("Tag not transformable to base yet; waiting...")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logdebug(f"TF exception while waiting for tag: {e}")
            rate.sleep()
        
        rospy.logerr(f"Timeout waiting for tag '{self.tag_frame}'. Check:\n"
                    f"  1. AprilTag detection is running: roslaunch icl_ur5_setup_bringup apriltag.launch\n"
                    f"  2. Tag is visible in camera view: rostopic echo /tag_detections\n"
                    f"  3. TF tree is complete: rosrun rqt_tf_tree rqt_tf_tree")
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

    def move_to_home(self, retries=2):
        rospy.loginfo('Moving to home joint configuration')
        for attempt in range(retries + 1):
            self.arm.set_joint_value_target(self.home_joint_positions)
            ok = self.arm.go(wait=True)
            self.arm.stop()
            self.arm.clear_pose_targets()
            if ok:
                return True
            rospy.logwarn(f'Home move failed (attempt {attempt + 1}/{retries + 1}); retrying...')
            rospy.sleep(1.0)
        rospy.logwarn('Failed to reach home position after retries')
        return False

    def move_to_pose(self, pose, label='pose', retries=1):
        rospy.loginfo(f"Moving to {label}: x={pose.position.x:.3f} y={pose.position.y:.3f} z={pose.position.z:.3f}")
        for attempt in range(retries + 1):
            self.arm.set_pose_target(pose)
            ok = self.arm.go(wait=True)
            self.arm.stop()
            self.arm.clear_pose_targets()
            if ok:
                return True
            rospy.logwarn(f"Move to {label} failed (attempt {attempt + 1}/{retries + 1})")
            rospy.sleep(0.5)
        return False

    def offset_pose(self, pose, dz):
        p = geometry_msgs.msg.Pose()
        p.position.x = pose.position.x
        p.position.y = pose.position.y
        p.position.z = pose.position.z + dz
        p.orientation = pose.orientation
        return p

    def add_collision_box(self, name, x, y, z, sx, sy, sz, timeout=2.0):
        """Add a box to the MoveIt planning scene and wait for it to register."""
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.w = 1.0
        self.scene.add_box(name, ps, size=(sx, sy, sz))
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < timeout and not rospy.is_shutdown():
            if name in self.scene.get_known_object_names():
                return True
            rospy.sleep(0.1)
        rospy.logwarn(f"Collision object '{name}' did not register within {timeout}s")
        return False

    def add_collision_scene(self, tag_pose):
        """Add the support shelf and the target cube to the planning scene so
        MoveIt avoids them in transit. The boxes are sized generously (and the
        cube box is tall) to tolerate the mono-camera tag-depth error."""
        cube_center_z = tag_pose.position.z - (self.cube_size / 2.0)
        # Support shelf: a box just below the cube
        self.add_collision_box('support_shelf',
                               tag_pose.position.x, tag_pose.position.y,
                               cube_center_z - self.cube_size / 2.0 - 0.03,
                               0.22, 0.22, 0.05)
        # Target cube: tall box covering the cube column (absorbs depth error)
        self.add_collision_box('target_object',
                               tag_pose.position.x, tag_pose.position.y,
                               cube_center_z + 0.03,
                               self.cube_size + 0.02, self.cube_size + 0.02, 0.16)

    def cartesian_move(self, target_pose, label='cartesian'):
        """Move the end-effector in a straight line to target_pose (used for the
        vertical grasp descent/retreat so we don't sweep into the cube)."""
        rospy.loginfo(f"Cartesian move to {label}: z={target_pose.position.z:.3f}")
        waypoints = [copy.deepcopy(target_pose)]
        # This MoveIt build's signature is (waypoints, eef_step, avoid_collisions).
        (plan, fraction) = self.arm.compute_cartesian_path(waypoints, 0.005, True)
        if fraction > 0.9:
            self.arm.execute(plan, wait=True)
            self.arm.stop()
            return True
        rospy.logwarn(f"Cartesian {label} only reached {fraction:.2f}; falling back to a planned move")
        return self.move_to_pose(target_pose, label)

    def command_gripper(self, close=True):
        """Control the gripper. Prefer the GripperCommand action (drives finger_joint
        to the target position and carries a max_effort hold cap), then the MoveIt
        group, then the Robotiq topic.

        close: True -> close gripper, False -> open gripper
        """
        target_position = self.gripper_closed_position if close else self.gripper_open_position

        # Primary: GripperCommand action -> position_controllers/GripperActionController
        if self.gripper_action is not None:
            try:
                goal = GripperCommandGoal()
                goal.command.position = target_position
                # Carry a non-zero max_effort so the position controller has a
                # holding-torque cap (and the action is well-formed). The gripper is
                # position-controlled, so this bounds the hold, not the squeeze.
                goal.command.max_effort = self.gripper_max_effort
                rospy.loginfo(f"Gripper {'close' if close else 'open'} -> pos={target_position:.3f}, "
                              f"max_effort={goal.command.max_effort:.1f}")
                self.gripper_action.send_goal(goal)
                self.gripper_action.wait_for_result(rospy.Duration(self.gripper_close_wait + 3.0))
                return
            except Exception as e:
                rospy.logwarn(f"GripperCommand action failed: {e}; falling back to MoveIt/topic")

        # Fallback: MoveIt gripper move group (position only)
        if hasattr(self, 'gripper_move_group') and self.gripper_move_group is not None:
            try:
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
