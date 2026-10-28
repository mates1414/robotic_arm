#!/usr/bin/env python3
"""Pick-and-place demo with moveit_commander: picks the cube of the ur5_workcell world and puts it down at another
spot. The pick position is fixed (~pick_x, ~pick_y) or, with ~use_apriltag, taken from the AprilTag on the cube as
seen by the wrist camera from the `home` pose. In Gazebo the grasped cube is fixed to the gripper through
/ur5/grasp/attach and released through /ur5/grasp/detach.

  rosrun ur5_robotiq_manipulation pick_place_demo.py [_use_apriltag:=true]
"""

import sys

import moveit_commander
import rospy
import tf2_ros
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty


def top_down_pose(x, y, z):
    """grasp_link pointing straight down (180 deg about the world y axis, the orientation of the `home` pose)"""
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.y = 1.0
    return pose


class PickPlaceDemo:
    def __init__(self):
        self.arm = moveit_commander.MoveGroupCommander("manipulator")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")
        self.arm.set_planning_time(5.0)
        self.arm.set_max_velocity_scaling_factor(0.3)
        self.arm.set_max_acceleration_scaling_factor(0.3)
        rospy.loginfo("planning frame %s, end effector %s", self.arm.get_planning_frame(),
                      self.arm.get_end_effector_link())

        self.attach_pub = rospy.Publisher("/ur5/grasp/attach", Empty, queue_size=1)
        self.detach_pub = rospy.Publisher("/ur5/grasp/detach", Empty, queue_size=1)

    def plan_and_execute(self, group, what):
        success, plan, _, error = group.plan()
        if not success:
            rospy.logerr("planning failed: %s (error code %d)", what, error.val)
            return False
        if not group.execute(plan, wait=True):
            rospy.logerr("execution failed: %s", what)
            return False
        rospy.loginfo("done: %s", what)
        return True

    def go_named(self, group, state):
        group.set_named_target(state)
        return self.plan_and_execute(group, "%s -> %s" % (group.get_name(), state))

    def go_pose(self, pose, what):
        self.arm.set_pose_target(pose)
        return self.plan_and_execute(self.arm, what)

    def go_straight(self, pose, what):
        """Straight-line motion of grasp_link, e.g. the final approach and the lift"""
        trajectory, fraction = self.arm.compute_cartesian_path([pose], 0.005)
        if fraction < 0.99:
            rospy.logerr("cartesian path %s: only %.0f %% feasible", what, fraction * 100.0)
            return False
        # the cartesian path is timed for full speed; slow it down like the planned motions
        trajectory = self.arm.retime_trajectory(self.arm.get_current_state(), trajectory,
                                                velocity_scaling_factor=0.1, acceleration_scaling_factor=0.1)
        if not self.arm.execute(trajectory, wait=True):
            rospy.logerr("execution failed: %s", what)
            return False
        rospy.loginfo("done: %s", what)
        return True

    def close_gripper(self, knuckle_position):
        """Closes to the object's width rather than fully, so that the fingers touch the object without pushing it"""
        self.gripper.set_joint_value_target({"finger_joint": knuckle_position})
        return self.plan_and_execute(self.gripper, "gripper -> %.2f rad" % knuckle_position)

    def send_grasp_command(self, pub, what):
        """Fixes the object to / releases it from the gripper in Gazebo (ur5_robotiq_gazebo_plugins)"""
        deadline = rospy.Time.now() + rospy.Duration(2.0)
        while pub.get_num_connections() == 0 and rospy.Time.now() < deadline:
            rospy.sleep(0.1)
        if pub.get_num_connections() == 0:
            rospy.logerr("nobody listens on %s (is the simulation running?)", pub.name)
            return False
        pub.publish(Empty())
        rospy.sleep(0.3)  # the joint is made in the next physics step
        rospy.loginfo("done: %s", what)
        return True

    def find_tag(self, tag_frame, timeout):
        """Position of the tag in the planning frame, or None"""
        tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tf_buffer)
        try:
            transform = tf_buffer.lookup_transform(self.arm.get_planning_frame(), tag_frame, rospy.Time(0),
                                                   rospy.Duration(timeout))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("no %s in TF (is apriltag.launch running and the tag in view?): %s", tag_frame, e)
            return None
        position = transform.transform.translation
        rospy.loginfo("%s at (%.3f, %.3f, %.3f)", tag_frame, position.x, position.y, position.z)
        return position

    def run(self):
        pick_x = rospy.get_param("~pick_x", 0.5)
        pick_y = rospy.get_param("~pick_y", 0.0)
        place_x = rospy.get_param("~place_x", 0.4)
        place_y = rospy.get_param("~place_y", -0.3)
        grasp_z = rospy.get_param("~grasp_z", 0.03)  # grasp_link height at the grasp (cube center 0.025)
        approach_z = rospy.get_param("~approach_z", 0.20)  # height of the approach / retreat poses
        # finger_joint at the grasp: the pads are ~5 cm apart at 0.37 rad, which just touches the 5 cm cube
        grasp_position = rospy.get_param("~grasp_position", 0.37)

        if not (self.go_named(self.arm, "home") and self.go_named(self.gripper, "open")):
            return False

        if rospy.get_param("~use_apriltag", False):
            # the tag is on the top face: the grasp height stays fixed, x and y come from the detection
            tag = self.find_tag(rospy.get_param("~tag_frame", "tag_0"), timeout=10.0)
            if tag is None:
                return False
            pick_x, pick_y = tag.x, tag.y

        return (
            self.go_pose(top_down_pose(pick_x, pick_y, approach_z), "pre-grasp")
            and self.go_straight(top_down_pose(pick_x, pick_y, grasp_z), "approach")
            and self.close_gripper(grasp_position)
            and self.send_grasp_command(self.attach_pub, "attach")
            and self.go_straight(top_down_pose(pick_x, pick_y, approach_z), "lift")
            and self.go_pose(top_down_pose(place_x, place_y, approach_z), "pre-place")
            and self.go_straight(top_down_pose(place_x, place_y, grasp_z + 0.005), "lower")
            and self.go_named(self.gripper, "open")
            and self.send_grasp_command(self.detach_pub, "detach")
            and self.go_straight(top_down_pose(place_x, place_y, approach_z), "retreat")
            and self.go_named(self.arm, "home")
        )


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("pick_place_demo")
    ok = PickPlaceDemo().run()
    rospy.loginfo("pick and place finished" if ok else "pick and place aborted")
    moveit_commander.roscpp_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
