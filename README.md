# 🤖 UR5 + Robotiq 85 Gripper - Gazebo Simulation

<div align="center">

![ROS](https://img.shields.io/badge/ROS-Noetic-blue?style=flat-square&logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-11-orange?style=flat-square)
![MoveIt](https://img.shields.io/badge/MoveIt-1.1-green?style=flat-square)
![Python](https://img.shields.io/badge/Python-3.8+-yellow?style=flat-square&logo=python)

*UR5 robot arm, Robotiq 85 gripper and an AprilTag-based pick-and-place system*

[🚀 Quick Start](#-quick-start) • [📦 Installation](#-installation) • [📁 File Structure](#-file-structure) • [⚙️ Configuration](#-configuration) • [🔧 Troubleshooting](#-troubleshooting)

</div>

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Quick Start](#-quick-start)
- [Installation](#-installation)
- [File Structure](#-file-structure)
- [Launch Commands](#-launch-commands)
- [Configuration](#-configuration)
- [System Architecture](#️-system-architecture)
- [TF Frame Tree](#️-tf-frame-tree)
- [Troubleshooting](#-troubleshooting)
- [Current Status](#-current-status-of-the-repository)

---

## 🎯 Overview

This project is designed to control a **UR5 robot arm** and a **Robotiq 85 gripper** in the Gazebo simulator. The system provides:

- ✅ **MoveIt** for motion planning
- ✅ **RViz** for real-time visualization
- ✅ **AprilTag**-based object detection
- ✅ **ROS Control** for controller management
- ✅ Simulation and hardware support

### 🎬 Simulation Flow

```
┌─────────────────────────────────────────────────┐
│  1. Gazebo world is loaded                       │
│  2. Robot model is spawned                       │
│  3. Controllers are started                      │
│  4. MoveIt planning scene is prepared            │
│  5. RViz visualization opens                     │
└─────────────────────────────────────────────────┘
```

---

## 🚀 Quick Start

### System Requirements

```bash
# ROS Noetic must be installed
# Gazebo 11.x
# MoveIt 1.1+
# Python 3.8+
```

> 🐳 On **Ubuntu 22.04** (or any non-Focal host) use the ROS Noetic container in
> the `docker/` folder instead of installing directly. See [DOCKER.md](DOCKER.md).

### Basic Commands

#### 1️⃣ Start the Simulation Environment

```bash
roslaunch icl_ur5_setup_gazebo ur5_gripper_simulation.launch
```

**This command:**
- Opens the Gazebo simulator
- Loads and spawns the robot model
- Starts the MoveIt planning node
- Opens the RViz visualization
- Starts the controllers

#### 2️⃣ Start AprilTag Detection

```bash
roslaunch icl_ur5_setup_bringup apriltag.launch
```

#### 3️⃣ Run the Pick-and-Place Algorithm

```bash
# Method 1: Via launch file (RECOMMENDED - parameters preset)
roslaunch icl_ur5_setup_bringup pick_and_place.launch

# Method 2: Override parameters
roslaunch icl_ur5_setup_bringup pick_and_place.launch gripper_tcp_offset:=0.17

# Method 3: Run the node directly (with default parameters)
rosrun icl_ur5_setup_bringup pick_and_place_task.py
```

> 💡 **Tip:** If the gripper collides with the object or does not get close enough,
> adjust the `gripper_tcp_offset` parameter.

---

## 📦 Installation

```bash
# Create the workspace
mkdir -p ~/ur5_ws/src
cd ~/ur5_ws

# Clone the package
git clone https://github.com/mates1414/robotic_arm.git src/

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
catkin_make
source devel/setup.bash
```

---

## 📁 File Structure

### 📊 Main Directory Layout

```
ur5_with_robotiq_gripper/
│
├── 🎮 icl_ur5_setup_gazebo/               # Gazebo Simulation
│   ├── launch/
│   │   ├── ur5_gripper_simulation.launch          ⭐ MAIN LAUNCH (starts everything)
│   │   └── controller_utils.launch               # Robot State Publisher + Controllers
│   ├── worlds/
│   │   └── icl_ur5_setup.world                   # Gazebo simulation world (AprilTag cube)
│   └── config/
│       ├── arm_controller_ur5.yaml               # UR5 trajectory controller config
│       ├── gripper_controller_robotiq.yaml       # Gripper controller config
│       ├── joint_state_controller.yaml           # Joint state publisher config
│       └── pid_gains.yaml                        # PID control parameters
│
├── 🤖 icl_ur5_setup_description/          # Robot Description (URDF/XACRO)
│   ├── robots/
│   │   ├── ur5_robotiq_85_joint_limited.xacro   # Main robot assembly (UR5 + Gripper)
│   │   └── ...
│   ├── urdf/
│   │   ├── robotiq_arg2f_85_model_macro.xacro   # Robotiq 85 gripper definition
│   │   ├── realsense.xacro                      # RealSense camera (optical frame)
│   │   └── ...
│   └── meshes/
│       └── [3D model files]
│
├── 📐 icl_ur5_setup_moveit_config/        # MoveIt Configuration
│   ├── launch/
│   │   ├── move_group.launch                    # MoveIt planning node
│   │   ├── moveit_rviz.launch                   # RViz launch
│   │   ├── trajectory_execution.launch.xml      # Trajectory execution
│   │   ├── planning_context.launch              # Planning scene
│   │   ├── ur5_gripper_moveit_controller_manager.launch.xml
│   │   └── ...
│   └── config/
│       ├── ur5_gripper.srdf                     # Semantic robot (planning groups)
│       ├── kinematics.yaml                      # IK solver (TRAC-IK)
│       ├── joint_limits.yaml                    # Joint limits
│       ├── ompl_planning.yaml                   # OMPL planner parameters
│       ├── controllers.yaml                     # ROS controller interface
│       └── moveit.rviz                          # RViz default configuration
│
├── 🎯 icl_ur5_setup_bringup/              # Pick-and-Place & AprilTag
│   ├── launch/
│   │   └── apriltag.launch                      # AprilTag detection launch
│   ├── node/
│   │   ├── pick_and_place_task.py               # Main pick-and-place algorithm
│   │   └── default_pick_and_place.py            # Test pick-and-place (fixed coords)
│   └── config/
│       ├── tags.yaml                            # AprilTag definitions (ID, size)
│       └── settings.yaml                        # Detection parameters
│
└── 📚 universal_robot/                    # UR5 Robot Driver (external)
    └── ur_description/
        └── urdf/
            ├── ur5.xacro                        # UR5 arm URDF definition
            └── common.gazebo.xacro             # Gazebo plugins
```

---

## ⭐ Launch File Analysis: `ur5_gripper_simulation.launch`

### 🔗 Include Chain

```
ur5_gripper_simulation.launch
├── empty_world.launch (gazebo_ros)
│   └── icl_ur5_setup.world ✓
│
├── ur5_robotiq_85_joint_limited.xacro ✓ (robot_description)
│   ├── ur5.xacro (universal_robot)
│   ├── robotiq_arg2f_85_model_macro.xacro ✓
│   └── realsense.xacro ✓
│
├── controller_utils.launch ✓
│   ├── robot_state_publisher
│   ├── joint_state_controller
│   └── fake_joint_calibration
│
├── arm_controller_ur5.yaml ✓
├── gripper_controller_robotiq.yaml ✓
│
├── move_group.launch ✓ (MoveIt)
│   ├── planning_context.launch
│   ├── ur5_gripper_moveit_controller_manager.launch.xml
│   └── trajectory_execution.launch.xml
│
└── moveit_rviz.launch ✓
    └── moveit.rviz
```

### 📝 Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `limited` | `true` | Use joint limits |
| `paused` | `false` | Start the simulation paused |
| `use_sim_time` | `true` | Use the Gazebo simulation clock |
| `gui` | `true` | Open the Gazebo GUI |
| `headless` | `false` | Run without a graphical interface |
| `debug` | `false` | Run in debug mode with gdb |
| `sim` | `true` | Simulation mode (for MoveIt) |

### ⚙️ Components Started

| Component | Type | Role |
|-----------|------|------|
| **Gazebo Server** | Simulator | Physics simulation |
| **Gazebo Client** | GUI | 3D visualization |
| **Robot State Publisher** | Node | Publishes TF |
| **Joint State Controller** | Controller | Publishes joint states to /joint_states |
| **Arm Controller** | Trajectory Controller | UR5 arm motion |
| **Gripper Controller** | Position Controller | Robotiq gripper control |
| **MoveIt Node** | Planning | Motion planning and execution |
| **RViz** | Visualization | Displays the planned trajectory |

---

## 🎮 Launch Commands

### 1. Full Simulation (Recommended)

```bash
roslaunch icl_ur5_setup_gazebo ur5_gripper_simulation.launch
```

**Starts:**
- Gazebo + GUI
- Robot model
- MoveIt
- RViz

---

### 2. Simulation Start Parameters

```bash
# Without a graphical interface (headless mode)
roslaunch icl_ur5_setup_gazebo ur5_gripper_simulation.launch headless:=true

# Paused at start
roslaunch icl_ur5_setup_gazebo ur5_gripper_simulation.launch paused:=true

# In debug mode (with gdb)
roslaunch icl_ur5_setup_gazebo ur5_gripper_simulation.launch debug:=true

# Without simulation clock
roslaunch icl_ur5_setup_gazebo ur5_gripper_simulation.launch use_sim_time:=false
```

---

### 3. AprilTag Detection

```bash
# Start AprilTag detection
roslaunch icl_ur5_setup_bringup apriltag.launch

# With parameters
roslaunch icl_ur5_setup_bringup apriltag.launch \
  camera_frame:=realsense_color_optical_frame \
  tag_size:=0.05
```

---

### 4. Pick-and-Place Node

```bash
# With default parameters
rosrun icl_ur5_setup_bringup pick_and_place_task.py

# With custom parameters (via rosparam)
rosparam set /pick_and_place_task/gripper_tcp_offset 0.17
rosparam set /pick_and_place_task/grasp_z_offset 0.0
rosparam set /pick_and_place_task/approach_height 0.15
rosrun icl_ur5_setup_bringup pick_and_place_task.py
```

#### 🎯 Pick-and-Place Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `~arm_group` | `manipulator` | MoveIt arm planning group |
| `~gripper_group` | `gripper` | MoveIt gripper group |
| `~base_frame` | `base_link` | Robot base frame |
| `~tag_frame` | `tag_0` | AprilTag frame name |
| `~camera_frame` | `realsense_color_optical_frame` | Camera optical frame |
| `~cube_size` | `0.05` | Target cube size (meters) |
| **`~gripper_tcp_offset`** | **`0.20`** | **Distance from tool0 to the gripper finger tips (m)** ⚠️ |
| `~grasp_z_offset` | `0.02` | Extra Z offset (for fine tuning) |
| `~approach_height` | `0.15` | Approach height (meters) |
| `~place_x/y/z` | `0.5/-0.2/0.5` | Place pose |

> ⚠️ **Important:** `gripper_tcp_offset` represents the physical extent of the
> gripper. This value prevents the gripper from going inside the object. For the
> Robotiq 85 it should be roughly **0.16–0.20 m**.

---

## ⚙️ Configuration

### 🎛️ Controller Configurations

#### `arm_controller_ur5.yaml`
```yaml
# UR5 robot arm trajectory controller
# Parameters:
#   - type: JointTrajectoryController
#   - joints: [shoulder_pan_joint, shoulder_lift_joint, ...]
#   - action_monitor_rate: 10
#   - constraints: Joint accuracy limits
```

#### `gripper_controller_robotiq.yaml`
```yaml
# Robotiq 85 gripper position controller
# Parameters:
#   - type: position_controllers/GripperActionController
#   - joint: finger_joint
# (Effort control was tried and reverted — see the Troubleshooting note.)
```

#### `joint_state_controller.yaml`
```yaml
# Publishes joint states to the /joint_states topic
# Publish rate: 50 Hz (default)
```

---

### 🗺️ MoveIt Configurations

#### `ur5_gripper.srdf`
- **Planning Groups:**
  - `manipulator`: the 6 joints of the UR5 arm
  - `gripper`: Robotiq gripper finger_joint
  - `ur5_gripper_group`: everything

- **End-effector:** `gripper` (mounted on tool0)

#### `kinematics.yaml`
```yaml
manipulator:
  kinematics_solver: trac_ik_kinematic_solver/TRAC_IKKinematicPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
  solve_type: Distance
```

#### `joint_limits.yaml`
- Velocity/acceleration limits for all joints
- Gripper finger_joint limits

#### `ompl_planning.yaml`
- **Planner:** RRT (default)
- **Sampling:** Uniform
- **Optimized:** RRT*

---

### 📷 Camera Configuration

**File:** `icl_ur5_setup_description/urdf/realsense.xacro`

The camera is now defined once in `realsense.xacro` (a `realsense_camera` macro that
the robot xacro includes) instead of being duplicated inline. It uses **two
co-located sensors**: a plain RGB camera (publishes `camera/image_raw` +
`camera/camera_info`, which AprilTag needs) and a depth sensor
(`camera/depth/image_raw` + `camera/depth/points`).

```xml
<!-- Camera mount (on the wrist) -->
<joint name="realsense_joint" type="fixed">
  <parent link="${parent}"/>
  <child link="realsense_link"/>
  <origin xyz="0 0.06 0.01" rpy="0 -1.5708 1.5708"/>
</joint>

<!-- RGB sensor plugin (AprilTag subscribes to these topics) -->
<plugin name="realsense_color_controller" filename="libgazebo_ros_camera.so">
  <robotNamespace>/ur5</robotNamespace>
  <cameraName>realsense</cameraName>
  <imageTopicName>camera/image_raw</imageTopicName>
  <cameraInfoTopicName>camera/camera_info</cameraInfoTopicName>
  <frameName>realsense_color_optical_frame</frameName>
</plugin>
```

---

## 🏗️ System Architecture

### 🔄 Data Flow

```
┌──────────────────────────────────────────────────────────┐
│                    GAZEBO SIMULATOR                       │
│  ┌─────────────┐         ┌──────────────┐                │
│  │ Robot Model │         │ Camera Sensor│                │
│  └─────┬───────┘         └──────┬───────┘                │
│        │                        │                        │
│        │ /joint_states          │ /ur5/realsense/        │
│        │                        │    camera/image_raw    │
└────────┼────────────────────────┼────────────────────────┘
         │                        │
         ▼                        ▼
    ┌─────────────┐          ┌──────────────────┐
    │ Joint State │          │ AprilTag         │
    │ Publisher   │          │ Detection Node   │
    └────┬────────┘          └──────┬───────────┘
         │                          │
         │ TF: /joint_states        │ TF: /tag_0
         │                          │
         └──────────┬───────────────┘
                    │
                    ▼
        ┌──────────────────────────┐
        │  Pick-and-Place Node     │
        │  (pick_and_place_task.py)│
        └──────────┬───────────────┘
                   │
    ┌──────────────┼──────────────┐
    │              │              │
    ▼              ▼              ▼
┌─────────┐  ┌──────────────┐  ┌──────────┐
│ MoveIt  │  │ Robot Pose   │  │ Gripper  │
│ Planning│──│ Calculation  │──│ Command  │
└────┬────┘  └──────────────┘  └────┬─────┘
     │                              │
     │ Trajectory                   │ Gripper Cmd
     │                              │
     └──────────┬───────────────────┘
                │
                ▼
    ┌──────────────────────────┐
    │  ROS Control Managers    │
    │  - arm_controller        │
    │  - gripper_controller    │
    └──────────┬───────────────┘
               │
               ▼
        ┌────────────────┐
        │ Gazebo Physics │
        │ (Robot Motion) │
        └────────────────┘
```

---

### 📡 ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Position/velocity of all joints |
| `/ur5/realsense/camera/image_raw` | `sensor_msgs/Image` | Camera image |
| `/ur5/realsense/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera calibration info |
| `/ur5/realsense/camera/depth/points` | `sensor_msgs/PointCloud2` | Depth point cloud (future use) |
| `/tf` | `tf2_msgs/TFMessage` | TF frame transforms |
| `/tag_detections` | `apriltag_ros/AprilTagDetectionArray` | AprilTag detections |
| `/arm_controller/follow_joint_trajectory/goal` | `control_msgs/FollowJointTrajectoryActionGoal` | Arm trajectory command |
| `/gripper/gripper_cmd/goal` | `control_msgs/GripperCommandActionGoal` | Gripper command (action) |

---

### 🔌 ROS Services

| Service | Description |
|---------|-------------|
| `/move_group/plan_execution/set_parameters` | Set MoveIt parameters |
| `/gazebo/set_physics_properties` | Physics parameters |
| `/gazebo/get_model_state` | Query a model's pose |
| `/controller_manager/list_controllers` | List loaded controllers and their state |

---

## 🗺️ TF Frame Tree

### Frame Hierarchy

```
world
└── base_link (UR5 base)
    ├── shoulder_link
    │   └── upper_arm_link
    │       └── forearm_link
    │           └── wrist_1_link
    │               └── wrist_2_link
    │                   └── wrist_3_link
    │                       ├── tool0 (End-effector)
    │                       │   └── robotiq_arg2f_base_link (Gripper)
    │                       │       ├── left_outer_knuckle
    │                       │       ├── right_outer_knuckle
    │                       │       └── [gripper fingers...]
    │                       │
    │                       └── realsense_link (Camera)
    │                           └── realsense_color_optical_frame
    │                               └── tag_0 (AprilTag TF)
    │
    └── base_link_inertia
```

### Important Frames

| Frame | Description | Parent |
|-------|-------------|--------|
| `world` | Gazebo world frame | - |
| `base_link` | UR5 robot base | `world` |
| `tool0` | End-effector frame (gripper mount) | `wrist_3_link` |
| `realsense_link` | Physical camera link | `wrist_3_link` |
| `realsense_color_optical_frame` | Camera optical frame (apriltag ref.) | `realsense_link` |
| `tag_0` | AprilTag frame (detection result) | `realsense_color_optical_frame` |
| `robotiq_arg2f_base_link` | Gripper base | `tool0` |

> ℹ️ **Note on the z=1.2 spawn offset:** the robot is spawned at `z=1.2` in Gazebo,
> but in the TF tree `world → base_link` is an identity transform. This is correct
> and intentional — `base_link` is the planning root, and everything (camera, cube
> target) is computed relative to `base_link`, not `world`. The Gazebo spawn height
> is a physics-world offset that does not (and should not) appear in TF.

---

### TF Publish Rates

| Component | Rate | Description |
|-----------|------|-------------|
| `robot_state_publisher` | 50 Hz | TF from URDF |
| `joint_state_controller` | 50 Hz | /joint_states |
| `Gazebo` | 1000 Hz | Physics simulation |
| `AprilTag` | 30 Hz | Tag detection |

---

## 🔧 Troubleshooting

> 🐳 If you are on **Ubuntu 22.04** (or any non-Focal system), use the ROS Noetic
> container in the `docker/` folder instead of installing the project directly.
> Details: [DOCKER.md](DOCKER.md).

### 🧪 Issues Found and Fixed During Testing

Issues found while testing the simulation end-to-end, all of which were **fixed**:

| Symptom | Cause | Fix |
|---------|-------|-----|
| Node dies immediately (`exit code 127`) | Python nodes used `#!/usr/bin/env python`; Noetic only has `python3` | Shebangs changed to `python3` |
| `rosdep` resolves no dependencies | Noetic reached EOL (May 2025); current `rosdep` skips EOL distros | `rosdep update --include-eol-distros` (in build_ws.sh) |
| `catkin_make` can't find `soem`/controllers | The container strips the apt index | `build_ws.sh` now runs `apt-get update` first |
| AprilTag not detecting (`/tag_detections` empty) | Camera was not publishing `camera_info` (a single depth plugin does not publish RGB camera_info) | Separate RGB + depth sensors ([realsense.xacro](ur5_with_robotiq_gripper/icl_ur5_setup_description/urdf/realsense.xacro)) |
| Cube not graspable / floating in Gazebo | Cube was `<static>true</static>` with no support beneath it | Cube made dynamic + thin shelf + `gazebo_grasp_fix` plugin |
| First move `ABORTED: CONTROL_FAILED` | First command sent before the robot settles after spawn | Added a settle delay + retry to the node |
| Cube flung across the world at contact | Default contact-physics impulse | `contact_max_correcting_vel` lowered; collision-aware approach; Cartesian descent |

> ⚠️ **About grasping:** reliably **gripping** a small 5 cm cube with the Robotiq 85
> in Gazebo classic is a known-hard problem (position-controlled fingers + contact
> physics). The full infrastructure is in place (dynamic cube, shelf, `grasp_fix`
> plugin, collision-aware approach, Cartesian descent). The motion runs cleanly and
> no longer flings the cube; a firm, repeatable lift still needs tuning of the grasp
> height (`grasp_z_offset`) and the gripper closed value (`gripper_closed_position`).
>
> 🔬 **Effort/force control was tried and reverted:** switching the gripper from
> position to effort control (`EffortJointInterface` +
> `effort_controllers/GripperActionController`) is the standard way to bound the
> squeeze force. However, the Robotiq's **underactuated closed-loop linkage + the
> kinematic mimic-joint plugin** is incompatible with single-joint effort control:
> `finger_joint` does not track the commanded position (the gripper won't close, the
> joint drifts the wrong way even at high effort). Position control was therefore
> restored (close=0.78, open=0.0, tracks precisely). The real fix would be a
> dedicated Robotiq Gazebo controller that models the linkage properly.

### ❌ Gazebo Won't Open

**Error:**
```
[Err] [World.cc:2214] Unable to read sdf string
```

**Fix:**
```bash
# Check the URDF syntax
rosrun xacro xacro ur5_robotiq_85_joint_limited.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf

# Check for duplicate links
grep -c 'link name="realsense_link"' /tmp/robot.urdf
# Expected: 1
```

---

### ❌ "link 'realsense_link' is not unique" Error

**Cause:** the same link is defined twice in the URDF

**Fix:**
1. Open `ur5_robotiq_85_joint_limited.xacro`
2. Find and remove the duplicate camera definitions
3. Only one `<link name="realsense_link">` should remain

---

### ❌ RViz Appears Empty

**Error:**
```
[ERROR] Unable to parse URDF from parameter '/robot_description'
[ERROR] Robot model not loaded
```

**Fix:**
```bash
# Check the robot_description parameter
rosparam get /robot_description | head -20

# Is robot_state_publisher running?
rosnode list | grep robot_state_publisher

# Restart the node
rosnode kill /robot_state_publisher
```

---

### ❌ AprilTag Not Detected

**Error:** `/tag_detections` is empty

**Fix:**
```bash
# Is the camera image coming through?
rosrun rqt_image_view rqt_image_view
# Select topic: /ur5/realsense/camera/image_raw

# ⭐ MOST COMMON CAUSE: camera_info is not being published.
# AprilTag waits for image_rect and camera_info together (synchronized).
# If these don't arrive you see a "Synchronized pairs: 0" warning.
rostopic hz /ur5/realsense/camera/image_raw    # should be ~30 Hz
rostopic hz /ur5/realsense/camera/camera_info  # should be the SAME rate (0 = the problem)

# Is the arm in a pose where the camera can see the cube?
# Is the tag_0 frame appearing?
rosrun tf tf_echo base_link tag_0
```

> 💡 A single `libgazebo_ros_depth_camera.so` plugin only publishes RGB
> `camera_info` once the depth stream is consumed; AprilTag subscribes only to RGB,
> so it receives 0 `camera_info`. That's why the camera is now defined as **two
> separate sensors** (RGB + depth) — see `realsense.xacro`.

---

### ❌ MoveIt Planning Fails

**Error:**
```
[ERROR] Solution found but result path has large
```

**Fix:**

```bash
# Check the joint limits
rosparam get /robot_description_planning/joint_limits

# Check the IK solver
rosparam get /robot_description_kinematics/manipulator

# Increase the planning time / range
rosparam set /move_group/planner_configs/RRTkConfigDefault/range 0.5

# Check the joint values
rostopic echo /joint_states
```

---

### ❌ Gripper Not Controlled

**Error:**
```
[WARN] Failed to control gripper
```

**Fix:**
```bash
# Is the gripper controller running?
rosservice call /controller_manager/list_controllers

# Test the gripper via the action interface
rostopic pub /gripper/gripper_cmd/goal control_msgs/GripperCommandActionGoal \
  "{goal: {command: {position: 0.5, max_effort: 40.0}}}" --once
```

---

### ❌ Simulation Clock Synchronization

**Error:**
```
[ERROR] TF: Cannot extrapolate into the future
```

**Fix:**
```bash
# Check the use_sim_time parameter
rosparam get /use_sim_time

# Check whether Gazebo is publishing the clock
rostopic list | grep clock
rostopic echo /clock | head -5
```

---

## 🐛 Debug Commands

```bash
# Visualize the TF tree
rosrun rqt_tf_tree rqt_tf_tree

# Watch topics
rqt_topic

# See the node graph
rqt_graph

# Edit ROS parameters
rqt_reconfigure

# Check Gazebo model states
rosservice call /gazebo/get_model_state '{model_name: "ur5_gripper", reference_frame: "world"}'

# Check joint values
rostopic echo /joint_states

# Controller status
rostopic echo /arm_controller/state
```

---

## 📚 File References

### Important Configuration Files

```
icl_ur5_setup_gazebo/
├── config/
│   ├── arm_controller_ur5.yaml           ← UR5 trajectory controller
│   ├── gripper_controller_robotiq.yaml   ← Gripper controller
│   └── joint_state_controller.yaml       ← Joint state publisher
│
└── worlds/
    └── icl_ur5_setup.world               ← AprilTag cube position (0.4, 0.1, 1.4)
```

### URDF/XACRO Files

```
icl_ur5_setup_description/
├── robots/
│   └── ur5_robotiq_85_joint_limited.xacro  ← Main assembly (camera + gripper)
│
└── urdf/
    ├── robotiq_arg2f_85_model_macro.xacro
    └── realsense.xacro
```

### MoveIt Configuration

```
icl_ur5_setup_moveit_config/
├── config/
│   ├── ur5_gripper.srdf                  ← Planning groups, end-effector
│   ├── kinematics.yaml                   ← TRAC-IK solver
│   ├── joint_limits.yaml                 ← Joint velocity/acceleration limits
│   └── ompl_planning.yaml                ← Planner parameters
│
└── launch/
    ├── move_group.launch                 ← MoveIt planning node
    └── moveit_rviz.launch                ← RViz launch
```

---

## 📖 Source Code

- **Pick-and-Place:** `icl_ur5_setup_bringup/node/pick_and_place_task.py`
- **AprilTag Configuration:** `icl_ur5_setup_bringup/config/tags.yaml`
- **Simulation World:** `icl_ur5_setup_gazebo/worlds/icl_ur5_setup.world`

---

## 🔗 External Resources

- [UR5 Robot Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
- [Robotiq Gripper ROS](https://github.com/ros-industrial/robotiq)
- [MoveIt Documentation](https://moveit.ros.org/)
- [Gazebo ROS Control](http://gazebosim.org/tutorials?tut=ros_control)
- [AprilTag ROS](https://github.com/AprilRobotics/apriltag_ros)

---

## 📊 Current Status of the Repository

*Last verified: 2026-06-04 (full end-to-end run inside the ROS Noetic Docker container).*

### ✅ Working

| Component | Status | Notes |
|-----------|--------|-------|
| **Docker (Noetic on Ubuntu 22.04)** | ✅ Working | Image builds; X11 + `/dev/dri` GUI passthrough; Intel GPU direct rendering confirmed (`Mesa Intel Xe Graphics`). See [DOCKER.md](DOCKER.md). |
| **Gazebo simulation** | ✅ Working | World, thin support shelf, and a dynamic AprilTag cube spawn correctly. GUI renders. |
| **MoveIt + RViz** | ✅ Working | `move_group` up; RViz opens with the MoveIt display; "You can start planning now!" |
| **Controllers** | ✅ Working | `arm_controller`, `joint_state_controller`, and `gripper` all report **running**. |
| **Camera (RGB-D)** | ✅ Working | `image_raw` and `camera_info` both publish in sync at ~30 Hz (the key fix that lets AprilTag work). Depth point cloud also available for future use. |
| **AprilTag detection** | ✅ Working | `/tag_detections` reports `id: [0]`; the `tag_0` TF resolves within ~5 mm of the cube's true position. |
| **First-move stability** | ✅ Fixed | Startup settle + gripper-hold + retry: the home move now succeeds on the first try (no more `CONTROL_FAILED`). |
| **Full pick-and-place motion** | ✅ Working | Runs end-to-end: home → detect tag → approach → Cartesian descend → close → retreat → place → open → home, with **no errors** ("Pick-and-place task completed"). |

### ⚠️ Known Limitation

| Component | Status | Notes |
|-----------|--------|-------|
| **Firm physical grasp / lift of the cube** | ⚠️ Not solved | The Robotiq 85 in Gazebo classic does not achieve a reliable grip on the 5 cm cube. Observed behavior brackets between two cases: at worst the grasp contact **ejects the cube** (it was flung ~7.7 m in one run); at best the gripper **brushes the cube ~5 cm** without lifting it. The cube is stable on its own (verified motionless for several seconds), so this is a gripper-contact problem, not a world-physics instability. |

### 🔬 What Was Tried for the Grasp

- **Position control (current):** `finger_joint` tracks precisely (close → 0.780, open → 0.000). Reliable for opening/closing, but the contact model doesn't hold the cube firmly.
- **Effort/force control (reverted):** intended to bound the squeeze force, but the Robotiq's underactuated closed-loop linkage + kinematic mimic-joint plugin does not track under single-joint effort control (`finger_joint` drifts the wrong way and the gripper won't close, even at high `max_effort`). Reverted to position control. See the Troubleshooting note above.
- **Infrastructure in place:** dynamic cube, thin support shelf, `gazebo_grasp_fix` plugin (attaches a grasped object via a fixed joint), collision-aware approach (adds shelf + cube to the planning scene), and a straight-down Cartesian descent so the arm doesn't sweep sideways into the cube.

### 🚀 The Two Real Paths Forward for Grasping

1. **A dedicated Robotiq Gazebo controller** that models the underactuated linkage properly (rather than the kinematic mimic-joint hack). This is the "correct" fix but a sizable undertaking.
2. **Accept the current state**: a complete, clean pick-and-place *motion* over a genuinely dynamic cube with all perception, planning, and infrastructure working, treating the firm grasp as out of scope for the simulation.

---

## 📝 License

This project was developed for educational and research purposes.

---

<div align="center">

**For questions:** use the [Issues](https://github.com/mates1414/robotic_arm/issues) section

**Last updated:** 2026-06-04

</div>
