# 🤖 UR5 + Robotiq 85 Gripper - Gazebo Simülasyonu

<div align="center">

![ROS](https://img.shields.io/badge/ROS-Noetic-blue?style=flat-square&logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-11-orange?style=flat-square)
![MoveIt](https://img.shields.io/badge/MoveIt-1.1-green?style=flat-square)
![Python](https://img.shields.io/badge/Python-3.8+-yellow?style=flat-square&logo=python)

*UR5 robot kolu, Robotiq 85 gripper ve AprilTag tabanlı pick-and-place sistemi*

[🚀 Hızlı Başlangıç](#-hızlı-başlangıç) • [📦 Kurulum](#-kurulum) • [📁 Dosya Yapısı](#-dosya-yapısı) • [⚙️ Konfigürasyon](#-konfigürasyon) • [🔧 Sorun Giderme](#-sorun-giderme)

</div>

---

## 📋 İçindekiler

- [Genel Bakış](#genel-bakış)
- [Hızlı Başlangıç](#-hızlı-başlangıç)
- [Kurulum](#-kurulum)
- [Dosya Yapısı](#-dosya-yapısı)
- [Launch Komutları](#-launch-komutları)
- [Konfigürasyon](#-konfigürasyon)
- [Sistem Mimarisi](#-sistem-mimarisi)
- [TF Frame Ağacı](#-tf-frame-ağacı)
- [Sorun Giderme](#-sorun-giderme)

---

## 🎯 Genel Bakış

Bu proje **UR5 robot kolu** ve **Robotiq 85 gripper**'ı Gazebo simülatöründe kontrol etmek için tasarlanmıştır. Sistem:

- ✅ **MoveIt** ile motion planning
- ✅ **RViz** ile gerçek zamanlı görselleştirme
- ✅ **AprilTag** tabanlı nesne tespiti
- ✅ **ROS Control** ile controller yönetimi
- ✅ Simülasyon ve donanım desteği

### 🎬 Simülasyon Akışı

```
┌─────────────────────────────────────────────────┐
│  1. Gazebo Dünyası Yüklenir                     │
│  2. Robot Modeli Spawn Edilir                   │
│  3. Controller'lar Başlatılır                   │
│  4. MoveIt Planning Scene Hazırlanır            │
│  5. RViz Görselleştirmesi Açılır                │
└─────────────────────────────────────────────────┘
```

---

## 🚀 Hızlı Başlangıç

### Sistem Gereksinimleri

```bash
# ROS Noetic kurulu olmalı
# Gazebo 11.x
# MoveIt 1.1+
# Python 3.8+
```

### Temel Komutlar

#### 1️⃣ Simülasyon Ortamını Başlat

```bash
roslaunch icl_ur5_setup_gazebo ur5_gripper_simulation.launch
```

**Bu komut:**
- Gazebo simülatörünü açar
- Robot modelini yükler ve spawn eder
- MoveIt planning node'unu başlatır
- RViz görselleştirmesini açar
- Controller'ları başlatır

#### 2️⃣ AprilTag Detection Başlat

```bash
roslaunch icl_ur5_setup_bringup apriltag.launch
```

#### 3️⃣ Pick-and-Place Algoritmasını Çalıştır

```bash
rosrun icl_ur5_setup_bringup pick_and_place_task.py
```

---

## 📦 Kurulum

```bash
# Workspace'i oluştur
mkdir -p ~/ur5_ws/src
cd ~/ur5_ws

# Paketi klonla
git clone https://github.com/mates1414/robotic_arm.git src/

# Bağımlılıkları yükle
rosdep install --from-paths src --ignore-src -r -y

# Build et
catkin_make
source devel/setup.bash
```

---

## 📁 Dosya Yapısı

### 📊 Ana Dizin Yapısı

```
ur5_with_robotiq_gripper/
│
├── 🎮 icl_ur5_setup_gazebo/               # Gazebo Simülasyon
│   ├── launch/
│   │   ├── ur5_gripper_simulation.launch          ⭐ ANA LAUNCH (hepsini başlatır)
│   │   └── controller_utils.launch               # Robot State Publisher + Controllers
│   ├── worlds/
│   │   └── icl_ur5_setup.world                   # Gazebo simülasyon dünyası (AprilTag küp)
│   └── config/
│       ├── arm_controller_ur5.yaml               # UR5 trajectory controller config
│       ├── gripper_controller_robotiq.yaml       # Gripper controller config
│       ├── joint_state_controller.yaml           # Joint state publisher config
│       └── pid_gains.yaml                        # PID kontrol parametreleri
│
├── 🤖 icl_ur5_setup_description/          # Robot Tanım (URDF/XACRO)
│   ├── robots/
│   │   ├── ur5_robotiq_85_joint_limited.xacro   # Ana robot montaj (UR5 + Gripper)
│   │   └── ...
│   ├── urdf/
│   │   ├── robotiq_arg2f_85_model_macro.xacro   # Robotiq 85 gripper tanımı
│   │   ├── realsense.xacro                      # RealSense kamera (optical frame)
│   │   └── ...
│   └── meshes/
│       └── [3D model dosyaları]
│
├── 📐 icl_ur5_setup_moveit_config/        # MoveIt Konfigürasyonu
│   ├── launch/
│   │   ├── move_group.launch                    # MoveIt planning node
│   │   ├── moveit_rviz.launch                   # RViz başlatması
│   │   ├── trajectory_execution.launch.xml      # Trajectory execution
│   │   ├── planning_context.launch              # Planning scene
│   │   ├── ur5_gripper_moveit_controller_manager.launch.xml
│   │   └── ...
│   └── config/
│       ├── ur5_gripper.srdf                     # Semantic robot (planning groups)
│       ├── kinematics.yaml                      # IK solver (TRAC-IK)
│       ├── joint_limits.yaml                    # Joint limitleri
│       ├── ompl_planning.yaml                   # OMPL planner parametreleri
│       ├── controllers.yaml                     # ROS controller interface
│       └── moveit.rviz                          # RViz default konfigürasyonu
│
├── 🎯 icl_ur5_setup_bringup/              # Pick-and-Place & AprilTag
│   ├── launch/
│   │   └── apriltag.launch                      # AprilTag detection başlatması
│   ├── node/
│   │   ├── pick_and_place_task.py               # Ana pick-and-place algoritması
│   │   └── default_pick_and_place.py            # Test pick-and-place (sabit koord.)
│   └── config/
│       ├── tags.yaml                            # AprilTag tanımları (ID, size)
│       └── settings.yaml                        # Detection parametreleri
│
└── 📚 universal_robot/                    # UR5 Robot Driver (external)
    └── ur_description/
        └── urdf/
            ├── ur5.xacro                        # UR5 kolu URDF tanımı
            └── common.gazebo.xacro             # Gazebo plugin'leri
```

---

## ⭐ Launch Dosyası Analizi: `ur5_gripper_simulation.launch`

### 🔗 Include Zinciri

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

### 📝 Launch Parametreleri

| Parametre | Default | Açıklama |
|-----------|---------|----------|
| `limited` | `true` | Joint limitlerini kullan |
| `paused` | `false` | Simülasyonun başladığında duraklatılı olması |
| `use_sim_time` | `true` | Gazebo simülasyon saatini kullan |
| `gui` | `true` | Gazebo GUI'sini aç |
| `headless` | `false` | Grafiksel arayüz olmadan çalış |
| `debug` | `false` | Debug modunda gdb ile çalıştır |
| `sim` | `true` | Simülasyon modu (MoveIt için) |

### ⚙️ Başlatılan Bileşenler

| Bileşen | Türü | Görev |
|---------|------|-------|
| **Gazebo Server** | Simülatör | Fizik simülasyonu |
| **Gazebo Client** | GUI | 3D görselleştirme |
| **Robot State Publisher** | Node | TF yayınlama |
| **Joint State Controller** | Controller | Joint state'lerini /joint_states'e publish |
| **Arm Controller** | Trajectory Controller | UR5 kolu hareketi |
| **Gripper Controller** | Position Controller | Robotiq gripper kontrolü |
| **MoveIt Node** | Planning | Motion planning ve execution |
| **RViz** | Görselleştirme | Planned trajectory gösterim |

---

## 🎮 Launch Komutları

### 1. Tam Simülasyon (Önerilen)

```bash
roslaunch icl_ur5_setup_gazebo ur5_gripper_simulation.launch
```

**Başlatan:**
- Gazebo + GUI
- Robot model
- MoveIt
- RViz

---

### 2. Simülasyon Başlangıç Parametreleri

```bash
# Grafiksel arayüz olmadan (headless mode)
roslaunch icl_ur5_setup_gazebo ur5_gripper_simulation.launch headless:=true

# Başlangıçta duraklatılı
roslaunch icl_ur5_setup_gazebo ur5_gripper_simulation.launch paused:=true

# Debug modunda (gdb ile)
roslaunch icl_ur5_setup_gazebo ur5_gripper_simulation.launch debug:=true

# Simulation saati kullanmadan
roslaunch icl_ur5_setup_gazebo ur5_gripper_simulation.launch use_sim_time:=false
```

---

### 3. AprilTag Detection

```bash
# AprilTag detection başlat
roslaunch icl_ur5_setup_bringup apriltag.launch

# Parametrelerle
roslaunch icl_ur5_setup_bringup apriltag.launch \
  camera_frame:=realsense_color_optical_frame \
  tag_size:=0.05
```

---

### 4. Pick-and-Place Node

```bash
# Varsayılan parametrelerle
rosrun icl_ur5_setup_bringup pick_and_place_task.py

# Özel parametrelerle (rosparam ile)
rosparam set /pick_and_place_task/grasp_z_offset -0.02
rosparam set /pick_and_place_task/approach_height 0.15
rosrun icl_ur5_setup_bringup pick_and_place_task.py
```

---

## ⚙️ Konfigürasyon

### 🎛️ Controller Konfigürasyonları

#### `arm_controller_ur5.yaml`
```yaml
# UR5 robot kolu trajectory controller
# Parametreler:
#   - type: JointTrajectoryController
#   - joints: [shoulder_pan_joint, shoulder_lift_joint, ...]
#   - action_monitor_rate: 10
#   - constraints: Joint accuracy limitleri
```

#### `gripper_controller_robotiq.yaml`
```yaml
# Robotiq 85 gripper position controller
# Parametreler:
#   - type: EffortJointInterface
#   - joint: finger_joint
#   - pid: PID kontrol parametreleri
```

#### `joint_state_controller.yaml`
```yaml
# Joint state'lerini /joint_states topic'ine publish eder
# Publish rate: 50 Hz (varsayılan)
```

---

### 🗺️ MoveIt Konfigürasyonları

#### `ur5_gripper.srdf`
- **Planning Groups:**
  - `manipulator`: UR5 kolunun 6 joint'i
  - `gripper`: Robotiq gripper finger_joint
  - `ur5_gripper_group`: Tamamı

- **End-effector:** `gripper` (tool0 üzerine monte)

#### `kinematics.yaml`
```yaml
manipulator:
  kinematics_solver: trac_ik_kinematic_solver/TRAC_IKKinematicPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
  solve_type: Distance
```

#### `joint_limits.yaml`
- Tüm joint'ler için hız/ivme limitleri
- Gripper finger_joint limitleri

#### `ompl_planning.yaml`
- **Planner:** RRT (default)
- **Sampling:** Uniform
- **Optimized:** RRT*

---

### 📷 Kamera Konfigürasyonu

**Dosya:** `icl_ur5_setup_description/robots/ur5_robotiq_85_joint_limited.xacro`

```xml
<!-- Camera mount -->
<joint name="realsense_joint" type="fixed">
  <parent link="wrist_3_link"/>
  <child link="realsense_link"/>
  <origin xyz="0 0.06 0.01" rpy="0.0 -1.5708 1.5708"/>
</joint>

<!-- Gazebo plugin -->
<plugin filename="libgazebo_ros_camera.so" name="realsense_camera_controller">
  <robotNamespace>/ur5</robotNamespace>
  <cameraName>realsense</cameraName>
  <imageTopicName>camera/image_raw</imageTopicName>
  <cameraInfoTopicName>camera/camera_info</cameraInfoTopicName>
  <frameName>realsense_link</frameName>
</plugin>
```

---

## 🏗️ Sistem Mimarisi

### 🔄 Veri Akışı

```
┌──────────────────────────────────────────────────────────┐
│                    GAZEBO SİMÜLATÖR                      │
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
        │ (Robot Hareket)│
        └────────────────┘
```

---

### 📡 ROS Topic'leri

| Topic | Tip | Açıklama |
|-------|-----|----------|
| `/joint_states` | `sensor_msgs/JointState` | Tüm joint'lerin konumu/hızı |
| `/ur5/realsense/camera/image_raw` | `sensor_msgs/Image` | Kamera görüntüsü |
| `/ur5/realsense/camera/camera_info` | `sensor_msgs/CameraInfo` | Kamera kalibrasyon bilgisi |
| `/tf` | `tf2_msgs/TFMessage` | TF frame dönüşümleri |
| `/tag_detections` | `apriltag_ros/AprilTagDetectionArray` | AprilTag tespitleri |
| `/arm_controller/follow_joint_trajectory/goal` | `control_msgs/FollowJointTrajectoryActionGoal` | Arm trajectory komutu |
| `/gripper/command` | `std_msgs/Float64` | Gripper position komutu |

---

### 🔌 ROS Services

| Service | Açıklama |
|---------|----------|
| `/move_group/plan_execution/set_parameters` | MoveIt parametreleri ayarla |
| `/gazebo/set_physics_properties` | Fizik parametreleri |
| `/gazebo/get_model_state` | Model pozisyonu sorgula |

---

## 🗺️ TF Frame Ağacı

### Frame Hiyerarşisi

```
world
└── base_link (UR5 tabanı)
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
    │                       └── realsense_link (Kamera)
    │                           └── realsense_color_optical_frame
    │                               └── tag_0 (AprilTag TF)
    │
    └── base_link_inertia
```

### Önemli Frame'ler

| Frame | Açıklama | Parent |
|-------|----------|--------|
| `world` | Gazebo dünya frame'i | - |
| `base_link` | UR5 robot tabanı | `world` |
| `tool0` | End-effector frame (gripper mount) | `wrist_3_link` |
| `realsense_link` | Kamera fiziksel link | `wrist_3_link` |
| `realsense_color_optical_frame` | Kamera optical frame (apriltag ref.) | `realsense_link` |
| `tag_0` | AprilTag frame (detection sonucu) | `realsense_color_optical_frame` |
| `robotiq_arg2f_base_link` | Gripper base | `tool0` |

---

### TF Yayınlama Frekansı

| Component | Frekans | Açıklama |
|-----------|---------|----------|
| `robot_state_publisher` | 50 Hz | URDF'den TF yayını |
| `joint_state_controller` | 50 Hz | /joint_states yayını |
| `Gazebo` | 1000 Hz | Fizik simülasyonu |
| `AprilTag` | 30 Hz | Tag detection |

---

## 🔧 Sorun Giderme

### ❌ Gazebo Açılmıyor

**Hata:**
```
[Err] [World.cc:2214] Unable to read sdf string
```

**Çözüm:**
```bash
# URDF syntax'ı kontrol et
rosrun xacro xacro ur5_robotiq_85_joint_limited.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf

# Duplicate link kontrol
grep -c 'link name="realsense_link"' /tmp/robot.urdf
# Beklenen: 1
```

---

### ❌ "link 'realsense_link' is not unique" Hatası

**Sebep:** URDF'de aynı link iki kez tanımlanmış

**Çözüm:**
1. `ur5_robotiq_85_joint_limited.xacro` aç
2. Duplicate camera tanımlarını bul ve sil
3. Sadece bir `<link name="realsense_link">` kalmalı

---

### ❌ RViz Boş Gözüküyor

**Hata:**
```
[ERROR] Unable to parse URDF from parameter '/robot_description'
[ERROR] Robot model not loaded
```

**Çözüm:**
```bash
# robot_description parametresi kontrol et
rosparam get /robot_description | head -20

# robot_state_publisher açık mı?
rosnode list | grep robot_state_publisher

# node'u restart et
rosnode kill /robot_state_publisher
```

---

### ❌ AprilTag Tespit Edilmiyor

**Hata:** `/tag_detections` boş

**Çözüm:**
```bash
# Kamera görüntüsü geliyor mu?
rosrun rqt_image_view rqt_image_view
# Topic seç: /ur5/realsense/camera/image_raw

# Tag görüntüde net görünüyor mu kontrol et
# Kamera pozisyonunu ayarla: ur5_robotiq_85_joint_limited.xacro

# Gazebo sensor'ü açık mı?
rosservice call /gazebo/get_entity_state '{name: "ur5_gripper"}'
```

---

### ❌ MoveIt Plan Başarısız Oluyor

**Hata:**
```
[ERROR] Solution found but result path has large
```

**Çözüm:**

```bash
# Joint limits kontrol et
rosparam get /robot_description_planning/joint_limits

# IK solver kontrol et
rosparam get /robot_description_kinematics/manipulator

# Plan zamanını artır
rosparam set /move_group/planner_configs/RRTkConfigDefault/range 0.5

# Joint değerleri kontrol et
rostopic echo /joint_states
```

---

### ❌ Gripper Kontrol Edilmiyor

**Hata:**
```
[WARN] Failed to control gripper
```

**Çözüm:**
```bash
# Gripper controller çalışıyor mu?
rosservice call /controller_manager/list_controllers

# Gripper status kontrol et
rosservice call /controller_manager/query_state_interface '{interface_type: EffortJointInterface}'

# Topic manuel test
rostopic pub /gripper/command std_msgs/Float64 -- 0.5
```

---

### ❌ Simulation Saati Senkronizasyonu

**Hata:**
```
[ERROR] TF: Cannot extrapolate into the future
```

**Çözüm:**
```bash
# use_sim_time parametresi kontrol et
rosparam get /use_sim_time

# Gazebo'dan clock yayını gelip gelmediğini kontrol et
rostopic list | grep clock
rostopic echo /clock | head -5
```

---

## 🐛 Debug Komutları

```bash
# TF ağacı görselleştir
rosrun rqt_tf_tree rqt_tf_tree

# Topic'leri izle
rqt_topic

# Node graph'ı gör
rqt_graph

# ROS parametrelerini düzenle
rqt_reconfigure

# Gazebo model state'lerini kontrol et
rosservice call /gazebo/get_model_state '{model_name: "ur5_gripper", reference_frame: "world"}'

# Joint values konrol et
rostopic echo /joint_states

# Controller status
rostopic echo /arm_controller/state
```

---

## 📚 Dosya Referansları

### Önemli Yapılandırma Dosyaları

```
icl_ur5_setup_gazebo/
├── config/
│   ├── arm_controller_ur5.yaml           ← UR5 trajectory controller
│   ├── gripper_controller_robotiq.yaml   ← Gripper PID controller
│   └── joint_state_controller.yaml       ← Joint state publisher
│
└── worlds/
    └── icl_ur5_setup.world               ← AprilTag küp konumu (0.4, 0.1, 1.4)
```

### URDF/XACRO Dosyaları

```
icl_ur5_setup_description/
├── robots/
│   └── ur5_robotiq_85_joint_limited.xacro  ← Ana montaj (camera + gripper)
│
└── urdf/
    ├── robotiq_arg2f_85_model_macro.xacro
    └── realsense.xacro
```

### MoveIt Konfigürasyonu

```
icl_ur5_setup_moveit_config/
├── config/
│   ├── ur5_gripper.srdf                  ← Planning groups, end-effector
│   ├── kinematics.yaml                   ← TRAC-IK solver
│   ├── joint_limits.yaml                 ← Joint hız/ivme limitleri
│   └── ompl_planning.yaml                ← Planner parametreleri
│
└── launch/
    ├── move_group.launch                 ← MoveIt planning node
    └── moveit_rviz.launch                ← RViz launch
```

---

## 📖 Kaynak Kodlar

- **Pick-and-Place:** `icl_ur5_setup_bringup/node/pick_and_place_task.py`
- **AprilTag Konfigürasyon:** `icl_ur5_setup_bringup/config/tags.yaml`
- **Simülasyon Dünyası:** `icl_ur5_setup_gazebo/worlds/icl_ur5_setup.world`

---

## 🔗 Harici Kaynaklar

- [UR5 Robot Driveri](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
- [Robotiq Gripper ROS](https://github.com/ros-industrial/robotiq)
- [MoveIt Dokümantasyonu](https://moveit.ros.org/)
- [Gazebo ROS Control](http://gazebosim.org/tutorials?tut=ros_control)
- [AprilTag ROS](https://github.com/AprilRobotics/apriltag_ros)

---

## 📝 Lisans

Bu proje eğitim ve araştırma amaçlı geliştirilmiştir.

---

<div align="center">

**Sorularınız için:** [Issues](https://github.com/mates1414/robotic_arm/issues) bölümünü kullanın

**Son güncelleme:** 29 Ekim 2025

</div>
