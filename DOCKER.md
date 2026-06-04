# 🐳 Docker — ROS Noetic on Ubuntu 22.04 (and any non-Focal host)

ROS Noetic officially targets **Ubuntu 20.04 (Focal)**. The `docker/` folder
runs the whole UR5 + Robotiq 2F-85 + AprilTag pick-and-place stack inside a
Noetic container so you can develop on **Ubuntu 22.04** (or anything with
Docker).

GUI apps (Gazebo, RViz) render on your host's X server, with GL accelerated
through your **Intel integrated GPU** (`/dev/dri`). No NVIDIA required.

---

## 📦 What's in `docker/`

| File | Purpose |
|------|---------|
| `Dockerfile` | Noetic + MoveIt + Gazebo control + AprilTag + TRAC-IK + UR deps, non-root user |
| `docker-compose.yml` | Workspace mount, X11 + `/dev/dri` passthrough, host networking |
| `entrypoint.sh` | Sources ROS / workspace / Gazebo paths for every command |
| `build_ws.sh` | `rosdep install` + `catkin_make` (run inside the container) |
| `run.sh` | Build image, start container, open a shell |
| `shell.sh` | Open extra shells (you need several terminals) |

The repository root is mounted **as the catkin `src/`** (it already contains the
`CMakeLists.txt` → catkin `toplevel.cmake` symlink), at
`/home/ros/catkin_ws/src` inside the container. Your host edits are live; the
`build/` and `devel/` folders are created in the container and owned by your
host user.

---

## 🚀 Quick start

From the `docker/` folder:

```bash
./run.sh                 # builds the image (first time ~10–15 min) and opens a shell
```

Inside the container, build the workspace **once**:

```bash
build_ws.sh              # rosdep install + catkin_make
```

Then run the demo. Open a **separate terminal per component** with `./shell.sh`:

```bash
# Terminal 1 — Gazebo + MoveIt + RViz
roslaunch icl_ur5_setup_gazebo ur5_gripper_simulation.launch

# Terminal 2 — AprilTag detection
roslaunch icl_ur5_setup_bringup apriltag.launch

# Terminal 3 — Pick and place
roslaunch icl_ur5_setup_bringup pick_and_place.launch
```

Stop everything:

```bash
docker compose down      # from the docker/ folder
```

---

## 🖥️ GUI / GPU notes

`run.sh` runs `xhost +local:root` so the container can reach your X server. If a
window still fails to open:

```bash
echo $DISPLAY            # should be :0 (or :1) on the host
xhost +local:root        # re-authorize
```

The compose file passes `/dev/dri` and adds the host's `video` (GID 44) and
`render` (GID 110) groups. These GIDs were detected on **this** machine — if you
move to another host, check and update them in `docker/docker-compose.yml`:

```bash
getent group render | cut -d: -f3   # render GID
getent group video  | cut -d: -f3   # video GID
```

Software rendering fallback (slow, but always works) if GL misbehaves:

```bash
export LIBGL_ALWAYS_SOFTWARE=1       # set inside the container before launching
```

---

## 🔧 Common tasks

```bash
# Rebuild the image after changing the Dockerfile
docker compose build

# Open another shell (sim/apriltag/task each want their own)
./shell.sh

# Rebuild the catkin workspace after adding packages
build_ws.sh

# Check the camera feed
rqt_image_view                       # topic: /ur5/realsense/camera/image_raw

# Confirm the tag is detected
rostopic echo /tag_detections
rosrun rqt_tf_tree rqt_tf_tree       # look for the tag_0 frame
```

---

## ✅ Project fixes applied

- **Texture path made portable.** `icl_ur5_setup.world` no longer hard-codes an
  absolute host path; its material `<uri>` entries are now relative
  (`materials/scripts`, `materials/textures`) and resolve via
  `GAZEBO_RESOURCE_PATH`, which `icl_ur5_setup_description` populates through
  `<gazebo_ros gazebo_media_path>` in its `package.xml`. That export only takes
  effect if the package **depends on** `gazebo_ros` — it didn't, so an
  `<exec_depend>gazebo_ros</exec_depend>` was added (verified with
  `rospack plugins --attrib=gazebo_media_path gazebo_ros`). Works in and out of
  Docker.
- **Dependencies declared.** `icl_ur5_setup_bringup/package.xml` now lists its
  real deps (apriltag_ros, moveit_commander, tf2, robotiq_2f_gripper_control,
  trac_ik, …) so `rosdep install` resolves them. The image still bakes the heavy
  ones for fast builds.
- **Camera consolidated + RGB-D.** The camera is now defined once, in
  `realsense.xacro` (a `realsense_camera` macro the robot xacro includes),
  instead of being duplicated inline. It uses **two co-located sensors**: a plain
  RGB camera (publishes `camera/image_raw` + `camera/camera_info`, which AprilTag
  needs) and a depth sensor (`camera/depth/image_raw` + `camera/depth/points`).
  A single depth plugin was tried first but only publishes `camera_info` when the
  depth stream is consumed, so AprilTag received 0 `camera_info` and never
  detected — hence the dedicated RGB sensor.
- **Pickable cube + grasp plugin.** `icl_ur5_setup.world` now has a **dynamic**
  cube on a thin support shelf (it was `<static>true</static>` and un-graspable),
  with stabilized contact physics. The Dockerfile builds `gazebo_grasp_fix`
  (JenniferBuehler/gazebo-pkgs, no Noetic binary) into `/opt/grasp_ws` and puts
  it on `GAZEBO_PLUGIN_PATH`, so the gripper can hold objects. The pick-and-place
  node is collision-aware (adds the shelf/cube to the planning scene, descends
  Cartesian) and has a startup settle + retry to avoid the first-move
  `CONTROL_FAILED`.
  > ⚠️ Reliably *gripping and lifting* the 5 cm cube with the position-controlled
  > Robotiq 85 in Gazebo classic is a known-hard problem. The full motion runs
  > cleanly over the dynamic cube without flinging it, but a solid lift needs
  > tuning of `grasp_z_offset` / `gripper_closed_position` (or an effort-controlled
  > gripper). The infrastructure (dynamic object, shelf, grasp_fix, collision-aware
  > approach) is all in place.

## ⚠️ Note on ROS Noetic being EOL

Noetic reached end-of-life in **May 2025**, so current `rosdep` skips it by
default and resolves nothing. The image and `build_ws.sh` always run
`rosdep update --include-eol-distros --rosdistro noetic`, so this is handled
for you — just be aware if you run `rosdep` manually. The `packages.ros.org`
apt repo still hosts the `ros-noetic-*` binaries.

See the repository root `README.md` for the full system architecture, topics,
TF tree, and troubleshooting.
