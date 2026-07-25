# Reliable Grasp Implementation Plan — UR5 + Robotiq 2F-85 in Gazebo Classic

## Context

The pick-and-place stack works end-to-end **except the physical grasp**: the gripper
either **flings the 5 cm cube away** on contact (worst case observed: ~7.7 m) or
**brushes past it without lifting** (best case: cube nudged ~5 cm, never lifted).
Everything else — AprilTag detection, MoveIt planning, Cartesian descent, the
first-move settle/retry, and a dynamic cube that is stable on its own — is verified
working.

This plan fixes the grasp with **targeted changes to existing files** (no architecture
rewrite). It keeps **position control** (effort control was already tried and fails:
the `roboticsgroup` mimic-joint plugin force-sets the coupled joints' positions while
`finger_joint` floats under torque, so single-joint effort control never tracks).

### Why the grasp fails (root causes)

| Failure | Root cause |
|---------|-----------|
| **Fling** | High-energy contact impulse when the fingers (Gazebo *default* surface: hard, bouncy, untuned) slam into the cube — amplified by **overclose** (0.78 rad drives the fingers ~30 mm *past* a 50 mm cube into its body) and a still-high `contact_max_correcting_vel=10`. `grasp_fix` with `grip_count_threshold=1 @ 15 Hz` can also attach mid-impact, freezing a body that already has ejection velocity. |
| **Brush-past (no lift)** | `disable_collisions_on_attach=true` means once `grasp_fix` attaches the fingers stop colliding — so if attach fires on a single-sided graze (threshold=1) the cube is "captured" with no real grip; or if friction is too low / contact too stiff the cube squirts out before a two-sided contact ever registers, so attach never fires and the gripper closes on air. |

### Three verified structural gaps

1. **Finger pads have NO friction/contact tuning.** `inner_finger` and `finger_tip`
   links in `robotiq_arg2f_85_model_macro.xacro` have no `<gazebo reference>` surface
   block — they grip with Gazebo defaults while the cube is carefully tuned.
2. **`pid_gains.yaml` is commented out** in `ur5_gripper_simulation.launch` (line 38),
   so the `PositionJointInterface` `finger_joint` has **no p-gain loaded** (runtime prints
   "No p gain specified for pid … finger_joint"). The mimic plugin's coupled joints need
   this gain to behave deterministically.
3. **Overclose:** the node commands `gripper_closed_position = 0.78` for *any* object;
   for a 5 cm cube the correct close-to-width angle is ~0.30–0.40 rad.

---

## The single most-likely fix (if only one change)

**Close-to-width (Layer 3): command ~0.35 rad instead of 0.78.** Overclose is the
dominant fling driver. But on its own it converts many flings into brush-pasts, so apply
the stabilization layers (1, 2, 5) first, then tune close-to-width.

## Recommended ordering for iterative tuning

1. **Layers 5 + 1 + 2 together** — foundational stabilizers (impulse cap, finger
   friction, finger PID). Pure stabilization, safe, no intent change.
2. **Layer 3** — close-to-width. The behavioral fix; tune live.
3. **Layer 4** — `grasp_fix` timing, so attach fires only on a stable two-sided grip.
4. **Layer 6** — node sequence/timing + grip verification.
5. **Layer 7** — optional MoveIt `attach_box` polish.

---

## Layer 1 — Finger-pad friction & contact

**File:** `ur5_with_robotiq_gripper/icl_ur5_setup_description/urdf/robotiq_arg2f_85_model_macro.xacro`

Add a `<gazebo reference>` surface block **inside the `inner_finger` macro** (after its
`</link>`, ~line 126) and **inside the `finger_tip` macro** (after its `</link>`, ~line
215). In-macro placement emits a correctly-prefixed block for left *and* right
automatically. Both links are tagged because the `finger_tip` box (`0.022 × 0.00635 ×
0.0375`) is the contact geometry but is fixed-jointed → **lumped into `inner_finger`** by
Gazebo, so whichever collision the solver evaluates carries the tuned surface.

```xml
<gazebo reference="${prefix}${fingerprefix}_inner_finger">
  <mu1>1.2</mu1>
  <mu2>1.2</mu2>
  <kp>3.0e4</kp>
  <kd>10.0</kd>
  <minDepth>0.0015</minDepth>
  <maxVel>0.05</maxVel>
  <fdir1>0 0 0</fdir1>
</gazebo>
<!-- and the identical block with reference="${prefix}${fingerprefix}_finger_tip" -->
```

**Values & why (all tunable):**
- `mu1=mu2=1.2` — parallel-jaw needs high tangential friction; making the pad higher than
  the cube's `mu=1.0` ensures the pad/cube pair is limited by the cube, not the pad.
- `kp=3.0e4` (softer than the cube's `1e5`) — **a softer pad absorbs the closing impact
  instead of rebounding it.** Primary anti-fling lever on the finger side. Too soft (<1e4)
  → pad visibly sinks/jitters.
- `kd=10.0` — damps the normal-direction bounce at contact (cube uses 1.0; the pad is the
  impacting body and wants more). Raise toward 50–100 if a bounce remains.
- `minDepth=0.0015` — force only applies past this penetration → kills contact chatter/jitter
  that wastes grip and micro-launches a light object.
- `maxVel=0.05` — local per-contact correcting-velocity cap, exactly where the cube is hit.
- `fdir1=0 0 0` — let Gazebo pick the friction direction (avoid anisotropic-friction bugs).

---

## Layer 2 — Load the finger PID

**File:** `ur5_with_robotiq_gripper/icl_ur5_setup_gazebo/launch/ur5_gripper_simulation.launch` (line 38)

Uncomment the load so the `PositionJointInterface` finger gets a gain:
```xml
<rosparam file="$(find icl_ur5_setup_gazebo)/config/pid_gains.yaml" command="load"/>
```

**File:** `ur5_with_robotiq_gripper/icl_ur5_setup_gazebo/config/pid_gains.yaml` (line 9)

Give the finger a gentle, *damped* gain (currently `{p:1.0, i:0, d:0}`):
```yaml
finger_joint: {p: 8.0, i: 0.0, d: 0.3, i_clamp: 0.0}
```

**Why:** under `gazebo_ros_control` the per-joint PID turns position error → effort each
tick. `p` sets squeeze firmness; with overclose removed (Layer 3) the steady-state error at
the cube face is small, so `p≈8` gives firm-but-not-violent (range 5–15). `d=0.3` opposes
finger velocity → **decelerates the fingers before impact** (PID-side anti-impact lever).
`i=0` so integral windup against the hard contact can't slowly crush/eject. The `effort=1000`
joint limit and the action's `max_effort=40` stay as hard ceilings. `libgazebo_ros_control.so`
runs in the **root** namespace, so it reads `gazebo_ros_control/pid_gains/finger_joint` — the
exact namespace this file uses.

---

## Layer 3 — Close-to-width gripper command

**Files:** `icl_ur5_setup_bringup/node/pick_and_place_task.py`, `icl_ur5_setup_bringup/launch/pick_and_place.launch`

Add a **new dedicated param** `gripper_grasp_position` (distinct from
`gripper_closed_position`, which stays for "close on nothing" semantics). Cleaner and more
tunable than computing inline from `cube_size`.

In `__init__` (~line 54):
```python
self.gripper_grasp_position = rospy.get_param('~gripper_grasp_position', 0.35)
```

In `command_gripper(...)` (line 382) add a `grasp` intent:
```python
def command_gripper(self, close=True, grasp=False):
    if grasp:
        target_position = self.gripper_grasp_position
    else:
        target_position = self.gripper_closed_position if close else self.gripper_open_position
```

At the pick site (line 181): `self.command_gripper(close=True, grasp=True)`.

Add to `pick_and_place.launch`: `<param name="gripper_grasp_position" value="0.35"/>`.

**Value:** `0.35` rad for the 5 cm cube, **to be tuned live**. Heuristic
`pos ≈ 0.78·(1 − 50/85) ≈ 0.32` for 50 mm; command **slightly tighter** (0.35) so
contact + friction + `grasp_fix` engage. Tuning band 0.30–0.42. Keep
`gripper_open_position = 0.0`.

**Risk (flagged):** after attach, collisions are disabled, so the controller keeps driving
toward the setpoint harmlessly — *unless* the commanded angle is large and attach fires
late, letting the fingers accelerate and punt the cube in the pre-attach instant.
Close-to-width removes that energy; keep the grasp angle modest.

---

## Layer 4 — grasp_fix tuning

**File:** `ur5_with_robotiq_gripper/icl_ur5_setup_description/robots/ur5_robotiq_85_joint_limited.xacro` (plugin block ~lines 70–90)

| Param | Now → New | Why |
|-------|-----------|-----|
| `grip_count_threshold` | 1 → **3** | Attach only after **sustained two-sided** contact, never mid-impact or on a one-sided graze. time-to-attach ≈ threshold/update_rate. |
| `update_rate` | 15 → **20** | Keeps time-to-attach low (3/20 = 150 ms) despite the higher threshold — rejects impact transients, commits well before retreat. |
| `max_grip_count` | 3 → **6** | Counter ceiling ≈ 2× threshold, so the latch rides out brief contact dropouts during the lift without releasing. |
| `release_tolerance` | 0.008 → **0.012** | Soft pads (Layer 1) let the fingers settle slightly after attach; too tight a tolerance triggers a spurious early release (drops cube mid-transport). Don't over-loosen or it won't release at place. |

Leave `forces_angle_tolerance=120`, the link names, `contact_topic`, and
`disable_collisions_on_attach=true` unchanged — all correct.

**Pass criterion:** the grasp_fix event topic prints *attached* **after** the fingers are
visibly in contact on both sides, not during descent or at the instant of touch.

---

## Layer 5 — Contact impulse / physics

**File:** `ur5_with_robotiq_gripper/icl_ur5_setup_gazebo/worlds/icl_ur5_setup.world` (`<physics>` block ~lines 17–33)

| Param | Now → New | Why |
|-------|-----------|-----|
| `contact_max_correcting_vel` | 10.0 → **2.0** | The biggest *global* fling lever. Caps solver velocity while resolving penetration. For a 0.2 kg cube, 1.0–5.0 is recommended; drop to 1.0 if any twitch-off remains. |
| `erp` | 0.2 → keep (drop to **0.1** only if fling persists) | Lower ERP = gentler per-step correction = less impulsive ejection, at the cost of softer contacts. |
| `cfm` | 0 → **1e-5** (optional) | Tiny non-zero CFM slightly softens constraints → better solver stability / less jitter on the light cube. Keep ≤1e-4. |
| `iters` | 100 → keep (or 150) | 100 is already generous; 150 marginally improves grip stability at CPU cost. |

Leave `sor`, `max_step_size`, `real_time_update_rate`, `contact_surface_layer` unchanged.
Layer 1's local `maxVel=0.05` and this global cap are belt-and-suspenders (local protects
the finger/cube contact; global protects the cube/shelf settle).

---

## Layer 6 — Node sequence / timing + grip verification

**File:** `icl_ur5_setup_bringup/node/pick_and_place_task.py` (`run()` ~lines 160–204)

**A. Settle after descend, before closing** — between `cartesian_move(pick_pose, 'grasp
descend')` (line 178) and `command_gripper` (line 181):
```python
rospy.sleep(0.5)   # let the arm settle before closing (param: grasp_settle_wait)
```
Closing while the wrist still micro-oscillates from the Cartesian stop adds lateral energy
that knocks the cube.

**B. Descend height** — keep `grasp_z_offset` (launch line 25) as a live knob; start 0.02,
increase toward **0.03–0.04** if the finger tips bottom on the cube top (pushing it into the
shelf → squirt). The 37.5 mm tip box vs 50 mm cube leaves room to bias the pinch upward for
a cleaner two-sided wrap.

**C. Staged close (fallback if impact persists)** — command ~0.25 (just-touching),
`rospy.sleep(0.4)`, then the final grasp angle ~0.35. Lets `grasp_fix` register first contact
and (disable-on-attach) drop finger/cube collision before final tightening. Implement only
if needed.

**D. Grip verification before lifting (important)** — before
`cartesian_move(retreat_pose, 'retreat')` (line 185):
- Add a read-only helper calling `/gazebo/get_model_state` (`gazebo_msgs/GetModelState`,
  `model_name=apriltag_cube`, `relative_entity_name=world`).
- After closing, do a small partial retreat (+3 cm), query cube z. If it rose by ≈ the
  retreat amount (±1 cm) → grip confirmed, continue. If not → **reopen**
  (`command_gripper(close=False)`), re-descend, retry up to N times, then abort.
- Cheaper signal: read `finger_joint` from `/joint_states` after closing — **stalled short**
  of the grasp angle ⇒ something between the fingers (good); **reached freely** ⇒ closed on
  air (bad). Strongest single signal = the `grasp_fix` attach event. Gate the lift on **attach
  event OR the get_model_state lift-check**.

**E.** Keep `gripper_close_wait=1.5` (covers the ~150 ms attach + action result wait).

---

## Layer 7 — MoveIt attach_box during transport (optional polish)

**File:** `icl_ur5_setup_bringup/node/pick_and_place_task.py`

Add `scene.attach_box(eef_link, 'target_object', touch_links=[finger links])` after a
**confirmed** grip (Layer 6D), before transport — **planning-scene bookkeeping only**;
`grasp_fix` stays the physical mechanism. They don't conflict: `grasp_fix` = physics
attachment; `attach_box` = MoveIt knows the cube rides with the EEF (collision-aware
transport, no phantom obstacle). The node currently `remove_world_object('target_object')`
at line 176 — re-add then attach, or attach a fresh box at the TCP. After release at place:
`remove_attached_object(...)` + `remove_world_object('target_object')`.

**Trade-offs:** Pro = collision-aware transport, clean place approach. Con = if the
planning-scene cube pose is slightly off (mono-camera depth error), MoveIt may see a phantom
collision and block valid plans — mitigate by sizing the attached box exactly `cube_size`
and placing it at the real TCP. Secondary to Layers 1–5; add once the physical grasp is solid.

---

## Verification & tuning loop (Dockerized sim)

**Reset the cube between attempts** (read/write sim state — matches the world spawn pose;
zero the twist so leftover fling velocity doesn't carry over):
```bash
rosservice call /gazebo/set_model_state '{model_state: {model_name: apriltag_cube, \
  pose: {position: {x: 0.4, y: 0.1, z: 1.41}, orientation: {w: 1.0}}, \
  twist: {}, reference_frame: world}}'
```

**Watch during each attempt:**
- `/gazebo/get_model_state` (service) for `apriltag_cube` — z rises with the gripper, ends
  near `place_pose` (0.5, −0.2, 0.5).
- **grasp_fix event topic** (`rostopic list | grep -i grasp`) — *attach* after two-sided
  contact, *release* after the open command. Clearest pass/fail.
- `/joint_states` `finger_joint` — stalled at/below setpoint (object present) vs reached
  freely (air).
- `/gripper/gripper_cmd/result` + `/feedback` — goal accepted / reached / stalled.
- Gazebo terminal — the "No p gain specified" warning must be **gone** after Layer 2.

**Quantitative pass/fail:**
- **Grasp success:** within 2 s of closing, cube z rose ≥ (retreat height − 1 cm) AND no
  horizontal jump > 5 cm. A fling = |Δx| or |Δy| > 0.5 m, or z below the shelf.
- **Place success:** final cube position within **3 cm** of `place_pose` in x/y and within
  **3 cm** of the expected resting z; cube at rest (twist ≈ 0).
- **Full pass:** pick + transport + place + release within 3 cm of target, gripper home, no
  `CONTROL_FAILED`, no fling, across **5/5 consecutive resets**.

**Per-layer test sequence:**
1. After 1+2+5: reset, run; confirm the "No p gain" warning is gone and that even with the
   *old* 0.78 close the cube no longer flies across the room (may still be crushed — expected
   until Layer 3).
2. After 3: tune live `rosparam set /pick_and_place_task/gripper_grasp_position 0.33` until
   captured without ejection; watch finger stall + attach event.
3. After 4: confirm attach fires *after* contact and the cube doesn't release during retreat
   (tune `release_tolerance`).
4. After 6: confirm the settle removes residual knock and the lift-check gates correctly
   (force a miss with too-small a grasp angle → it retries/aborts instead of lifting air).
5. After 7: confirm transport plans succeed and the place approach is collision-clean.

**Live-tunable (no rebuild):** `rosparam set` for `gripper_grasp_position`, `grasp_z_offset`,
`grasp_settle_wait`, `gripper_close_wait`. **Relaunch needed:** `pid_gains.yaml` finger p/d
(loaded at gazebo_ros_control init). **Gazebo restart + xacro re-expansion:** world physics
and URDF friction — get these in the right ballpark first, iterate less often.

---

## Risks & gotchas

- **Attach-before-touch:** `grip_count_threshold` too low → attach on a one-sided graze →
  cube stuck to one finger then dropped. Layer 4 (threshold=3) + Layer 1 friction mitigate;
  verify via attach-event timing.
- **disable_collisions_on_attach + large close angle:** late attach lets fingers accelerate
  and punt the cube pre-attach. Keep the grasp angle modest (Layer 3).
- **Over-loosened `release_tolerance`** → cube won't release at place. Tune in tandem with the
  soft `kp`.
- **PID `p` too high** → re-introduces crushing/ejection; **too low** → brush-past. Interacts
  with the commanded angle — tune together.
- **Soft `kp`** can let the finger box visibly interpenetrate the cube before attach —
  cosmetic, functionally fine; raise toward 6e4 if it bothers.
- **MoveIt attached-box pose error** (mono-depth) can block transport plans — size exactly,
  place at the real TCP.
- **Do NOT** revert finger_joint to effort control — the mimic plugin breaks under single-joint
  effort control. All of the above keeps position control.

---

## Critical files

| File | Layer(s) |
|------|----------|
| `icl_ur5_setup_description/urdf/robotiq_arg2f_85_model_macro.xacro` | 1 — finger friction/contact `<gazebo reference>` blocks |
| `icl_ur5_setup_gazebo/launch/ur5_gripper_simulation.launch` | 2 — uncomment pid_gains.yaml load (line 38) |
| `icl_ur5_setup_gazebo/config/pid_gains.yaml` | 2 — finger_joint gain |
| `icl_ur5_setup_bringup/node/pick_and_place_task.py` | 3, 6, 7 — close-to-width, settle/verify, attach_box |
| `icl_ur5_setup_bringup/launch/pick_and_place.launch` | 3, 6 — `gripper_grasp_position`, `grasp_settle_wait` params |
| `icl_ur5_setup_description/robots/ur5_robotiq_85_joint_limited.xacro` | 4 — grasp_fix grip_count_threshold/update_rate/max_grip_count/release_tolerance |
| `icl_ur5_setup_gazebo/worlds/icl_ur5_setup.world` | 5 — contact_max_correcting_vel + physics |
