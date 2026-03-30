# JAKA KARGO ROS2 Workspace

ROS 2 workspace for controlling, simulating and planning for the **JAKA KARGO** robot.
This repo contains a low-level driver that wraps the JAKA K1 SDK, a MoveIt server/action bridge, URDF/meshes, and an Isaac Sim integration layer.


## 🏁 Quick start 

**Prereqs:** ROS 2 Humble , compiler with C++17, and JAKA K1 SDK library shipped in this workspace with its header files .

### 🔧 Build Instructions

```bash
git clone git@github.com:JAKARobotics/JAKA_KARGO.git
cd JAKA_KARGO/jaka_kargo_ros2/
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 📁 Workspace Structure

```bash
jaka_kargo_ros2/
├── jaka_kargo_driver            # ROS2 node: wraps JAKA K1 SDK, publishes topics, offers services
├── jaka_kargo_msgs              # custom ROS2 messages & services used by driver
├── jaka_kargo_moveit_config     # MoveIt2 configuration for planning groups (full-robot, left-arm, right-arm, body / external-axis)
├── jaka_kargo_planner           # MoveIt action server / trajectory bridge (FollowJointTrajectory)
├── jaka_kargo_description       # URDFs + meshes + RViz configs
├── jaka_kargo_isaacsim          # Isaac Sim bridge / scripts and launch files
└── README.md
```

### 🚀 Launch Instructions  

#### 1) Start the driver (connects to real controller)

```bash
# adjust IP parameter as needed
ros2 launch jaka_kargo_driver kargo_driver_start.launch.py ip:=10.5.5.100
```

#### 2) Start MoveIt server (action server)

```bash
# adjust IP parameter as needed
ros2 launch jaka_kargo_planner kargo_moveit_server.launch.py ip:=10.5.5.100
```

#### 3) Start MoveIt / RViz demo
```bash
ros2 launch jaka_kargo_moveit_config demo.launch.py
```

## 📖 Package Description

### 1. jaka_kargo_driver

- Node `jaka_kargo_driver` — connects to controller using JAKA's SDK `libjakaAPI_2_3_0_DUAL_9.so` included in **jaka_kargo_driver/lib**, and exposes:

    - Topics
        - `/jaka_kargo_driver/tool_position` (geometry_msgs/TwistStamped) — dual-arm TCP cartesian pose
        - `/jaka_kargo_driver/joint_position` (sensor_msgs/JointState) — full-robot joint feedback: left-arm, right-arm, body / external-axis joints
        - `/jaka_kargo_driver/robot_states` (jaka_kargo_msgs/RobotStates) — generic full-robot state flags

    - Services 
        - `/jaka_kargo_driver/linear_move` — cartesian linear movement (dual/single)
        - `/jaka_kargo_driver/joint_move` — joint movement (dual/single)
        - `/jaka_kargo_driver/servo_move_enable` — enable servo mode
        - `/jaka_kargo_driver/edg_servo_p` — EDG servo in Cartesian (per-arm)
        - `/jaka_kargo_driver/edg_servo_j` — EDG servo in joint space (per-arm)
        - `/jaka_kargo_driver/stop_move` — abort current motion
        - `/jaka_kargo_driver/get_fk` - forward kinematics solution
        - `/jaka_kargo_driver/get_ik` - inverse kinematics solution
        - `/jaka_kargo_driver/drag_mode` - enable free drag mode
        - `/jaka_kargo_driver/set_tool_offset,` `/jaka_kargo_driver/get_tool_offset`
        - `/jaka_kargo_driver/set_tool_payload`, `/jaka_kargo_driver/get_tool_payload`
        - `/jaka_kargo_driver/set_collision_level`, `/jaka_kargo_driver/get_collision_level`
        - `/jaka_kargo_driver/get_default_base`, `/jaka_kargo_driver/clear_error,` `/jaka_kargo_driver/get_sdk_version,` `/jaka_kargo_driver/set_debug_mode`, `/jaka_kargo_driver/get_dh_params`
  
        - `/jaka_kargo_driver/ext_enable` — enable or disable a single external axis
        - `/jaka_kargo_driver/jog_ext` — jog a single external axis
        - `/jaka_kargo_driver/multi_move_ext` — generic multi-axes motion command for external axes and/or robot arms

### 2. jaka_kargo_msgs

- ROS2 message & service definitions used by the `jaka_kargo_driver` package.

### 3. jaka_kargo_planner

- links against `libjakaAPI_2_3_0_DUAL_9.so` included in **jaka_kargo_planner/lib**. 
- Provides `FollowJointTrajectory` action servers for moveit:
    - `/jaka_kargo_full_robot_controller/follow_joint_trajectory` (18 joints — full-robot: 7 left arm + 7 right arm + 4 body / external axis joints)
    - `/jaka_kargo_arm_l_controller/follow_joint_trajectory `(7 joints — left-arm)
    - `/jaka_kargo_arm_r_controller/follow_joint_trajectory` (7 joints — right-arm)
    - `/jaka_kargo_body_controller/follow_joint_trajectory` (4 joints — body / external axis)

- Publishes `/joint_states` — full-robot state expected by MoveIt.

### 4. jaka_kargo_moveit_config

- URDF/XACRO, SRDF, joint limits and MoveIt controller configuration for: **left-arm**, **right-arm**, **body / external-axis**, and **full-robot** controllers.

### 5. jaka_kargo_isaacsim

- Scripts and launch files to run Isaac Sim with the robot USD for simulation + MoveIt integration.

### Example service calls

#### 1. Joint move (right arm only)

```bash
ros2 service call /jaka_kargo_driver/joint_move jaka_kargo_msgs/srv/Move "{
  robot_index: 1,
  is_block: true,
  mode_left: 0,
  mode_right: 0,
  pose_left:  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],   # placeholder (will be filled)
  pose_right: [1.57 ,0 ,0 ,-1.57 ,-1.57 ,1.57 ,0],   # radians
  vel_left: 0.0,
  vel_right: 0.1,
  acc_left: 0.0,
  acc_right: 0.1
}"
```

#### 2. Get Inverse kinematics solution (right arm)

```bash
ros2 service call /jaka_kargo_driver/get_ik jaka_kargo_msgs/srv/GetIK "{robot_index: 1, ref_joints: [1.57 ,0 ,0 ,-1.57 ,0.0 ,0.0 ,0.0], cartesian_pose: [449.133, -353.160, 227.806, -3.1319, 0.0037, 0.5220]}"
```

#### 3. Enable Drag mode (left arm)

```bash
ros2 service call /jaka_kargo_driver/drag_mode jaka_kargo_msgs/srv/DragMode "{robot_index: 0, enable: 1}"
```

#### 4. Get tool payload (left arm)

```bash
ros2 service call /jaka_kargo_driver/get_tool_payload jaka_kargo_msgs/srv/GetToolPayload "{robot_index: 0}"
```

#### 5. Get Collision Level (right arm)

```bash
ros2 service call /jaka_kargo_driver/get_collision_level jaka_kargo_msgs/srv/GetCollisionLevel "{robot_index: 1}"
```

#### 6. Clear error

```bash
ros2 service call /jaka_kargo_driver/clear_error jaka_kargo_msgs/srv/ClearError 
```

#### 7. Get SDK version

```bash
ros2 service call /jaka_kargo_driver/get_sdk_version jaka_kargo_msgs/srv/GetSdkVersion 
```

### External axis service calls

#### 8. Enable one external-axis  

```bash
ros2 service call /jaka_kargo_driver/ext_enable jaka_kargo_msgs/srv/ExtEnable "{ext_id: 0, enable: true}"
```

#### 9. Disable one external-axis

```bash
ros2 service call /jaka_kargo_driver/ext_enable jaka_kargo_msgs/srv/ExtEnable "{ext_id: 2, enable: false}"
```
#### 10. Jog external-axis

```bash
ros2 service call /jaka_kargo_driver/jog_ext jaka_kargo_msgs/srv/JogExt "{ext_id: 1, is_abs: 1, vel: 10.0, step: 5.0}"
```

### multi_move_ext examples

The `/jaka_kargo_driver/multi_move_ext` service is a generic wrapper around the SDK interface `multi_mov_with_ext(...)`.
It can be used for:
- external axis joint motion
- robot arm joint motion
- robot arm linear motion
- robot arm circular motion
- mixed motion requests containing multiple motion units  

#### 11. Move one external-axis 

```bash
ros2 service call /jaka_kargo_driver/multi_move_ext jaka_kargo_msgs/srv/MultiMoveExt "{
  is_block: true,
  use_di_info: false,
  moves: [
    {
      motion_unit_type: 1,
      motion_unit_id: 0,
      move_type: 0,
      move_mode: 0,
      movej_end_pos: [100.0],
      movej_j_vel: 80.0,
      movej_j_acc: 80.0,
      movej_j_jerk: 0.0,
      movej_blend_tol: 0.0,
    }
  ]
}"
```

#### 12. Move all 4 external-axes together

```bash
ros2 service call /jaka_kargo_driver/multi_move_ext jaka_kargo_msgs/srv/MultiMoveExt "{
  is_block: true,
  use_di_info: false,
  moves: [
    {
      motion_unit_type: 1,
      motion_unit_id: 0,
      move_type: 0,
      move_mode: 0,
      movej_end_pos: [200.0],
      movej_j_vel: 80.0,
      movej_j_acc: 80.0,
      movej_j_jerk: 0.0,
      movej_blend_tol: 0.0,
    },
    {
      motion_unit_type: 1,
      motion_unit_id: 1,
      move_type: 0,
      move_mode: 0,
      movej_end_pos: [10.0],
      movej_j_vel: 80.0,
      movej_j_acc: 80.0,
      movej_j_jerk: 0.0,
      movej_blend_tol: 0.0,
    },
    {
      motion_unit_type: 1,
      motion_unit_id: 2,
      move_type: 0,
      move_mode: 0,
      movej_end_pos: [15.0],
      movej_j_vel: 80.0,
      movej_j_acc: 80.0,
      movej_j_jerk: 0.0,
      movej_blend_tol: 0.0,
    },
    {
      motion_unit_type: 1,
      motion_unit_id: 3,
      move_type: 0,
      move_mode: 0,
      movej_end_pos: [20.0],
      movej_j_vel: 80.0,
      movej_j_acc: 80.0,
      movej_j_jerk: 0.0,
      movej_blend_tol: 0.0,
    }
  ]
}"
```

#### 13. Move 2 external-axis + 2 robot-arms (joint motion)
```bash
ros2 service call /jaka_kargo_driver/multi_move_ext jaka_kargo_msgs/srv/MultiMoveExt "{
  is_block: true,
  use_di_info: false,
  moves: [
    {
      motion_unit_type: 1,
      motion_unit_id: 0,
      move_type: 0,
      move_mode: 0,
      movej_end_pos: [150.0], 
      movej_j_vel: 80.0,
      movej_j_acc: 80.0,
      movej_j_jerk: 0.0,
      movej_blend_tol: 0.0,
    },
    {
      motion_unit_type: 1,
      motion_unit_id: 1,
      move_type: 0,
      move_mode: 0,
      movej_end_pos: [20.0],
      movej_j_vel: 80.0,
      movej_j_acc: 80.0,
      movej_j_jerk: 0.0,
      movej_blend_tol: 0.0,
    },
    {
      motion_unit_type: 0, 
      motion_unit_id: 0, 
      move_type: 0,
      move_mode: 0,
      movej_end_pos: [1.57 ,0 ,0 ,-1.57 ,-1.57 ,1.57 ,0], 
      movej_j_vel: 80.0,
      movej_j_acc: 80.0,
      movej_j_jerk: 0.0,
      movej_blend_tol: 0.0,
    },
    {
      motion_unit_type: 0, 
      motion_unit_id: 1, 
      move_type: 0,
      move_mode: 0,
      movej_end_pos: [1.57 ,0 ,0 ,-1.57 ,-1.57 ,1.57 ,0], 
      movej_j_vel: 80.0,
      movej_j_acc: 80.0,
      movej_j_jerk: 0.0,
      movej_blend_tol: 0.0,
    }
  ]
}"
```

## 🧪 MoveIt Simulation Modes

To support both RViz-only simulation and Isaac Sim integration, we modified the `launches.py` file used by MoveIt2:  

🛠️ **Modifications**:

- Adds **use_rviz_sim** and **use_isaac_sim** arguments.
- Determines whether fake controllers or Isaac Sim interfaces are spawned, based on the provided argument.

⚙️ **Setup**:

- Replace the original `launches.py` from MoveIt2 with the modified version provided in **jaka_kargo_ros2** package.
- To find where to replace:  
  ```bash
  find /opt/ros/humble/ -name launches.py
  ```

### 🚀 Launch Moveit Simulation Modes  

### a. RViz-only simulation mode

Launch MoveIt2 in RViz-only simulation mode using the following command:

```bash
ros2 launch jaka_kargo_moveit_config demo.launch.py use_rviz_sim:=true
```

### b. Isaac Sim simulation mode 

#### 1. Installing Isaac Sim

Follow the [official instructions](
https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_workstation.html) to install Isaac-sim.

#### 2. Launching Isaac Sim

We provide the ROS 2 launch interface for running Isaac Sim and loading the robot model directly into a USD stage.  

- Start Isaac Sim and load the robot model:

  ```bash
  ros2 launch jaka_kargo_isaacsim run_kargo_isaacsim.launch.py
  ```

    This internally:

    - Launches Isaac Sim through `python.sh`.
    - Loads the USD from `jaka_kargo_description`.
    - Sets up **/isaac_joint_states** and **isaac_joint_commands** topics for communication with MoveIt2 and RViz.

- Launch MoveIt2 and RViz with Isaac Sim integration enabled:
  ```bash
  ros2 launch jaka_kargo_moveit_config demo.launch.py use_isaac_sim:=true
  ```

This setup enables planning trajectories in RViz and executing them directly in Isaac Sim.

### 🎮 Controller Switching in RViz

In RViz:
- Plan and execute trajectories to target poses or joint states.
- Only one planning group (left-arm or right-arm or body / external-axis or full_robot) can be selected at a time.
- A full-robot controller can be enabled to control left-arm, right-arm and body / external-axis together.

⚠️ **Note:**  
ROS 2 controllers use a claiming property:
- A joint can be controlled by only one controller at a time.
- If the full-robot controller is enabled, the left-arm, right-arm and body controllers are automatically disabled, and vice-versa.

#### 🔧 Switching Controllers

Activate the full-robot controller (disable subgroup controllers):

```bash
ros2 control switch_controllers \
  --activate jaka_kargo_full_robot_controller \
  --deactivate jaka_kargo_arm_l_controller jaka_kargo_arm_r_controller jaka_kargo_body_controller \
  --strict
```

Revert to left-arm + right-arm + body controllers:

```bash
ros2 control switch_controllers \
  --activate jaka_kargo_arm_l_controller jaka_kargo_arm_r_controller jaka_kargo_body_controller \
  --deactivate jaka_kargo_full_robot_controller \
  --strict
```


### 🧯 Troubleshooting Isaac Sim

#### ❌ Segmentation Fault When Using ROS2 Launch

If you see an error like:

```swift
Fatal Python error: Segmentation fault
...
```

Try these solutions:

✅ 1: Clear Corrupted IsaacSim Configs

```bash
rm -rf ~/.nvidia-omniverse/logs
rm -rf ~/.nvidia-omniverse/config
```

Then relaunch:

```bash
ros2 launch jaka_kargo_isaacsim run_kargo_isaacsim.launch.py
```

✅ 2: Test IsaacSim Without ROS

```bash
cd jaka_kargo_isaacsim/scripts
./python.sh isaacsim_moveit.py
```

- If this works, your Isaac Sim install and python shell are fine → the problem is in the ROS launch environment.
- If it crashes: the issue is deeper (likely the Python shell or an Isaac Sim extension).  

✅ 3. Launch Empty IsaacSim and Load Manually  
As a fallback, start Isaac Sim without your robot usd loaded:
  ```bash
  ros2 launch jaka_kargo_isaacsim run_isaacsim.launch.py
  ```
In the GUI:

  1. Go to File → Open.
  2. Browse to and select kargo's usd file: `jaka_kargo_description/kargo-de/urdf/jaka_kargo/jaka_kargo_moveit.usd`.
  3. Press Play ▶ to start simulation.

To confirm the ROS bridge is active, check topics:
  ```bash
  ros2 topic list
  ```
  
 You should see:
- /isaac_joint_commands
- /isaac_joint_states