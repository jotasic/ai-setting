---
name: moveit-setup
description: Configure MoveIt2 for robot motion planning
argument-hint: <robot_name> [--assistant]
allowed-tools: Bash, Read, Edit, Write, Grep, Glob
model: sonnet
category: setup
---

# MoveIt Setup

MoveIt2 설정 패키지를 생성/구성합니다.

## Arguments

- `robot_name`: 로봇 이름 (필수)
- `--assistant`: Setup Assistant GUI 실행

## Workflow

```
1. Launch MoveIt Setup Assistant (or manual config)
2. Generate SRDF
3. Configure kinematics
4. Set up planning pipelines
5. Define named poses
6. Configure controllers
```

## Setup Assistant

```bash
# Launch GUI
ros2 launch moveit_setup_assistant setup_assistant.launch.py

# Select URDF
# Configure:
#   - Planning Groups (arm, gripper)
#   - Robot Poses (home, pick_ready, place_ready)
#   - End Effectors
#   - Passive Joints
#   - Collision Matrix
```

## Manual Configuration

### Package Structure
```
palletizing_moveit_config/
├── config/
│   ├── joint_limits.yaml
│   ├── kinematics.yaml
│   ├── ompl_planning.yaml
│   ├── pilz_cartesian_limits.yaml
│   └── moveit_controllers.yaml
├── srdf/
│   └── palletizing_robot.srdf
└── launch/
    └── move_group.launch.py
```

### kinematics.yaml
```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  kinematics_solver_attempts: 3
```

### joint_limits.yaml
```yaml
joint_limits:
  joint_1:
    has_velocity_limits: true
    max_velocity: 3.14
    has_acceleration_limits: true
    max_acceleration: 5.0
```

### ompl_planning.yaml
```yaml
planner_configs:
  RRTConnect:
    type: geometric::RRTConnect
    range: 0.0
  PRM:
    type: geometric::PRM
    max_nearest_neighbors: 10

arm:
  default_planner_config: RRTConnect
  planner_configs:
    - RRTConnect
    - PRM
```

## Launch File

```python
def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "palletizing_robot",
        package_name="palletizing_moveit_config"
    ).to_moveit_configs()

    return LaunchDescription([
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            parameters=[moveit_config.to_dict()],
            output="screen",
        ),
    ])
```

## Examples

```bash
/moveit-setup palletizing_robot              # 설정 생성
/moveit-setup palletizing_robot --assistant  # GUI 실행
```

## Agent Integration

경로 계획 문제 시:
```
Use the motion-planner agent to optimize MoveIt2 configuration
```
