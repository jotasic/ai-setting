---
name: launch-sim
description: Launch full palletizing simulation (ROS2 + Unity)
argument-hint: [--ros-only] [--unity-only] [--docker]
allowed-tools: Bash, Read, Grep, Glob
model: haiku
category: simulation
---

# Launch Simulation

ROS2와 Unity 팔로타이징 시뮬레이션을 실행합니다.

## Arguments

- `--ros-only`: ROS2 노드만 실행
- `--unity-only`: Unity만 실행
- `--docker`: Docker 컨테이너에서 ROS2 실행

## Full Stack Launch

```
┌─────────────────────────────────────────────────────────┐
│  Terminal 1: ROS2 Core                                  │
│  ros2 launch palletizing_robot full_sim.launch.py      │
├─────────────────────────────────────────────────────────┤
│  Terminal 2: Unity Simulation                           │
│  ./Build/PalletizingSim                                 │
├─────────────────────────────────────────────────────────┤
│  Terminal 3: Visualization (optional)                   │
│  rviz2 -d config/palletizing.rviz                      │
└─────────────────────────────────────────────────────────┘
```

## Commands

### Native Launch
```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Launch ROS2 nodes
ros2 launch palletizing_robot palletizing_sim.launch.py

# Run Unity (separate terminal)
./unity_ws/Build/PalletizingSim
```

### Docker Launch
```bash
# Start ROS2 container
docker-compose up -d ros2

# Run Unity on host
./unity_ws/Build/PalletizingSim

# View ROS2 logs
docker-compose logs -f ros2
```

### launch.py Example
```python
def generate_launch_description():
    return LaunchDescription([
        # TCP Endpoint
        Node(package='ros_tcp_endpoint', executable='default_server_endpoint',
             parameters=[{'ROS_IP': '0.0.0.0', 'ROS_TCP_PORT': 10000}]),

        # MoveIt
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('palletizing_moveit_config'),
                    'launch', 'move_group.launch.py'
                ])
            ])
        ),

        # Behavior Tree
        Node(package='palletizing_robot', executable='palletizing_bt_node',
             output='screen'),
    ])
```

## Examples

```bash
/launch-sim                  # 전체 시뮬레이션 시작
/launch-sim --ros-only       # ROS2만 시작
/launch-sim --unity-only     # Unity만 시작
/launch-sim --docker         # Docker 기반 실행
```

## Agent Integration

실행 문제 시:
```
Use the simulation-debugger agent to diagnose launch issues
```
