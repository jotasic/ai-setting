---
name: ros-topic
description: Monitor and debug ROS2 topics
argument-hint: <command> [topic_name]
allowed-tools: Bash, Read, Grep
model: haiku
category: debug
---

# ROS Topic Monitor

ROS2 토픽을 모니터링하고 디버깅합니다.

## Arguments

- `command`: list | echo | hz | info | pub
- `topic_name`: 토픽 이름 (선택)

## Commands

### List Topics
```bash
ros2 topic list
ros2 topic list -t  # with types
```

### Echo Topic
```bash
ros2 topic echo /joint_states
ros2 topic echo /detected_box --once
ros2 topic echo /joint_trajectory --no-arr  # hide arrays
```

### Check Frequency
```bash
ros2 topic hz /joint_states
ros2 topic hz /camera/image_raw
```

### Topic Info
```bash
ros2 topic info /joint_trajectory -v
```

### Publish Test Message
```bash
# JointState
ros2 topic pub /test_joint sensor_msgs/msg/JointState \
  "{name: ['joint_1'], position: [1.57]}" --once

# Pose
ros2 topic pub /test_pose geometry_msgs/msg/Pose \
  "{position: {x: 1.0, y: 0.5, z: 0.3}}" --once

# Bool
ros2 topic pub /gripper/grasp std_msgs/msg/Bool "{data: true}" --once
```

## Common Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | sensor_msgs/JointState | 관절 상태 |
| `/joint_trajectory` | trajectory_msgs/JointTrajectory | 궤적 명령 |
| `/detected_box` | geometry_msgs/Pose | 박스 위치 |
| `/box_info` | palletizing_msgs/BoxInfo | 박스 정보 |
| `/gripper/status` | std_msgs/Bool | 그리퍼 상태 |
| `/tf` | tf2_msgs/TFMessage | 좌표 변환 |

## Useful Aliases

```bash
# Add to .bashrc
alias rtl='ros2 topic list'
alias rte='ros2 topic echo'
alias rth='ros2 topic hz'
alias rti='ros2 topic info -v'
```

## Examples

```bash
/ros-topic list                    # 토픽 목록
/ros-topic echo /joint_states      # 토픽 데이터 확인
/ros-topic hz /joint_states        # 주파수 확인
/ros-topic info /joint_trajectory  # 토픽 정보
/ros-topic pub /test Bool true     # 테스트 발행
```

## Agent Integration

통신 문제 시:
```
Use the simulation-debugger agent to diagnose communication issues
```
