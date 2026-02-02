---
name: ros2-build
description: Build ROS2 workspace with colcon
argument-hint: [package] [--clean]
allowed-tools: Bash, Read, Grep, Glob
model: haiku
category: build
---

# ROS2 Build

ROS2 워크스페이스를 colcon으로 빌드합니다.

## Arguments

- `package`: 특정 패키지만 빌드 (선택)
- `--clean`: 클린 빌드 (build, install, log 폴더 삭제)

## Workflow

```
1. Source ROS2 environment
2. Navigate to workspace
3. Run colcon build
4. Source install/setup.bash
```

## Commands

```bash
# Full build
cd ~/ros2_ws && colcon build --symlink-install

# Single package
colcon build --packages-select palletizing_robot

# Clean build
rm -rf build install log && colcon build

# Build with dependencies
colcon build --packages-up-to palletizing_robot

# Parallel jobs
colcon build --parallel-workers 4
```

## Common Issues

| Issue | Solution |
|-------|----------|
| Missing dependency | `rosdep install --from-paths src -y` |
| CMake error | Check CMakeLists.txt syntax |
| Python import error | Check setup.py entry_points |

## Examples

```bash
/ros2-build                           # 전체 빌드
/ros2-build palletizing_robot         # 특정 패키지
/ros2-build --clean                   # 클린 빌드
/ros2-build palletizing_msgs --clean  # 메시지 패키지 클린 빌드
```

## Agent Integration

빌드 실패 시:
```
Use the ros2-developer agent to fix package configuration issues
```
