---
name: docker-ros
description: Manage Docker-based ROS2 environment
argument-hint: <command> [args]
allowed-tools: Bash, Read, Edit, Write, Grep, Glob
model: haiku
category: infrastructure
---

# Docker ROS2 Environment

Docker 기반 ROS2 환경을 관리합니다.

## Arguments

- `command`: build | run | exec | logs | stop
- `args`: 추가 인자

## docker-compose.yml

```yaml
version: '3.8'

services:
  ros2:
    build:
      context: .
      dockerfile: Dockerfile.ros2
    image: palletizing-ros2:humble
    container_name: palletizing_ros2
    ports:
      - "10000:10000"  # TCP bridge
      - "5005:5005"    # Debug
    volumes:
      - ./ros2_ws:/root/ros2_ws
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=0
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    network_mode: host  # or use bridge with proper config
    privileged: true
    stdin_open: true
    tty: true
    command: bash -c "source /opt/ros/humble/setup.bash && tail -f /dev/null"
```

## Dockerfile.ros2

```dockerfile
FROM osrf/ros:humble-desktop

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-moveit \
    ros-humble-py-trees \
    ros-humble-py-trees-ros \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install py_trees py_trees_ros

# Clone ROS-TCP-Endpoint
RUN mkdir -p /root/ros2_ws/src && \
    cd /root/ros2_ws/src && \
    git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git -b ROS2v0.7.0

# Build workspace
WORKDIR /root/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Setup entrypoint
COPY ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
```

## Commands

### Build
```bash
docker-compose build ros2
```

### Run
```bash
# Start container
docker-compose up -d ros2

# Interactive shell
docker-compose exec ros2 bash

# Run specific node
docker-compose exec ros2 ros2 launch palletizing_robot sim.launch.py
```

### Logs
```bash
docker-compose logs -f ros2
```

### Stop
```bash
docker-compose down
```

## Network Configuration

### For Unity on Host (Windows/Mac)
```bash
# Get container IP
docker inspect palletizing_ros2 | grep IPAddress

# Or use host.docker.internal in Unity ROS Settings
```

### For WSL2
```bash
# Get WSL IP
hostname -I | awk '{print $1}'

# Set in Unity: use this IP instead of localhost
```

## Examples

```bash
/docker-ros build                 # 이미지 빌드
/docker-ros run                   # 컨테이너 시작
/docker-ros exec bash             # 셸 접속
/docker-ros logs                  # 로그 확인
/docker-ros stop                  # 컨테이너 중지
```

## Agent Integration

Docker 문제 시:
```
Use the simulation-debugger agent to troubleshoot Docker networking
```
