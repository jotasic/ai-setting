# ROS2-Unity Robot Simulation - Environment Setup

이 에이전트들을 사용하기 위한 환경 구성 가이드입니다.

## 요구사항

| Component | Version | 필수 |
|-----------|---------|------|
| Ubuntu | 22.04 LTS | ✅ |
| ROS2 | Humble / Jazzy | ✅ |
| Unity | 2021.3 LTS+ | ✅ |
| Docker | 24.0+ | 권장 |
| Python | 3.10+ | ✅ |

---

## 1. ROS2 설치

### Ubuntu 22.04 + ROS2 Humble

```bash
# Locale 설정
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS2 apt repository 추가
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS2 Humble 설치
sudo apt update
sudo apt install ros-humble-desktop -y

# 환경 설정
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 추가 ROS2 패키지

```bash
# MoveIt2
sudo apt install ros-humble-moveit -y

# Navigation2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y

# ros2_control
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers -y

# Behavior Tree
pip install py_trees py_trees_ros

# State Machine (SMACH)
pip install smach
sudo apt install ros-humble-executive-smach -y

# ROS-TCP-Endpoint (Unity 연동)
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git -b main-ros2
cd ~/ros2_ws
colcon build --packages-select ros_tcp_endpoint
```

---

## 2. Unity 설치

### Unity Hub 설치

```bash
# Unity Hub 다운로드 (Ubuntu)
wget -qO - https://hub.unity3d.com/linux/keys/public | sudo apt-key add -
sudo sh -c 'echo "deb https://hub.unity3d.com/linux/repos/deb stable main" > /etc/apt/sources.list.d/unityhub.list'
sudo apt update
sudo apt install unityhub -y
```

### Unity Editor 설치

1. Unity Hub 실행
2. Installs → Install Editor
3. **2021.3 LTS** 이상 버전 선택
4. 모듈 추가: Linux Build Support

### Unity Robotics 패키지 설치

Unity 프로젝트에서 **Window → Package Manager**:

1. **+** 버튼 → Add package from git URL
2. 다음 패키지 추가:

```
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
```

또는 `Packages/manifest.json`에 직접 추가:

```json
{
  "dependencies": {
    "com.unity.robotics.ros-tcp-connector": "https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector",
    "com.unity.robotics.urdf-importer": "https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer"
  }
}
```

---

## 3. Docker 환경 (권장)

Windows/Mac 사용자이거나 환경 격리가 필요한 경우 Docker 사용을 권장합니다.

### Docker 설치

```bash
# Docker 설치
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER

# Docker Compose
sudo apt install docker-compose-plugin -y
```

### ROS2 Docker 이미지

```bash
# 공식 ROS2 이미지
docker pull ros:humble

# 또는 Unity Robotics 튜토리얼 이미지
docker pull unityrobotics/ros2:humble-ros-tcp-endpoint
```

### Docker Compose 예시

```yaml
# docker-compose.yml
version: '3.8'
services:
  ros2:
    image: ros:humble
    container_name: ros2_sim
    network_mode: host
    volumes:
      - ./ros2_ws:/root/ros2_ws
      - /tmp/.X11-unix:/tmp/.X11-unix
    environment:
      - DISPLAY=$DISPLAY
      - ROS_DOMAIN_ID=0
    command: bash -c "source /opt/ros/humble/setup.bash && ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0"
```

```bash
# 실행
docker-compose up -d
```

---

## 4. 프로젝트 구조

```
your_project/
├── .claude/                    # 이 폴더를 복사
│   ├── agents/
│   ├── skills/
│   ├── settings.json
│   └── CLAUDE.md
├── ros2_ws/                    # ROS2 워크스페이스
│   └── src/
│       ├── robot_description/  # URDF/Xacro
│       ├── robot_moveit/       # MoveIt2 설정
│       ├── robot_control/      # 제어 노드
│       └── ros_tcp_endpoint/   # Unity 브리지
└── unity_project/              # Unity 프로젝트
    ├── Assets/
    │   ├── Scripts/
    │   ├── URDF/
    │   └── Scenes/
    └── Packages/
```

---

## 5. 연결 테스트

### ROS2 측 (터미널 1)

```bash
# ROS-TCP-Endpoint 실행
source ~/ros2_ws/install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

### Unity 측

1. **Robotics → ROS Settings** 메뉴 열기
2. ROS IP Address: `127.0.0.1` (또는 Docker IP)
3. ROS Port: `10000`
4. Protocol: `ROS2`
5. **Connect** 클릭

### 연결 확인

```bash
# ROS2에서 토픽 확인
ros2 topic list

# Unity에서 발행하는 토픽이 보이면 성공
/unity/joint_states
/unity/clock
```

---

## 6. 문제 해결

### 연결 안 됨

```bash
# 방화벽 확인
sudo ufw allow 10000/tcp

# Docker 네트워크 확인
docker network inspect bridge

# ROS_IP 환경변수 확인
echo $ROS_IP
```

### Unity에서 메시지 타입 오류

```
# Unity Editor에서 메시지 재생성
Robotics → Generate ROS Messages...
```

### URDF 임포트 실패

```bash
# URDF 문법 검사
check_urdf robot.urdf

# Xacro → URDF 변환
xacro robot.urdf.xacro > robot.urdf
```

---

## 7. 추가 도구 (선택)

### 시각화

```bash
# RViz2
sudo apt install ros-humble-rviz2 -y

# rqt
sudo apt install ros-humble-rqt* -y

# PlotJuggler (데이터 시각화)
sudo apt install ros-humble-plotjuggler-ros -y
```

### 디버깅

```bash
# py_trees 시각화
sudo apt install ros-humble-py-trees-ros-viewer -y

# SMACH 시각화
sudo apt install ros-humble-smach-viewer -y
```

### 비전 AI (선택)

```bash
# OpenCV
pip install opencv-python

# PyTorch (CUDA)
pip install torch torchvision

# Ultralytics YOLO
pip install ultralytics
```

---

## Quick Start

```bash
# 1. 이 설정을 프로젝트에 복사
cp -r ros2-unity-palletizing /your/project/.claude

# 2. ROS2 워크스페이스 생성
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build

# 3. Unity 프로젝트 생성 후 Robotics 패키지 설치

# 4. Claude Code에서 에이전트 사용
# "Use the ros2-architect agent to design the system"
```

---

## 참고 자료

- [ROS2 Humble 설치 가이드](https://docs.ros.org/en/humble/Installation.html)
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [MoveIt2 튜토리얼](https://moveit.picknik.ai/main/index.html)
- [py_trees 문서](https://py-trees.readthedocs.io/)
