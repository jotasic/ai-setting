# ROS2-Unity Palletizing Simulation

ROS2와 Unity를 연동한 로봇 팔로타이징 시뮬레이션 프로젝트를 위한 AI 코딩 어시스턴트 설정입니다.

## Quick Start

```bash
# 이 설정 폴더를 프로젝트에 복사
cp -r ros2-unity-palletizing /your/project/.claude

# 또는 심볼릭 링크
ln -s /path/to/ai-setting/ros2-unity-palletizing /your/project/.claude
```

## Project Structure

```
.
├── settings.json          # ROS2/Unity 권한 설정
├── mcp.json              # MCP 서버 설정 (Docker, GitHub 등)
├── hooks.json            # URDF/Python 자동 검증 훅
├── agents/               # 9개 전문 에이전트
│   ├── ros2-architect.md
│   ├── unity-developer.md
│   ├── ros2-developer.md
│   ├── motion-planner.md
│   ├── behavior-tree-designer.md
│   ├── urdf-specialist.md
│   ├── ros-unity-bridge.md
│   ├── physics-tuner.md
│   └── simulation-debugger.md
├── skills/               # 10개 스킬
│   ├── ros2-build/
│   ├── unity-build/
│   ├── launch-sim/
│   ├── urdf-import/
│   ├── moveit-setup/
│   ├── bt-design/
│   ├── ros-topic/
│   ├── docker-ros/
│   ├── coordinate-check/
│   └── physics-tune/
└── CLAUDE.md             # 이 파일
```

---

## System Architecture: Split-Brain

```
┌─────────────────────────────────────────────────────────────┐
│              ROS2 (Control Brain)                           │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │  MoveIt2    │  │  py_trees   │  │   Custom Nodes      │ │
│  │  Planner    │  │     BT      │  │  (BoxDetector etc)  │ │
│  └──────┬──────┘  └──────┬──────┘  └──────────┬──────────┘ │
│         └────────────────┼───────────────────┘             │
│                          │ DDS                              │
└──────────────────────────┼──────────────────────────────────┘
                           │
             ┌─────────────┴─────────────┐
             │    ROS-TCP-Endpoint       │
             │      (Port 10000)         │
             └─────────────┬─────────────┘
                           │ TCP/IP
             ┌─────────────┴─────────────┐
             │    ROS-TCP-Connector      │
             └─────────────┬─────────────┘
                           │
┌──────────────────────────┼──────────────────────────────────┐
│                          │                                   │
│  ┌─────────────┐  ┌──────┴──────┐  ┌─────────────────────┐  │
│  │Articulation │  │    ROS      │  │   Virtual Sensors   │  │
│  │   Body      │  │ Connection  │  │  (Camera, Lidar)    │  │
│  └─────────────┘  └─────────────┘  └─────────────────────┘  │
│              Unity (Simulation Body)                         │
└──────────────────────────────────────────────────────────────┘
```

---

## Agents (9개)

| Model | Agent | Description |
|-------|-------|-------------|
| **opus** | `ros2-architect` | 시스템 아키텍처, Split-Brain 설계 |
| **sonnet** | `unity-developer` | Unity 시뮬레이션, ArticulationBody, 그리퍼 |
| **sonnet** | `ros2-developer` | ROS2 노드, 패키지, 토픽/서비스 |
| **sonnet** | `motion-planner` | MoveIt2 설정, 경로 계획 |
| **sonnet** | `behavior-tree-designer` | py_trees BT 로직 설계 |
| **sonnet** | `urdf-specialist` | URDF/Xacro 로봇 모델링 |
| **sonnet** | `ros-unity-bridge` | TCP 브리지, 메시지 정의, 좌표 변환 |
| **sonnet** | `physics-tuner` | ArticulationBody 물리 튜닝 |
| **sonnet** | `simulation-debugger` | 시스템 디버깅, 시각화 |

### 에이전트 사용 예시

```
Use the ros2-architect agent to design the system topology
Have the unity-developer agent implement the vacuum gripper
Have the ros2-developer agent create the box detector node
Use the motion-planner agent to configure MoveIt2
Have the behavior-tree-designer agent implement pick-and-place logic
Use the urdf-specialist agent to configure robot joint limits
Have the ros-unity-bridge agent set up custom messages
Use the physics-tuner agent to fix joint vibration issues
Have the simulation-debugger agent diagnose communication problems
```

---

## Skills (10개)

| Skill | Usage | Description |
|-------|-------|-------------|
| `/ros2-build` | `/ros2-build [package] [--clean]` | colcon 빌드 |
| `/unity-build` | `/unity-build [--headless]` | Unity 프로젝트 빌드 |
| `/launch-sim` | `/launch-sim [--ros-only\|--docker]` | 시뮬레이션 실행 |
| `/urdf-import` | `/urdf-import <path>` | URDF를 Unity에 임포트 |
| `/moveit-setup` | `/moveit-setup <robot> [--assistant]` | MoveIt2 설정 |
| `/bt-design` | `/bt-design <task> [--visualize]` | Behavior Tree 설계 |
| `/ros-topic` | `/ros-topic list\|echo\|hz <topic>` | 토픽 모니터링 |
| `/docker-ros` | `/docker-ros build\|run\|exec\|logs` | Docker ROS2 관리 |
| `/coordinate-check` | `/coordinate-check [pose]` | 좌표계 변환 검증 |
| `/physics-tune` | `/physics-tune [joint] [--smooth]` | 물리 파라미터 튜닝 |

---

## Workflow: 팔로타이징 시뮬레이션 구축 5단계

### 1. 준비 (Preparation)
```bash
# Docker ROS2 환경 시작
/docker-ros build
/docker-ros run

# Unity Robotics Hub 패키지 설치 (Unity Package Manager)
com.unity.robotics.ros-tcp-connector
com.unity.robotics.urdf-importer
```

### 2. 모델링 (Modeling)
```bash
# URDF 유효성 검사 및 임포트
/urdf-import ~/ros2_ws/src/robot_description/urdf/robot.urdf

# 물리 파라미터 튜닝
/physics-tune --smooth
```

### 3. 연동 (Bridging)
```bash
# 브리지 설정 확인
/ros-topic list
/coordinate-check "1.0, 0.0, 0.5"

# 커스텀 메시지 생성 (Unity Editor)
# Robotics > Generate ROS Messages
```

### 4. 지능화 (Intelligence)
```bash
# MoveIt2 설정
/moveit-setup palletizing_robot

# Behavior Tree 설계
/bt-design "pick box from conveyor and place on pallet"
```

### 5. 검증 (Validation)
```bash
# 시뮬레이션 실행
/launch-sim

# 토픽 모니터링
/ros-topic hz /joint_states
/ros-topic echo /detected_box
```

---

## Coordinate System

```
ROS2 (Right-Handed)     Unity (Left-Handed)
      Z (up)                 Y (up)
      │                      │
      └──── Y (left)         └──── X (right)
     /                      /
    X (forward)            Z (forward)

변환: Unity.X = -ROS.Y, Unity.Y = ROS.Z, Unity.Z = ROS.X
```

---

## Key Technologies

| Component | Technology | Purpose |
|-----------|------------|---------|
| Robot Control | ROS2 Humble/Jazzy | 노드 통신, 미들웨어 |
| Motion Planning | MoveIt2 + OMPL | 충돌 회피 경로 생성 |
| Task Logic | py_trees | Behavior Tree 작업 관리 |
| Physics | Unity PhysX + ArticulationBody | 로봇 물리 시뮬레이션 |
| Communication | ROS-TCP-Connector | ROS2-Unity 브리지 |
| Robot Model | URDF/Xacro | 로봇 구조 정의 |
| Container | Docker | ROS2 환경 격리 |

---

## Common Issues & Solutions

| Issue | Solution |
|-------|----------|
| Unity에서 ROS 연결 안 됨 | ros_tcp_endpoint 실행 확인, 포트 10000 방화벽 열기 |
| 좌표가 이상함 | `/coordinate-check`로 변환 검증 |
| 로봇 팔이 떨림 | `/physics-tune`로 damping 증가 |
| 메시지 타입 불일치 | Unity에서 ROS Messages 재생성 |
| BT가 멈춤 | `rqt_py_trees`로 노드 상태 확인 |
| Docker 네트워크 문제 | host network 모드 또는 포트 포워딩 확인 |

---

## Output Rules

settings.json의 preferences를 따릅니다:

- **verbosity: minimal** - 간결한 출력
- **codeOnly: true** - 코드 중심
- **maxExplanationLines: 3** - 설명 최대 3줄

---

## Resources

- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [MoveIt2 Documentation](https://moveit.ros.org/)
- [py_trees Documentation](https://py-trees.readthedocs.io/)
- [ArticulationBody API](https://docs.unity3d.com/ScriptReference/ArticulationBody.html)
