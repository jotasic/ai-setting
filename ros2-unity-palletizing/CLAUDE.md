# ROS2-Unity Robot Simulation

ROS2와 Unity를 연동한 로봇 시뮬레이션 (디지털 트윈) 프로젝트를 위한 AI 코딩 어시스턴트 설정입니다.

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
├── agents/               # 12개 전문 에이전트
│   ├── ros2-architect.md
│   ├── unity-developer.md
│   ├── ros2-developer.md
│   ├── motion-planner.md
│   ├── behavior-tree-designer.md
│   ├── state-machine-designer.md    # NEW
│   ├── urdf-specialist.md
│   ├── ros-unity-bridge.md
│   ├── virtual-sensor-developer.md  # NEW
│   ├── scenario-designer.md         # NEW
│   ├── physics-tuner.md
│   └── simulation-debugger.md
├── skills/               # 12개 스킬
│   ├── ros2-build/
│   ├── unity-build/
│   ├── launch-sim/
│   ├── urdf-import/
│   ├── moveit-setup/
│   ├── bt-design/
│   ├── state-diagram/    # NEW
│   ├── scenario-run/     # NEW
│   ├── ros-topic/
│   ├── docker-ros/
│   ├── coordinate-check/
│   └── physics-tune/
└── CLAUDE.md             # 이 파일
```

---

## System Architecture: Digital Twin Simulation

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Unity (Digital Twin)                                 │
│  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐  ┌─────────────┐  │
│  │    Robot      │  │    Virtual    │  │   Virtual     │  │   Physics   │  │
│  │   (URDF)      │  │    Camera     │  │   Sensors     │  │   Engine    │  │
│  └───────┬───────┘  └───────┬───────┘  └───────┬───────┘  └──────┬──────┘  │
│          └──────────────────┼──────────────────┴─────────────────┘         │
└─────────────────────────────┼───────────────────────────────────────────────┘
                              │ TCP Bridge (Port 10000)
┌─────────────────────────────┼───────────────────────────────────────────────┐
│                             ▼              ROS2 (Control)                    │
│  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐  ┌─────────────┐  │
│  │   Vision AI   │  │     State     │  │    MoveIt2    │  │  py_trees   │  │
│  │   Pipeline    │  │    Machine    │  │    Planner    │  │     BT      │  │
│  └───────────────┘  └───────────────┘  └───────────────┘  └─────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Agents (12개)

| Model | Agent | Description |
|-------|-------|-------------|
| **설계** |
| opus | `ros2-architect` | 시스템 아키텍처, Split-Brain 설계 |
| **Unity 시뮬레이션** |
| sonnet | `unity-developer` | Unity 환경, ArticulationBody, 그리퍼 |
| sonnet | `physics-tuner` | 물리 엔진 파라미터 튜닝 |
| sonnet | `urdf-specialist` | URDF/Xacro 로봇 모델링 |
| sonnet | `virtual-sensor-developer` | 가상 카메라, Lidar, 센서 시뮬레이션 |
| **ROS2 제어** |
| sonnet | `ros2-developer` | ROS2 노드, 패키지, 토픽/서비스 |
| sonnet | `ros-unity-bridge` | TCP 브리지, 메시지 정의, 좌표 변환 |
| sonnet | `motion-planner` | MoveIt2 설정, 경로 계획 |
| **로직 설계** |
| sonnet | `behavior-tree-designer` | py_trees BT 로직 설계 |
| sonnet | `state-machine-designer` | SMACH, FlexBE, YASMIN 상태 머신 |
| **테스트/디버깅** |
| sonnet | `scenario-designer` | 시뮬레이션 시나리오 설계 |
| sonnet | `simulation-debugger` | 시스템 디버깅, 시각화 |

### 에이전트 사용 예시

```
Use the ros2-architect agent to design the system topology
Have the unity-developer agent implement the robot gripper
Have the virtual-sensor-developer agent create RGB-D camera simulation
Use the state-machine-designer agent to design the workflow state machine
Have the scenario-designer agent create test scenarios
Use the simulation-debugger agent to diagnose communication problems
```

---

## Skills (12개)

| Skill | Usage | Description |
|-------|-------|-------------|
| **빌드/실행** |
| `/ros2-build` | `/ros2-build [package] [--clean]` | colcon 빌드 |
| `/unity-build` | `/unity-build [--headless]` | Unity 프로젝트 빌드 |
| `/launch-sim` | `/launch-sim [--ros-only\|--docker]` | 시뮬레이션 실행 |
| `/docker-ros` | `/docker-ros build\|run\|exec\|logs` | Docker ROS2 관리 |
| **로봇 설정** |
| `/urdf-import` | `/urdf-import <path>` | URDF를 Unity에 임포트 |
| `/moveit-setup` | `/moveit-setup <robot> [--assistant]` | MoveIt2 설정 |
| `/physics-tune` | `/physics-tune [joint] [--smooth]` | 물리 파라미터 튜닝 |
| `/coordinate-check` | `/coordinate-check [pose]` | 좌표계 변환 검증 |
| **로직 설계** |
| `/bt-design` | `/bt-design <task> [--visualize]` | Behavior Tree 설계 |
| `/state-diagram` | `/state-diagram <task> [--format]` | 상태 다이어그램 생성 |
| **테스트/모니터링** |
| `/ros-topic` | `/ros-topic list\|echo\|hz <topic>` | 토픽 모니터링 |
| `/scenario-run` | `/scenario-run <scenario> [--cycles N]` | 시나리오 실행/검증 |

---

## Workflow: 시뮬레이션 구축 5단계

### 1. 환경 준비 (Setup)
```bash
/docker-ros build
/docker-ros run
# Unity: Package Manager에서 Robotics Hub 설치
```

### 2. 로봇 모델링 (Modeling)
```bash
/urdf-import ~/ros2_ws/src/robot_description/urdf/robot.urdf
/physics-tune --smooth
```

### 3. 센서 구성 (Sensors)
```
Use the virtual-sensor-developer agent to create camera and lidar simulation
```

### 4. 로직 구현 (Logic)
```bash
/state-diagram "detect, pick, place"
/bt-design "pick object and place on target"
```

### 5. 테스트 (Testing)
```bash
/scenario-run smoke --cycles 10
/scenario-run stress --report
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
| Task Logic | py_trees / SMACH | BT / State Machine |
| Physics | Unity PhysX + ArticulationBody | 로봇 물리 시뮬레이션 |
| Sensors | Unity Perception | 가상 카메라, Lidar |
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
| 상태 머신 멈춤 | rqt_smach_viewer로 상태 확인 |
| 시나리오 실패 | `/scenario-run --report`로 상세 로그 확인 |

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
- [SMACH Documentation](http://wiki.ros.org/smach)
- [ArticulationBody API](https://docs.unity3d.com/ScriptReference/ArticulationBody.html)
