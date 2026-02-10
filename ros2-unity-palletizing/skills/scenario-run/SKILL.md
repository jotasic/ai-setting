---
name: scenario-run
description: Run and validate simulation scenarios
argument-hint: <scenario_file|scenario_name> [--report] [--cycles N]
allowed-tools: Bash, Read, Edit, Write, Grep, Glob
model: sonnet
category: testing
---

# Scenario Run

시뮬레이션 시나리오를 실행하고 검증합니다.

## Arguments

- `scenario_file`: 시나리오 YAML 파일 경로
- `scenario_name`: 사전 정의된 시나리오 이름
- `--report`: 상세 리포트 생성
- `--cycles N`: N회 반복 실행

## Workflow

```
1. Load scenario configuration
2. Setup simulation environment
3. Execute scenario steps
4. Collect metrics
5. Generate report
```

## Scenario File Format

```yaml
name: "Pick and Place Test"
setup:
  robot: "robot_arm"
  objects:
    - id: "box_1"
      position: [0.5, 0, 0.3]
steps:
  - action: "detect"
    timeout: 5.0
  - action: "pick"
    target: "box_1"
  - action: "place"
    position: [0.5, 0.5, 0.3]
criteria:
  success_rate: 0.95
  max_cycle_time: 30.0
```

## Built-in Scenarios

| Name | Description |
|------|-------------|
| `smoke` | 기본 동작 확인 |
| `stress` | 100회 반복 테스트 |
| `boundary` | 경계 조건 테스트 |
| `failure` | 에러 복구 테스트 |

## Output Report

```
═══════════════════════════════════════
Scenario Report: Pick and Place Test
═══════════════════════════════════════
Total Runs:     100
Success:        98 (98.0%)
Failed:         2 (2.0%)

Cycle Time:
  Average:      12.5s
  Min:          10.2s
  Max:          18.7s

Metrics:
  Collisions:   0
  Position Error: 2.3mm (avg)
═══════════════════════════════════════
```

## Examples

```bash
/scenario-run scenarios/pick_place.yaml
/scenario-run smoke --cycles 10
/scenario-run stress --report
/scenario-run scenarios/edge_cases.yaml --cycles 50 --report
```

## Agent Integration

시나리오 설계:
```
Use the scenario-designer agent to create comprehensive test scenarios
```
