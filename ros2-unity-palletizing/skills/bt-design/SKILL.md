---
name: bt-design
description: Design and implement Behavior Tree for palletizing
argument-hint: <task> [--visualize]
allowed-tools: Bash, Read, Edit, Write, Grep, Glob
model: sonnet
category: logic
---

# Behavior Tree Design

py_trees를 사용한 팔로타이징 비헤이비어 트리를 설계합니다.

## Arguments

- `task`: 작업 설명 (필수)
- `--visualize`: rqt_py_trees 뷰어 실행

## Workflow

```
1. Analyze task requirements
2. Design tree structure
3. Implement custom behaviors
4. Set up Blackboard
5. Test and visualize
```

## Tree Structure Template

```
Palletizing BT
├── [Parallel] Root
│   ├── [Sequence] SafetyMonitor
│   │   ├── CheckEStop
│   │   └── CheckRobotHealth
│   │
│   └── [Sequence*] MainTask  (* = memory)
│       ├── WaitForBox
│       ├── GetBoxInfo
│       ├── [Selector] PickSequence
│       │   ├── [Sequence] NormalPick
│       │   │   ├── PlanApproach
│       │   │   ├── ExecuteApproach
│       │   │   ├── Grasp
│       │   │   └── VerifyGrasp
│       │   └── [Sequence] RetryPick
│       │       └── ...
│       ├── ComputePlacePose
│       ├── PlanPlace
│       ├── ExecutePlace
│       ├── Release
│       └── ReturnHome
```

## Code Template

```python
#!/usr/bin/env python3
import py_trees
import py_trees_ros

class PalletizingBT:
    def __init__(self, node):
        self.node = node
        self.tree = self._create_tree()

    def _create_tree(self):
        root = py_trees.composites.Parallel(
            name="Root",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll()
        )

        # Safety monitor
        safety = py_trees.composites.Sequence("SafetyMonitor", memory=False)
        safety.add_children([
            CheckEStop("CheckEStop"),
            CheckRobotHealth("CheckRobotHealth"),
        ])

        # Main task
        main = py_trees.composites.Sequence("MainTask", memory=True)
        main.add_children([
            WaitForBox("WaitForBox"),
            # ... more behaviors
        ])

        root.add_children([safety, main])
        return root
```

## Blackboard Keys

```python
# Define at tree setup
blackboard = py_trees.blackboard.Client(name="palletizing")
blackboard.register_key(key="box_pose", access=py_trees.common.Access.WRITE)
blackboard.register_key(key="place_pose", access=py_trees.common.Access.WRITE)
blackboard.register_key(key="grasp_success", access=py_trees.common.Access.WRITE)
blackboard.register_key(key="pallet_count", access=py_trees.common.Access.WRITE)
```

## Visualization

```bash
# Run BT viewer
ros2 run py_trees_ros_viewer py_trees_tree_watcher

# Or in rqt
rqt --standalone py_trees_ros_viewer
```

## Examples

```bash
/bt-design "pick and place with retry logic"
/bt-design "add pallet layer completion check" --visualize
/bt-design "implement error recovery behavior"
```

## Agent Integration

BT 설계 시:
```
Use the behavior-tree-designer agent for complex logic implementation
```
