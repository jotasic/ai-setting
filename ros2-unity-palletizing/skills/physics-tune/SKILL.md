---
name: physics-tune
description: Tune Unity physics parameters for robot simulation
argument-hint: [joint_name] [--aggressive|--smooth]
allowed-tools: Bash, Read, Edit, Write, Grep, Glob
model: sonnet
category: simulation
---

# Physics Tune

Unity ArticulationBody 물리 파라미터를 튜닝합니다.

## Arguments

- `joint_name`: 특정 관절 튜닝 (선택)
- `--aggressive`: 빠른 응답 설정
- `--smooth`: 부드러운 동작 설정

## Tuning Parameters

### ArticulationDrive

| Parameter | Description | Palletizing Range |
|-----------|-------------|-------------------|
| Stiffness | Position spring | 100,000 - 1,000,000 |
| Damping | Velocity damper | 10,000 - 100,000 |
| ForceLimit | Max motor torque | 5,000 - 50,000 |

### Presets

**Aggressive (Fast Response)**
```csharp
stiffness = 800000f;
damping = 30000f;
forceLimit = 40000f;
```

**Smooth (Gentle Motion)**
```csharp
stiffness = 200000f;
damping = 80000f;
forceLimit = 20000f;
```

**Balanced (Default)**
```csharp
stiffness = 400000f;
damping = 50000f;
forceLimit = 30000f;
```

## Per-Joint Recommendations

```csharp
// Base rotation (J1) - High inertia
j1.stiffness = 500000f;
j1.damping = 60000f;

// Shoulder (J2) - Heavy lifting
j2.stiffness = 800000f;
j2.damping = 80000f;

// Elbow (J3) - Medium load
j3.stiffness = 400000f;
j3.damping = 40000f;

// Wrist (J4-J6) - Fine positioning
wrist.stiffness = 150000f;
wrist.damping = 15000f;
```

## PhysX Project Settings

```
Edit > Project Settings > Physics

Time.fixedDeltaTime: 0.005 (200Hz)
Default Solver Iterations: 20
Default Solver Velocity Iterations: 10
Enable Enhanced Determinism: true
```

## Tuning Script

```csharp
public class PhysicsTuner : MonoBehaviour
{
    [System.Serializable]
    public class JointPreset
    {
        public string jointName;
        public float stiffness = 400000f;
        public float damping = 50000f;
        public float forceLimit = 30000f;
    }

    public JointPreset[] presets;

    [ContextMenu("Apply Presets")]
    public void ApplyPresets()
    {
        foreach (var preset in presets)
        {
            var joint = transform.Find(preset.jointName)?.GetComponent<ArticulationBody>();
            if (joint != null)
            {
                var drive = joint.xDrive;
                drive.stiffness = preset.stiffness;
                drive.damping = preset.damping;
                drive.forceLimit = preset.forceLimit;
                joint.xDrive = drive;
            }
        }
    }
}
```

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| Vibration | Low damping | Increase damping 2x |
| Slow response | High damping | Decrease damping |
| Arm droops | Low stiffness | Increase stiffness |
| Overshoot | Low damping | Increase damping |
| Motor limit | Low forceLimit | Increase forceLimit |

## Examples

```bash
/physics-tune                       # 현재 설정 확인
/physics-tune joint_2 --aggressive  # 특정 관절 빠르게
/physics-tune --smooth              # 전체 부드럽게
```

## Agent Integration

물리 문제 시:
```
Use the physics-tuner agent for detailed parameter optimization
```
