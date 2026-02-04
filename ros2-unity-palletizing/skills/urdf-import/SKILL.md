---
name: urdf-import
description: Import URDF robot model into Unity
argument-hint: <urdf_path>
allowed-tools: Bash, Read, Grep, Glob
model: haiku
category: setup
---

# URDF Import

URDF 로봇 모델을 Unity에 임포트합니다.

## Arguments

- `urdf_path`: URDF 파일 경로 (필수)

## Workflow

```
1. Validate URDF syntax
2. Convert xacro to URDF (if needed)
3. Copy to Unity Assets
4. Import via URDF Importer
5. Configure ArticulationBody settings
```

## Pre-Import Validation

```bash
# Check URDF syntax
check_urdf robot.urdf

# Convert xacro to URDF
xacro robot.urdf.xacro > robot.urdf

# Visualize structure
urdf_to_graphviz robot.urdf
evince robot.pdf
```

## Unity Import Steps

### Via Editor
```
1. Assets > Import Robot from URDF
2. Select URDF file
3. Configure import settings:
   - Axis Type: Y-Axis (Unity default)
   - Mesh Decomposer: VHACD
   - Convex Mesh: Enable for collision
```

### Via Script
```csharp
using Unity.Robotics.UrdfImporter;

public class URDFImportScript
{
    [MenuItem("Robotics/Import Palletizing Robot")]
    public static void ImportRobot()
    {
        var settings = new ImportSettings
        {
            choosenAxis = ImportSettings.axisType.yAxis,
            convexMethod = ImportSettings.convexDecomposer.vHACD,
        };

        UrdfRobotExtensions.Create("Assets/URDF/robot.urdf", settings);
    }
}
```

## Post-Import Configuration

```csharp
// Set base as immovable
var baseBody = robot.GetComponent<ArticulationBody>();
baseBody.immovable = true;

// Configure drives
foreach (var joint in robot.GetComponentsInChildren<ArticulationBody>())
{
    if (joint.jointType == ArticulationJointType.RevoluteJoint)
    {
        var drive = joint.xDrive;
        drive.stiffness = 100000;
        drive.damping = 10000;
        joint.xDrive = drive;
    }
}
```

## File Structure

```
unity_ws/
└── Assets/
    └── URDF/
        └── palletizing_robot/
            ├── robot.urdf
            └── meshes/
                ├── visual/
                │   └── *.dae
                └── collision/
                    └── *.stl
```

## Examples

```bash
/urdf-import ~/ros2_ws/src/robot_description/urdf/robot.urdf
/urdf-import ./robot.urdf.xacro  # xacro 자동 변환
```

## Agent Integration

URDF 문제 시:
```
Use the urdf-specialist agent to fix robot model issues
```
