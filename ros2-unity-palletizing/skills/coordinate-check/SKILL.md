---
name: coordinate-check
description: Verify ROS2-Unity coordinate system transformations
argument-hint: [pose_data]
allowed-tools: Bash, Read, Grep, Glob
model: haiku
category: debug
---

# Coordinate Check

ROS2와 Unity 간의 좌표계 변환을 검증합니다.

## Coordinate Systems

```
ROS2 (Right-Handed)              Unity (Left-Handed)
      Z (up)                          Y (up)
      │                               │
      │                               │
      └──── Y (left)                  └──── X (right)
     /                               /
    X (forward)                     Z (forward)
```

## Transformation Rules

| ROS2 | Unity |
|------|-------|
| X | Z |
| Y | -X |
| Z | Y |

### Position
```
Unity.X = -ROS.Y
Unity.Y = ROS.Z
Unity.Z = ROS.X
```

### Rotation (Quaternion)
```
Unity.X = -ROS.Y
Unity.Y = -ROS.Z
Unity.Z = ROS.X
Unity.W = ROS.W
```

## Verification Script (Python)

```python
#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Pose

def ros_to_unity(ros_pose):
    unity_pose = Pose()
    # Position
    unity_pose.position.x = -ros_pose.position.y
    unity_pose.position.y = ros_pose.position.z
    unity_pose.position.z = ros_pose.position.x
    # Rotation
    unity_pose.orientation.x = -ros_pose.orientation.y
    unity_pose.orientation.y = -ros_pose.orientation.z
    unity_pose.orientation.z = ros_pose.orientation.x
    unity_pose.orientation.w = ros_pose.orientation.w
    return unity_pose

# Test with known pose
ros_pose = Pose()
ros_pose.position.x = 1.0  # Forward
ros_pose.position.y = 0.0
ros_pose.position.z = 0.5  # Up

unity_pose = ros_to_unity(ros_pose)
print(f"ROS: ({ros_pose.position.x}, {ros_pose.position.y}, {ros_pose.position.z})")
print(f"Unity: ({unity_pose.position.x}, {unity_pose.position.y}, {unity_pose.position.z})")
# Expected Unity: (0, 0.5, 1.0) - X=0, Y=0.5(up), Z=1.0(forward)
```

## Verification Script (C#)

```csharp
public static class CoordinateVerifier
{
    public static void VerifyTransform(Vector3 rosPos, Vector3 expectedUnityPos)
    {
        Vector3 convertedPos = new Vector3(
            -rosPos.y,
            rosPos.z,
            rosPos.x
        );

        Debug.Log($"ROS: {rosPos}");
        Debug.Log($"Converted: {convertedPos}");
        Debug.Log($"Expected: {expectedUnityPos}");
        Debug.Log($"Match: {Vector3.Distance(convertedPos, expectedUnityPos) < 0.001f}");
    }
}
```

## Test Cases

| ROS2 Position | Expected Unity |
|---------------|---------------|
| (1, 0, 0) - forward | (0, 0, 1) - forward |
| (0, 1, 0) - left | (-1, 0, 0) - left |
| (0, 0, 1) - up | (0, 1, 0) - up |
| (1, 0.5, 0.3) | (-0.5, 0.3, 1) |

## Quick Debug

```bash
# Publish test pose in ROS2
ros2 topic pub /test_pose geometry_msgs/msg/Pose \
  "{position: {x: 1.0, y: 0.0, z: 0.5}}" --once

# Check in Unity Console if received as (0, 0.5, 1.0)
```

## Examples

```bash
/coordinate-check                           # 좌표계 설명
/coordinate-check "1.0, 0.5, 0.3"          # 특정 좌표 변환 확인
```

## Agent Integration

좌표 문제 시:
```
Use the ros-unity-bridge agent to debug coordinate transformations
```
