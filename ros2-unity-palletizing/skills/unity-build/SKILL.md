---
name: unity-build
description: Build Unity project for simulation
argument-hint: [--headless] [--development]
allowed-tools: Bash, Read, Grep, Glob
model: haiku
category: build
---

# Unity Build

Unity 시뮬레이션 프로젝트를 빌드합니다.

## Arguments

- `--headless`: 헤드리스(서버) 빌드
- `--development`: 개발 빌드 (디버깅 가능)

## Workflow

```
1. Check Unity project path
2. Validate project settings
3. Execute Unity build command
4. Verify build output
```

## Commands

```bash
# Linux standalone build
Unity -batchmode -nographics -projectPath ./unity_ws \
  -buildLinux64Player ./Build/PalletizingSim \
  -quit

# Development build
Unity -batchmode -nographics -projectPath ./unity_ws \
  -buildLinux64Player ./Build/PalletizingSim \
  -Development -quit

# Headless server build
Unity -batchmode -nographics -projectPath ./unity_ws \
  -executeMethod BuildScript.BuildHeadless \
  -quit

# Generate ROS messages
Unity -batchmode -projectPath ./unity_ws \
  -executeMethod Unity.Robotics.ROSTCPConnector.MessageGeneration.MessageAutoGen.GenerateAllMessages \
  -quit
```

## Build Script (BuildScript.cs)

```csharp
// Assets/Editor/BuildScript.cs
using UnityEditor;

public class BuildScript
{
    public static void BuildHeadless()
    {
        BuildPipeline.BuildPlayer(
            new[] { "Assets/Scenes/PalletizingSimulation.unity" },
            "Build/PalletizingSim",
            BuildTarget.StandaloneLinux64,
            BuildOptions.EnableHeadlessMode
        );
    }
}
```

## Examples

```bash
/unity-build                    # 기본 빌드
/unity-build --headless         # 서버용 헤드리스 빌드
/unity-build --development      # 개발 빌드
```

## Agent Integration

빌드 실패 시:
```
Use the unity-developer agent to fix Unity project issues
```
