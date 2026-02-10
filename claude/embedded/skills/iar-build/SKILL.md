---
name: iar-build
description: Build IAR EWARM project from command line
argument-hint: [project.ewp] [--config Release|Debug] [--clean]
allowed-tools: Bash, Read, Grep, Glob
model: haiku
category: embedded-workflow
---

# IAR Build

IAR EWARM 프로젝트를 커맨드라인에서 빌드합니다.

## Arguments

- `$ARGUMENTS`: .ewp 파일 경로, 빌드 설정
- `--config`: 빌드 구성 (Release/Debug, 기본: Release)
- `--clean`: 클린 빌드

## Workflow

```
┌─────────────────────────────────────┐
│  1. Find .ewp project file         │
│  2. Detect build configuration      │
│  3. Execute iarbuild                │
│  4. Parse build output              │
│  5. Report errors/warnings/.map     │
└─────────────────────────────────────┘
```

## Commands

```bash
# Standard build
iarbuild Project.ewp -build Release

# Clean build
iarbuild Project.ewp -clean Release
iarbuild Project.ewp -build Release

# Debug configuration
iarbuild Project.ewp -build Debug
```

## Output Format

```
IAR Build Result
═══════════════════════════════════════
Project: Project.ewp
Config:  Release
Status:  SUCCESS / FAILED

Errors:   0
Warnings: 3
  Pe177: main.c(42) - variable declared but not referenced
  Pe550: uart.c(18) - variable set but never used
  Pa082: timer.c(95) - undefined behavior

Output:
  ELF: Release/Exe/Project.out
  HEX: Release/Exe/Project.hex
  MAP: Release/List/Project.map

Memory Usage:
  Flash: 45,312 / 262,144 bytes (17.3%)
  RAM:   8,704 /  49,152 bytes (17.7%)
═══════════════════════════════════════
```

## Post-Build

- .map 파일에서 메모리 사용량 자동 추출
- 에러 발생 시 관련 소스 코드 위치 표시
- 워닝 수 증가 시 경고

## Related Skills

- `/memory-map`: 상세 메모리 분석
- `/static-analysis`: MISRA-C 분석
