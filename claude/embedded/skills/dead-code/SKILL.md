---
name: dead-code
description: Detect unused functions, variables, macros, and includes
argument-hint: [file|directory|--all]
allowed-tools: Read, Grep, Glob, Bash
model: sonnet
category: embedded-analysis
---

# Dead Code Detection

미사용 함수, 변수, 매크로, include를 탐지합니다.

## Arguments

- `$ARGUMENTS`: 분석 대상 (기본: 전체 프로젝트)
- `--all`: 전체 프로젝트

## Workflow

```
┌─────────────────────────────────────┐
│  1. List all function definitions   │
│  2. Search for each function usage  │
│  3. List all global variables       │
│  4. Search for each variable usage  │
│  5. Check #define macro usage       │
│  6. Check #include necessity        │
│  7. Report unused items             │
└─────────────────────────────────────┘
```

## Detection Categories

| Category | Method | Notes |
|---------|--------|-------|
| Unused functions | Defined but never called | Exclude ISR handlers, `__root` |
| Unused variables | Declared but never read | Exclude `volatile` ISR flags carefully |
| Unused macros | `#define` but never referenced | |
| Unused includes | `#include` but no symbol used from it | |
| Unused SPL modules | .c in project but no API called | Saves Flash |
| Dead branches | `if(0)`, `#if 0` blocks | |

## Exceptions (Do NOT Flag)

- ISR handlers (called by hardware, not code)
- `__root` annotated variables/functions (intentionally kept)
- `startup_stm32f10x_hd.s` vector table entries
- Callback functions registered via function pointers
- `assert_failed()` (called by SPL assert macro)

## Output Format

```
Dead Code Report
═══════════════════════════════════════
Files Scanned: 25

Unused Functions (5):
  old_parse_v1()     protocol.c:145  [~480 bytes Flash]
  debug_dump()       utils.c:89      [~320 bytes Flash]
  SPI3_Init()        spi_driver.c:67 [~256 bytes Flash]
  ...

Unused Variables (3):
  g_debug_mode       main.c:12       [4 bytes RAM]
  s_old_buffer[256]  comm.c:34       [256 bytes RAM]
  ...

Unused SPL Modules (2):
  stm32f10x_dac.c   [~1.2 KB Flash]  ← Remove from project
  stm32f10x_can.c   [~2.8 KB Flash]  ← Remove from project

Potential Savings:
  Flash: ~5.1 KB
  RAM:   ~260 bytes
═══════════════════════════════════════
```

## Related Skills

- `/memory-map`: 메모리 사용 분석
- `/refactor-legacy`: 코드 정리
