---
name: module-extract
description: Extract a module from a monolithic C file into separate .c/.h pair
argument-hint: <source-file> <module-name> [function-list]
allowed-tools: Read, Edit, Write, Grep, Glob, Bash
model: sonnet
category: embedded-feature
---

# Module Extract

거대한 단일 .c 파일에서 기능별 모듈을 분리합니다.

## Arguments

- `$ARGUMENTS`:
  - `<source-file>`: 원본 파일 (예: main.c)
  - `<module-name>`: 추출할 모듈명 (예: bsp_uart)
  - `[function-list]`: 추출할 함수 목록 (선택, 없으면 자동 분석)

## Workflow

```
┌─────────────────────────────────────────┐
│  1. Analyze source file                 │
│  2. Identify function group             │
│  3. Map variable dependencies           │
│  4. Create module_name.h (public API)   │
│  5. Create module_name.c (implementation)│
│  6. Update original file                │
│  7. Build verification                  │
└─────────────────────────────────────────┘
```

## Extraction Rules

1. **Move together**: Functions + their private variables + their types
2. **Public API**: Only functions called from outside the module
3. **Static by default**: All moved functions start as `static` unless externally called
4. **Preserve volatile**: Never drop `volatile` on moved variables
5. **ISR stays**: ISR handlers remain in original or dedicated ISR file
6. **Build after move**: Every extraction must compile cleanly

## Generated Module Structure

### Header (module_name.h)
```c
#ifndef MODULE_NAME_H
#define MODULE_NAME_H

#include "stm32f10x.h"

/* Types */
/* Public API */

#endif /* MODULE_NAME_H */
```

### Source (module_name.c)
```c
#include "module_name.h"

/* Private defines */
/* Private variables (static) */
/* Private functions (static) */
/* Public functions */
```

## Output Format

```
Module Extraction Complete
═══════════════════════════════════════
Source: main.c (2400 → 1920 lines, -20%)
Created: bsp_uart.c (320 lines), bsp_uart.h (45 lines)

Moved: 8 functions, 5 variables
  Public:  UART_Init, UART_Send, UART_Receive
  Static:  configure_gpio, configure_nvic, ...

Build: PASS
Memory: Flash +96B (+0.04%), RAM +0B
═══════════════════════════════════════
```

## Related Skills

- `/refactor-legacy`: 전체 리팩토링 워크플로우
- `/dead-code`: 추출 후 미사용 코드 확인
