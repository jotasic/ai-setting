---
name: static-analysis
description: Run MISRA-C static analysis on embedded C code
argument-hint: [file|directory] [--misra] [--all]
allowed-tools: Bash, Read, Grep, Glob
model: sonnet
category: embedded-workflow
---

# Static Analysis

임베디드 C 코드에 대한 정적 분석을 수행합니다. MISRA-C:2012 규칙 위반 검출에 중점을 둡니다.

## Arguments

- `$ARGUMENTS`: 분석 대상 파일/디렉토리
- `--misra`: MISRA-C 규칙만 검사
- `--all`: 전체 프로젝트 분석

## Workflow

```
┌─────────────────────────────────────┐
│  1. Identify target files (.c/.h)   │
│  2. Check IAR C-STAT if available   │
│  3. Manual MISRA pattern scan       │
│  4. Volatile/ISR safety check       │
│  5. Generate report                 │
└─────────────────────────────────────┘
```

## Analysis Categories

### 1. MISRA-C:2012 Pattern Scan
- Type conversion issues (Rule 10.x)
- Pointer cast violations (Rule 11.x)
- Missing parentheses (Rule 12.1)
- Control flow issues (Rule 14-16)
- Missing function return usage (Rule 17.7)
- Dynamic allocation usage (Rule 21.3)

### 2. ISR Safety
- Missing `volatile` on shared variables
- Blocking calls in ISR context
- Non-atomic read-modify-write on shared data
- Missing interrupt pending bit clear

### 3. Memory Safety
- Buffer overflow potential
- Stack depth estimation
- Pointer arithmetic beyond bounds
- Uninitialized variable usage

### 4. IAR-Specific
- Incorrect `#pragma` usage
- Misused `__no_init` / `__root`
- Linker configuration issues

## Output Format

```
Static Analysis Report
═══════════════════════════════════════
Files Analyzed: 15
Total Issues: 23

CRITICAL (5):
  [MISRA 21.3] main.c:42 - malloc() usage detected
  [ISR-SAFETY] uart.c:105 - Missing volatile: g_rx_count
  [ISR-SAFETY] timer.c:78 - delay_ms() called in ISR context
  [MISRA 17.2] protocol.c:200 - Recursive function call
  [BUFFER] spi.c:55 - Array index may exceed bounds

WARNING (12):
  [MISRA 15.7] main.c:89 - if-else chain missing final else
  [MISRA 12.1] adc.c:34 - Missing parentheses in expression
  ...

INFO (6):
  [MISRA 8.7] utils.c:12 - Function could be static
  ...

Summary: 5 Critical | 12 Warning | 6 Info
═══════════════════════════════════════
```

## Related Skills

- `/iar-build`: 빌드 후 분석
- `/refactor-legacy`: 발견된 이슈 수정
