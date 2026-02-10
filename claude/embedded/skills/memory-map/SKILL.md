---
name: memory-map
description: Analyze IAR .map file for Flash/RAM usage breakdown
argument-hint: [map-file-path]
allowed-tools: Bash, Read, Grep, Glob
model: sonnet
category: embedded-workflow
---

# Memory Map Analysis

IAR 링커의 .map 파일을 파싱하여 Flash/RAM 사용량을 분석합니다.

## Arguments

- `$ARGUMENTS`: .map 파일 경로 (없으면 프로젝트에서 자동 탐색)

## Workflow

```
┌─────────────────────────────────────┐
│  1. Find .map file                  │
│  2. Parse MODULE SUMMARY section    │
│  3. Parse PLACEMENT SUMMARY         │
│  4. Calculate per-module usage      │
│  5. Identify optimization targets   │
│  6. Generate visual report          │
└─────────────────────────────────────┘
```

## Map File Key Sections

1. **MODULE SUMMARY**: Per-object file breakdown (ro code, ro data, rw data)
2. **ENTRY LIST**: All symbols with sizes
3. **PLACEMENT SUMMARY**: Section placement in memory

## Output Format

```
Memory Usage Report - STM32F103VCT6
═══════════════════════════════════════

Flash: 87,040 / 262,144 bytes (33.2%)
▓▓▓▓▓▓▓▓▓▓░░░░░░░░░░░░░░░░░░░░ 33.2%

RAM:   12,288 / 49,152 bytes (25.0%)
▓▓▓▓▓▓▓░░░░░░░░░░░░░░░░░░░░░░░ 25.0%

─── Flash Breakdown (Top 10) ───────────
  1. main.o            12,480 B  (14.3%)
  2. protocol.o         8,192 B  ( 9.4%)
  3. stm32f10x_rcc.o    5,120 B  ( 5.9%)
  4. sprintf.o           4,096 B  ( 4.7%)  ← [CAN OPTIMIZE]
  5. uart_driver.o      3,072 B  ( 3.5%)
  ...

─── RAM Breakdown (Top 10) ─────────────
  1. rx_buffer          2,048 B  (16.7%)  ← [CAN REDUCE?]
  2. tx_buffer          1,024 B  ( 8.3%)
  3. CSTACK             2,048 B  (16.7%)
  4. frame_buffer         512 B  ( 4.2%)
  ...

─── Section Summary ────────────────────
  .text     68,544 B  (code)
  .rodata   14,336 B  (const data)
  .data      4,096 B  (initialized vars → in Flash + RAM)
  .bss       8,192 B  (zero-init vars → RAM only)
  CSTACK     2,048 B  (stack → RAM)

═══════════════════════════════════════
```

## Related Skills

- `/iar-build`: 빌드 후 맵 파일 생성
- `/dead-code`: 미사용 코드 탐지
