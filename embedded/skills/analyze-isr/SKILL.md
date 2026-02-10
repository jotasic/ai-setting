---
name: analyze-isr
description: Analyze ISR handlers for safety, priority conflicts, and timing
argument-hint: [file|--all]
allowed-tools: Read, Grep, Glob
model: sonnet
category: embedded-analysis
---

# ISR Analysis

인터럽트 핸들러의 안전성, 우선순위 충돌, 실행 시간을 분석합니다.

## Arguments

- `$ARGUMENTS`: 분석 대상 파일 (기본: stm32f10x_it.c + 전체 프로젝트)
- `--all`: 모든 ISR 포함 파일 분석

## Workflow

```
┌─────────────────────────────────────┐
│  1. Find all ISR handlers           │
│  2. Map shared variables per ISR    │
│  3. Check volatile correctness      │
│  4. Detect blocking calls in ISR    │
│  5. Analyze priority assignments    │
│  6. Estimate execution depth        │
│  7. Report findings                 │
└─────────────────────────────────────┘
```

## Analysis Points

### Safety Checks
- `volatile` on all ISR-shared variables
- No `delay_ms()`, `printf()`, `malloc()` in ISR
- Pending bit cleared in every ISR
- Critical section protection for shared RMW operations

### Priority Analysis
- NVIC priority group configuration
- Priority assignment per ISR
- Potential priority inversion
- Nesting depth estimation

### Timing Estimation
- Function call depth from ISR
- Loop iterations in ISR path
- Worst-case execution estimate (instruction count)

## Output Format

```
ISR Analysis Report
═══════════════════════════════════════
ISR Handlers Found: 8

┌──────────────────┬──────┬──────────┬─────────────┐
│ Handler          │ Prio │ Shared   │ Issues      │
├──────────────────┼──────┼──────────┼─────────────┤
│ SysTick_Handler  │ 15   │ tick_cnt │ OK          │
│ USART1_IRQHandler│ 2    │ rx_buf   │ volatile!   │
│ TIM2_IRQHandler  │ 4    │ flags    │ blocking!   │
│ DMA1_CH4_IRQ     │ 3    │ tx_buf   │ OK          │
└──────────────────┴──────┴──────────┴─────────────┘

CRITICAL:
  [1] USART1_IRQHandler: g_rx_count missing volatile
  [2] TIM2_IRQHandler: delay_ms(10) called in ISR

WARNING:
  [3] No NVIC_PriorityGroupConfig found in init code
═══════════════════════════════════════
```

## Related Skills

- `/static-analysis`: 전체 정적 분석
- `/refactor-legacy`: ISR 리팩토링
