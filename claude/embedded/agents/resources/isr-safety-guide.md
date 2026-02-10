# ISR Safety Guide for STM32F103 (Cortex-M3)

## Golden Rules

1. **Keep ISRs short**: Set flag → exit. Process in main loop.
2. **Always clear pending bit**: Before or at start of ISR.
3. **volatile for all shared variables**: Compiler must not optimize away reads/writes.
4. **Protect read-modify-write**: Even 32-bit operations are not atomic if RMW.
5. **No blocking**: No `delay_ms()`, no `while(busy)`, no `printf`.

## volatile Correctness

### When volatile is Required
```c
/* Shared between ISR and main loop */
volatile uint8_t  g_uart_rx_flag;       /* REQUIRED */
volatile uint16_t g_adc_value;          /* REQUIRED */
volatile uint32_t g_systick_count;      /* REQUIRED */

/* Only used within ISR */
static uint8_t s_isr_local_counter;     /* Not needed */

/* Only used in main loop (never touched by ISR) */
static uint8_t s_config_value;          /* Not needed */
```

### volatile + const (Read-only from ISR perspective)
```c
/* Written in main, read in ISR (still needs volatile) */
volatile uint32_t g_timeout_value;
```

### volatile Pointer vs Pointer to volatile
```c
volatile uint8_t *p;   /* Pointer to volatile data (common) */
uint8_t * volatile p;  /* Volatile pointer to data (rare) */
volatile uint8_t * volatile p;  /* Both volatile (very rare) */
```

## Atomic Access on Cortex-M3

### What IS Atomic (Single Instruction)
```c
/* 8-bit, 16-bit, 32-bit aligned LOAD/STORE are atomic */
volatile uint32_t flag;
flag = 1;           /* Single STR instruction → atomic */
x = flag;           /* Single LDR instruction → atomic */
```

### What is NOT Atomic
```c
/* Read-Modify-Write is NEVER atomic */
counter++;          /* LDR + ADD + STR → NOT atomic */
flags |= 0x01;     /* LDR + ORR + STR → NOT atomic */
flags &= ~0x02;    /* LDR + BIC + STR → NOT atomic */

/* 64-bit access is NOT atomic on Cortex-M3 */
volatile uint64_t timestamp;  /* Two 32-bit operations */
```

## Critical Section Patterns

### Pattern 1: Disable/Enable Interrupts (Simple)
```c
__disable_interrupt();
shared_counter++;
__enable_interrupt();
```
**Problem**: If called from code that already has interrupts disabled, `__enable_interrupt()` prematurely re-enables them.

### Pattern 2: Save/Restore PRIMASK (Safe)
```c
uint32_t primask = __get_PRIMASK();
__disable_interrupt();
shared_counter++;
__set_PRIMASK(primask);
```
**Preferred**: Nestable, preserves caller's interrupt state.

### Pattern 3: Specific IRQ Disable (Minimal Latency)
```c
NVIC_DisableIRQ(TIM2_IRQn);
/* Modify data shared only with TIM2 ISR */
shared_tim2_data++;
NVIC_EnableIRQ(TIM2_IRQn);
```
**Best for**: Only blocking specific ISR while keeping others responsive.

## ISR Template

```c
/**
 * @brief  TIM2 Interrupt Handler
 * @note   Keep minimal: clear flag, set notification, exit
 */
void TIM2_IRQHandler(void)
{
    /* 1. Check interrupt source */
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        /* 2. Clear pending bit FIRST */
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

        /* 3. Minimal processing: set flag or buffer data */
        s_tim2_flag = 1;

        /* 4. Optional: quick counter/accumulator */
        s_tick_count++;
    }
    /* 5. NO delay, NO printf, NO blocking calls */
}
```

## NVIC Priority Configuration

### Priority Group (Set Once at Startup)
```c
/* Recommended: 4 bits preemption, 0 bits sub-priority */
NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
```

### Priority Assignment Strategy
```
Priority 0 (Highest) ── Fault handlers, Watchdog
Priority 1           ── Safety-critical timing
Priority 2           ── Communication RX (UART, SPI)
Priority 3           ── Communication TX
Priority 4           ── Timer interrupts
Priority 5           ── ADC conversion complete
Priority 6-14        ── Application-specific
Priority 15 (Lowest) ── Background tasks, SysTick
```

### Nested Interrupt Rules
- Higher preemption priority **can interrupt** lower priority ISR
- Same preemption priority: earlier arrival wins (no nesting)
- Within same preemption: sub-priority determines order in pending state

## Common ISR Bugs

### Bug 1: Forgotten Pending Bit Clear
```c
/* BUG: ISR fires continuously, system freezes */
void TIM2_IRQHandler(void)
{
    s_flag = 1;
    /* Missing: TIM_ClearITPendingBit(TIM2, TIM_IT_Update); */
}
```

### Bug 2: Data Race on Multi-Byte Read
```c
/* BUG: Main loop reads partial update */
/* ISR writes: s_timestamp = 0x0001_0000 → 0x0000_FFFF */
/* Main reads between two 16-bit writes → gets 0x0000_0000 or 0x0001_FFFF */

/* FIX: Read with interrupts disabled */
__disable_interrupt();
uint32_t local_ts = s_timestamp;
__enable_interrupt();
```

### Bug 3: Flag Lost Due to Non-Atomic Clear
```c
/* BUG: Check-then-clear is not atomic */
if (s_rx_flag) {          /* ISR sets flag HERE between check and clear */
    s_rx_flag = 0;        /* Clears the flag we didn't process! */
    process_rx();
}

/* FIX: Clear first, then process */
__disable_interrupt();
uint8_t flag = s_rx_flag;
s_rx_flag = 0;
__enable_interrupt();
if (flag) {
    process_rx();
}
```

### Bug 4: printf in ISR
```c
/* BUG: printf is NOT reentrant, takes milliseconds */
void USART1_IRQHandler(void)
{
    printf("UART IRQ\n");  /* NEVER DO THIS */
}
```
