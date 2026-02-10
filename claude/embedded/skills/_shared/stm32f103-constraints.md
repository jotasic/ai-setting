# STM32F103VCT6 Constraints

## Hard Limits

| Resource | Limit | Critical Threshold (80%) |
|---------|-------|-------------------------|
| Flash | 256 KB (262,144 B) | 204 KB |
| SRAM | 48 KB (49,152 B) | 38 KB |
| GPIO Pins | 80 (LQFP100) | - |
| Max Clock | 72 MHz | - |
| APB1 Clock | 36 MHz max | - |

## Stack Guidelines

| Scenario | Recommended Stack (CSTACK) |
|---------|---------------------------|
| Simple (no nesting, few locals) | 1 KB |
| Moderate (1-2 ISR nesting levels) | 2 KB |
| Complex (deep calls, large locals) | 4 KB |
| With printf/sprintf | +2 KB (sprintf uses ~1.5KB stack) |

## Peripheral Limits

| Peripheral | Max Count | Notes |
|-----------|-----------|-------|
| USART | 3 | USART1=APB2, USART2-3=APB1 |
| SPI | 3 | SPI1=APB2, SPI2-3=APB1 |
| I2C | 2 | Both APB1 |
| ADC | 3 | 16 channels shared |
| Timer (Advanced) | 2 | TIM1, TIM8 |
| Timer (General) | 4 | TIM2-TIM5 |
| Timer (Basic) | 2 | TIM6-TIM7 |
| DMA Channels | 12 | DMA1=7ch, DMA2=5ch |
| CAN | 1 | Shared pins with USB |
| USB | 1 | Shared pins with CAN |

## Timing Constraints

| Operation | Approximate Duration |
|----------|---------------------|
| Flash write (half-word) | ~40 us |
| Flash page erase (2KB) | ~20 ms |
| ISR entry latency | 12 cycles (167 ns @ 72MHz) |
| Context save (ISR) | ~12-16 cycles |

## Common Pitfalls

1. **APB1 prescaler**: If HCLK > 36MHz, APB1 must have prescaler ≥ 2
2. **Timer clocks**: If APB prescaler ≠ 1, timer clock = APBx_clock × 2
3. **ADC clock**: Max 14 MHz (use prescaler from APB2)
4. **USB clock**: Must be exactly 48 MHz (PLL must provide)
5. **Flash latency**: 0 WS (≤24MHz), 1 WS (≤48MHz), 2 WS (≤72MHz)
