# STM32F103VCT6 Quick Reference

## Chip Overview

| Item | Value |
|------|-------|
| Core | ARM Cortex-M3 |
| Max Clock | 72 MHz |
| Flash | 256 KB |
| SRAM | 48 KB |
| Package | LQFP100 |
| Density | High-density |
| Operating Voltage | 2.0 - 3.6V |
| I/O Pins | 80 (5V tolerant on most) |

## Memory Map

| Region | Start | End | Size |
|--------|-------|-----|------|
| Flash | 0x0800_0000 | 0x0803_FFFF | 256 KB |
| SRAM | 0x2000_0000 | 0x2000_BFFF | 48 KB |
| Peripherals | 0x4000_0000 | 0x4002_3FFF | - |
| Cortex-M3 Internal | 0xE000_0000 | 0xE00F_FFFF | - |
| System Memory (Bootloader) | 0x1FFF_F000 | 0x1FFF_F7FF | 2 KB |
| Option Bytes | 0x1FFF_F800 | 0x1FFF_F80F | 16 B |

## Clock Tree

```
HSE (4-16 MHz, typical 8 MHz)
  │
  ├──→ PLLSRC ──→ PLLMUL (×2..×16) ──→ PLLCLK
  │                                        │
HSI (8 MHz)                                 │
  │                                        │
  └──→ PLLSRC (/2) ──→ PLLMUL ───────────┘
                                           │
                          SW (System Clock Source Select)
                           │
                           ▼
                     SYSCLK (max 72 MHz)
                           │
                    AHB Prescaler (/1,2,4,...,512)
                           │
                     HCLK (max 72 MHz)
                       │       │
                APB1 Prescaler  APB2 Prescaler
                 (/1,2,4,8,16)  (/1,2,4,8,16)
                       │              │
                PCLK1 (max 36MHz)  PCLK2 (max 72MHz)
                       │              │
                APB1 Timers ×2    APB2 Timers ×2
                (if prescaler≠1)  (if prescaler≠1)
```

## Standard Clock Configuration (72 MHz)

```c
/* HSE = 8MHz, PLL = 8MHz × 9 = 72MHz */
RCC_HSEConfig(RCC_HSE_ON);
RCC_WaitForHSEStartUp();
RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
RCC_PLLCmd(ENABLE);
RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
RCC_HCLKConfig(RCC_SYSCLK_Div1);      /* AHB  = 72 MHz */
RCC_PCLK1Config(RCC_HCLK_Div2);       /* APB1 = 36 MHz */
RCC_PCLK2Config(RCC_HCLK_Div1);       /* APB2 = 72 MHz */
FLASH_SetLatency(FLASH_Latency_2);     /* 2 wait states for 72MHz */
```

## Peripheral Bus Assignment

### APB2 (max 72 MHz)
GPIOA-E, USART1, SPI1, TIM1, TIM8, ADC1-3, AFIO, EXTI

### APB1 (max 36 MHz)
USART2-3, UART4-5, SPI2-3, I2C1-2, TIM2-7, CAN1, USB, DAC, BKP, PWR

### AHB
DMA1-2, SDIO, FSMC, CRC, SRAM

## GPIO Pin Modes (SPL)

| Mode | SPL Constant | Usage |
|------|-------------|-------|
| Analog Input | `GPIO_Mode_AIN` | ADC input |
| Floating Input | `GPIO_Mode_IN_FLOATING` | UART RX, default |
| Pull-up/Pull-down | `GPIO_Mode_IPU` / `GPIO_Mode_IPD` | Button input |
| Push-Pull Output | `GPIO_Mode_Out_PP` | LED, chip select |
| Open-Drain Output | `GPIO_Mode_Out_OD` | I2C (legacy) |
| AF Push-Pull | `GPIO_Mode_AF_PP` | UART TX, SPI |
| AF Open-Drain | `GPIO_Mode_AF_OD` | I2C SDA/SCL |

## Common Pin Assignments (Default, No Remap)

| Peripheral | TX/MOSI/SDA | RX/MISO/SCL | CLK/SCK | NSS/CS |
|-----------|-------------|-------------|---------|--------|
| USART1 | PA9 | PA10 | - | - |
| USART2 | PA2 | PA3 | - | - |
| USART3 | PB10 | PB11 | - | - |
| SPI1 | PA7 | PA6 | PA5 | PA4 |
| SPI2 | PB15 | PB14 | PB13 | PB12 |
| I2C1 | PB7 (SDA) | - | PB6 (SCL) | - |
| I2C2 | PB11 (SDA) | - | PB10 (SCL) | - |
| CAN1 | PA12 (TX) | PA11 (RX) | - | - |

## NVIC Interrupt Vectors (Key)

| IRQn | Handler Name | Source |
|------|-------------|--------|
| -3 | HardFault_Handler | All faults |
| -5 | SVC_Handler | Supervisor call |
| -2 | PendSV_Handler | Pendable SV |
| -1 | SysTick_Handler | System tick |
| 6 | EXTI0_IRQHandler | EXTI Line 0 |
| 23 | EXTI9_5_IRQHandler | EXTI Lines 5-9 |
| 25 | TIM1_UP_IRQHandler | TIM1 Update |
| 28 | TIM2_IRQHandler | TIM2 global |
| 29 | TIM3_IRQHandler | TIM3 global |
| 30 | TIM4_IRQHandler | TIM4 global |
| 37 | USART1_IRQHandler | USART1 global |
| 38 | USART2_IRQHandler | USART2 global |
| 39 | USART3_IRQHandler | USART3 global |
| 35 | SPI1_IRQHandler | SPI1 global |
| 36 | SPI2_IRQHandler | SPI2 global |
| 11 | DMA1_Channel1_IRQHandler | DMA1 CH1 |

## Flash Page Size

- High-density (STM32F103VCT6): **2KB per page**
- Total pages: 128 (256KB / 2KB)
