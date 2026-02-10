---
name: peripheral-init
description: Generate STM32 peripheral initialization code using SPL
argument-hint: <peripheral> [options]
allowed-tools: Read, Write, Edit, Grep, Glob
model: sonnet
category: embedded-feature
---

# Peripheral Init Generator

STM32F103VCT6의 페리페럴 초기화 코드를 SPL 기반으로 생성합니다.

## Arguments

- `$ARGUMENTS`: 페리페럴 이름 및 설정
- 예: `USART1 115200`, `SPI1 master`, `TIM2 1ms`, `ADC1 PA0`, `I2C1 100kHz`

## Supported Peripherals

| Peripheral | Options |
|-----------|---------|
| `USART1-3` | baudrate, parity, stopbits |
| `SPI1-3` | master/slave, speed, mode(0-3) |
| `I2C1-2` | speed (100k/400k), address |
| `TIM1-8` | period, mode (basic/pwm/input-capture) |
| `ADC1-3` | channel, trigger, DMA |
| `DMA` | peripheral, direction, mode |
| `GPIO` | port, pin, mode |
| `EXTI` | line, trigger (rising/falling/both) |

## Generated Code Structure

```c
/* 1. RCC clock enable */
RCC_APBxPeriphClockCmd(..., ENABLE);

/* 2. GPIO configuration */
GPIO_InitTypeDef GPIO_InitStructure;
GPIO_Init(GPIOx, &GPIO_InitStructure);

/* 3. NVIC configuration (if interrupt) */
NVIC_InitTypeDef NVIC_InitStructure;
NVIC_Init(&NVIC_InitStructure);

/* 4. DMA configuration (if DMA) */
DMA_InitTypeDef DMA_InitStructure;
DMA_Init(DMAx_Channely, &DMA_InitStructure);

/* 5. Peripheral configuration */
XXX_InitTypeDef XXX_InitStructure;
XXX_Init(XXXx, &XXX_InitStructure);
XXX_Cmd(XXXx, ENABLE);
```

## Output

- Complete init function with all dependencies
- Correct RCC, GPIO, NVIC configuration
- ISR template if interrupt mode selected
- Pin mapping comment

## Related Skills

- `/register-audit`: 레지스터 직접 접근 코드 변환
