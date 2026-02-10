---
name: register-audit
description: Find direct register access and suggest SPL API replacements
argument-hint: [file|directory|--all]
allowed-tools: Read, Grep, Glob
model: sonnet
category: embedded-analysis
---

# Register Access Audit

직접 레지스터 접근 코드를 찾아 SPL API 대체안을 제시합니다.

## Arguments

- `$ARGUMENTS`: 분석 대상 (파일/디렉토리)
- `--all`: 전체 프로젝트

## Workflow

```
┌─────────────────────────────────────┐
│  1. Scan for register patterns      │
│     (GPIOx->, RCC->, TIMx->...)    │
│  2. Classify each access            │
│  3. Map to equivalent SPL API       │
│  4. Assess conversion risk          │
│  5. Generate conversion guide       │
└─────────────────────────────────────┘
```

## Detection Patterns

```c
/* Direct register access patterns to detect */
GPIOx->CRL, CRH, IDR, ODR, BSRR, BRR, LCKR
RCC->CR, CFGR, APB1ENR, APB2ENR, AHBENR
TIMx->CR1, CR2, ARR, PSC, CCR1-4, SR, DIER
USARTx->SR, DR, CR1, CR2, CR3, BRR
SPIx->CR1, CR2, SR, DR
I2Cx->CR1, CR2, SR1, SR2, DR, OAR1
ADCx->CR1, CR2, SQR1-3, DR, SR
DMAx_Channely->CCR, CNDTR, CPAR, CMAR
EXTI->IMR, EMR, RTSR, FTSR, PR
AFIO->MAPR, EXTICR1-4
SCB->AIRCR, SCR, CCR, SHP, CFSR, HFSR
FLASH->ACR, SR, CR, AR
```

## Conversion Examples

| Register Access | SPL Equivalent |
|----------------|----------------|
| `GPIOA->ODR \|= (1<<5)` | `GPIO_SetBits(GPIOA, GPIO_Pin_5)` |
| `GPIOA->ODR &= ~(1<<5)` | `GPIO_ResetBits(GPIOA, GPIO_Pin_5)` |
| `GPIOA->IDR & (1<<3)` | `GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3)` |
| `RCC->APB2ENR \|= (1<<2)` | `RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE)` |
| `TIM2->SR &= ~(1<<0)` | `TIM_ClearFlag(TIM2, TIM_FLAG_Update)` |
| `USART1->DR = byte` | `USART_SendData(USART1, byte)` |
| `while(!(USART1->SR & (1<<7)))` | `while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)` |

## Risk Assessment

| Risk | Condition | Action |
|------|-----------|--------|
| **LOW** | Simple GPIO set/reset | Safe to convert |
| **LOW** | RCC clock enable | Safe to convert |
| **MEDIUM** | Timer configuration | Verify prescaler/period math |
| **MEDIUM** | UART baud rate | Verify BRR calculation |
| **HIGH** | ISR flag manipulation | Test timing carefully |
| **HIGH** | DMA register setup | Verify all fields |
| **SKIP** | Performance-critical tight loops | Keep register access |

## Output Format

```
Register Access Audit
═══════════════════════════════════════
Files Scanned: 15
Direct Register Accesses Found: 42

LOW RISK (Convert):
  main.c:55     GPIOB->ODR |= (1<<5)
                → GPIO_SetBits(GPIOB, GPIO_Pin_5)

  main.c:120    RCC->APB2ENR |= (1<<3)
                → RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE)

MEDIUM RISK (Review):
  timer.c:34    TIM2->PSC = 71;
                → TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;

HIGH RISK (Careful):
  isr.c:12      TIM2->SR = ~(1<<0);
                → TIM_ClearITPendingBit(TIM2, TIM_IT_Update)

SKIP (Keep as-is):
  fast_loop.c:89  GPIOA->BSRR in tight timing loop

Summary: 28 LOW | 8 MEDIUM | 4 HIGH | 2 SKIP
═══════════════════════════════════════
```

## Related Skills

- `/peripheral-init`: SPL 초기화 코드 생성
- `/refactor-legacy`: 변환 실행
