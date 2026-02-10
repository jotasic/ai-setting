# SPL API Patterns & Anti-Patterns

## Standard Initialization Pattern

Every SPL peripheral follows this sequence:

```c
/* 1. Declare init struct */
XXX_InitTypeDef XXX_InitStructure;

/* 2. Enable clock */
RCC_APBxPeriphClockCmd(RCC_APBxPeriph_XXX, ENABLE);

/* 3. Fill init struct */
XXX_InitStructure.field1 = value1;
XXX_InitStructure.field2 = value2;

/* 4. Call Init */
XXX_Init(XXXx, &XXX_InitStructure);

/* 5. Enable peripheral */
XXX_Cmd(XXXx, ENABLE);
```

## Common SPL Struct Defaults to Watch

### GPIO_InitTypeDef
```c
/* Always set ALL fields - no default values */
GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_X;    /* REQUIRED */
GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_XXX; /* REQUIRED */
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_XXX; /* Required for output modes */
```

### USART_InitTypeDef
```c
USART_InitStructure.USART_BaudRate            = 115200;
USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
USART_InitStructure.USART_StopBits            = USART_StopBits_1;
USART_InitStructure.USART_Parity              = USART_Parity_No;
USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
```

### TIM_TimeBaseInitTypeDef
```c
/* Timer frequency = TIMxCLK / (Prescaler + 1) / (Period + 1) */
TIM_TimeBaseStructure.TIM_Prescaler     = prescaler - 1;  /* NOTE: -1 */
TIM_TimeBaseStructure.TIM_Period        = period - 1;      /* NOTE: -1 */
TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;  /* Only TIM1/TIM8 */
```

## Anti-Patterns

### 1. Forgetting RCC Clock Enable
```c
/* BAD: Peripheral access without clock → HardFault or silent failure */
GPIO_Init(GPIOA, &GPIO_InitStructure);

/* GOOD: Always enable clock first */
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
GPIO_Init(GPIOA, &GPIO_InitStructure);
```

### 2. Missing AFIO Clock for Remapping
```c
/* BAD: Remap without AFIO clock */
GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);

/* GOOD: Enable AFIO clock first */
RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
```

### 3. Wrong Struct Reuse
```c
/* BAD: Reusing struct without clearing */
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOA, &GPIO_InitStructure);

/* Forgot to change Mode for input pin */
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
/* GPIO_Mode still Out_PP! */
GPIO_Init(GPIOA, &GPIO_InitStructure);

/* GOOD: Reset all fields or use StructInit */
GPIO_StructInit(&GPIO_InitStructure);  /* Reset to defaults */
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
GPIO_Init(GPIOA, &GPIO_InitStructure);
```

### 4. Incorrect Timer Frequency Calculation
```c
/* BAD: Off-by-one */
/* Wanted 1kHz from 72MHz APB2 timer */
TIM_TimeBaseStructure.TIM_Prescaler = 72;    /* Actually /73 */
TIM_TimeBaseStructure.TIM_Period = 1000;      /* Actually /1001 */
/* Real freq = 72MHz / 73 / 1001 = 985.6 Hz (wrong!) */

/* GOOD: Account for +1 in hardware */
TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;   /* /72 */
TIM_TimeBaseStructure.TIM_Period = 1000 - 1;      /* /1000 */
/* Real freq = 72MHz / 72 / 1000 = 1000 Hz (correct) */
```

### 5. Not Clearing Interrupt Pending Bit
```c
/* BAD: Pending bit not cleared → ISR fires endlessly */
void TIM2_IRQHandler(void)
{
    /* process... */
}

/* GOOD: Clear pending bit FIRST */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        /* process... */
    }
}
```

### 6. Enabling Interrupt Before Configuration Complete
```c
/* BAD: ISR fires before peripheral is ready */
NVIC_Init(&NVIC_InitStructure);  /* Enable IRQ */
USART_Init(USART1, &USART_InitStructure);  /* Init after! */

/* GOOD: Configure fully, then enable IRQ last */
USART_Init(USART1, &USART_InitStructure);
USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
NVIC_Init(&NVIC_InitStructure);  /* Enable IRQ last */
```

## StructInit Functions (Reset to Defaults)

```c
GPIO_StructInit(&GPIO_InitStructure);
USART_StructInit(&USART_InitStructure);
TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
SPI_StructInit(&SPI_InitStructure);
I2C_StructInit(&I2C_InitStructure);
DMA_StructInit(&DMA_InitStructure);
ADC_StructInit(&ADC_InitStructure);
```

## SPL Error Checking

```c
/* SPL provides assert_param() macro */
/* Enable in stm32f10x_conf.h: */
#define USE_FULL_ASSERT

/* Implement in main.c: */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* Log or breakpoint for debugging */
    while (1);
}
```
