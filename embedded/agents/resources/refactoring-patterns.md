# Embedded Legacy Refactoring Pattern Catalog

## Pattern 1: God-File Decomposition

### Problem
Single file (main.c or system.c) contains 2000+ lines mixing initialization, business logic, ISR handlers, and peripheral code.

### Solution
```
Before:                          After:
main.c (2500 lines)             main.c (100 lines)
                                 ├── bsp_gpio.c/.h
                                 ├── bsp_uart.c/.h
                                 ├── bsp_spi.c/.h
                                 ├── bsp_timer.c/.h
                                 ├── app_comm.c/.h
                                 └── app_control.c/.h
```

### Steps
1. Map all functions with their categories (init, ISR, logic, peripheral)
2. Group by peripheral or feature
3. Create new module files with clean APIs
4. Move functions one group at a time
5. Make internals `static`, export only API
6. Build after each move

---

## Pattern 2: Global Variable Encapsulation

### Problem
```c
/* Scattered across multiple files */
extern uint8_t g_rx_buffer[512];
extern volatile uint16_t g_rx_count;
extern volatile uint8_t g_rx_complete;
extern uint8_t g_tx_buffer[512];
extern volatile uint16_t g_tx_count;
```

### Solution
```c
/* uart_driver.h - Clean API */
typedef enum {
    UART_OK = 0,
    UART_BUSY,
    UART_TIMEOUT,
    UART_ERROR
} UART_Status_t;

UART_Status_t UART_Send(const uint8_t *data, uint16_t len);
UART_Status_t UART_Receive(uint8_t *data, uint16_t *len, uint32_t timeout_ms);
uint8_t UART_IsRxReady(void);

/* uart_driver.c - Encapsulated state */
static uint8_t s_rx_buffer[512];
static volatile uint16_t s_rx_count;
static volatile uint8_t s_rx_complete;
```

---

## Pattern 3: Callback Decoupling

### Problem
ISR directly calls application functions, creating tight coupling.

```c
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE)) {
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        protocol_parse_byte(USART_ReceiveData(USART1));  /* Tight coupling */
    }
}
```

### Solution
```c
/* uart_driver.h */
typedef void (*UART_RxCallback_t)(uint8_t byte);
void UART_RegisterRxCallback(UART_RxCallback_t cb);

/* uart_driver.c */
static UART_RxCallback_t s_rx_callback = NULL;

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE)) {
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        uint8_t byte = USART_ReceiveData(USART1);
        if (s_rx_callback) {
            s_rx_callback(byte);
        }
    }
}

/* app_init.c - Registration */
UART_RegisterRxCallback(protocol_parse_byte);
```

---

## Pattern 4: State Machine Extraction

### Problem
State logic implemented with nested if-else and scattered flags.

```c
if (g_mode == 1 && g_step == 0 && !g_waiting) {
    if (g_counter > 100) {
        g_step = 1;
        g_counter = 0;
    }
} else if (g_mode == 1 && g_step == 1) {
    /* ... 50 more lines ... */
}
```

### Solution
```c
/* state_machine.h */
typedef enum {
    SM_STATE_IDLE,
    SM_STATE_INIT,
    SM_STATE_RUNNING,
    SM_STATE_ERROR,
    SM_STATE_COUNT
} SM_State_t;

typedef enum {
    SM_EVENT_START,
    SM_EVENT_TIMEOUT,
    SM_EVENT_ERROR,
    SM_EVENT_RESET,
    SM_EVENT_COUNT
} SM_Event_t;

void SM_Init(void);
void SM_ProcessEvent(SM_Event_t event);
SM_State_t SM_GetState(void);

/* state_machine.c */
typedef SM_State_t (*SM_Handler_t)(SM_Event_t event);

static SM_State_t s_current_state = SM_STATE_IDLE;

static SM_State_t handle_idle(SM_Event_t event);
static SM_State_t handle_init(SM_Event_t event);
static SM_State_t handle_running(SM_Event_t event);
static SM_State_t handle_error(SM_Event_t event);

static const SM_Handler_t s_handlers[SM_STATE_COUNT] = {
    [SM_STATE_IDLE]    = handle_idle,
    [SM_STATE_INIT]    = handle_init,
    [SM_STATE_RUNNING] = handle_running,
    [SM_STATE_ERROR]   = handle_error,
};

void SM_ProcessEvent(SM_Event_t event)
{
    SM_State_t next = s_handlers[s_current_state](event);
    if (next != s_current_state) {
        s_current_state = next;
    }
}
```

---

## Pattern 5: Magic Number Elimination

### Problem
```c
TIM_TimeBaseStructure.TIM_Prescaler = 71;
TIM_TimeBaseStructure.TIM_Period = 999;
if (adc_value > 2482) { ... }
GPIOB->ODR |= 0x0020;
delay_ms(150);
```

### Solution
```c
/* config.h or module-specific header */
#define SYSTEM_CLOCK_MHZ        72
#define TIM_1MS_PRESCALER       (SYSTEM_CLOCK_MHZ - 1)     /* 71 */
#define TIM_1MS_PERIOD          (1000 - 1)                   /* 999 */

#define ADC_VREF_MV             3300
#define ADC_MAX_VALUE           4096
#define MV_TO_ADC(mv)           ((mv) * ADC_MAX_VALUE / ADC_VREF_MV)
#define VOLTAGE_THRESHOLD_MV    2000

#define LED_STATUS_PIN          GPIO_Pin_5
#define LED_STATUS_PORT         GPIOB

#define DEBOUNCE_TIME_MS        150

/* Usage */
TIM_TimeBaseStructure.TIM_Prescaler = TIM_1MS_PRESCALER;
TIM_TimeBaseStructure.TIM_Period = TIM_1MS_PERIOD;
if (adc_value > MV_TO_ADC(VOLTAGE_THRESHOLD_MV)) { ... }
GPIO_SetBits(LED_STATUS_PORT, LED_STATUS_PIN);
delay_ms(DEBOUNCE_TIME_MS);
```

---

## Pattern 6: Layered Architecture Introduction

### Problem
Flat structure where application code directly manipulates registers/peripherals.

### Solution
```
┌─────────────────────────────────┐
│  Application Layer              │  app_main.c, app_comm.c
│  (Business logic, state machines)│
├─────────────────────────────────┤
│  Service Layer (optional)       │  svc_protocol.c, svc_datalog.c
│  (Cross-cutting concerns)       │
├─────────────────────────────────┤
│  Driver Layer                   │  drv_uart.c, drv_spi.c
│  (Peripheral abstraction)       │
├─────────────────────────────────┤
│  BSP Layer                      │  bsp_gpio.c, bsp_clock.c
│  (Board-specific pin mapping)   │
├─────────────────────────────────┤
│  SPL / CMSIS                    │  stm32f10x_*.c, core_cm3.c
│  (Vendor library)               │
└─────────────────────────────────┘

Rules:
- Each layer only calls the layer directly below
- No upward calls (use callbacks instead)
- No skipping layers
```

---

## Refactoring Priority Matrix

| Code Smell | Risk if Unfixed | Effort | Priority |
|-----------|----------------|--------|----------|
| Missing volatile | Runtime bug | Low | **P0** |
| ISR blocking | System hang | Low | **P0** |
| God-file (2000+ lines) | Unmaintainable | Medium | **P1** |
| Global variable jungle | Race conditions | Medium | **P1** |
| Magic numbers | Misconfiguration | Low | **P2** |
| Copy-paste code | Inconsistency | Medium | **P2** |
| Mixed register/SPL | Confusion | Medium | **P3** |
| Missing error handling | Silent failure | Medium | **P3** |
