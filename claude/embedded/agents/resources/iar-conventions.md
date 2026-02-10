# IAR EWARM Conventions & Extensions

## IAR-Specific Keywords

### Variable Placement
```c
/* Place variable at absolute address */
#pragma location = 0x20000000
uint32_t special_var;

/* Place in named section */
#pragma location = "MY_SECTION"
const uint8_t config_data[] = { ... };

/* Skip zero-initialization (faster startup, saves time) */
__no_init uint8_t dma_buffer[1024];

/* Prevent linker from removing (even if appears unused) */
__root const uint32_t firmware_version = 0x0100;

/* Pack struct (no padding) */
__packed struct {
    uint8_t  id;
    uint32_t value;  /* No padding before this */
};
```

### Function Attributes
```c
/* Execute function from RAM (faster than Flash) */
__ramfunc void time_critical_function(void);

/* Interrupt handler */
__irq __arm void USART1_IRQHandler(void);

/* No return function */
__noreturn void system_fatal_error(void);

/* Inline hint */
#pragma inline = forced
static void critical_inline_func(void);
```

### Intrinsic Functions
```c
#include <intrinsics.h>

__disable_interrupt();         /* CPSID I - disable IRQ */
__enable_interrupt();          /* CPSIE I - enable IRQ */
__no_operation();              /* NOP */
__get_MSP();                   /* Read Main Stack Pointer */
__set_MSP(value);              /* Set Main Stack Pointer */
__get_PSP();                   /* Read Process Stack Pointer */
__get_PRIMASK();               /* Read PRIMASK */
__set_PRIMASK(value);          /* Set PRIMASK */
__CLZ(value);                  /* Count Leading Zeros */
__RBIT(value);                 /* Reverse Bits */
__REV(value);                  /* Reverse Byte Order (32-bit) */
__REV16(value);                /* Reverse Byte Order (16-bit) */
__WFI();                       /* Wait For Interrupt */
__WFE();                       /* Wait For Event */
__DSB();                       /* Data Synchronization Barrier */
__ISB();                       /* Instruction Synchronization Barrier */
__DMB();                       /* Data Memory Barrier */
```

## Critical Section Pattern (IAR)

```c
/* Safe critical section using PRIMASK */
static inline uint32_t critical_section_enter(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_interrupt();
    return primask;
}

static inline void critical_section_exit(uint32_t primask)
{
    __set_PRIMASK(primask);
}

/* Usage */
uint32_t irq_state = critical_section_enter();
/* Critical code here - interrupts disabled */
shared_variable++;
critical_section_exit(irq_state);
```

## Pragma Directives

```c
/* Optimization control */
#pragma optimize = none             /* Disable optimization for this function */
#pragma optimize = speed            /* Optimize for speed */
#pragma optimize = size             /* Optimize for size */
#pragma optimize = balanced         /* Balance speed/size */

/* Data alignment */
#pragma data_alignment = 4          /* 4-byte align next variable */
#pragma data_alignment = 32         /* Cache-line align (for DMA) */

/* Pack control */
#pragma pack(push, 1)               /* Start packing */
struct my_packed_struct { ... };
#pragma pack(pop)                    /* Restore packing */

/* Section placement */
#pragma section = "MY_DATA"
#pragma location = "MY_DATA"

/* Diagnostics */
#pragma diag_suppress = Pe177       /* Suppress specific warning */
#pragma diag_warning = Pe177        /* Demote to warning */
#pragma diag_error = Pe177          /* Promote to error */
```

## Linker Configuration (.icf)

```
/* stm32f103vc.icf - IAR linker configuration */

define symbol __ICFEDIT_region_ROM_start__ = 0x08000000;
define symbol __ICFEDIT_region_ROM_end__   = 0x0803FFFF;  /* 256KB */
define symbol __ICFEDIT_region_RAM_start__ = 0x20000000;
define symbol __ICFEDIT_region_RAM_end__   = 0x2000BFFF;  /* 48KB */

define symbol __ICFEDIT_size_cstack__ = 0x800;   /* 2KB stack */
define symbol __ICFEDIT_size_heap__   = 0x000;   /* No heap */

define memory mem with size = 4G;
define region ROM_region = mem:[from __ICFEDIT_region_ROM_start__
                                to __ICFEDIT_region_ROM_end__];
define region RAM_region = mem:[from __ICFEDIT_region_RAM_start__
                                to __ICFEDIT_region_RAM_end__];

define block CSTACK with alignment = 8, size = __ICFEDIT_size_cstack__ { };

place at address mem:0x08000000 { readonly section .intvec };
place in ROM_region { readonly };
place in RAM_region { readwrite, block CSTACK };

initialize by copy { readwrite };
do not initialize { section .noinit };
```

## Project File Structure (.ewp)

```
Project.eww (Workspace)
└── Project.ewp (Project)
    ├── Application/
    │   ├── main.c
    │   └── stm32f10x_it.c
    ├── Drivers/
    │   ├── stm32f10x_conf.h
    │   └── system_stm32f10x.c
    ├── SPL/
    │   ├── stm32f10x_gpio.c
    │   ├── stm32f10x_rcc.c
    │   ├── stm32f10x_usart.c
    │   └── ...
    ├── CMSIS/
    │   ├── core_cm3.c
    │   └── startup_stm32f10x_hd.s
    └── Linker/
        └── stm32f103vc.icf
```

## IAR Build from Command Line

```bash
# Build project
iarbuild Project.ewp -build Release

# Clean and rebuild
iarbuild Project.ewp -clean Release
iarbuild Project.ewp -build Release

# Build with specific configuration
iarbuild Project.ewp -build "Debug"
iarbuild Project.ewp -build "Release"

# Output files (Release config)
# Project/Release/Exe/Project.out    (ELF)
# Project/Release/Exe/Project.hex    (Intel HEX)
# Project/Release/Exe/Project.bin    (Binary)
# Project/Release/List/Project.map   (Map file)
```

## Common IAR Warning IDs

| Warning | Description | Action |
|---------|-------------|--------|
| Pe177 | Variable declared but not referenced | Remove or add `(void)var;` |
| Pe550 | Variable set but never used | Remove assignment |
| Pa082 | Undefined behavior: signed overflow | Fix calculation |
| Pe068 | Integer conversion resulted in truncation | Add explicit cast |
| Pe188 | Enumerated type mixed with another type | Use explicit cast |
