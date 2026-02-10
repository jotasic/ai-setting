# MISRA-C:2012 Key Rules Checklist

## Required Rules (Must Follow)

### Types & Conversions (Rules 10-11)
- [ ] **10.1**: No implicit conversion of operands with different essential types
- [ ] **10.3**: No narrowing assignment without explicit cast
- [ ] **10.4**: Both operands of arithmetic operators must have same essential type
- [ ] **10.8**: No cast to wider essential type of complex expression
- [ ] **11.1**: No conversions between function pointer and other types
- [ ] **11.3**: No cast between pointer to object and pointer to different object
- [ ] **11.6**: No cast between void pointer and arithmetic type

### Expressions (Rules 12-13)
- [ ] **12.1**: Explicit precedence with parentheses
- [ ] **12.2**: Shift count must be in range [0, bit-width - 1]
- [ ] **12.4**: Constant expression evaluation must not overflow
- [ ] **13.1**: Initializer lists must not contain side effects
- [ ] **13.5**: RHS of `&&` or `||` must not have persistent side effects

### Control Flow (Rules 14-16)
- [ ] **14.1**: No unreachable code (except defensive `default:`)
- [ ] **14.3**: No dead code (code that executes but has no effect)
- [ ] **14.4**: Controlling expression of if/while must be boolean
- [ ] **15.1**: No `goto` (exception: error cleanup pattern)
- [ ] **15.7**: All `if...else if` chains must end with `else`
- [ ] **16.1**: All switch statements must have `default` case
- [ ] **16.3**: Every switch clause must end with `break` or comment `/* fall through */`
- [ ] **16.4**: Every switch must have at least two clauses
- [ ] **16.6**: Every switch must have at least two non-default clauses

### Functions (Rules 17)
- [ ] **17.1**: No `<stdarg.h>` (no variadic functions)
- [ ] **17.2**: No recursion
- [ ] **17.3**: No implicit function declarations
- [ ] **17.7**: Return value of non-void function must be used

### Pointers (Rules 18)
- [ ] **18.1**: No pointer arithmetic resulting in out-of-bounds
- [ ] **18.4**: No `+`, `-`, `+=`, `-=` on pointers (prefer array indexing)
- [ ] **18.6**: No use of address of automatic variable beyond its lifetime

### Standard Library (Rules 21)
- [ ] **21.1**: No `#define` or `#undef` of standard library identifiers
- [ ] **21.3**: No `<stdlib.h>` memory functions (`malloc`, `calloc`, `realloc`, `free`)
- [ ] **21.6**: No `<stdio.h>` input/output functions

## Advisory Rules (Recommended)

### Declarations (Rules 8)
- [ ] **8.7**: Functions/objects used in single file should be `static`
- [ ] **8.9**: Objects used in single function should be local, not file-scope
- [ ] **8.13**: Pointer parameters not modified should be `const`

### Expressions
- [ ] **12.3**: Comma operator not used
- [ ] **13.3**: Side effects in full expressions only

### Control Flow
- [ ] **15.4**: No more than one `break`/`goto` per loop
- [ ] **15.5**: No more than one loop exit point

## Embedded-Critical Subset

For STM32 firmware, these are the most impactful rules:

### Memory Safety
```c
/* Rule 18.1: Array bounds */
uint8_t buf[10];
buf[10] = 0;  /* VIOLATION: out of bounds */

/* Rule 21.3: No dynamic allocation */
void *p = malloc(100);  /* VIOLATION */
```

### Type Safety
```c
/* Rule 10.3: No implicit narrowing */
uint8_t x = 256;  /* VIOLATION: truncation */

/* Rule 10.4: Same essential type */
uint16_t a = 1;
int32_t b = 2;
uint16_t c = a + b;  /* VIOLATION: mixed types */
```

### Volatile & ISR Safety (Not MISRA but Critical)
```c
/* All ISR-shared variables MUST be volatile */
volatile uint8_t isr_flag;  /* CORRECT */
uint8_t isr_flag;           /* BUG: optimizer may cache */

/* Read-modify-write is NOT atomic */
__disable_interrupt();
shared_counter++;
__enable_interrupt();
```

### Defensive Programming
```c
/* Rule 15.7: Complete else chain */
if (state == STATE_IDLE) {
    /* ... */
} else if (state == STATE_RUNNING) {
    /* ... */
} else {
    /* Required: handle unexpected state */
    Error_Handler();
}

/* Rule 16.1: Default in switch */
switch (cmd) {
    case CMD_START: break;
    case CMD_STOP: break;
    default:
        Error_Handler();  /* Required */
        break;
}
```
