# Refactoring Protocol (5 Phases)

## Phase 1: ANALYZE (embedded-architect)

### Input
- Target file(s) or module name

### Actions
1. Count total lines per file
2. List all `extern` global variables
3. Map function call graph (who calls whom)
4. Identify ISR handlers and their data dependencies
5. List all `#include` dependencies
6. Measure coupling (number of cross-file references)

### Output
```
Analysis Report
├── File inventory: [files with line counts]
├── Global variable map: [var → files that use it]
├── Function call graph: [caller → callee]
├── ISR dependency list: [ISR → shared variables]
└── Coupling score: [high/medium/low per file pair]
```

---

## Phase 2: PLAN (embedded-refactorer)

### Input
- Phase 1 analysis report

### Actions
1. Group related functions into proposed modules
2. Identify extraction order (least-coupled first)
3. Assess risk per extraction (ISR involvement = high risk)
4. Estimate Flash/RAM impact
5. Define new public API for each module

### Output
```
Refactoring Plan
├── Module 1: bsp_uart
│   ├── Functions to move: [list]
│   ├── Globals to encapsulate: [list]
│   ├── New API: [function signatures]
│   ├── Risk: LOW (no ISR changes)
│   └── Order: 1st
├── Module 2: bsp_timer
│   ├── Functions to move: [list]
│   ├── ISR affected: TIM2_IRQHandler
│   ├── Risk: MEDIUM
│   └── Order: 2nd
└── ...
```

---

## Phase 3: EXTRACT (firmware-developer + peripheral-specialist)

### Per Module Extraction

1. **Create header file** (.h)
   - Include guard
   - Public type definitions
   - Public function declarations
   - No implementation details

2. **Create source file** (.c)
   - Include own header
   - Move functions from original file
   - Make internal functions `static`
   - Make internal variables `static`
   - Preserve `volatile` on ISR-shared vars

3. **Update original file**
   - Add `#include "new_module.h"`
   - Remove moved functions
   - Remove moved variables
   - Replace direct access with API calls

4. **Build verification**
   - `iarbuild Project.ewp -build Release`
   - Zero new errors
   - Warning count should not increase

### ISR Extraction Rules
- ISR handler function stays in `stm32f10x_it.c` or dedicated ISR file
- ISR calls module API (callback or flag check)
- Shared data stays in the module that owns it
- Never move ISR handler to application layer

---

## Phase 4: VERIFY (embedded-code-reviewer + memory-optimizer)

### Code Review Checks
- [ ] All moved functions compile
- [ ] No new `extern` globals introduced
- [ ] All `volatile` preserved
- [ ] `static` applied to all internal symbols
- [ ] Include guards present
- [ ] No circular dependencies
- [ ] MISRA rule compliance maintained

### Memory Verification
- [ ] Flash usage delta ≤ +2% (overhead from function calls)
- [ ] RAM usage delta ≈ 0 (no new buffers)
- [ ] Stack depth unchanged (verify in .map)

---

## Phase 5: DOCUMENT

### Generate per-module summary
```
Refactoring Summary: [module_name]
═══════════════════════════════════════
Source: main.c (2400 lines → 1800 lines)
New Module: bsp_uart.c/.h (280 lines)

Moved:
  Functions: 8 (3 public, 5 static)
  Variables: 5 (all now static, 2 volatile)
  ISR: USART1_IRQHandler (calls UART_IRQCallback)

API:
  void UART_Init(uint32_t baudrate);
  void UART_Send(const uint8_t *data, uint16_t len);
  uint8_t UART_IsRxReady(void);
  uint16_t UART_Read(uint8_t *buf, uint16_t max_len);

Memory Impact:
  Flash: +128 bytes (+0.05%)
  RAM:   +0 bytes

Build: PASS (0 errors, 0 new warnings)
═══════════════════════════════════════
```
