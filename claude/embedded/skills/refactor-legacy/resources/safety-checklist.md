# Refactoring Safety Checklist

## Pre-Refactoring

- [ ] Current code builds without errors
- [ ] .map file baseline saved (Flash/RAM usage)
- [ ] All modified files committed to git (clean state)
- [ ] ISR handler list documented
- [ ] Global variable inventory complete

## During Refactoring (Per Module)

### Code Move
- [ ] All related functions moved together
- [ ] All related variables moved together
- [ ] `volatile` keyword preserved on ISR-shared variables
- [ ] `static` applied to file-internal functions
- [ ] `static` applied to file-internal variables
- [ ] Include guard added to new header
- [ ] Original file updated to `#include` new header

### Build Check
- [ ] `iarbuild` succeeds (zero errors)
- [ ] No new warnings introduced
- [ ] Warning count ≤ previous count

### Memory Check
- [ ] Flash delta ≤ +2% (function call overhead acceptable)
- [ ] RAM delta ≈ 0
- [ ] Stack size unchanged in .icf

### ISR Safety
- [ ] ISR handler names unchanged (match startup .s file)
- [ ] ISR pending bit clear logic unchanged
- [ ] ISR execution path not lengthened
- [ ] No new function calls added inside ISR body
- [ ] Shared variable access pattern unchanged

## Post-Refactoring

- [ ] Full clean build succeeds
- [ ] Final .map file compared with baseline
- [ ] No circular includes (`A.h → B.h → A.h`)
- [ ] All `extern` declarations reviewed and minimized
- [ ] Module dependency is unidirectional (no cycles)
- [ ] Changes committed with descriptive message
