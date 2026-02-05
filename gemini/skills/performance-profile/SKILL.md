---
name: performance-profile
description: detailed performance profiling and benchmarking.
argument-hint: [target]
---

# Performance Profile

Analyze performance hotspots and run benchmarks.

## Arguments

- `$ARGUMENTS`: Target (file/function) - optional

## Workflow

1. **Identify Target**
2. **Run Profiling**
   - Node.js: `node --prof`
   - Frontend: `lighthouse`
   - Build: `npm run build -- --analyze`
3. **Analyze Results**
   - CPU/Memory/Network/IO
4. **Generate Report**

## Common Checks

### Frontend
- Bundle size, FCP, TTI, CLS

### Backend
- Response time, DB query time, Memory/CPU

### Database
- `EXPLAIN ANALYZE`

## Output
- Current Metrics
- Bottlenecks
- Recommendations
