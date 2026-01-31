---
name: performance-profile
description: 성능 프로파일링 및 벤치마크 실행
argument-hint: [target]
---

# Performance Profile

성능 병목 지점을 분석하고 벤치마크를 실행합니다.

## Arguments

- `$ARGUMENTS`: 프로파일링 대상 (optional)

## Workflow

1. **Identify Target**
   - Specific file/function if provided
   - Otherwise, scan for performance hotspots

2. **Run Profiling**
   ```bash
   # Node.js
   node --prof app.js
   node --prof-process isolate-*.log > profile.txt

   # React/Frontend
   npx lighthouse http://localhost:3000 --output html

   # Bundle analysis
   npm run build -- --analyze
   ```

3. **Analyze Results**
   - CPU time analysis
   - Memory usage patterns
   - I/O bottlenecks
   - Network requests

4. **Generate Report**
   - Current performance metrics
   - Identified bottlenecks
   - Optimization recommendations
   - Expected improvements

## Common Checks

### Frontend
- Bundle size and chunks
- First Contentful Paint (FCP)
- Time to Interactive (TTI)
- Cumulative Layout Shift (CLS)

### Backend
- Response time (p50, p95, p99)
- Database query time
- Memory usage
- CPU utilization

### Database
```sql
EXPLAIN (ANALYZE, BUFFERS) SELECT ...
```

## Output

Provide a detailed performance report with:
- Current metrics
- Bottleneck analysis
- Prioritized recommendations
- Benchmark results (before/after if applicable)
