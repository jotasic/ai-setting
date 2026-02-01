---
name: performance-profile
description: 성능 프로파일링 및 벤치마크 실행
argument-hint: [target] [--report]
allowed-tools: Bash, Read, Grep, Glob
model: sonnet
category: infrastructure
---

# Performance Profile

성능 병목 지점을 분석하고 벤치마크를 실행합니다.

## Triggers (사용 조건)

- "성능 분석해줘", "profile performance"
- "느린 부분 찾아줘", "benchmark"
- 성능 최적화 필요시

## Arguments

- `$ARGUMENTS`: 프로파일링 대상
- `--report`: 상세 리포트 생성

## Workflow

```
┌─────────────────────────────────────┐
│  1. Identify target                 │
│  2. Run profiling tools             │
│  3. Analyze results                 │
│  4. Generate recommendations        │
└─────────────────────────────────────┘
```

## Profiling Tools

| Platform | Tool |
|----------|------|
| Node.js | `node --prof` |
| React | `React DevTools` |
| Frontend | `Lighthouse` |
| Bundle | `webpack-bundle-analyzer` |
| DB | `EXPLAIN ANALYZE` |

## Agent Integration

**성능 최적화:**
```
Use the performance-optimizer agent to implement optimizations for [bottleneck]
```

**코드 리팩토링:**
```
Use the refactorer agent to optimize [slow-function]
```

**DB 쿼리 최적화:**
```
Use the database-specialist agent to optimize slow queries
```

## Output Format

```
Performance Profile: [target]
═══════════════════════════════════════
Metrics:
  Response time (p50): 120ms
  Response time (p99): 450ms
  Memory usage: 256MB

Bottlenecks:
  1. [High] DB query in getUserList (200ms)
  2. [Med] Image processing (80ms)

Recommendations:
  1. Add index on users.email
  2. Implement lazy loading
  3. Enable caching

Expected Improvement: 40%
═══════════════════════════════════════
```

## Examples

```bash
/performance-profile                    # 전체 분석
/performance-profile src/api/users      # 특정 모듈
/performance-profile --report           # 상세 리포트
```

## Related Skills

- `/architecture-review`: 구조 분석
- `/code-quality`: 품질 검사
- `/db-migrate`: DB 최적화
