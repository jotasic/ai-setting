---
name: estimate-effort
description: 코드 변경 규모 및 영향도 분석
argument-hint: <task-description>
allowed-tools: Read, Grep, Glob
model: sonnet
category: understanding
---

# Estimate Effort

작업의 규모와 영향 범위를 분석합니다.

## Triggers (사용 조건)

- "이거 얼마나 걸려?", "effort estimate"
- "작업 규모 분석", "impact analysis"
- 작업 계획 전 규모 파악 필요시

## Arguments

- `$ARGUMENTS`: 작업 설명

## Workflow

```
┌─────────────────────────────────────┐
│  1. Parse task description          │
│  2. Find related files              │
│  3. Analyze dependencies            │
│  4. Assess risks                    │
│  5. Generate estimate               │
└─────────────────────────────────────┘
```

## Complexity Levels

| Level | Description | Files |
|-------|-------------|-------|
| Low | 단순 변경 | 1-3 |
| Medium | 여러 컴포넌트 | 4-10 |
| High | 횡단 관심사 | 10+ |
| Very High | 아키텍처 변경 | 전체 |

## Agent Integration

**상세 설계 분석:**
```
Use the architect agent to analyze technical approach for [task]
```

**영향 범위 확인:**
```
Use the code-reviewer agent to identify all affected areas
```

## Output Format

```
Effort Estimate: [task]
═══════════════════════════════════════
Complexity: Medium

Scope:
  Files to modify: 8
  Files to create: 2
  Estimated LOC: ~250
  Tests to add: 12

Affected Areas:
  - src/api/users/
  - src/services/auth/
  - tests/

Risks:
  ⚠ Breaking API changes (Medium)
  ○ Performance impact (Low)

Approach:
  1. Type definitions
  2. Core logic
  3. API endpoints
  4. Tests

Checklist:
  - [ ] Schema changes
  - [ ] Business logic
  - [ ] Tests
  - [ ] Documentation
═══════════════════════════════════════
```

## Examples

```bash
/estimate-effort Add user profile image upload
/estimate-effort Implement OAuth2 authentication
/estimate-effort Migrate to new payment provider
```

## Related Skills

- `/architecture-review`: 시스템 구조 파악
- `/full-dev`: 전체 개발 플로우
- `/new-feature`: 기능 구현
