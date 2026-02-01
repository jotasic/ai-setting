---
name: architecture-review
description: 현재 시스템 아키텍처 분석 및 리뷰
argument-hint: [area] [--deep]
allowed-tools: Read, Grep, Glob, Bash
model: sonnet
category: understanding
---

# Architecture Review

시스템 아키텍처를 분석하고 개선 기회를 식별합니다.

## Triggers (사용 조건)

- "아키텍처 분석해줘", "review architecture"
- "구조 리뷰", "프로젝트 분석"
- 시스템 이해 또는 리팩토링 계획시

## Arguments

- `$ARGUMENTS`: 분석 영역
  - `structure`: 프로젝트 구조
  - `dependencies`: 의존성 관계
  - `data-flow`: 데이터 흐름
  - `security`: 보안 아키텍처
- `--deep`: 심층 분석

## Workflow

```
┌─────────────────────────────────────┐
│  1. Scan project structure          │
│  2. Analyze components              │
│  3. Map dependencies                │
│  4. Identify patterns               │
│  5. Generate report                 │
└─────────────────────────────────────┘
```

## Agent Integration

**상세 설계 분석:**
```
Use the architect agent to evaluate design decisions and trade-offs
```

**보안 분석:**
```
Use the security-auditor agent to review security architecture
```

**성능 분석:**
```
Use the performance-optimizer agent to analyze performance bottlenecks
```

## Output Format

```
Architecture Review: [project]
═══════════════════════════════════════
Pattern: Layered Architecture

Structure:
  src/
  ├── api/          # Presentation
  ├── services/     # Business Logic
  └── repositories/ # Data Access

Component Diagram:
  ┌─────────┐     ┌──────────┐
  │   API   │────▶│ Services │
  └─────────┘     └──────────┘
                        │
                        ▼
                  ┌──────────┐
                  │   Data   │
                  └──────────┘

Strengths:
  ✓ Clear separation of concerns
  ✓ Consistent naming

Concerns:
  ⚠ Circular dependency in services/
  ⚠ UserService too large (500+ LOC)

Recommendations:
  1. [High] Split UserService
  2. [Med] Add caching layer
═══════════════════════════════════════
```

## Examples

```bash
/architecture-review
/architecture-review dependencies
/architecture-review security --deep
```

## Related Skills

- `/explain-code`: 특정 코드 이해
- `/search-code`: 코드 검색
- `/full-dev`: 새 기능 개발시 구조 참고
