---
name: type-check-improve
description: TypeScript/Python 타입 검사 강화
argument-hint: [path] [--strict]
allowed-tools: Bash, Read, Edit, Write, Grep, Glob
model: sonnet
category: documentation
---

# Type Check & Improve

타입 커버리지를 분석하고 타입 안전성을 강화합니다.

## Triggers (사용 조건)

- "타입 검사해줘", "type check"
- "타입 커버리지 개선", "fix any types"
- 타입 안전성 강화 필요시

## Arguments

- `$ARGUMENTS`: 검사 경로 (default: `./src`)
- `--strict`: strict 모드 적용

## Workflow

```
┌─────────────────────────────────────┐
│  1. Run type checker                │
│  2. Analyze coverage                │
│  3. Identify issues                 │
│  4. Generate improvements           │
└─────────────────────────────────────┘
```

## Type Check Commands

| Language | Command |
|----------|---------|
| TypeScript | `npx tsc --noEmit` |
| Python | `mypy .` / `pyright` |

## Agent Integration

**타입 에러 수정:**
```
Use the debugger agent to fix type errors in [file]
```

**타입 정의 작성:**
```
Use the backend-developer agent to add proper type definitions
```

## Output Format

```
Type Coverage Report
═══════════════════════════════════════
Coverage: 78% → Target: 95%

Issues Found:
  - src/api.ts:45 - Implicit any
  - src/utils.ts:23 - Missing return type

Improvements Made:
  ✓ Added types to 15 functions
  ✓ Replaced 8 `any` types
  ✓ Added 3 type guards

Remaining:
  - 12 implicit any
  - 5 missing return types
═══════════════════════════════════════
```

## Examples

```bash
/type-check-improve                # 전체 검사
/type-check-improve src/api        # 특정 경로
/type-check-improve --strict       # strict 모드
```

## Related Skills

- `/lint`: 코드 스타일 검사
- `/code-quality`: 전체 품질 검사
