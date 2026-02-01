---
name: skill-name
description: 스킬 설명 (한 줄)
argument-hint: <required> [optional]
allowed-tools: Read, Edit, Write, Bash, Grep, Glob
model: sonnet  # haiku | sonnet | opus
category: development  # development | infrastructure | testing | documentation | workflow
---

# Skill Name

간단한 설명.

## Triggers (사용 조건)

다음 상황에서 이 스킬을 사용:
- "키워드1" 언급 시
- "키워드2" 요청 시
- 특정 상황 설명

## Arguments

- `$ARGUMENTS`: 인자 설명
- `--option`: 옵션 설명

## Workflow

```
┌─────────────────────────────────────┐
│  1. Step 1                          │
│  2. Step 2                          │
│  3. Step 3                          │
└─────────────────────────────────────┘
```

## Agent Integration

이 스킬은 다음 에이전트를 순차적으로 호출:

```
Use the [agent-name] agent to [task]
```

## Output Format

```
Result Summary
═══════════════════════════════════════
✓ Step 1: [result]
✓ Step 2: [result]
═══════════════════════════════════════
```

## Examples

```bash
/skill-name argument --option
```

## Related Skills

- `/related-skill-1`
- `/related-skill-2`
