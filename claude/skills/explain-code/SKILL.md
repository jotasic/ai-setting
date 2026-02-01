---
name: explain-code
description: 코드를 시각적 다이어그램과 비유를 사용하여 설명합니다
argument-hint: <file|function> [--detail]
allowed-tools: Read, Grep, Glob
model: sonnet
category: understanding
---

# Code Explanation

코드를 이해하기 쉽게 시각적으로 설명합니다.

## Triggers (사용 조건)

- "이 코드 설명해줘", "explain this"
- "이게 뭐하는 코드야?", "how does this work"
- 복잡한 로직 이해 필요시

## Arguments

- `$ARGUMENTS`: 파일 경로 또는 함수명
- `--detail`: 상세 설명 모드

## Workflow

```
┌─────────────────────────────────────┐
│  1. Read target code                │
│  2. Create analogy                  │
│  3. Draw visual diagram             │
│  4. Step-by-step walkthrough        │
│  5. Highlight gotchas               │
└─────────────────────────────────────┘
```

## Explanation Structure

### 1. Analogy First
> "This code is like a restaurant kitchen where..."

### 2. Visual Diagram
```
Input → Process → Output
  │         │
  └─────────┘
```

### 3. Step-by-Step Walkthrough
1. What happens first
2. How data transforms
3. What gets returned

### 4. Key Gotchas
- Edge cases
- Performance considerations
- Common misunderstandings

## Agent Integration

**심층 분석:**
```
Use the architect agent to analyze the design patterns used in [file]
```

**코드 개선:**
```
Use the refactorer agent to suggest improvements for [code]
```

## Output Format

```
Code: [file:function]
═══════════════════════════════════════
Analogy: [relatable comparison]

Flow:
  [ASCII diagram]

Steps:
  1. [first step]
  2. [second step]

Gotchas:
  - [important note]
═══════════════════════════════════════
```

## Examples

```bash
/explain-code src/auth/login.ts
/explain-code useAuth --detail
/explain-code src/utils/parser.py
```

## Related Skills

- `/search-code`: 관련 코드 찾기
- `/architecture-review`: 전체 아키텍처 이해
