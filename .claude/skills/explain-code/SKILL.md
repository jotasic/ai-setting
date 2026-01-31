---
name: explain-code
description: 코드를 시각적 다이어그램과 비유를 사용하여 설명합니다
argument-hint: [filename or code snippet]
allowed-tools: Read, Grep, Glob
---

# Code Explanation Skill

When explaining code, follow this structure:

## 1. Analogy First

Start with a relatable analogy:
> "This code is like a restaurant kitchen where..."

## 2. Visual Diagram

Create an ASCII diagram showing the flow:

```
Input → Process → Output
  │         │
  └─────────┘
```

## 3. Step-by-Step Walkthrough

Break down the code into logical steps:
1. What happens first
2. How data transforms
3. What gets returned

## 4. Key Gotchas

Highlight common mistakes or misconceptions:
- Edge cases to watch for
- Performance considerations
- Common misunderstandings

---

**File to explain:** $ARGUMENTS

If no file is specified, ask the user which file or code they want explained.
