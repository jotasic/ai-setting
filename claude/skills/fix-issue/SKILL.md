---
name: fix-issue
description: GitHub 이슈를 분석하고 수정 사항을 구현합니다
argument-hint: <issue-number> [--no-test]
allowed-tools: Read, Edit, Write, Bash, Grep, Glob
model: sonnet
category: development
context: fork
---

# Fix GitHub Issue

GitHub 이슈를 분석하고 버그를 수정합니다.

## Triggers (사용 조건)

- "이슈 #123 수정해줘", "fix issue"
- "버그 수정", "bug fix"
- GitHub 이슈 번호 언급 시

## Arguments

- `$ARGUMENTS`: 이슈 번호 (예: 123, #123)
- `--no-test`: 테스트 작성 스킵

## Workflow

```
┌─────────────────────────────────────┐
│  1. Fetch issue details (gh)        │
│  2. Analyze with debugger agent     │
│  3. Implement fix                   │
│  4. Write tests                     │
│  5. Verify & create PR              │
└─────────────────────────────────────┘
```

## Agent Integration

**버그 분석:**
```
Use the debugger agent to analyze issue #[number] and identify root cause
```

**코드 수정:**
```
Use the backend-developer agent to fix [bug description] in [file]
```
또는
```
Use the frontend-developer agent to fix [bug description] in [file]
```

**테스트 추가:**
```
Use the test-writer agent to write regression tests for issue #[number]
```

**코드 리뷰:**
```
Use the code-reviewer agent to review the fix for issue #[number]
```

## Step-by-Step

### 1. Fetch Issue Details

```bash
gh issue view $ARGUMENTS --json title,body,labels,comments
```

### 2. Analyze the Issue

- What is the expected behavior?
- What is the actual behavior?
- Are there reproduction steps?
- Any related issues or PRs?

### 3. Locate & Fix

- Search for relevant files
- Use debugger agent for root cause analysis
- Implement minimal, targeted fix

### 4. Verify

```bash
npm test                    # Run tests
npm run build              # Verify build
```

## Output Format

```
Issue #[number] Fix Summary
═══════════════════════════════════════
Root Cause: [description]

Files Modified:
  - [file path]: [change description]

Tests Added:
  - [test description]

Verification:
  ✓ Tests passing
  ✓ Build successful
  ✓ Issue scenario resolved
═══════════════════════════════════════
```

## Examples

```bash
/fix-issue 123
/fix-issue #456 --no-test
```

## Related Skills

- `/new-feature`: 새 기능 구현
- `/review-pr`: PR 리뷰
- `/run-tests`: 테스트 실행
