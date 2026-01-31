---
name: fix-issue
description: GitHub 이슈를 분석하고 수정 사항을 구현합니다
argument-hint: [issue-number]
allowed-tools: Read, Edit, Write, Bash, Grep, Glob
context: fork
---

# Fix GitHub Issue

## Issue Information

Issue to fix: $ARGUMENTS

## Workflow

### 1. Fetch Issue Details

```bash
gh issue view $ARGUMENTS
```

### 2. Analyze the Issue

- What is the expected behavior?
- What is the actual behavior?
- Are there reproduction steps?
- Any related issues or PRs?

### 3. Locate Relevant Code

Search for:
- Files mentioned in the issue
- Error messages or stack traces
- Related functionality

### 4. Implement Fix

- Make minimal, targeted changes
- Follow existing code patterns
- Add tests if applicable

### 5. Verify

- Run existing tests
- Test the specific scenario from the issue
- Check for regressions

## Output

Provide:
1. Summary of changes made
2. Files modified
3. How to test the fix
4. Any follow-up items
