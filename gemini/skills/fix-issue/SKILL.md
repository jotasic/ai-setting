---
name: fix-issue
description: Analyze and fix GitHub issues.
argument-hint: [issue-number]
allowed-tools: Read, Edit, Write, Bash, Grep, Glob
context: fork
---

# Fix GitHub Issue

## Issue Information
Issue: $ARGUMENTS

## Workflow

### 1. Fetch Issue
`gh issue view $ARGUMENTS`

### 2. Analyze
- Expected vs Actual
- Reproduction steps

### 3. Locate Code
- Search files, error messages

### 4. Implement Fix
- Minimal changes
- Add tests

### 5. Verify
- Run tests
- Check regressions

## Output
1. Summary
2. Files modified
3. Verification steps
