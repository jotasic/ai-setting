---
name: code-quality
description: Run code quality pipeline (lint, test, security).
argument-hint: [--fix]
allowed-tools: Bash, Read, Grep, Glob
---

# Code Quality Pipeline

Runs sequential code quality checks.

## Arguments

- `--fix`: Fix fixable issues automatically.

## Pipeline Steps

1. **Type Check**: TypeScript, Python (mypy), Go (vet)
2. **Lint**: ESLint, Prettier, Ruff, Black, Golangci-lint
3. **Test**: Unit/Integration tests with coverage
4. **Security**: Audit dependencies and secrets
5. **Build**: Verify build succeeds

## Output Summary

```
Code Quality Report
═══════════════════════════════════════
✓ Type Check    : PASSED
✓ Lint          : PASSED
✓ Tests         : PASSED
✓ Security      : PASSED
✓ Build         : PASSED
═══════════════════════════════════════
Overall: PASSED
```
