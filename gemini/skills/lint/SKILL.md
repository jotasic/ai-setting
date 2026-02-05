---
name: lint
description: Run lint and format checks.
argument-hint: [file-path or --fix]
allowed-tools: Bash, Read, Grep, Glob
---

# Lint & Format

Check and fix code style.

## Arguments

- `$ARGUMENTS`: File path or `--fix`

## Workflow

### 1. Detect Linter
- ESLint, Flake8, Golangci-lint, Clippy

### 2. Run Linter
- Check: `eslint .`, `prettier --check .`
- Fix: `eslint . --fix`, `prettier --write .`

### 3. Report Issues
- List violations
- Auto-fix status
