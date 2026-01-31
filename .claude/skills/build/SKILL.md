---
name: build
description: 프로젝트를 빌드합니다
argument-hint: [build-target]
allowed-tools: Bash, Read, Grep, Glob
---

# Build Project

프로젝트를 빌드하고 결과를 확인합니다.

## Arguments

- `$ARGUMENTS`: 빌드 타겟 (예: production, development)

## Workflow

### 1. Detect Build System

```bash
# Node.js
test -f package.json && cat package.json | grep -E "\"build\""

# Python
test -f setup.py || test -f pyproject.toml

# Go
test -f go.mod

# Rust
test -f Cargo.toml

# Make
test -f Makefile
```

### 2. Run Build

| System | Command |
|--------|---------|
| npm | `npm run build` |
| yarn | `yarn build` |
| pnpm | `pnpm build` |
| Go | `go build ./...` |
| Cargo | `cargo build --release` |
| Make | `make build` |
| Python | `python -m build` |

### 3. Verify Output

- 빌드 결과물 확인
- 빌드 에러 분석
- 경고 메시지 검토

## Output

```
Build Status: [Success/Failed]

Output:
  - [build artifacts]

Warnings:
  - [warning messages]

Errors (if any):
  - [error details]
  - [suggested fixes]
```
