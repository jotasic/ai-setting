---
name: lint
description: 코드 린트 및 포맷팅을 실행합니다
argument-hint: [file-path or --fix]
allowed-tools: Bash, Read, Grep, Glob
---

# Lint & Format

코드 스타일을 검사하고 자동 수정합니다.

## Arguments

- `$ARGUMENTS`: 대상 파일 또는 `--fix` 옵션

## Workflow

### 1. Detect Linter

```bash
# JavaScript/TypeScript
test -f .eslintrc* || test -f eslint.config.*

# Python
test -f .flake8 || test -f pyproject.toml

# Go
which golangci-lint

# Rust
which clippy
```

### 2. Run Linter

| Tool | Check | Fix |
|------|-------|-----|
| ESLint | `npx eslint .` | `npx eslint . --fix` |
| Prettier | `npx prettier --check .` | `npx prettier --write .` |
| Ruff | `ruff check .` | `ruff check . --fix` |
| Black | `black --check .` | `black .` |
| golangci-lint | `golangci-lint run` | `golangci-lint run --fix` |
| Clippy | `cargo clippy` | `cargo clippy --fix` |

### 3. Report Issues

- 스타일 위반 목록
- 자동 수정 가능 여부
- 수동 수정 필요 항목

## Output

```
Lint Results:
  Errors: X
  Warnings: Y
  Auto-fixable: Z

Issues:
  [file:line] [rule] [message]

To auto-fix, run:
  /lint --fix
```
