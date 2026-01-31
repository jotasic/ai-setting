---
name: run-tests
description: 프로젝트의 테스트를 실행합니다
argument-hint: [test-path or pattern]
allowed-tools: Bash, Read, Grep, Glob
---

# Run Tests

테스트를 실행하고 결과를 분석합니다.

## Arguments

- `$ARGUMENTS`: 특정 테스트 파일/패턴 (선택사항)

## Workflow

### 1. Detect Test Framework

프로젝트의 테스트 프레임워크를 감지합니다:

```bash
# Node.js
test -f package.json && cat package.json | grep -E "jest|mocha|vitest"

# Python
test -f pytest.ini || test -f setup.py || test -f pyproject.toml

# Go
test -f go.mod

# Rust
test -f Cargo.toml
```

### 2. Run Tests

프레임워크에 맞는 테스트 명령 실행:

| Framework | Command |
|-----------|---------|
| Jest | `npm test` or `npx jest $ARGUMENTS` |
| Vitest | `npm test` or `npx vitest $ARGUMENTS` |
| Pytest | `pytest $ARGUMENTS` |
| Go | `go test ./... $ARGUMENTS` |
| Cargo | `cargo test $ARGUMENTS` |

### 3. Analyze Results

- 실패한 테스트 식별
- 에러 메시지 분석
- 수정 방안 제안

## Output

```
Test Results:
  Passed: X
  Failed: Y
  Skipped: Z

Failed Tests:
  - [test name]: [error summary]

Recommendations:
  - [fix suggestions]
```
