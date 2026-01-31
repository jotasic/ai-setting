---
name: run-tests
description: Run project tests.
argument-hint: [test-path or pattern]
allowed-tools: Bash, Read, Grep, Glob
---

# Run Tests

Executes tests and analyzes results.

## Arguments

- `$ARGUMENTS`: path or pattern (optional)

## Workflow

### 1. Detect Framework
- Node (jest, vitest, mocha)
- Python (pytest, unittest)
- Go (go test)
- Rust (cargo test)

### 2. Run Tests
- Execute appropriate command

### 3. Analyze Results
- Identify failures
- Suggest fixes

## Output
- Results summary (Pass/Fail)
- Specific failures with error messages
- Recommendations
