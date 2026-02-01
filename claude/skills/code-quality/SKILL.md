---
name: code-quality
description: 코드 품질 검사 파이프라인을 실행합니다 (lint, test, type-check, security)
argument-hint: [--fix] [--skip=<steps>]
allowed-tools: Bash, Read, Grep, Glob
model: haiku
category: workflow
---

# Code Quality Pipeline

전체 코드 품질 검사를 순차적으로 실행합니다.

## Triggers (사용 조건)

- "코드 품질 검사해줘", "quality check"
- "PR 전에 검사", "commit 전 확인"
- "lint, test 다 돌려줘"

## Arguments

- `--fix`: 자동 수정 가능한 문제 수정
- `--skip=<steps>`: 특정 단계 스킵 (예: `--skip=security,build`)

## Pipeline Steps

```
┌─────────────────────────────────────────────────────┐
│  1. Type Check    → 타입 에러 검사                    │
│  2. Lint          → 코드 스타일 검사                   │
│  3. Test          → 단위/통합 테스트                   │
│  4. Security      → 보안 취약점 스캔                   │
│  5. Build         → 빌드 검증                        │
└─────────────────────────────────────────────────────┘
```

## Agent Integration

문제 발견 시 전문 에이전트 호출:

**타입 에러 수정:**
```
Use the debugger agent to fix type errors in [file]
```

**보안 취약점:**
```
Use the security-auditor agent to review and fix vulnerabilities
```

**코드 리뷰:**
```
Use the code-reviewer agent to review changes before commit
```

## Step 1: Type Check

```bash
# TypeScript
npx tsc --noEmit

# Python (mypy)
mypy .

# Go
go vet ./...
```

## Step 2: Lint

```bash
# JavaScript/TypeScript
npx eslint . $FIX_FLAG
npx prettier --check . $FIX_FLAG

# Python
ruff check . $FIX_FLAG
black --check . $FIX_FLAG

# Go
golangci-lint run $FIX_FLAG
```

## Step 3: Test

```bash
# with coverage
npm test -- --coverage
pytest --cov
go test -cover ./...
```

## Step 4: Security Scan

```bash
# Dependencies
npm audit
pip-audit
cargo audit

# Secrets
git secrets --scan
gitleaks detect
```

## Step 5: Build Verification

```bash
npm run build
python -m build
go build ./...
cargo build --release
```

## Output Summary

```
Code Quality Report
═══════════════════════════════════════
✓ Type Check    : PASSED
✓ Lint          : PASSED (3 warnings)
✓ Tests         : PASSED (42/42)
✓ Security      : PASSED (0 vulnerabilities)
✓ Build         : PASSED
═══════════════════════════════════════
Overall: PASSED
```

## Examples

```bash
/code-quality                    # 전체 검사
/code-quality --fix              # 자동 수정 포함
/code-quality --skip=security    # 보안 검사 스킵
```

## Related Skills

- `/lint`: 린트만 실행
- `/run-tests`: 테스트만 실행
- `/dependency-audit`: 의존성 보안 검사
- `/commit`: 커밋 (품질 검사 후)
