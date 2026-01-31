---
name: new-feature
description: 새로운 기능을 구현합니다
argument-hint: [feature description]
allowed-tools: Read, Edit, Write, Bash, Grep, Glob
---

# Implement New Feature

새로운 기능을 체계적으로 구현합니다.

## Arguments

- `$ARGUMENTS`: 기능 설명

## Workflow

### 1. Requirements Analysis

- 기능 요구사항 파악
- 기존 코드베이스 분석
- 영향 범위 확인

### 2. Design

- 구현 방식 결정
- 파일 구조 계획
- API/인터페이스 설계

### 3. Implementation

```
┌─────────────────────────────────────┐
│  1. Create feature branch           │
│  2. Implement core logic            │
│  3. Add error handling              │
│  4. Write tests                     │
│  5. Update documentation            │
└─────────────────────────────────────┘
```

### 4. Verification

- 단위 테스트 실행
- 통합 테스트 실행
- 수동 테스트

### 5. Cleanup

- 코드 리뷰 준비
- 불필요한 코드 제거
- 린트/포맷팅 확인

## Output

```
Feature: [feature name]

Files Created:
  - [file path]

Files Modified:
  - [file path]

Tests Added:
  - [test descriptions]

Next Steps:
  - [ ] Code review
  - [ ] Merge to main
```

## Guidelines

- 작은 단위로 커밋
- 테스트 먼저 작성 (TDD) 고려
- 기존 패턴과 일관성 유지
