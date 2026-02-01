---
name: new-feature
description: 새로운 기능을 구현합니다
argument-hint: <feature description>
allowed-tools: Read, Edit, Write, Bash, Grep, Glob
model: sonnet
category: development
---

# Implement New Feature

새로운 기능을 체계적으로 구현합니다.

## Triggers (사용 조건)

- "새 기능 만들어줘", "feature 구현"
- "기능 추가해줘"
- 단일 기능 구현 요청 시 (전체 플로우는 `/full-dev` 사용)

## Arguments

- `$ARGUMENTS`: 기능 설명

## Workflow

```
┌─────────────────────────────────────┐
│  1. Analyze requirements            │
│  2. Design implementation           │
│  3. Implement with agent            │
│  4. Write tests                     │
│  5. Verify & cleanup                │
└─────────────────────────────────────┘
```

## Agent Integration

구현 대상에 따라 적절한 에이전트 호출:

**프론트엔드 기능:**
```
Use the frontend-developer agent to implement [feature] UI component
```

**백엔드 기능:**
```
Use the backend-developer agent to implement [feature] API endpoint
```

**유틸리티/스크립트:**
```
Use the general-developer agent to implement [feature] utility
```

**테스트 작성:**
```
Use the test-writer agent to write tests for [feature]
```

## Output Format

```
Feature: [feature name]
═══════════════════════════════════════
Files Created:
  - [file path]

Files Modified:
  - [file path]

Tests Added:
  - [test descriptions]

Next Steps:
  - [ ] Code review
  - [ ] Merge to main
═══════════════════════════════════════
```

## Examples

```bash
/new-feature 사용자 프로필 이미지 업로드
/new-feature 다크모드 토글 버튼
/new-feature 결제 내역 조회 API
```

## Related Skills

- `/full-dev`: 기획부터 문서화까지 전체 플로우
- `/fix-issue`: 버그 수정
- `/run-tests`: 테스트 실행
