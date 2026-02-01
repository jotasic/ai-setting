---
name: full-dev
description: 요구사항부터 개발 완료까지 전체 플로우 실행
argument-hint: <feature-description>
allowed-tools: Task
model: haiku
category: workflow
---

# Full Development Flow

요구사항: $ARGUMENTS

## ⚡ 즉시 실행 - Phase 1부터 순차 진행

**Phase 1: 기획** (즉시 시작)
```
Use the spec-writer agent to create PRD for: $ARGUMENTS
```

PRD 완료 후 → Phase 2 진행

**Phase 2: 설계**
```
Use the architect agent to design system based on the PRD
```

설계 완료 후 → Phase 3 진행

**Phase 3: 구현** (프로젝트 타입에 따라 선택)
```
Use the backend-developer agent to implement the backend
Use the frontend-developer agent to implement the frontend
```

구현 완료 후 → Phase 4 진행

**Phase 4: 품질**
```
Use the test-writer agent to write tests
Use the code-reviewer agent to review the code
```

## Related Skills

- `/write-spec`: PRD만 작성
- `/new-feature`: 구현만 (설계 없이)
- `/architecture-review`: 기존 아키텍처 분석
