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

## 전체 플로우 요약

```
┌─────────────────────────────────────────────────────────────────┐
│  Phase 1: Planning                                              │
│  ├─ spec-writer     → 요구사항 구체화, PRD 작성                   │
│  └─ architect       → 시스템 설계, 기술 스택 결정                  │
├─────────────────────────────────────────────────────────────────┤
│  Phase 2: Design                                                │
│  ├─ database-specialist → DB 스키마 설계                         │
│  ├─ api-designer        → API 설계 (REST/GraphQL)               │
│  └─ devops-specialist   → 인프라 설계 (필요시)                    │
├─────────────────────────────────────────────────────────────────┤
│  Phase 3: Implementation                                        │
│  ├─ backend-developer   → 백엔드 구현 (API, 비즈니스 로직)         │
│  ├─ frontend-developer  → 프론트엔드 구현 (UI, 상태관리)           │
│  └─ general-developer   → 기타 구현 (스크립트, 유틸리티)           │
├─────────────────────────────────────────────────────────────────┤
│  Phase 4: Quality                                               │
│  ├─ test-writer         → 테스트 코드 작성                        │
│  ├─ code-reviewer       → 코드 리뷰                              │
│  └─ security-auditor    → 보안 검토 (민감 기능시)                  │
├─────────────────────────────────────────────────────────────────┤
│  Phase 5: Documentation                                         │
│  └─ doc-writer          → API 문서, README 업데이트               │
└─────────────────────────────────────────────────────────────────┘
```

## 실행 방법

### Phase 1: Planning

```
Use the spec-writer agent to create PRD for: $ARGUMENTS
```

PRD 완료 후:

```
Use the architect agent to design system architecture based on the PRD
```

### Phase 2: Design

```
Use the database-specialist agent to design database schema based on the architecture
```

```
Use the api-designer agent to design API endpoints based on the architecture
```

### Phase 3: Implementation

```
Use the backend-developer agent to implement the API endpoints
```

```
Use the frontend-developer agent to implement the UI components
```

### Phase 4: Quality

```
Use the test-writer agent to write tests for the implemented code
```

```
Use the code-reviewer agent to review all changes
```

### Phase 5: Documentation

```
Use the doc-writer agent to update documentation
```

## 단계별 산출물

| Phase | Agent | 산출물 |
|-------|-------|--------|
| Planning | spec-writer | `docs/specs/{feature}-prd.md` |
| Planning | architect | `docs/architecture/{feature}-design.md` |
| Design | database-specialist | `docs/database/{feature}-schema.md`, migrations |
| Design | api-designer | `docs/api/{feature}-api.md` |
| Implementation | backend-developer | `src/api/`, `src/services/` |
| Implementation | frontend-developer | `src/components/`, `src/pages/` |
| Quality | test-writer | `tests/`, `__tests__/` |
| Documentation | doc-writer | `README.md`, `docs/` |

## 진행 체크리스트

각 단계 완료시 체크:

- [ ] **Planning**
  - [ ] PRD 작성 완료
  - [ ] 아키텍처 설계 완료
- [ ] **Design**
  - [ ] DB 스키마 설계 완료
  - [ ] API 설계 완료
- [ ] **Implementation**
  - [ ] 백엔드 구현 완료
  - [ ] 프론트엔드 구현 완료
- [ ] **Quality**
  - [ ] 테스트 작성 완료
  - [ ] 코드 리뷰 완료
- [ ] **Documentation**
  - [ ] 문서화 완료

## 스킵 가능한 단계

프로젝트 특성에 따라 스킵:

| 조건 | 스킵 가능 |
|------|----------|
| 백엔드만 | frontend-developer |
| 프론트엔드만 | backend-developer, database-specialist |
| DB 없음 | database-specialist |
| 내부 도구 | security-auditor, doc-writer |

## 주의사항

1. **각 단계 완료 확인**: 다음 단계 진행 전 산출물 확인
2. **에이전트 간 문서 공유**: PRD, 설계 문서를 기반으로 구현
3. **점진적 진행**: 한 번에 모든 단계 실행하지 않고, 단계별 확인
4. **피드백 반영**: 각 단계에서 문제 발견시 이전 단계로 돌아가 수정

## Examples

```bash
/full-dev 사용자 알림 시스템
/full-dev 결제 기능 - 카드, 계좌이체 지원
/full-dev 관리자 대시보드
```

## Related Skills

- `/write-spec`: PRD만 작성
- `/new-feature`: 구현만 (설계 없이)
- `/architecture-review`: 기존 아키텍처 분석
