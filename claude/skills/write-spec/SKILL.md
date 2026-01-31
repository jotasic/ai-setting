---
name: write-spec
description: 사용자 요구사항을 기획서, 기술 스펙, CLAUDE.md로 변환하여 다른 에이전트가 구현할 수 있도록 문서화
argument-hint: <feature-description> [--prd-only] [--tech-only] [--update-claude]
---

# Write Specification

사용자의 요구사항을 분석하여 기획서(PRD)와 기술 스펙을 생성하고, CLAUDE.md를 업데이트합니다.

## Arguments

- `$ARGUMENTS`:
  - `<feature-description>`: 구현하려는 기능 설명 (필수)
  - `--prd-only`: 기획서만 생성
  - `--tech-only`: 기술 스펙만 생성
  - `--update-claude`: CLAUDE.md만 업데이트
  - `--output <dir>`: 출력 디렉토리 지정 (기본: docs/specs/)

## Workflow

```
┌─────────────────────────────────────────────────────────────┐
│  1. Analyze     → 요구사항 분석 및 질문                       │
│  2. Explore     → 프로젝트 구조 파악                         │
│  3. Draft PRD   → 기획서 초안 작성                          │
│  4. Draft Tech  → 기술 스펙 초안 작성                        │
│  5. Update      → CLAUDE.md 업데이트                        │
│  6. Review      → 문서 검토 및 확인 요청                      │
└─────────────────────────────────────────────────────────────┘
```

## Output Files

### 1. 기획서 (PRD)
`docs/specs/{feature-name}-prd.md`

```markdown
# {Feature} 기획서

## 개요
## 기능 요구사항
## 사용자 시나리오
## 비기능 요구사항
## 제약사항
## 성공 지표
```

### 2. 기술 스펙
`docs/specs/{feature-name}-tech-spec.md`

```markdown
# {Feature} 기술 스펙

## 아키텍처
## 데이터 모델
## API 설계
## 구현 계획
## 테스트 계획
## 에이전트 가이드
```

### 3. CLAUDE.md 업데이트

```markdown
## Current Feature: {Feature}

### Overview
### Implementation Status
### Key Files
### Conventions
### Agent Instructions
```

## Examples

### 기본 사용
```
/write-spec 사용자 알림 시스템 - 이메일과 푸시 알림 지원, 알림 설정 관리
```

### 기획서만 생성
```
/write-spec 결제 시스템 통합 --prd-only
```

### 기술 스펙만 생성
```
/write-spec API 레이트 리미팅 --tech-only
```

### CLAUDE.md만 업데이트
```
/write-spec 현재 작업 중인 검색 기능 --update-claude
```

### 커스텀 출력 경로
```
/write-spec 대시보드 기능 --output design/
```

## Integration with spec-writer Agent

더 상세한 문서가 필요한 경우:

```
Use the spec-writer agent to create comprehensive documentation for:
- User authentication with OAuth2
- Password reset flow
- Session management

Include:
- Detailed user journeys
- Security considerations
- API contracts
- Database schema
```

## Document Structure

생성되는 문서의 전체 구조:

```
project/
├── docs/
│   └── specs/
│       ├── {feature}-prd.md          # 기획서
│       ├── {feature}-tech-spec.md    # 기술 스펙
│       └── README.md                 # 스펙 인덱스 (자동 업데이트)
├── CLAUDE.md                         # 프로젝트 컨텍스트
└── ...
```

## For Other Agents

생성된 문서를 다른 에이전트가 활용하는 방법:

```markdown
## Agent Instructions (in CLAUDE.md)

### architect
- PRD의 "아키텍처" 섹션 참조
- 기술 스펙의 컴포넌트 다이어그램 기반 설계

### api-designer
- 기술 스펙의 "API 설계" 섹션 확장
- PRD의 기능 요구사항 반영

### database-specialist
- 기술 스펙의 "데이터 모델" 섹션 기반 스키마 설계
- PRD의 비기능 요구사항(성능) 고려

### test-writer
- 기술 스펙의 "테스트 계획" 따르기
- PRD의 사용자 시나리오 기반 E2E 테스트

### code-reviewer
- PRD의 제약사항 확인
- 기술 스펙의 컨벤션 준수 검토
```

## Quick Templates

### Simple Feature
```
/write-spec 간단한 CRUD 기능 - 사용자 프로필 관리
```

### Complex System
```
/write-spec 복잡한 시스템:
- 실시간 채팅 기능
- 메시지 저장 및 검색
- 읽음 표시
- 타이핑 인디케이터
- 파일 첨부
```

### API Integration
```
/write-spec 외부 API 연동:
- Stripe 결제 통합
- 웹훅 처리
- 에러 핸들링
- 재시도 로직
```

## Post-Generation Steps

문서 생성 후 권장 워크플로우:

1. **검토**: 생성된 문서 확인 및 수정
2. **설계**: `architect` 에이전트로 아키텍처 검토
3. **구현**: 기술 스펙의 구현 계획 따라 진행
4. **테스트**: `test-writer`로 테스트 작성
5. **리뷰**: `code-reviewer`로 코드 검토
