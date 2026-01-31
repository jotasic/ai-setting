---
name: write-spec
description: 사용자 요구사항을 기획서(PRD)로 변환. 기술적 구현은 architect 에이전트가 담당
argument-hint: <feature-description>
---

# Write Specification (PRD)

사용자의 요구사항을 분석하여 기획서(PRD)를 생성합니다.
기술적 설계와 구현은 다른 에이전트(architect 등)가 담당합니다.

## Arguments

- `$ARGUMENTS`: 구현하려는 기능 설명 (필수)

## Workflow

```
┌─────────────────────────────────────────────────────────────┐
│  1. Analyze     → 요구사항 분석                              │
│  2. Clarify     → 불명확한 부분 질문                         │
│  3. Write PRD   → 기획서 작성                               │
│  4. Handoff     → architect 에이전트로 전달 안내             │
└─────────────────────────────────────────────────────────────┘
```

## Output

### 기획서 (PRD)
`docs/specs/{feature-name}-prd.md`

```markdown
# {Feature} 기획서 (PRD)

## 1. 개요
- 배경, 목적, 대상 사용자, 기대 효과

## 2. 기능 요구사항
- 필수 기능 (Must Have) - P0
- 권장 기능 (Should Have) - P1
- 선택 기능 (Nice to Have) - P2

## 3. 사용자 시나리오
- 시나리오별 사용자 여정

## 4. 비기능 요구사항
- 성능, 보안, 사용성, 확장성 기대치

## 5. 제약사항
- 비즈니스/시스템 제약

## 6. 성공 지표 (KPI)

## 7. 범위 외 (Out of Scope)

## 8. 다음 단계
- architect 에이전트에게 전달 안내
```

## What This Skill Does

- ✅ 비즈니스 요구사항 정의
- ✅ 사용자 시나리오 작성
- ✅ 기능 목록 및 우선순위
- ✅ 성공 지표 정의
- ✅ 제약사항 명시

## What This Skill Does NOT Do

- ❌ 아키텍처 설계 → `architect` 에이전트
- ❌ API 설계 → `api-designer` 에이전트
- ❌ DB 스키마 → `database-specialist` 에이전트
- ❌ 기술 스택 결정 → `architect` 에이전트
- ❌ 코드 작성 → 구현 에이전트

## Examples

### 기본 사용
```
/write-spec 사용자 알림 시스템 - 이메일, 푸시 알림 지원, 알림 설정 관리
```

### 상세 요구사항
```
/write-spec 결제 시스템:
- 신용카드, 계좌이체 지원
- 정기 결제 기능
- 영수증 발행
- 환불 처리
```

### 간단한 기능
```
/write-spec 사용자 프로필 페이지 - 프로필 사진, 이름, 소개글 수정
```

## After PRD is Created

기획서 작성 완료 후 다음 단계:

```
기획서가 완성되었습니다: docs/specs/{feature}-prd.md

다음 단계로 architect 에이전트에게 시스템 설계를 요청하세요:

┌─────────────────────────────────────────────────────────────┐
│  Use the architect agent to design the system architecture  │
│  based on docs/specs/{feature}-prd.md                       │
└─────────────────────────────────────────────────────────────┘
```

## Full Development Flow

```
사용자 요구사항
     │
     ▼
/write-spec (기획서 작성)
     │
     │  📄 docs/specs/{feature}-prd.md
     │
     ▼
architect 에이전트 (시스템 설계)
     │
     │  📄 기술 스펙, 아키텍처 문서
     │
     ├──▶ api-designer (API 설계)
     ├──▶ database-specialist (DB 설계)
     │
     ▼
구현 에이전트들
     │
     ├──▶ test-writer (테스트)
     ├──▶ code-reviewer (리뷰)
     │
     ▼
완료
```

## Tips

1. **구체적으로 설명**: "알림 기능" 보다 "이메일/푸시 알림, 사용자 설정 관리"
2. **비즈니스 목표 포함**: 왜 필요한지 설명
3. **대상 사용자 명시**: 누가 사용하는지
4. **제약사항 언급**: 일정, 예산, 기존 시스템 등

## Integration with spec-writer Agent

더 상세한 기획이 필요한 경우 에이전트 직접 호출:

```
Use the spec-writer agent to create a detailed PRD for:
- User authentication system
- Social login (Google, GitHub)
- Password reset flow
- Remember me functionality
```
