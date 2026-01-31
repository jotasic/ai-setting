# Agent Selection Guide

상황에 맞는 적절한 에이전트를 선택하는 가이드입니다.

## Quick Reference

| 상황 | 추천 에이전트 | 모델 | 이유 |
|------|--------------|------|------|
| 시스템 설계 | `architect` | opus | 복잡한 트레이드오프 분석 필요 |
| 보안 검토 | `security-auditor` | opus | 취약점 놓치면 안됨 |
| 코드 리뷰 | `code-reviewer` | sonnet | 일반적인 리뷰 작업 |
| 버그 수정 | `debugger` | sonnet | 분석 + 수정 균형 |
| 테스트 작성 | `test-writer` | sonnet | 코드 생성 작업 |
| 리팩토링 | `refactorer` | sonnet | 점진적 개선 작업 |
| 문서 작성 | `doc-writer` | haiku | 빠른 응답, 간단한 작업 |

## Model Selection Criteria

### Opus (복잡한 추론)

**사용 시점:**
- 아키텍처/설계 결정
- 보안 분석 (놓치면 안되는 경우)
- 복잡한 트레이드오프 분석
- 장기적 영향 평가

**특징:**
- 가장 높은 추론 능력
- 깊은 분석과 통찰
- 비용이 높음
- 응답 속도 느림

### Sonnet (균형)

**사용 시점:**
- 일반적인 코딩 작업
- 코드 리뷰
- 버그 수정
- 테스트 작성
- 리팩토링

**특징:**
- 코딩 작업에 최적화
- 비용 대비 성능 우수
- 적절한 응답 속도

### Haiku (빠른 응답)

**사용 시점:**
- 간단한 문서화
- 빠른 설명 필요
- 단순 변환 작업
- 대량 처리

**특징:**
- 가장 빠른 응답
- 낮은 비용
- 간단한 작업에 적합

## Decision Tree

```
작업 유형은?
│
├── 설계/분석
│   └── 복잡도가 높은가?
│       ├── Yes → architect (opus)
│       └── No → 직접 질문
│
├── 보안 관련
│   └── security-auditor (opus)
│
├── 코드 작성/수정
│   ├── 테스트 → test-writer (sonnet)
│   ├── 리팩토링 → refactorer (sonnet)
│   └── 일반 코딩 → 직접 작성
│
├── 코드 리뷰
│   └── code-reviewer (sonnet)
│
├── 버그 수정
│   └── debugger (sonnet)
│
└── 문서화
    └── doc-writer (haiku)
```

## Examples

### Example 1: New Feature
```
# 1단계: 설계 (opus)
Use the architect agent to design the payment processing system

# 2단계: 구현 (sonnet - 기본 모델)
Implement the PaymentService based on the design

# 3단계: 테스트 (sonnet)
Use the test-writer agent to add tests for PaymentService

# 4단계: 리뷰 (sonnet)
Use the code-reviewer agent to review the implementation

# 5단계: 문서화 (haiku)
Use the doc-writer agent to document the payment API
```

### Example 2: Security Fix
```
# 1단계: 보안 감사 (opus)
Use the security-auditor agent to audit the auth module

# 2단계: 버그 수정 (sonnet)
Use the debugger agent to fix the identified vulnerabilities

# 3단계: 재검증 (opus)
Use the security-auditor agent to verify the fixes
```

### Example 3: Quick Documentation
```
# 빠른 문서화가 필요할 때 (haiku)
Use the doc-writer agent to add JSDoc comments to utils.ts
```

## Cost Optimization Tips

1. **작업 분류 먼저**: 복잡도에 따라 적절한 모델 선택
2. **병렬 처리**: 독립적인 작업은 동시에 요청
3. **캐시 활용**: 반복적인 질문 피하기
4. **점진적 접근**: 큰 작업은 단계별로 나누기

## Anti-Patterns

### 피해야 할 것들

❌ **모든 작업에 opus 사용**
- 비용 낭비
- 간단한 작업에는 오버킬

❌ **복잡한 보안 분석에 haiku 사용**
- 중요한 취약점을 놓칠 수 있음

❌ **에이전트 없이 복잡한 작업 요청**
- 컨텍스트 부족으로 결과 품질 저하

### 권장 사항

✅ **작업 복잡도에 맞는 모델 선택**
✅ **에이전트의 전문성 활용**
✅ **단계별 검증**
