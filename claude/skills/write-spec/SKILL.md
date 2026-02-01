---
name: write-spec
description: 사용자 요구사항을 기획서(PRD)로 변환
argument-hint: <feature-description>
allowed-tools: Read, Write, Grep, Glob
model: opus
category: documentation
---

# Write Specification (PRD)

사용자의 요구사항을 분석하여 기획서(PRD)를 생성합니다.

## Triggers (사용 조건)

- "기획서 작성해줘", "write spec"
- "PRD 만들어줘", "요구사항 정리"
- 새 기능 기획 필요시

## Arguments

- `$ARGUMENTS`: 기능 설명 (필수)

## Workflow

```
┌─────────────────────────────────────┐
│  1. Analyze requirements            │
│  2. Ask clarifying questions        │
│  3. Write PRD document              │
│  4. Handoff to architect            │
└─────────────────────────────────────┘
```

## What This Does

- ✅ 비즈니스 요구사항 정의
- ✅ 사용자 시나리오 작성
- ✅ 기능 우선순위 설정
- ✅ 성공 지표 정의

## What This Does NOT Do

- ❌ 아키텍처 설계 → `architect`
- ❌ API 설계 → `api-designer`
- ❌ DB 설계 → `database-specialist`

## Agent Integration

**상세 기획:**
```
Use the spec-writer agent to create detailed PRD with stakeholder questions
```

**기술 설계:**
```
Use the architect agent to design system based on PRD
```

## Output Format

```
PRD Created
═══════════════════════════════════════
Feature: [name]
Output: docs/specs/[feature]-prd.md

Sections:
  ✓ Overview
  ✓ Requirements (P0: 3, P1: 5, P2: 2)
  ✓ User Scenarios
  ✓ Success Metrics

Next Step:
  Use the architect agent to design
  the system based on [prd-path]
═══════════════════════════════════════
```

## Examples

```bash
/write-spec 사용자 알림 시스템
/write-spec 결제 기능 - 카드, 계좌이체 지원
/write-spec 프로필 페이지 - 사진, 이름, 소개글 수정
```

## Related Skills

- `/full-dev`: 전체 개발 플로우
- `/estimate-effort`: 규모 추정
- `/architecture-review`: 구조 분석
