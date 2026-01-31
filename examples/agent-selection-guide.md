# Agent Selection Guide

상황에 맞는 적절한 에이전트를 선택하는 가이드입니다.

## 에이전트 지정 방식

### 명시적 지정 (권장)

에이전트를 직접 지정하여 호출합니다:

```
Use the {agent-name} agent to {task}
```

**예시:**
```
Use the architect agent to design the notification system
Use the frontend-developer agent to implement the dashboard components
Use the general-developer agent to create a data migration script
```

### 미지정 시 동작

에이전트를 지정하지 않으면 **기본 Claude**가 직접 처리합니다.

```
# 에이전트 미지정 - Claude가 직접 처리
Fix the bug in the login function

# 에이전트 지정 - debugger 에이전트가 처리
Use the debugger agent to fix the bug in the login function
```

**에이전트 사용이 권장되는 경우:**
- 전문적인 분석이 필요한 작업 (보안, 아키텍처)
- 일관된 패턴/템플릿이 필요한 작업 (테스트, 문서)
- 특정 도메인 지식이 필요한 작업 (DB, DevOps)

---

## Quick Reference (17개 에이전트)

| 상황 | 추천 에이전트 | 모델 |
|------|--------------|------|
| 시스템 설계, 아키텍처 | `architect` | opus |
| 보안 검토, 취약점 분석 | `security-auditor` | opus |
| 코드 품질 리뷰 | `code-reviewer` | sonnet |
| 버그 분석 및 수정 | `debugger` | sonnet |
| 테스트 코드 작성 | `test-writer` | sonnet |
| 웹 프론트엔드 구현 | `frontend-developer` | sonnet |
| API/서버 구현 | `backend-developer` | sonnet |
| 스크립트, CLI, 봇 | `general-developer` | sonnet |
| 코드 리팩토링 | `refactorer` | sonnet |
| 성능 분석 및 최적화 | `performance-optimizer` | sonnet |
| 인프라, CI/CD | `devops-specialist` | sonnet |
| API 설계 | `api-designer` | sonnet |
| DB 스키마, 쿼리 | `database-specialist` | sonnet |
| 기획서(PRD) 작성 | `spec-writer` | sonnet |
| CLAUDE.md 생성 | `claudemd-generator` | sonnet |
| 문서화, README | `doc-writer` | haiku |
| 의존성 관리 | `dependency-manager` | haiku |

---

## Decision Tree

```
작업 유형은?
│
├── 기획/설계
│   ├── 비즈니스 요구사항 정리 → spec-writer
│   ├── 시스템 아키텍처 → architect (opus)
│   ├── API 스펙 설계 → api-designer
│   └── DB 스키마 설계 → database-specialist
│
├── 구현
│   ├── 웹 UI/컴포넌트 → frontend-developer
│   ├── API/서버 로직 → backend-developer
│   ├── 스크립트/CLI/봇 → general-developer
│   └── 인프라/배포 설정 → devops-specialist
│
├── 품질
│   ├── 코드 리뷰 → code-reviewer
│   ├── 테스트 작성 → test-writer
│   ├── 리팩토링 → refactorer
│   ├── 성능 최적화 → performance-optimizer
│   └── 보안 검토 → security-auditor (opus)
│
├── 문서화
│   ├── 기술 문서/README → doc-writer (haiku)
│   └── CLAUDE.md 생성 → claudemd-generator
│
└── 유지보수
    ├── 버그 수정 → debugger
    └── 의존성 업데이트 → dependency-manager (haiku)
```

---

## 에이전트별 사용 예시

### 설계 에이전트

#### architect (opus)
```
# 시스템 설계
Use the architect agent to design a real-time notification system
with WebSocket support and message persistence

# 기술 스택 결정
Use the architect agent to evaluate whether to use GraphQL or REST
for our mobile app backend

# 마이크로서비스 분리
Use the architect agent to plan the decomposition of our monolith
into microservices
```

#### api-designer (sonnet)
```
# REST API 설계
Use the api-designer agent to design RESTful endpoints for
user management with proper resource naming

# GraphQL 스키마 설계
Use the api-designer agent to design GraphQL schema for
the e-commerce product catalog
```

#### database-specialist (sonnet)
```
# 스키마 설계
Use the database-specialist agent to design the schema for
a multi-tenant SaaS application

# 쿼리 최적화
Use the database-specialist agent to optimize the slow queries
in the reporting module
```

#### spec-writer (sonnet)
```
# 기획서 작성
Use the spec-writer agent to write a PRD for the user
notification preferences feature

# 요구사항 정리
Use the spec-writer agent to document requirements for
the payment integration
```

### 구현 에이전트

#### frontend-developer (sonnet)
```
# React 컴포넌트 구현
Use the frontend-developer agent to implement the dashboard
components with charts and filters

# 상태 관리 구현
Use the frontend-developer agent to set up Zustand store
for the shopping cart feature

# 폼 구현
Use the frontend-developer agent to create the multi-step
registration form with validation
```

#### backend-developer (sonnet)
```
# API 엔드포인트 구현
Use the backend-developer agent to implement the user
authentication endpoints with JWT

# 비즈니스 로직 구현
Use the backend-developer agent to implement the order
processing service with inventory checks

# DB 연동 구현
Use the backend-developer agent to implement the repository
layer for the notification module
```

#### general-developer (sonnet)
```
# 스크립트 작성
Use the general-developer agent to create a data migration
script from MySQL to PostgreSQL

# CLI 도구 개발
Use the general-developer agent to build a CLI tool for
managing project configurations

# 봇 개발
Use the general-developer agent to create a Slack bot
that posts daily standup reminders

# 크롤러 작성
Use the general-developer agent to build a web scraper
for collecting competitor pricing data
```

#### devops-specialist (sonnet)
```
# Docker 설정
Use the devops-specialist agent to create Dockerfile and
docker-compose.yml for the application

# CI/CD 파이프라인
Use the devops-specialist agent to set up GitHub Actions
for automated testing and deployment

# Kubernetes 설정
Use the devops-specialist agent to create K8s manifests
for the microservices deployment
```

### 품질 에이전트

#### code-reviewer (sonnet)
```
# 코드 리뷰
Use the code-reviewer agent to review the authentication
module for best practices

# PR 리뷰
Use the code-reviewer agent to review the changes in
the current pull request
```

#### test-writer (sonnet)
```
# 단위 테스트
Use the test-writer agent to add unit tests for the
PaymentService class

# 통합 테스트
Use the test-writer agent to write integration tests
for the REST API endpoints

# E2E 테스트
Use the test-writer agent to create Playwright tests
for the checkout flow
```

#### security-auditor (opus)
```
# 보안 감사
Use the security-auditor agent to audit the authentication
module for vulnerabilities

# OWASP 체크
Use the security-auditor agent to check the API for
OWASP Top 10 vulnerabilities

# 의존성 보안 검토
Use the security-auditor agent to review dependencies
for known security issues
```

#### performance-optimizer (sonnet)
```
# 성능 분석
Use the performance-optimizer agent to analyze the slow
database queries in the reporting module

# 최적화 제안
Use the performance-optimizer agent to suggest optimizations
for the image processing pipeline
```

#### refactorer (sonnet)
```
# 코드 리팩토링
Use the refactorer agent to refactor the legacy user
service into smaller modules

# 패턴 적용
Use the refactorer agent to apply the repository pattern
to the data access layer
```

### 문서/관리 에이전트

#### doc-writer (haiku)
```
# API 문서화
Use the doc-writer agent to generate API documentation
for the user endpoints

# README 작성
Use the doc-writer agent to create a comprehensive README
for the project

# 코드 주석
Use the doc-writer agent to add JSDoc comments to the
utility functions
```

#### claudemd-generator (sonnet)
```
# CLAUDE.md 생성
Use the claudemd-generator agent to create CLAUDE.md
based on our conversation about the project setup

# CLAUDE.md 업데이트
Use the claudemd-generator agent to update CLAUDE.md
with the new conventions we discussed
```

#### dependency-manager (haiku)
```
# 의존성 업데이트
Use the dependency-manager agent to update outdated
packages with security patches

# 의존성 분석
Use the dependency-manager agent to analyze and clean up
unused dependencies
```

#### debugger (sonnet)
```
# 버그 분석
Use the debugger agent to investigate the null pointer
exception in the checkout flow

# 에러 수정
Use the debugger agent to fix the race condition in
the notification service
```

---

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

**에이전트:** architect, security-auditor

### Sonnet (균형)

**사용 시점:**
- 일반적인 코딩 작업
- 코드 리뷰
- 구현 작업
- 테스트 작성

**특징:**
- 코딩 작업에 최적화
- 비용 대비 성능 우수
- 적절한 응답 속도

**에이전트:** code-reviewer, debugger, test-writer, frontend-developer, backend-developer, general-developer, refactorer, performance-optimizer, devops-specialist, api-designer, database-specialist, spec-writer, claudemd-generator

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

**에이전트:** doc-writer, dependency-manager

---

## Workflow Examples

### Example 1: New Feature (Full Flow)

```
# 1단계: 기획서 작성 (sonnet)
Use the spec-writer agent to write a PRD for the notification feature

# 2단계: 시스템 설계 (opus)
Use the architect agent to design the notification system
based on docs/specs/notification-prd.md

# 3단계: API 설계 (sonnet)
Use the api-designer agent to design notification API endpoints

# 4단계: DB 설계 (sonnet)
Use the database-specialist agent to design the notification schema

# 5단계: 백엔드 구현 (sonnet)
Use the backend-developer agent to implement the notification service

# 6단계: 프론트엔드 구현 (sonnet)
Use the frontend-developer agent to implement the notification UI

# 7단계: 테스트 (sonnet)
Use the test-writer agent to add tests for the notification feature

# 8단계: 코드 리뷰 (sonnet)
Use the code-reviewer agent to review the implementation

# 9단계: 문서화 (haiku)
Use the doc-writer agent to document the notification API
```

### Example 2: Script/Automation

```
# 데이터 마이그레이션 스크립트
Use the general-developer agent to create a migration script
that transfers user data from the legacy system

# 테스트 추가
Use the test-writer agent to add tests for the migration script
```

### Example 3: Infrastructure Setup

```
# Docker 설정
Use the devops-specialist agent to create Docker configuration
for the application

# CI/CD 설정
Use the devops-specialist agent to set up GitHub Actions
for automated deployment
```

### Example 4: Security Fix

```
# 1단계: 보안 감사 (opus)
Use the security-auditor agent to audit the auth module

# 2단계: 버그 수정 (sonnet)
Use the debugger agent to fix the identified vulnerabilities

# 3단계: 재검증 (opus)
Use the security-auditor agent to verify the fixes
```

---

## Anti-Patterns

### 피해야 할 것들

❌ **모든 작업에 opus 사용**
- 비용 낭비
- 간단한 작업에는 오버킬

❌ **복잡한 보안 분석에 haiku 사용**
- 중요한 취약점을 놓칠 수 있음

❌ **에이전트 혼동**
- frontend-developer에게 API 구현 요청
- backend-developer에게 UI 구현 요청
- general-developer에게 웹 서버 구현 요청

### 권장 사항

✅ **작업 복잡도에 맞는 모델 선택**
✅ **에이전트의 전문성 활용**
✅ **단계별 검증**
✅ **명확한 작업 범위 지정**
