# Development Workflows with Claude Code

Claude Code를 활용한 일반적인 개발 워크플로우 가이드입니다.

## 1. Feature Development Workflow

새로운 기능을 개발할 때의 워크플로우입니다.

```
┌─────────────────────────────────────────────────────────────┐
│  1. Spec        → spec-writer로 기획서(PRD) 작성             │
│  2. Design      → architect 에이전트로 시스템 설계            │
│  3. API Design  → api-designer로 API 설계                   │
│  4. Backend     → backend-developer로 서버 구현             │
│  5. Frontend    → frontend-developer로 UI 구현              │
│  6. Test        → test-writer 에이전트로 테스트 작성          │
│  7. Review      → code-reviewer 에이전트로 리뷰              │
│  8. Document    → doc-writer 에이전트로 문서화               │
│  9. Ship        → /commit, /review-pr로 배포 준비            │
└─────────────────────────────────────────────────────────────┘
```

### Step 1: Specification (PRD)
```
Use the spec-writer agent to write a PRD for the user notification feature.
Requirements:
- Real-time notifications via WebSocket
- Email fallback for offline users
- Notification preferences per user
```

### Step 2: System Design
```
Use the architect agent to design the notification system
based on docs/specs/notification-prd.md
```

### Step 3: API Design
```
Use the api-designer agent to design notification API endpoints
including WebSocket events and REST endpoints
```

### Step 4: Backend Implementation
```
Use the backend-developer agent to implement:
- NotificationService with WebSocket support
- Email notification fallback
- User preference storage
```

### Step 5: Frontend Implementation
```
Use the frontend-developer agent to implement:
- NotificationBell component
- NotificationList with real-time updates
- NotificationPreferences settings page
```

### Step 6: Testing
```
Use the test-writer agent to add comprehensive tests for:
- NotificationService unit tests
- WebSocket integration tests
- Frontend component tests
```

### Step 7: Review
```
Use the code-reviewer agent to review all notification-related changes
```

### Step 8: Documentation
```
Use the doc-writer agent to document the notification API
```

### Step 9: Ship
```
/commit feat(notifications): add real-time notification system
/review-pr
```

---

## 2. Bug Fix Workflow

버그를 수정할 때의 워크플로우입니다.

```
┌─────────────────────────────────────────────────────────────┐
│  1. Reproduce   → 버그 재현 및 확인                          │
│  2. Investigate → debugger 에이전트로 원인 분석              │
│  3. Fix         → 수정 구현                                 │
│  4. Test        → 회귀 테스트 추가                           │
│  5. Verify      → /run-tests로 검증                         │
└─────────────────────────────────────────────────────────────┘
```

### Example
```
# Step 1: Describe the bug
Users report that login fails with "Invalid token" error
after being idle for more than 30 minutes.

# Step 2: Investigate
Use the debugger agent to investigate the token expiration bug.
Error occurs in src/auth/middleware.ts

# Step 3-4: Fix and Test
Fix the issue and add a regression test

# Step 5: Verify
/run-tests src/auth
```

---

## 3. Code Review Workflow

코드 리뷰를 수행할 때의 워크플로우입니다.

```
┌─────────────────────────────────────────────────────────────┐
│  1. Overview    → PR 변경사항 파악                           │
│  2. Security    → security-auditor로 보안 검토              │
│  3. Quality     → code-reviewer로 품질 검토                 │
│  4. Test        → 테스트 커버리지 확인                       │
│  5. Feedback    → 피드백 정리 및 전달                        │
└─────────────────────────────────────────────────────────────┘
```

### Example
```
# Step 1: Overview
/review-pr

# Step 2: Security Review
Use the security-auditor agent to check for vulnerabilities
in PR #123

# Step 3: Quality Review
Use the code-reviewer agent to review code quality

# Step 4: Test Coverage
/run-tests --coverage
```

---

## 4. Refactoring Workflow

코드를 리팩토링할 때의 워크플로우입니다.

```
┌─────────────────────────────────────────────────────────────┐
│  1. Identify    → 리팩토링 대상 식별                         │
│  2. Plan        → architect로 리팩토링 전략 수립             │
│  3. Test First  → 기존 동작 테스트로 보장                    │
│  4. Refactor    → refactorer로 점진적 리팩토링              │
│  5. Verify      → 테스트 통과 확인                          │
└─────────────────────────────────────────────────────────────┘
```

### Example
```
# Step 1: Identify
The PaymentService has grown to 500+ lines and handles
multiple responsibilities.

# Step 2: Plan
Use the architect agent to plan the PaymentService refactoring.
Goals:
- Split into smaller services
- Apply Single Responsibility Principle
- Maintain backward compatibility

# Step 3: Ensure Tests
/run-tests src/services/payment

# Step 4: Refactor
Use the refactorer agent to extract payment validation
into a separate PaymentValidator class

# Step 5: Verify
/run-tests src/services/payment
/code-quality
```

---

## 5. Security Audit Workflow

보안 감사를 수행할 때의 워크플로우입니다.

```
┌─────────────────────────────────────────────────────────────┐
│  1. Dependency  → 의존성 취약점 스캔                         │
│  2. Code Scan   → security-auditor로 코드 스캔              │
│  3. Secrets     → 하드코딩된 시크릿 검사                     │
│  4. OWASP       → OWASP Top 10 체크                        │
│  5. Report      → 결과 정리 및 수정 계획                     │
└─────────────────────────────────────────────────────────────┘
```

### Example
```
# Step 1: Dependency Scan
npm audit
# or
pip-audit

# Step 2-4: Full Security Audit
Use the security-auditor agent to perform a comprehensive
security audit of the authentication module.

# Step 5: Fix critical issues
Fix the identified vulnerabilities starting with Critical severity
```

---

## 6. Documentation Workflow

문서화 작업 시의 워크플로우입니다.

```
┌─────────────────────────────────────────────────────────────┐
│  1. Inventory   → 문서화 대상 파악                           │
│  2. Generate    → doc-writer로 초안 생성                    │
│  3. Examples    → 실제 동작하는 예제 추가                    │
│  4. Review      → 정확성 및 완성도 검토                      │
│  5. Publish     → 문서 배포                                 │
└─────────────────────────────────────────────────────────────┘
```

### Example
```
# Step 1: Identify what needs documentation
The new API endpoints added in v2.0 need documentation:
- /api/v2/orders
- /api/v2/subscriptions
- /api/v2/webhooks

# Step 2: Generate
Use the doc-writer agent to create API documentation for
the v2 endpoints. Include request/response examples.

# Step 3: Add Examples
Ensure all examples are tested and working

# Step 4: Review
Verify accuracy against actual API behavior
```

---

## 7. Onboarding New Project

새 프로젝트를 시작하거나 기존 프로젝트에 참여할 때의 워크플로우입니다.

```
┌─────────────────────────────────────────────────────────────┐
│  1. Explore     → /search-code로 구조 파악                  │
│  2. Understand  → /explain-code로 핵심 코드 이해            │
│  3. Setup       → 개발 환경 설정                            │
│  4. First Task  → 작은 이슈부터 시작                         │
└─────────────────────────────────────────────────────────────┘
```

### Example
```
# Step 1: Explore
/search-code authentication flow
/search-code database models
/search-code API routes

# Step 2: Understand Core Logic
/explain-code src/core/engine.ts
/explain-code src/services/auth.ts

# Step 3: Setup
/build
/run-tests

# Step 4: First Contribution
/fix-issue 42  # Start with a good-first-issue
```

---

## Tips for Efficient Workflows

### 1. Parallel Execution
독립적인 작업은 병렬로 요청:
```
In parallel:
1. Use test-writer to add tests for UserService
2. Use doc-writer to document UserService API
```

### 2. Context Preservation
연관 작업 시 컨텍스트 유지:
```
Continue with the authentication refactoring.
Apply the same patterns to SessionService.
```

### 3. Incremental Progress
큰 작업은 작은 단위로 분할:
```
Let's refactor in stages:
Stage 1: Extract validation logic
Stage 2: Create separate validator class
Stage 3: Update all callers
Stage 4: Remove old code
```

### 4. Verification at Each Step
각 단계마다 검증:
```
After each change:
1. Run related tests
2. Verify no regressions
3. Commit if successful
```

---

## 8. Performance Optimization Workflow

성능 최적화 작업 시의 워크플로우입니다.

```
┌─────────────────────────────────────────────────────────────┐
│  1. Measure     → /performance-profile로 현재 상태 측정      │
│  2. Identify    → performance-optimizer로 병목 분석          │
│  3. Prioritize  → 영향도 기반 우선순위 결정                   │
│  4. Optimize    → 최적화 구현                                │
│  5. Verify      → 개선 효과 측정                             │
│  6. Document    → 최적화 결과 문서화                          │
└─────────────────────────────────────────────────────────────┘
```

### Example
```
# Step 1: Measure Baseline
/performance-profile src/api

# Step 2: Identify Bottlenecks
Use the performance-optimizer agent to analyze:
- API response times > 500ms
- Database queries with EXPLAIN ANALYZE
- Memory usage patterns

# Step 3: Prioritize
Focus on:
1. [High] /api/orders endpoint (2s response time)
2. [Medium] Database N+1 queries in UserService
3. [Low] Bundle size optimization

# Step 4: Optimize
Implement:
- Add database indexes
- Implement query batching
- Add Redis caching

# Step 5: Verify Improvement
/performance-profile src/api
# Compare before/after metrics

# Step 6: Document
Use doc-writer to document:
- Optimizations made
- Performance improvements achieved
- Monitoring recommendations
```

---

## 9. Dependency Update Workflow

의존성 업데이트 작업 시의 워크플로우입니다.

```
┌─────────────────────────────────────────────────────────────┐
│  1. Audit       → /dependency-audit로 취약점 스캔            │
│  2. Plan        → 업데이트 순서 및 위험도 분석                │
│  3. Update      → 점진적 업데이트                            │
│  4. Test        → 호환성 테스트                              │
│  5. Commit      → 변경사항 커밋                              │
└─────────────────────────────────────────────────────────────┘
```

### Example
```
# Step 1: Security Audit
/dependency-audit

# Step 2: Plan Updates
Use the dependency-manager agent to:
- Prioritize security vulnerabilities
- Check breaking changes in major updates
- Create update sequence

# Step 3: Update (one at a time for majors)
npm update lodash
npm install react@18 --save

# Step 4: Test Compatibility
/run-tests
/build

# Step 5: Commit
/commit chore(deps): update lodash to fix CVE-2021-xxxx
/commit feat(deps): upgrade to React 18
```

---

## 10. API Design & Implementation Workflow

새 API를 설계하고 구현할 때의 워크플로우입니다.

```
┌─────────────────────────────────────────────────────────────┐
│  1. Design      → api-designer로 엔드포인트 설계             │
│  2. Spec        → OpenAPI 스펙 작성                         │
│  3. Implement   → 백엔드 구현                                │
│  4. Validate    → API 스키마 검증                           │
│  5. Document    → /api-docs-generate로 문서 생성            │
│  6. Test        → API 통합 테스트                           │
└─────────────────────────────────────────────────────────────┘
```

### Example
```
# Step 1: Design API
Use the api-designer agent to design order management API:
- CRUD operations for orders
- Order status transitions
- Webhook for status changes

# Step 2: Create OpenAPI Spec
Based on design, create openapi.yaml with:
- Request/response schemas
- Error responses
- Authentication requirements

# Step 3: Implement
Implement endpoints following the spec:
- POST /api/v1/orders
- GET /api/v1/orders/:id
- PATCH /api/v1/orders/:id/status

# Step 4: Validate
Validate implementation matches spec

# Step 5: Generate Docs
/api-docs-generate ./docs/api

# Step 6: Test
Use test-writer to add API integration tests
```

---

## 11. Database Migration Workflow

데이터베이스 스키마 변경 시의 워크플로우입니다.

```
┌─────────────────────────────────────────────────────────────┐
│  1. Design      → database-specialist로 스키마 설계          │
│  2. Plan        → 마이그레이션 전략 수립                      │
│  3. Create      → /db-migrate create로 마이그레이션 생성     │
│  4. Test        → 스테이징에서 테스트                         │
│  5. Backup      → 프로덕션 백업                              │
│  6. Execute     → /db-migrate run으로 실행                  │
│  7. Verify      → 데이터 무결성 확인                          │
└─────────────────────────────────────────────────────────────┘
```

### Example
```
# Step 1: Design Schema Change
Use the database-specialist agent to design:
- Add "subscription_tier" to users table
- Add "subscriptions" table
- Define relationships

# Step 2: Plan Migration Strategy
For zero-downtime:
1. Add nullable column
2. Deploy code that writes to both
3. Backfill existing data
4. Add NOT NULL constraint
5. Deploy code that reads new column

# Step 3: Create Migration
/db-migrate create add-subscription-support

# Step 4: Test on Staging
/db-migrate run --env=staging
/run-tests --env=staging

# Step 5: Backup Production
pg_dump production > backup_$(date +%Y%m%d).sql

# Step 6: Execute
/db-migrate run --env=production

# Step 7: Verify
Check data integrity and application health
```

---

## 12. Release Management Workflow

릴리즈 준비 및 배포 시의 워크플로우입니다.

```
┌─────────────────────────────────────────────────────────────┐
│  1. Freeze      → 기능 동결, 안정화                          │
│  2. Test        → 전체 테스트 스위트 실행                     │
│  3. Changelog   → /changelog로 릴리즈 노트 생성              │
│  4. Version     → 버전 번호 업데이트                          │
│  5. Tag         → Git 태그 생성                              │
│  6. Deploy      → 프로덕션 배포                               │
│  7. Monitor     → 배포 후 모니터링                            │
└─────────────────────────────────────────────────────────────┘
```

### Example
```
# Step 1: Feature Freeze
Merge only bug fixes to release branch

# Step 2: Full Test Suite
/run-tests
/code-quality

# Step 3: Generate Changelog
/changelog 2.0.0

# Step 4: Update Version
npm version 2.0.0

# Step 5: Create Tag
git tag -a v2.0.0 -m "Release 2.0.0"
git push origin v2.0.0

# Step 6: Deploy
Deploy to production via CI/CD

# Step 7: Monitor
- Check error rates
- Monitor performance metrics
- Watch user feedback channels
```

---

## 13. Script/Automation Workflow

스크립트, CLI 도구, 봇 등을 개발할 때의 워크플로우입니다.

```
┌─────────────────────────────────────────────────────────────┐
│  1. Define      → 요구사항 정의                               │
│  2. Design      → 입출력, 옵션 설계                           │
│  3. Implement   → general-developer로 구현                   │
│  4. Test        → 테스트 및 엣지 케이스 확인                   │
│  5. Document    → 사용법 문서화                               │
│  6. Deploy      → 실행 환경에 배포                            │
└─────────────────────────────────────────────────────────────┘
```

### Example: Data Migration Script
```
# Step 1: Define Requirements
Migrate user data from legacy MySQL to new PostgreSQL database
- Handle data transformation
- Support incremental migration
- Provide rollback capability

# Step 2: Design
Use the general-developer agent to design:
- CLI interface with --source, --target, --batch-size options
- Progress reporting
- Error handling and logging

# Step 3: Implement
Use the general-developer agent to create:
- migration/migrate_users.py script
- Configuration handling
- Database connection management

# Step 4: Test
Test with sample data:
- Empty dataset
- Large dataset (1M+ records)
- Data with special characters

# Step 5: Document
Use the doc-writer agent to create README with:
- Installation instructions
- Usage examples
- Configuration options

# Step 6: Deploy
Deploy to scheduled job or run manually
```

### Example: Slack Bot
```
# Step 1: Define
Create a Slack bot for daily standup reminders

# Step 2-3: Design & Implement
Use the general-developer agent to create a Slack bot that:
- Posts daily standup reminders at 9 AM
- Collects responses via threads
- Summarizes standup results

# Step 4-6: Test, Document, Deploy
Test in staging workspace, document setup, deploy to production
```

---

## 14. Infrastructure Setup Workflow

인프라 및 CI/CD를 설정할 때의 워크플로우입니다.

```
┌─────────────────────────────────────────────────────────────┐
│  1. Requirements → 인프라 요구사항 정의                        │
│  2. Design       → architect로 인프라 아키텍처 설계            │
│  3. Containers   → devops-specialist로 Docker 설정           │
│  4. CI/CD        → devops-specialist로 파이프라인 설정        │
│  5. IaC          → devops-specialist로 인프라 코드화          │
│  6. Test         → 스테이징 환경에서 테스트                    │
│  7. Document     → 운영 문서 작성                             │
└─────────────────────────────────────────────────────────────┘
```

### Example: Kubernetes Deployment
```
# Step 1: Requirements
- Multi-environment support (dev, staging, prod)
- Auto-scaling based on CPU/memory
- Zero-downtime deployments
- Secret management

# Step 2: Infrastructure Design
Use the architect agent to design:
- Kubernetes cluster architecture
- Service mesh requirements
- Monitoring and logging stack

# Step 3: Containerization
Use the devops-specialist agent to create:
- Dockerfile with multi-stage build
- docker-compose.yml for local development
- Container security best practices

# Step 4: CI/CD Pipeline
Use the devops-specialist agent to set up:
- GitHub Actions for build and test
- Automated deployment to staging
- Manual approval for production

# Step 5: Infrastructure as Code
Use the devops-specialist agent to create:
- Kubernetes manifests (Deployment, Service, Ingress)
- Helm charts for configuration
- Terraform for cloud resources

# Step 6: Test
- Deploy to staging environment
- Run integration tests
- Verify auto-scaling

# Step 7: Document
Use the doc-writer agent to create:
- Deployment runbook
- Troubleshooting guide
- Architecture diagrams
```

---

## 15. Project Setup Workflow

새 프로젝트의 CLAUDE.md를 생성하고 설정할 때의 워크플로우입니다.

```
┌─────────────────────────────────────────────────────────────┐
│  1. Explore     → 프로젝트 구조 파악                          │
│  2. Discuss     → 프로젝트 컨벤션 논의                        │
│  3. Generate    → claudemd-generator로 CLAUDE.md 생성       │
│  4. Review      → 생성된 내용 검토 및 수정                    │
│  5. Commit      → 저장소에 커밋                               │
└─────────────────────────────────────────────────────────────┘
```

### Example
```
# Step 1: Explore Project
/search-code project structure
What frameworks and tools does this project use?

# Step 2: Discuss Conventions
Let's establish:
- Code style guidelines
- Git commit conventions
- Testing requirements
- Deployment process

# Step 3: Generate CLAUDE.md
Use the claudemd-generator agent to create CLAUDE.md
based on our discussion and project analysis

# Step 4: Review
Review the generated CLAUDE.md:
- Verify accuracy of project description
- Check conventions match team agreements
- Ensure all important patterns are documented

# Step 5: Commit
/commit docs: add CLAUDE.md for AI assistant context
```

---

## Workflow Selection Guide

| 상황 | 권장 워크플로우 |
|------|----------------|
| 새 기능 개발 | Feature Development (#1) |
| 버그 수정 | Bug Fix (#2) |
| PR 리뷰 | Code Review (#3) |
| 코드 개선 | Refactoring (#4) |
| 보안 점검 | Security Audit (#5) |
| 문서 작성 | Documentation (#6) |
| 새 프로젝트 참여 | Onboarding (#7) |
| 속도 개선 | Performance Optimization (#8) |
| 패키지 업데이트 | Dependency Update (#9) |
| API 개발 | API Design (#10) |
| DB 변경 | Database Migration (#11) |
| 버전 배포 | Release Management (#12) |
| 스크립트/CLI/봇 | Script/Automation (#13) |
| 인프라/CI/CD | Infrastructure Setup (#14) |
| CLAUDE.md 생성 | Project Setup (#15) |
