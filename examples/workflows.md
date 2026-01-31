# Development Workflows with Claude Code

Claude Code를 활용한 일반적인 개발 워크플로우 가이드입니다.

## 1. Feature Development Workflow

새로운 기능을 개발할 때의 워크플로우입니다.

```
┌─────────────────────────────────────────────────────────────┐
│  1. Plan        → architect 에이전트로 설계                   │
│  2. Implement   → /new-feature 스킬로 구현                   │
│  3. Test        → test-writer 에이전트로 테스트 작성          │
│  4. Review      → code-reviewer 에이전트로 리뷰              │
│  5. Document    → doc-writer 에이전트로 문서화               │
│  6. Ship        → /commit, /review-pr로 배포 준비            │
└─────────────────────────────────────────────────────────────┘
```

### Step 1: Planning
```
Use the architect agent to design the user notification feature.
Requirements:
- Real-time notifications via WebSocket
- Email fallback for offline users
- Notification preferences per user
```

### Step 2: Implementation
```
/new-feature notification system based on the architect's design
```

### Step 3: Testing
```
Use the test-writer agent to add comprehensive tests for:
- NotificationService
- WebSocket connection handling
- Email sending logic
```

### Step 4: Review
```
Use the code-reviewer agent to review all notification-related changes
```

### Step 5: Documentation
```
Use the doc-writer agent to document the notification API
```

### Step 6: Ship
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
