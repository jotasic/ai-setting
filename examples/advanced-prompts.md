# Advanced Prompts

복잡한 작업을 위한 고급 프롬프트 예제입니다.

## 1. 대규모 리팩토링

### 모놀리스에서 마이크로서비스 분리
```
Use the architect agent to plan microservice extraction:

Current state:
- Monolithic application in src/
- User, Order, Payment modules tightly coupled
- Shared database

Goals:
- Extract Order service as independent microservice
- Define API contracts between services
- Plan data migration strategy
- Maintain backward compatibility during transition

Constraints:
- Zero downtime deployment required
- Existing API consumers must not break
- Budget for 3 months migration
```

### 레거시 코드 현대화
```
Analyze and plan modernization of legacy codebase:

Current state:
- jQuery-based frontend
- Callback-based Node.js backend
- No TypeScript, no tests

Target state:
- React with TypeScript
- Async/await patterns
- 80% test coverage

Approach:
1. First, analyze current code structure
2. Identify high-risk areas
3. Create migration plan with phases
4. Prioritize by business impact
```

---

## 2. 복잡한 기능 구현

### 결제 시스템 구현
```
Design and implement payment processing system:

Requirements:
- Support Stripe and PayPal
- Handle webhooks for async events
- Implement idempotency for retries
- Store transaction history
- PCI DSS compliance considerations

Use these agents in sequence:
1. architect - System design and security review
2. api-designer - API endpoint design
3. database-specialist - Transaction schema
4. security-auditor - Security review
5. test-writer - Payment flow tests
```

### 실시간 협업 기능
```
Implement real-time collaboration feature:

Requirements:
- Multiple users editing same document
- Conflict resolution (CRDT or OT)
- Presence indicators
- Offline support with sync

Technical decisions needed:
- WebSocket vs SSE vs polling
- State synchronization strategy
- Database design for versioning
```

---

## 3. 성능 최적화

### 데이터베이스 최적화
```
Use the performance-optimizer and database-specialist agents:

Problem:
- API response time > 2 seconds
- Database CPU at 80%
- 1M+ rows in orders table

Analyze:
1. Identify slow queries (EXPLAIN ANALYZE)
2. Review index usage
3. Check for N+1 queries
4. Evaluate caching opportunities

Provide:
- Query optimization recommendations
- Index creation scripts
- Caching strategy
- Expected performance improvements
```

### 프론트엔드 번들 최적화
```
Use the performance-optimizer agent to analyze frontend bundle:

Current state:
- Bundle size: 2.5MB
- First paint: 4 seconds
- Many unused dependencies

Goals:
- Bundle size < 500KB
- First paint < 1.5 seconds

Analyze:
1. Run bundle analyzer
2. Identify large dependencies
3. Find code splitting opportunities
4. Check for tree-shaking issues
```

---

## 4. 보안 감사

### 전체 보안 리뷰
```
Use the security-auditor agent for comprehensive security audit:

Scope:
- Authentication and authorization
- API security
- Data validation
- Dependency vulnerabilities
- Secrets management
- OWASP Top 10 compliance

Focus areas:
1. src/auth/ - Authentication logic
2. src/api/ - API endpoints
3. src/db/ - Database queries

Output:
- Vulnerability report with severity
- Remediation steps for each issue
- Security improvement roadmap
```

---

## 5. 아키텍처 설계

### 새 시스템 설계
```
Use the architect agent to design notification system:

Requirements:
- Support email, SMS, push notifications
- Template management
- User preferences
- Rate limiting
- Delivery tracking

Constraints:
- Must integrate with existing user service
- High availability (99.9% uptime)
- Handle 1M+ notifications/day

Provide:
- Architecture diagram
- Component descriptions
- Technology recommendations
- Scaling strategy
- Failure handling
```

---

## 6. 다단계 워크플로우

### 기능 개발 전체 사이클
```
Implement user analytics dashboard:

Step 1 - Design (architect):
- Define data models
- API design
- Frontend components

Step 2 - Backend (direct implementation):
- Create database migrations
- Implement API endpoints
- Add caching layer

Step 3 - Frontend (direct implementation):
- Create React components
- Add charts and visualizations
- Implement filters

Step 4 - Testing (test-writer):
- API integration tests
- Component unit tests
- E2E tests

Step 5 - Documentation (doc-writer):
- API documentation
- User guide
- Architecture decision records

Step 6 - Review (code-reviewer + security-auditor):
- Code quality review
- Security review
- Performance review
```

---

## 7. 조건부 프롬프트

### 컨텍스트 기반 분기
```
Analyze the codebase and recommend improvements:

If TypeScript project:
- Check strict mode settings
- Review type coverage
- Suggest type improvements

If Python project:
- Check type hints coverage
- Review mypy configuration
- Suggest typing improvements

If no tests exist:
- Prioritize adding tests
- Start with critical paths
- Suggest testing framework

If tests exist but coverage < 60%:
- Identify uncovered code
- Prioritize by risk
- Add missing tests
```

---

## 8. 복합 에이전트 활용

### 병렬 분석
```
Analyze this PR from multiple perspectives simultaneously:

1. code-reviewer: Code quality and best practices
2. security-auditor: Security implications
3. performance-optimizer: Performance impact
4. architect: Architectural consistency

Combine findings into unified review with:
- Must fix (blocking)
- Should fix (important)
- Nice to have (suggestions)
```
