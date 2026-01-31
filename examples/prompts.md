# Effective Prompts for Claude Code

Claude Code를 효과적으로 사용하기 위한 프롬프트 예제 모음입니다.

## 1. 코드 작성

### 새 기능 구현
```
Create a user authentication module with:
- Email/password login
- JWT token generation
- Password hashing with bcrypt
- Session management

Use the existing database connection in src/db.ts
Follow the error handling pattern in src/utils/errors.ts
```

### API 엔드포인트 추가
```
Add a REST API endpoint for user profile:
- GET /api/users/:id - get user profile
- PUT /api/users/:id - update user profile
- DELETE /api/users/:id - delete user account

Include input validation and proper error responses
```

## 2. 버그 수정

### 에러 분석
```
I'm getting this error:
[에러 메시지 붙여넣기]

The error occurs when [상황 설명].
Expected behavior: [예상 동작]
Actual behavior: [실제 동작]
```

### 테스트 실패 수정
```
This test is failing:
[테스트 코드 또는 파일 경로]

Error message:
[에러 메시지]

Please investigate and fix the issue.
```

## 3. 코드 리뷰

### PR 리뷰 요청
```
Review the changes in this PR for:
- Code quality and readability
- Potential bugs or edge cases
- Security vulnerabilities
- Performance implications

Focus especially on [특정 영역].
```

### 코드 개선 제안
```
Suggest improvements for src/services/payment.ts:
- Better error handling
- Performance optimization
- Code organization

Don't change the public API.
```

## 4. 리팩토링

### 코드 정리
```
Refactor the OrderService class:
- Extract common logic into helper functions
- Reduce method complexity
- Improve naming

Ensure all existing tests still pass.
```

### 패턴 적용
```
Apply the Repository pattern to the data access layer.
Currently, database queries are scattered across services.
Centralize them while maintaining the same functionality.
```

## 5. 테스트 작성

### 단위 테스트 추가
```
Add unit tests for src/utils/validator.ts:
- Test all validation functions
- Include edge cases (empty, null, invalid format)
- Aim for >90% coverage
```

### 통합 테스트 추가
```
Add integration tests for the checkout flow:
1. Add items to cart
2. Apply discount code
3. Process payment
4. Verify order creation

Use the test database and mock the payment gateway.
```

## 6. 문서화

### API 문서 생성
```
Generate API documentation for the /api/orders endpoints.
Include:
- Request/response examples
- Authentication requirements
- Error codes
- Rate limits
```

### README 업데이트
```
Update the README with:
- New features added in v2.0
- Updated installation steps
- Environment variables reference
- Troubleshooting section
```

## 7. 아키텍처 설계

### 시스템 설계 요청
```
Design a notification system that:
- Supports email, SMS, and push notifications
- Handles high volume (10k+ messages/minute)
- Includes retry logic and failure handling
- Allows user preferences

Consider existing infrastructure and scalability needs.
```

### 기술 선택 조언
```
We need to add real-time features (chat, live updates).
Current stack: Node.js, PostgreSQL, React

Compare options:
- WebSockets vs Server-Sent Events
- Self-hosted vs managed service
- Impact on existing architecture
```

## 8. 보안 검토

### 보안 감사 요청
```
Perform a security audit on the authentication module.
Check for:
- OWASP Top 10 vulnerabilities
- Secure password handling
- Session management issues
- Input validation gaps
```

### 취약점 수정
```
Fix the SQL injection vulnerability in:
src/repositories/user.ts:45

The current query uses string concatenation.
Use parameterized queries instead.
```

## 9. 성능 최적화

### 성능 분석
```
The /api/dashboard endpoint is slow (>2s response time).
Profile the code and identify bottlenecks.
Suggest optimizations without changing the API response format.
```

### 쿼리 최적화
```
Optimize this database query that's causing performance issues:
[쿼리 붙여넣기]

Current execution time: 5s
Target: <100ms
Consider indexing and query restructuring.
```

## 10. 마이그레이션

### 버전 업그레이드
```
Upgrade the project from React 17 to React 18:
- Update dependencies
- Fix breaking changes
- Adopt new features where beneficial
- Ensure all tests pass
```

### 데이터베이스 마이그레이션
```
Create a migration to:
- Add 'status' column to orders table
- Backfill existing rows with 'completed' status
- Add index on status column

Use the existing migration framework.
```

---

## Tips for Better Prompts

1. **구체적으로 작성**: 모호한 요청보다 구체적인 요청이 더 좋은 결과
2. **컨텍스트 제공**: 관련 파일, 기존 패턴, 제약 조건 언급
3. **예상 결과 명시**: 원하는 출력 형태 설명
4. **단계별 분리**: 큰 작업은 작은 단계로 나누어 요청
5. **피드백 반복**: 결과를 확인하고 필요시 추가 지시
