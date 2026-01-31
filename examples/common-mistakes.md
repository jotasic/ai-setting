# Common Mistakes Guide

Claude Code 사용 시 흔히 발생하는 실수와 해결 방법입니다.

## 1. 에이전트 선택 실수

### 잘못된 에이전트 사용

| 상황 | 잘못된 선택 | 올바른 선택 |
|------|-------------|-------------|
| 복잡한 아키텍처 설계 | `code-reviewer` | `architect` (opus) |
| 보안 취약점 검사 | `code-reviewer` | `security-auditor` (opus) |
| 간단한 문서화 | `architect` | `doc-writer` (haiku) |
| 성능 문제 분석 | `debugger` | `performance-optimizer` |

### 해결 방법
```
# 작업 복잡도에 맞는 에이전트 선택
- 높은 복잡도 → opus 에이전트 (architect, security-auditor)
- 일반 코딩 → sonnet 에이전트 (code-reviewer, test-writer)
- 단순 작업 → haiku 에이전트 (doc-writer)
```

---

## 2. 프롬프트 작성 실수

### 너무 모호한 프롬프트

```
# Bad
Fix the bug

# Good
Fix the authentication bug in src/auth/login.ts:45
where users with special characters in passwords
cannot log in. The bcrypt comparison fails.
```

### 컨텍스트 부족

```
# Bad
Add tests

# Good
Use the test-writer agent to add Jest tests for:
- src/utils/validation.ts
- Focus on email and password validation
- Include edge cases (empty, null, special chars)
- Target 90% coverage
```

### 여러 작업을 한 번에 요청

```
# Bad
Refactor the code, add tests, fix bugs, and update docs

# Good (단계별 접근)
Step 1: Use refactorer to improve src/services/user.ts
Step 2: Use test-writer to add missing tests
Step 3: Use debugger to fix failing tests
Step 4: Use doc-writer to update API docs
```

---

## 3. 스킬 사용 실수

### 인자 누락

```
# Bad
/fix-issue

# Good
/fix-issue 123

# Bad
/db-migrate

# Good
/db-migrate create add-user-roles
```

### 잘못된 순서

```
# Bad (테스트 없이 커밋)
/commit "Add new feature"

# Good
/run-tests
# 테스트 통과 확인 후
/commit "Add new feature"
```

---

## 4. 설정 실수

### 권한 설정 오류

```json
// Bad - 너무 광범위
{
  "allow": ["Bash(*)"]
}

// Good - 필요한 것만 허용
{
  "allow": [
    "Bash(npm run *)",
    "Bash(git status*)",
    "Bash(git log*)"
  ]
}
```

### MCP 서버 환경 변수 누락

```json
// Bad - 환경 변수 없음
{
  "github": {
    "command": "npx",
    "args": ["-y", "@anthropic/mcp-server-github"]
  }
}

// Good - 환경 변수 설정
{
  "github": {
    "command": "npx",
    "args": ["-y", "@anthropic/mcp-server-github"],
    "env": {
      "GITHUB_TOKEN": "${GITHUB_TOKEN}"
    }
  }
}
```

---

## 5. 워크플로우 실수

### 검증 없이 진행

```
# Bad
1. 코드 작성
2. 커밋
3. 푸시

# Good
1. 코드 작성
2. /run-tests (테스트 실행)
3. /lint (린트 검사)
4. code-reviewer로 리뷰
5. /commit
6. 푸시
```

### 의존성 무시

```
# Bad
새 기능 바로 구현 시작

# Good
1. /architecture-review로 영향도 분석
2. /estimate-effort로 규모 파악
3. architect로 설계
4. 구현 시작
```

---

## 6. 보안 실수

### 민감 정보 노출

```
# Bad - 비밀 하드코딩
const API_KEY = 'sk-1234567890abcdef';

# Good - 환경 변수 사용
const API_KEY = process.env.API_KEY;
```

### 보안 검토 생략

```
# Bad
새 API 엔드포인트 배포

# Good
1. 구현
2. security-auditor로 검토
3. 취약점 수정
4. 재검토
5. 배포
```

---

## 7. 성능 실수

### 프로파일링 없이 최적화

```
# Bad
"이 부분이 느린 것 같으니 최적화하자"

# Good
/performance-profile
# 실제 병목 지점 확인 후 최적화
```

### 조기 최적화

```
# Bad
처음부터 복잡한 캐싱 레이어 추가

# Good
1. 먼저 동작하는 코드 작성
2. 성능 측정
3. 필요시 최적화
```

---

## 8. 테스트 실수

### 테스트 없이 리팩토링

```
# Bad
1. 코드 리팩토링
2. "잘 되는 것 같다"

# Good
1. 기존 테스트 확인 (커버리지)
2. 필요시 테스트 추가
3. 리팩토링
4. 모든 테스트 통과 확인
```

### 해피 패스만 테스트

```
# Bad
test('creates user', () => {
  expect(createUser(validData)).toBeDefined();
});

# Good
describe('createUser', () => {
  it('creates user with valid data', () => {...});
  it('throws on invalid email', () => {...});
  it('throws on duplicate email', () => {...});
  it('handles null input', () => {...});
  it('trims whitespace', () => {...});
});
```

---

## 9. 문서화 실수

### 코드만 작성하고 문서 무시

```
# Bad
기능 구현 완료, 문서 없음

# Good
1. 기능 구현
2. doc-writer로 API 문서 생성
3. README 업데이트
4. 변경 로그 추가
```

### 과도한 문서화

```
# Bad - 모든 것을 문서화
// 변수 i를 1로 초기화합니다
let i = 1;

# Good - 의미 있는 문서화
/**
 * Validates user email format and domain restrictions.
 * @param email - Email address to validate
 * @returns true if valid, throws ValidationError otherwise
 */
function validateEmail(email: string): boolean
```

---

## 10. 협업 실수

### 대규모 변경을 한 번에 커밋

```
# Bad
git commit -m "Refactor everything"

# Good
git commit -m "refactor(auth): extract token validation"
git commit -m "refactor(auth): add refresh token support"
git commit -m "test(auth): add token validation tests"
```

### PR 리뷰 요청 없이 머지

```
# Bad
코드 작성 → 바로 main에 푸시

# Good
1. 기능 브랜치 생성
2. 코드 작성
3. PR 생성
4. code-reviewer로 자체 리뷰
5. 팀 리뷰 요청
6. 피드백 반영
7. 머지
```

---

## 체크리스트

### 작업 시작 전
- [ ] 적절한 에이전트 선택했는가?
- [ ] 명확한 프롬프트를 작성했는가?
- [ ] 컨텍스트를 충분히 제공했는가?

### 작업 중
- [ ] 단계별로 검증하고 있는가?
- [ ] 테스트를 함께 작성하고 있는가?
- [ ] 보안을 고려하고 있는가?

### 작업 완료 후
- [ ] 테스트가 통과하는가?
- [ ] 린트 검사를 통과하는가?
- [ ] 문서를 업데이트했는가?
- [ ] 코드 리뷰를 받았는가?
