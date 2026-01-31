# Testing Strategy Guide

효과적인 테스트 전략 수립 가이드입니다.

## 테스트 피라미드

```
        /\
       /  \      E2E Tests (10%)
      /----\     - Critical user journeys
     /      \    - Cross-browser testing
    /--------\
   /          \  Integration Tests (20%)
  /            \ - API endpoints
 /--------------\- Database operations
/                \
/==================\ Unit Tests (70%)
                    - Functions
                    - Components
                    - Utilities
```

---

## 테스트 유형별 가이드

### 1. Unit Tests

#### 무엇을 테스트할까?
- 순수 함수
- 유틸리티 함수
- 비즈니스 로직
- React/Vue 컴포넌트

#### 프롬프트 예시
```
Use the test-writer agent to add unit tests for src/utils/validation.ts:
- Test each validation function
- Include edge cases (empty, null, special characters)
- Test error messages
- Aim for 100% branch coverage
```

#### Jest 예시
```typescript
describe('validateEmail', () => {
  it('should return true for valid email', () => {
    expect(validateEmail('test@example.com')).toBe(true);
  });

  it('should return false for invalid email', () => {
    expect(validateEmail('invalid')).toBe(false);
  });

  it('should handle edge cases', () => {
    expect(validateEmail('')).toBe(false);
    expect(validateEmail(null)).toBe(false);
  });
});
```

---

### 2. Integration Tests

#### 무엇을 테스트할까?
- API 엔드포인트
- 데이터베이스 연동
- 외부 서비스 통합
- 미들웨어 체인

#### 프롬프트 예시
```
Use the test-writer agent to add integration tests for src/api/users:
- Test CRUD operations
- Test authentication middleware
- Test error responses
- Test pagination and filtering
- Mock external services
```

#### Supertest 예시
```typescript
describe('POST /api/users', () => {
  it('should create user with valid data', async () => {
    const response = await request(app)
      .post('/api/users')
      .send({ email: 'test@example.com', name: 'Test' })
      .expect(201);

    expect(response.body).toHaveProperty('id');
  });

  it('should return 400 for invalid data', async () => {
    await request(app)
      .post('/api/users')
      .send({ email: 'invalid' })
      .expect(400);
  });
});
```

---

### 3. E2E Tests

#### 무엇을 테스트할까?
- 핵심 사용자 여정
- 결제 플로우
- 회원가입/로그인
- 크로스 브라우저 호환성

#### 프롬프트 예시
```
Use the test-writer agent to add Playwright E2E tests:
- Test user registration flow
- Test login with valid/invalid credentials
- Test checkout process
- Run on Chrome, Firefox, Safari
```

#### Playwright 예시
```typescript
test('user can complete purchase', async ({ page }) => {
  await page.goto('/products');
  await page.click('[data-testid="add-to-cart"]');
  await page.click('[data-testid="checkout"]');
  await page.fill('#email', 'test@example.com');
  await page.fill('#card', '4242424242424242');
  await page.click('button[type="submit"]');
  await expect(page.locator('.success')).toBeVisible();
});
```

---

## 테스트 커버리지 전략

### 커버리지 목표
| 영역 | 목표 | 우선순위 |
|------|------|----------|
| 비즈니스 로직 | 90%+ | 높음 |
| API 엔드포인트 | 80%+ | 높음 |
| UI 컴포넌트 | 70%+ | 중간 |
| 유틸리티 | 100% | 중간 |
| 설정/초기화 | 50%+ | 낮음 |

### 커버리지 측정
```bash
# Jest
npm test -- --coverage

# pytest
pytest --cov=src --cov-report=html

# Go
go test -cover ./...
```

---

## 테스트 데이터 관리

### Fixtures 생성
```
Use /create-testdata User 50 to generate:
- 50 realistic user records
- Various edge cases
- Consistent test data
```

### Factory 패턴
```typescript
// factories/user.ts
import { faker } from '@faker-js/faker';

export const createUser = (overrides = {}) => ({
  id: faker.string.uuid(),
  email: faker.internet.email(),
  name: faker.person.fullName(),
  createdAt: faker.date.past(),
  ...overrides,
});
```

---

## 모킹 전략

### 외부 서비스 모킹
```typescript
// Mock Stripe
jest.mock('stripe', () => ({
  Stripe: jest.fn(() => ({
    charges: {
      create: jest.fn().mockResolvedValue({ id: 'ch_123' }),
    },
  })),
}));
```

### API 모킹 (MSW)
```typescript
import { rest } from 'msw';
import { setupServer } from 'msw/node';

const server = setupServer(
  rest.get('/api/users', (req, res, ctx) => {
    return res(ctx.json([{ id: 1, name: 'Test' }]));
  })
);

beforeAll(() => server.listen());
afterEach(() => server.resetHandlers());
afterAll(() => server.close());
```

---

## CI/CD 통합

### GitHub Actions
```yaml
jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - run: npm ci
      - run: npm test -- --coverage
      - uses: codecov/codecov-action@v3
```

### 테스트 병렬화
```yaml
strategy:
  matrix:
    shard: [1, 2, 3, 4]
steps:
  - run: npm test -- --shard=${{ matrix.shard }}/4
```

---

## 테스트 작성 체크리스트

### 새 기능 추가 시
- [ ] 단위 테스트 작성
- [ ] 통합 테스트 작성 (API인 경우)
- [ ] E2E 테스트 업데이트 (주요 플로우인 경우)
- [ ] 엣지 케이스 테스트
- [ ] 에러 케이스 테스트

### 버그 수정 시
- [ ] 버그 재현 테스트 작성
- [ ] 수정 후 테스트 통과 확인
- [ ] 회귀 테스트 추가

### 리팩토링 시
- [ ] 기존 테스트 통과 확인
- [ ] 필요시 테스트 업데이트
- [ ] 커버리지 유지 확인
