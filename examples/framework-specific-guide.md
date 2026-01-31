# Framework-Specific Guide

프레임워크별 Claude Code 활용 가이드입니다.

## Frontend Frameworks

### React

#### 프로젝트 구조
```
src/
├── components/     # UI 컴포넌트
├── hooks/          # 커스텀 훅
├── pages/          # 페이지 컴포넌트
├── services/       # API 호출
├── store/          # 상태 관리
├── types/          # TypeScript 타입
└── utils/          # 유틸리티
```

#### 권장 프롬프트
```
Create a React component for [description]:
- Use functional components with hooks
- Include TypeScript types
- Add proper error boundaries
- Include unit tests with React Testing Library
```

#### 성능 최적화
```
Use the performance-optimizer agent to analyze:
- Unnecessary re-renders
- Bundle size optimization
- Code splitting opportunities
- Memoization candidates (useMemo, useCallback)
```

---

### Vue.js

#### 프로젝트 구조
```
src/
├── components/     # 컴포넌트
├── composables/    # Composition API
├── views/          # 페이지 뷰
├── stores/         # Pinia 스토어
├── router/         # Vue Router
└── types/          # TypeScript
```

#### 권장 프롬프트
```
Create a Vue component for [description]:
- Use Composition API with <script setup>
- Include TypeScript types
- Add Pinia store integration if needed
- Include Vitest unit tests
```

---

### Next.js

#### 프로젝트 구조 (App Router)
```
app/
├── (routes)/       # 라우트 그룹
├── api/            # API 라우트
├── components/     # 컴포넌트
└── lib/            # 유틸리티
```

#### 권장 프롬프트
```
Create a Next.js page for [description]:
- Use App Router conventions
- Implement server/client components appropriately
- Add proper metadata for SEO
- Include loading and error states
```

---

## Backend Frameworks

### Express.js

#### 프로젝트 구조
```
src/
├── controllers/    # 요청 핸들러
├── middleware/     # 미들웨어
├── models/         # 데이터 모델
├── routes/         # 라우트 정의
├── services/       # 비즈니스 로직
└── utils/          # 유틸리티
```

#### 권장 프롬프트
```
Create an Express API endpoint for [description]:
- Include input validation (Zod/Joi)
- Add proper error handling
- Include authentication middleware
- Add OpenAPI documentation
```

---

### FastAPI

#### 프로젝트 구조
```
app/
├── api/
│   └── v1/         # API 버전
├── core/           # 설정, 보안
├── models/         # SQLAlchemy 모델
├── schemas/        # Pydantic 스키마
├── services/       # 비즈니스 로직
└── tests/          # 테스트
```

#### 권장 프롬프트
```
Create a FastAPI endpoint for [description]:
- Use Pydantic models for validation
- Add proper HTTP status codes
- Include dependency injection
- Add OpenAPI documentation
```

---

### NestJS

#### 프로젝트 구조
```
src/
├── modules/
│   └── users/
│       ├── users.controller.ts
│       ├── users.service.ts
│       ├── users.module.ts
│       └── dto/
├── common/         # 공통 유틸
└── config/         # 설정
```

#### 권장 프롬프트
```
Create a NestJS module for [description]:
- Use proper decorators
- Include DTOs with class-validator
- Add service layer
- Include unit tests with Jest
```

---

## Database & ORM

### Prisma
```
Use the database-specialist agent to:
- Design Prisma schema
- Create migrations
- Optimize queries
- Set up database indexes
```

### TypeORM
```
Create TypeORM entities for [description]:
- Include proper decorators
- Define relationships
- Add migration scripts
```

### SQLAlchemy
```
Create SQLAlchemy models for [description]:
- Use declarative base
- Define relationships
- Include Alembic migrations
```

---

## 프레임워크별 테스트

| Framework | 테스트 도구 | 프롬프트 |
|-----------|-------------|----------|
| React | Jest + RTL | `Use test-writer to add React Testing Library tests` |
| Vue | Vitest + VTL | `Use test-writer to add Vue Test Utils tests` |
| Express | Jest + Supertest | `Use test-writer to add API integration tests` |
| FastAPI | pytest + httpx | `Use test-writer to add pytest API tests` |
| NestJS | Jest | `Use test-writer to add NestJS e2e tests` |

---

## CI/CD 템플릿

### GitHub Actions (Node.js)
```yaml
name: CI
on: [push, pull_request]
jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
      - run: npm ci
      - run: npm run lint
      - run: npm test
      - run: npm run build
```

### GitHub Actions (Python)
```yaml
name: CI
on: [push, pull_request]
jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
      - run: pip install -r requirements.txt
      - run: ruff check .
      - run: mypy .
      - run: pytest
```
