---
name: create-testdata
description: 테스트 데이터 및 픽스처 생성
argument-hint: <model> [count] [--format=<json|sql|factory>]
allowed-tools: Read, Write, Grep, Glob
model: sonnet
category: testing
---

# Create Test Data

테스트용 목업 데이터와 픽스처를 생성합니다.

## Triggers (사용 조건)

- "테스트 데이터 만들어줘", "create fixtures"
- "목업 데이터", "seed data"
- 테스트용 데이터 필요시

## Arguments

- `<model>`: 데이터 모델명 (예: User, Order)
- `[count]`: 생성 개수 (default: 10)
- `--format=<json|sql|factory>`: 출력 포맷

## Workflow

```
┌─────────────────────────────────────┐
│  1. Analyze model schema            │
│  2. Generate realistic data         │
│  3. Respect constraints             │
│  4. Output in requested format      │
└─────────────────────────────────────┘
```

## Output Formats

| Format | File |
|--------|------|
| JSON | `fixtures/users.json` |
| SQL | `seeds/users.sql` |
| Factory | `factories/userFactory.ts` |

## Agent Integration

**스키마 분석:**
```
Use the database-specialist agent to analyze [model] schema
```

**테스트 작성:**
```
Use the test-writer agent to create tests using the generated data
```

## Data Types

| Field | Generator |
|-------|-----------|
| UUID | `faker.string.uuid()` |
| Name | `faker.person.fullName()` |
| Email | `faker.internet.email()` |
| Phone | `faker.phone.number()` |
| Date | `faker.date.past()` |

## Output Format

```
Test Data Generated
═══════════════════════════════════════
Model: User
Count: 10

Output:
  fixtures/users.json
  factories/userFactory.ts

Sample:
  {
    "id": "uuid-1",
    "name": "John Doe",
    "email": "john@example.com"
  }
═══════════════════════════════════════
```

## Examples

```bash
/create-testdata User 10                 # 기본
/create-testdata Order 50 --format=sql   # SQL 시드
/create-testdata Product --format=factory # 팩토리
```

## Related Skills

- `/run-tests`: 테스트 실행
- `/db-migrate`: DB 마이그레이션
