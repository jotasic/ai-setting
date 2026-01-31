---
name: create-testdata
description: 테스트 데이터 및 픽스처 생성
argument-hint: <model> [count]
---

# Create Test Data

테스트용 목업 데이터와 픽스처를 생성합니다.

## Arguments

- `$ARGUMENTS`:
  - `<model>`: 데이터 모델명 (e.g., `User`, `Order`)
  - `[count]`: 생성할 개수 (default: 10)

## Workflow

1. **Analyze Model**
   - Read model/schema definition
   - Identify field types and constraints
   - Check relationships

2. **Generate Data**
   - Realistic fake data
   - Respect constraints (unique, format)
   - Maintain referential integrity

3. **Output Formats**
   - JSON fixtures
   - SQL inserts
   - Factory functions

## Libraries

```bash
# JavaScript
npm install @faker-js/faker

# Python
pip install faker factory_boy
```

## Output Examples

### JSON Fixture
```json
{
  "users": [
    {
      "id": "uuid-1",
      "name": "John Doe",
      "email": "john@example.com",
      "createdAt": "2024-01-15T10:00:00Z"
    }
  ]
}
```

### Factory (TypeScript)
```typescript
import { faker } from '@faker-js/faker';

export const createUser = (overrides = {}) => ({
  id: faker.string.uuid(),
  name: faker.person.fullName(),
  email: faker.internet.email(),
  createdAt: faker.date.past(),
  ...overrides
});

export const createUsers = (count: number) =>
  Array.from({ length: count }, () => createUser());
```

### Factory (Python)
```python
import factory
from faker import Faker

fake = Faker()

class UserFactory(factory.Factory):
    class Meta:
        model = User

    id = factory.LazyFunction(lambda: str(uuid4()))
    name = factory.LazyFunction(fake.name)
    email = factory.LazyFunction(fake.email)
```

### SQL Seeds
```sql
INSERT INTO users (id, name, email, created_at) VALUES
('uuid-1', 'John Doe', 'john@example.com', NOW()),
('uuid-2', 'Jane Smith', 'jane@example.com', NOW());
```

## Data Types

| Field Type | Generator |
|------------|-----------|
| UUID | `faker.string.uuid()` |
| Name | `faker.person.fullName()` |
| Email | `faker.internet.email()` |
| Phone | `faker.phone.number()` |
| Address | `faker.location.streetAddress()` |
| Date | `faker.date.past()` |
| Number | `faker.number.int()` |
| Boolean | `faker.datatype.boolean()` |
