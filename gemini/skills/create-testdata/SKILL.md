---
name: create-testdata
description: Generate test data and fixtures.
argument-hint: <model> [count]
---

# Create Test Data

Generates mock data and fixtures for testing.

## Arguments

- `<model>`: Data model name (e.g., `User`)
- `[count]`: Number of items (default: 10)

## Workflow

1. **Analyze Model**: Check schema/types.
2. **Generate Data**: Realistic fake data respecting constraints.
3. **Output**: JSON, SQL, or Factory code.

## Libraries

- `@faker-js/faker`
- `faker`, `factory_boy`

## Output Examples

### JSON Fixture
```json
{ "users": [{ "id": "...", "name": "..." }] }
```

### Factory
Code using faker/factory library.

### SQL Insert
`INSERT INTO ... VALUES ...`
