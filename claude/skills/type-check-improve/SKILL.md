---
name: type-check-improve
description: TypeScript/Python 타입 검사 강화
argument-hint: [path]
---

# Type Check & Improve

타입 커버리지를 분석하고 타입 안전성을 강화합니다.

## Arguments

- `$ARGUMENTS`: 검사할 경로 (default: `./src`)

## Workflow

1. **Run Type Check**
   ```bash
   # TypeScript
   npx tsc --noEmit

   # Python
   mypy .
   pyright .
   ```

2. **Analyze Coverage**
   ```bash
   # TypeScript
   npx type-coverage

   # Python
   mypy --html-report ./type-report .
   ```

3. **Identify Issues**
   - Implicit `any` types
   - Missing return types
   - Unsafe type assertions
   - Missing null checks

4. **Generate Improvements**
   - Add explicit type annotations
   - Replace `any` with proper types
   - Add type guards
   - Fix type errors

## TypeScript Improvements

### Before
```typescript
function process(data) {
  return data.map(item => item.value);
}
```

### After
```typescript
interface DataItem {
  value: string;
}

function process(data: DataItem[]): string[] {
  return data.map(item => item.value);
}
```

## Python Improvements

### Before
```python
def process(data):
    return [item.value for item in data]
```

### After
```python
from typing import List

class DataItem:
    value: str

def process(data: List[DataItem]) -> List[str]:
    return [item.value for item in data]
```

## Strict Mode Checklist

### TypeScript (tsconfig.json)
```json
{
  "compilerOptions": {
    "strict": true,
    "noImplicitAny": true,
    "strictNullChecks": true,
    "noImplicitReturns": true,
    "noUncheckedIndexedAccess": true
  }
}
```

### Python (mypy.ini)
```ini
[mypy]
strict = true
warn_return_any = true
warn_unused_ignores = true
```

## Output

```markdown
## Type Coverage Report

### Current: 78% → Target: 95%

### Issues Found
| File | Line | Issue | Suggestion |
|------|------|-------|------------|
| api.ts | 45 | Implicit any | Add User type |

### Improvements Made
- [x] Added types to 15 functions
- [x] Replaced 8 `any` types
- [x] Added 3 type guards
```
