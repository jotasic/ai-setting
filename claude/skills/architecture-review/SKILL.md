---
name: architecture-review
description: 현재 시스템 아키텍처 분석 및 리뷰
argument-hint: [focus-area]
---

# Architecture Review

시스템 아키텍처를 분석하고 개선 기회를 식별합니다.

## Arguments

- `$ARGUMENTS`: 집중 분석 영역 (optional)
  - `structure`: 프로젝트 구조
  - `dependencies`: 의존성 관계
  - `data-flow`: 데이터 흐름
  - `security`: 보안 아키텍처

## Workflow

1. **Scan Project Structure**
   ```bash
   tree -I 'node_modules|dist|.git' -L 3
   ```

2. **Analyze Components**
   - Entry points
   - Core modules
   - Shared utilities
   - External integrations

3. **Map Dependencies**
   - Internal dependencies
   - External packages
   - Circular dependencies

4. **Identify Patterns**
   - Design patterns in use
   - Anti-patterns detected
   - Consistency issues

5. **Generate Report**

## Output Format

```markdown
## Architecture Review

### Project Structure
```
src/
├── api/          # REST endpoints
├── services/     # Business logic
├── repositories/ # Data access
├── models/       # Domain models
└── utils/        # Shared utilities
```

### Architecture Pattern
**Layered Architecture** with:
- Presentation Layer (api/)
- Business Layer (services/)
- Data Access Layer (repositories/)

### Component Diagram
```
┌─────────────┐     ┌─────────────┐
│   API       │────▶│  Services   │
└─────────────┘     └─────────────┘
                           │
                           ▼
                    ┌─────────────┐
                    │ Repositories│
                    └─────────────┘
                           │
                           ▼
                    ┌─────────────┐
                    │  Database   │
                    └─────────────┘
```

### Strengths
- Clear separation of concerns
- Consistent naming conventions
- Good test coverage

### Concerns
| Issue | Severity | Location | Recommendation |
|-------|----------|----------|----------------|
| Circular dependency | Medium | services/ | Extract shared interface |
| God class | High | UserService | Split responsibilities |
| Missing abstraction | Low | utils/ | Create proper interfaces |

### Recommendations
1. **High Priority**
   - Refactor UserService (500+ LOC)
   - Add caching layer

2. **Medium Priority**
   - Standardize error handling
   - Add API versioning

3. **Low Priority**
   - Improve logging structure
   - Add performance monitoring
```

## Analysis Areas

### Structure
- Directory organization
- Module boundaries
- Naming conventions

### Dependencies
- Package dependencies
- Internal module coupling
- Circular dependencies

### Data Flow
- Request/response flow
- State management
- Event handling

### Security
- Authentication flow
- Authorization checks
- Data validation
