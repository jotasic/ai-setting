---
name: architecture-review
description: Architecture analysis and review.
argument-hint: [focus-area]
---

# Architecture Review

Analyze system architecture and identify improvements.

## Arguments

- `$ARGUMENTS`: Focus area (optional)
  - `structure`: Project structure
  - `dependencies`: Dependency graph
  - `data-flow`: Data flow
  - `security`: Security architecture

## Workflow

1. **Scan Project Structure**
2. **Analyze Components**
3. **Map Dependencies**
4. **Identify Patterns**
   - Design patterns
   - Anti-patterns
5. **Generate Report**

## Output Format

```markdown
## Architecture Review

### Project Structure
...

### Architecture Pattern
Layered Architecture / Hexagonal / etc.

### Component Diagram
...

### Strengths
...

### Concerns
| Issue | Severity | Recommendation |
|-------|----------|----------------|
| ... | ... | ... |

### Recommendations
1. High Priority
2. Medium Priority
3. Low Priority
```
