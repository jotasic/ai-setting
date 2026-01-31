---
name: estimate-effort
description: Analyze scale and impact of code changes.
argument-hint: <description>
---

# Estimate Effort

Analyze task scope and impact.

## Arguments

- `$ARGUMENTS`: Task description

## Workflow

1. **Understand Scope**
2. **Analyze Codebase**
3. **Estimate Impact**
   - Files, LOC, Tests, Docs
4. **Risk Assessment**
   - Breaking changes, Performance

## Output Format

```markdown
## Effort Estimation

### Task: [Description]

### Scope Analysis
| Metric | Count |
|--------|-------|
| Files | ... |
| LOC | ... |

### Affected Areas
...

### Risk Assessment
...

### Recommended Approach
1. ...
2. ...
```
