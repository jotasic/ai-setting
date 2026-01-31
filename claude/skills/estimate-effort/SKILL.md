---
name: estimate-effort
description: 코드 변경 규모 및 영향도 분석
argument-hint: <description>
---

# Estimate Effort

작업의 규모와 영향 범위를 분석합니다.

## Arguments

- `$ARGUMENTS`: 작업 설명

## Workflow

1. **Understand Scope**
   - Parse task description
   - Identify affected areas

2. **Analyze Codebase**
   - Find related files
   - Check dependencies
   - Identify test coverage

3. **Estimate Impact**
   - Files to modify
   - Lines of code
   - Test updates needed
   - Documentation updates

4. **Risk Assessment**
   - Breaking changes
   - Integration points
   - Performance impact

## Output Format

```markdown
## Effort Estimation

### Task: [Description]

### Scope Analysis

| Metric | Count |
|--------|-------|
| Files to modify | 8 |
| Files to create | 2 |
| Estimated LOC changes | ~250 |
| Tests to add/update | 12 |

### Affected Areas
- `src/api/users/` - Main implementation
- `src/services/auth/` - Integration point
- `src/types/` - Type definitions
- `tests/` - Test updates

### Dependencies
- Requires: None
- Blocks: User management feature

### Risk Assessment

| Risk | Level | Mitigation |
|------|-------|------------|
| Breaking API changes | Medium | Version API |
| Performance impact | Low | Add caching |
| Test coverage | Low | Existing tests cover |

### Complexity: Medium

### Recommended Approach
1. Start with type definitions
2. Implement core logic
3. Add API endpoints
4. Update tests
5. Update documentation

### Checklist
- [ ] Schema/type changes
- [ ] Business logic
- [ ] API endpoints
- [ ] Unit tests
- [ ] Integration tests
- [ ] Documentation
- [ ] Migration (if needed)
```

## Complexity Levels

| Level | Description | Typical Files |
|-------|-------------|---------------|
| Low | Simple change, isolated | 1-3 files |
| Medium | Multiple components | 4-10 files |
| High | Cross-cutting concern | 10+ files |
| Very High | Architecture change | System-wide |
