---
name: changelog
description: Automatic CHANGELOG generation from git history.
argument-hint: [version]
---

# Changelog Generator

Generates CHANGELOG from Git commit history using Conventional Commits.

## Arguments

- `$ARGUMENTS`: Version number (optional, e.g., `1.2.0`)

## Workflow

1. **Analyze Commits**
   `git log` check.

2. **Categorize**
   - feat, fix, docs, perf, refactor, test, chore, BREAKING CHANGE

3. **Generate Changelog Entry**

## Output Format

```markdown
## [1.2.0] - 2024-01-15

### Breaking Changes
- ...

### Features
- ...

### Bug Fixes
- ...

### Performance
- ...
```
