---
name: dependency-audit
description: Check dependencies for security and compatibility.
argument-hint: [--fix]
---

# Dependency Audit

Audit project dependencies for vulnerabilities and updates.

## Arguments

- `--fix`: Attempt auto-fix

## Workflow

1. **Security Audit**: `npm audit`, `pip-audit`, etc.
2. **Outdated Check**: `npm outdated`
3. **License Check**: `license-checker`
4. **Bundle Impact**: `cost-of-modules`

## Auto-Fix
- `npm audit fix`
- `npm audit fix --force` (careful)

## Report Format

```markdown
## Dependency Audit Report

### Security Vulnerabilities
| Package | Severity | Fix |
|---------|----------|-----|
| ... | ... | ... |

### Outdated Packages
...
```
