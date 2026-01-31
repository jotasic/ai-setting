---
name: dependency-audit
description: 의존성 보안 및 호환성 검사
argument-hint: [--fix]
---

# Dependency Audit

프로젝트 의존성의 보안 취약점과 업데이트 상태를 검사합니다.

## Arguments

- `$ARGUMENTS`: `--fix` 자동 수정 시도 (optional)

## Workflow

1. **Security Audit**
   ```bash
   # npm
   npm audit

   # yarn
   yarn audit

   # pnpm
   pnpm audit

   # Python
   pip-audit
   ```

2. **Outdated Check**
   ```bash
   npm outdated
   ```

3. **License Check**
   ```bash
   npx license-checker --summary
   ```

4. **Bundle Impact**
   ```bash
   npx cost-of-modules --less
   ```

## Auto-Fix (if --fix)

```bash
# Safe fixes only
npm audit fix

# Including breaking changes (with confirmation)
npm audit fix --force
```

## Report Format

```markdown
## Dependency Audit Report

### Security Vulnerabilities
| Package | Severity | Issue | Fix |
|---------|----------|-------|-----|
| example | High | CVE-XXX | 1.2.3 |

### Outdated Packages
| Package | Current | Wanted | Latest |
|---------|---------|--------|--------|
| react | 17.0.2 | 17.0.2 | 18.2.0 |

### Recommendations
1. [Critical] Fix security issues
2. [High] Update major versions
3. [Medium] Update minor versions

### License Summary
All licenses compatible: MIT, Apache-2.0, ISC
```

## Priority

- **Critical**: Security vulnerabilities (high/critical)
- **High**: Security vulnerabilities (medium/low)
- **Medium**: Major version updates available
- **Low**: Minor/patch updates
