---
name: changelog
description: 커밋 로그에서 CHANGELOG 자동 생성
argument-hint: [version]
---

# Changelog Generator

Git 커밋 히스토리에서 CHANGELOG를 자동 생성합니다.

## Arguments

- `$ARGUMENTS`: 버전 번호 (optional, e.g., `1.2.0`)

## Workflow

1. **Analyze Commits**
   ```bash
   git log --oneline --since="last tag"
   # or
   git log v1.0.0..HEAD --oneline
   ```

2. **Categorize by Conventional Commits**
   - `feat:` → Features
   - `fix:` → Bug Fixes
   - `docs:` → Documentation
   - `perf:` → Performance
   - `refactor:` → Refactoring
   - `test:` → Tests
   - `chore:` → Maintenance
   - `BREAKING CHANGE:` → Breaking Changes

3. **Generate Changelog Entry**

## Output Format

```markdown
## [1.2.0] - 2024-01-15

### Breaking Changes
- Removed deprecated `oldFunction()` API

### Features
- Add user authentication (#123)
- Implement dark mode support (#145)

### Bug Fixes
- Fix memory leak in data processing (#156)
- Resolve race condition in async handler (#162)

### Performance
- Optimize database queries (30% faster)

### Documentation
- Update API reference
- Add migration guide
```

## Auto-Detection

If no version provided:
1. Check latest git tag
2. Suggest next version based on changes:
   - Breaking changes → Major bump
   - Features → Minor bump
   - Fixes only → Patch bump

## Integration

```bash
# Generate and prepend to CHANGELOG.md
# Update package.json version
# Create git tag
```

## Templates

### Keep a Changelog Format
Following https://keepachangelog.com/

### Conventional Changelog
Following https://conventionalcommits.org/
