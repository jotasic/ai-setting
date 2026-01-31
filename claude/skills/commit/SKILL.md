---
name: commit
description: 변경사항을 커밋합니다 (Conventional Commits)
argument-hint: [commit message or --amend]
disable-model-invocation: true
allowed-tools: Bash, Read, Grep
---

# Git Commit

변경사항을 Conventional Commits 형식으로 커밋합니다.

## Arguments

- `$ARGUMENTS`: 커밋 메시지 또는 옵션

## Workflow

### 1. Check Status

```bash
git status
git diff --staged
```

### 2. Stage Changes (if needed)

```bash
git add -p  # Interactive staging
```

### 3. Create Commit

Conventional Commits 형식:

```
<type>(<scope>): <description>

[optional body]

[optional footer]
```

#### Types
| Type | Description |
|------|-------------|
| feat | 새로운 기능 |
| fix | 버그 수정 |
| docs | 문서 변경 |
| style | 코드 스타일 (포맷팅) |
| refactor | 리팩토링 |
| test | 테스트 추가/수정 |
| chore | 빌드, 설정 등 |
| perf | 성능 개선 |

### 4. Verify

```bash
git log -1 --oneline
git show --stat
```

## Examples

```bash
# Feature
git commit -m "feat(auth): add OAuth2 login support"

# Bug fix
git commit -m "fix(api): handle null response correctly"

# Breaking change
git commit -m "feat(api)!: change response format

BREAKING CHANGE: response now returns array instead of object"
```

## Guidelines

- 제목은 50자 이내
- 본문은 72자에서 줄바꿈
- 명령형 현재시제 사용 (Add, not Added)
- Why를 설명, What은 코드가 설명
