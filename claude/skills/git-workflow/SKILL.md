---
name: git-workflow
description: Git 브랜치 워크플로우를 관리합니다 (feature branch, PR)
argument-hint: [action: start|finish|sync]
disable-model-invocation: true
allowed-tools: Bash, Read
---

# Git Workflow

Git 브랜치 기반 워크플로우를 자동화합니다.

## Arguments

- `start [feature-name]`: 새 feature 브랜치 생성
- `finish`: 현재 브랜치 PR 생성 준비
- `sync`: main 브랜치와 동기화

## Workflow: Start Feature

```bash
# 1. main 브랜치로 이동 및 최신화
git checkout main
git pull origin main

# 2. feature 브랜치 생성
git checkout -b feature/$FEATURE_NAME

# 3. 확인
git status
```

## Workflow: Finish Feature

```bash
# 1. 변경사항 확인
git status
git diff --stat main...HEAD

# 2. 커밋 정리 (선택)
git log --oneline main..HEAD

# 3. push
git push -u origin $(git branch --show-current)

# 4. PR 생성
gh pr create --fill
```

## Workflow: Sync with Main

```bash
# 1. main 최신화
git fetch origin main

# 2. rebase 또는 merge
git rebase origin/main
# 또는
git merge origin/main

# 3. 충돌 해결 후
git push --force-with-lease
```

## Branch Naming Convention

| Type | Pattern | Example |
|------|---------|---------|
| Feature | `feature/[name]` | `feature/user-auth` |
| Bugfix | `fix/[issue]` | `fix/login-error` |
| Hotfix | `hotfix/[name]` | `hotfix/security-patch` |
| Release | `release/[version]` | `release/v1.2.0` |
