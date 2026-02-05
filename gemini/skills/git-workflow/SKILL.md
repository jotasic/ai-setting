---
name: git-workflow
description: Manages Git branch workflows (feature branch, PR)
argument-hint: [action: start|finish|sync]
disable-model-invocation: true
allowed-tools: Bash, Read
---

# Git Workflow

Automates git branch-based workflows.

## Arguments

- `start [feature-name]`: Create new feature branch
- `finish`: Prepare PR for current branch
- `sync`: Sync with main branch

## Workflow: Start Feature

```bash
# 1. Checkout and pull main
git checkout main
git pull origin main

# 2. Create feature branch
git checkout -b feature/$FEATURE_NAME

# 3. Check status
git status
```

## Workflow: Finish Feature

```bash
# 1. Check changes
git status
git diff --stat main...HEAD

# 2. Push
git push -u origin $(git branch --show-current)

# 3. Create PR
gh pr create --fill
```

## Workflow: Sync with Main

```bash
# 1. Fetch main
git fetch origin main

# 2. Rebase
git rebase origin/main

# 3. Push if needed
git push --force-with-lease
```
