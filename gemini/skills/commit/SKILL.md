---
name: commit
description: Commit changes using Conventional Commits standard.
argument-hint: [commit message or --amend]
disable-model-invocation: true
allowed-tools: Bash, Read, Grep
---

# Git Commit

Commits changes using the Conventional Commits format.

## Arguments

- `$ARGUMENTS`: Commit message or options

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

Conventional Commits Format:

```
<type>(<scope>): <description>

[optional body]

[optional footer]
```

#### Types
| Type | Description |
|------|-------------|
| feat | New feature |
| fix | Bug fix |
| docs | Documentation |
| style | Style changes (formatting) |
| refactor | Refactoring |
| test | Tests |
| chore | Build/Tooling |
| perf | Performance |

### 4. Verify

```bash
git log -1 --oneline
git show --stat
```
