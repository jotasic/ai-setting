---
name: setup-env
description: Automate development environment setup.
argument-hint: [--clean]
---

# Setup Environment

Sets up dev environment automatically.

## Arguments

- `--clean`: Clean install

## Workflow

1. **Detect Project Type** (Node, Python, Go, Rust)
2. **Install Dependencies** (npm/yarn/pnpm, pip/poetry, go mod, cargo)
3. **Setup Configuration** (.env, hooks)
4. **Database Setup** (Migrate, Seed)
5. **Verify Setup** (Health check, tests, start server)

## Clean Install
Removes node_modules/.venv before install.

## Output
- Installed packages count
- Config status
- Next steps
