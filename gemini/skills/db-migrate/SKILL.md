---
name: db-migrate
description: Create and run database migrations.
argument-hint: <create|run|rollback> [name]
---

# Database Migration

Manage database migrations.

## Arguments

- `create <name>`: Create new migration
- `run`: Apply migrations
- `rollback`: Revert migration
- `status`: Check status

## Workflow

### Create
Detect tool (Prisma, TypeORM, Django, Alembic) and generate migration file.

### Run/Rollback
Execute corresponding command (e.g., `npx prisma migrate deploy`).

## Safe Migration Checklist

- [ ] Backup
- [ ] Test on staging
- [ ] Check locks
- [ ] Rollback plan
- [ ] Monitor
