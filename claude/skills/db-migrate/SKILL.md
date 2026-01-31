---
name: db-migrate
description: 데이터베이스 마이그레이션 생성 및 실행
argument-hint: <create|run|rollback> [name]
---

# Database Migration

데이터베이스 마이그레이션을 생성하고 관리합니다.

## Arguments

- `$ARGUMENTS`:
  - `create <name>`: 새 마이그레이션 생성
  - `run`: 마이그레이션 실행
  - `rollback`: 마지막 마이그레이션 롤백
  - `status`: 마이그레이션 상태 확인

## Workflow

### Create Migration

1. Detect migration tool in use:
   - Prisma: `npx prisma migrate dev --name <name>`
   - TypeORM: `npx typeorm migration:create`
   - Knex: `npx knex migrate:make <name>`
   - Django: `python manage.py makemigrations`
   - Alembic: `alembic revision --autogenerate -m "<name>"`

2. Generate migration file with:
   - Up migration (apply changes)
   - Down migration (rollback)

### Run Migration

```bash
# Prisma
npx prisma migrate deploy

# TypeORM
npx typeorm migration:run

# Knex
npx knex migrate:latest

# Django
python manage.py migrate

# Alembic
alembic upgrade head
```

### Rollback Migration

```bash
# Knex
npx knex migrate:rollback

# TypeORM
npx typeorm migration:revert

# Alembic
alembic downgrade -1
```

## Safe Migration Checklist

- [ ] Backup database before migration
- [ ] Test on staging environment first
- [ ] Check for long-running locks
- [ ] Prepare rollback plan
- [ ] Monitor after deployment

## Migration Template

```sql
-- migrate:up
BEGIN;

ALTER TABLE users ADD COLUMN new_field VARCHAR(255);

COMMIT;

-- migrate:down
BEGIN;

ALTER TABLE users DROP COLUMN new_field;

COMMIT;
```

## Zero-Downtime Strategy

1. Add new column (nullable)
2. Deploy code that writes to both
3. Backfill data
4. Add constraints
5. Deploy code that reads from new
6. Remove old column
