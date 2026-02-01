---
name: db-migrate
description: 데이터베이스 마이그레이션 생성 및 실행
argument-hint: <create|run|rollback|status> [name]
allowed-tools: Bash, Read, Write, Edit
model: sonnet
category: infrastructure
---

# Database Migration

데이터베이스 마이그레이션을 생성하고 관리합니다.

## Triggers (사용 조건)

- "마이그레이션 만들어줘", "create migration"
- "DB 변경 적용", "run migrate"
- "롤백해줘", "rollback migration"

## Arguments

- `create <name>`: 새 마이그레이션 생성
- `run`: 마이그레이션 실행
- `rollback`: 마지막 마이그레이션 롤백
- `status`: 현재 상태 확인

## Workflow

```
┌─────────────────────────────────────┐
│  create → Generate migration file   │
│  run    → Apply pending migrations  │
│  rollback → Revert last migration   │
│  status → Show migration status     │
└─────────────────────────────────────┘
```

## Migration Tools

| Framework | Create | Run |
|-----------|--------|-----|
| Prisma | `prisma migrate dev` | `prisma migrate deploy` |
| TypeORM | `typeorm migration:create` | `typeorm migration:run` |
| Knex | `knex migrate:make` | `knex migrate:latest` |
| Django | `makemigrations` | `migrate` |

## Agent Integration

**스키마 설계:**
```
Use the database-specialist agent to design schema for [feature]
```

**마이그레이션 검토:**
```
Use the database-specialist agent to review migration for potential issues
```

## Safety Checklist

- [ ] Backup database before migration
- [ ] Test on staging first
- [ ] Check for long-running locks
- [ ] Prepare rollback plan
- [ ] Monitor after deployment

## Output Format

```
DB Migration: [action]
═══════════════════════════════════════
Action: create
Name: add_user_avatar

Generated:
  migrations/20240115_add_user_avatar.sql

Content:
  -- Up
  ALTER TABLE users ADD COLUMN avatar_url TEXT;

  -- Down
  ALTER TABLE users DROP COLUMN avatar_url;

Next: /db-migrate run
═══════════════════════════════════════
```

## Examples

```bash
/db-migrate create add_user_avatar
/db-migrate run
/db-migrate rollback
/db-migrate status
```

## Related Skills

- `/setup-env`: 환경 설정
- `/full-dev`: 전체 개발 플로우
