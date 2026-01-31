---
name: setup-env
description: 개발 환경 자동 설정
argument-hint: [--clean]
---

# Setup Environment

프로젝트 개발 환경을 자동으로 설정합니다.

## Arguments

- `$ARGUMENTS`: `--clean` 클린 설치 (optional)

## Workflow

1. **Detect Project Type**
   - Node.js (package.json)
   - Python (requirements.txt, pyproject.toml)
   - Go (go.mod)
   - Rust (Cargo.toml)

2. **Install Dependencies**
   ```bash
   # Node.js
   npm install
   # or
   yarn install
   # or
   pnpm install

   # Python
   pip install -r requirements.txt
   # or
   poetry install

   # Go
   go mod download

   # Rust
   cargo build
   ```

3. **Setup Configuration**
   - Copy `.env.example` to `.env` if exists
   - Generate required config files
   - Setup pre-commit hooks if configured

4. **Database Setup**
   - Run migrations
   - Seed development data

5. **Verify Setup**
   - Run health checks
   - Execute test suite
   - Start development server

## Clean Install (--clean)

```bash
# Node.js
rm -rf node_modules
rm package-lock.json
npm install

# Python
rm -rf .venv
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Common Setup Tasks

### Git Hooks
```bash
npx husky install
```

### Environment Variables
```bash
if [ ! -f .env ]; then
  cp .env.example .env
  echo "Created .env from .env.example"
fi
```

### Database
```bash
npm run db:migrate
npm run db:seed
```

## Output

```markdown
## Environment Setup Complete

### Installed
- Node.js dependencies: 342 packages
- Python dependencies: 28 packages

### Configured
- [x] .env file created
- [x] Git hooks installed
- [x] Database migrated

### Next Steps
1. Update .env with your credentials
2. Run `npm run dev` to start
```
