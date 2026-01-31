---
name: generate-claudemd
description: 세션 대화 기록과 프로젝트 분석을 통해 CLAUDE.md 자동 생성
argument-hint: [--output <path>] [--append] [--minimal]
---

# Generate CLAUDE.md

현재 세션의 대화 기록과 프로젝트 구조를 분석하여 CLAUDE.md를 자동 생성합니다.

## Arguments

- `$ARGUMENTS`:
  - `--output <path>`: 출력 경로 지정 (기본: ./CLAUDE.md)
  - `--append`: 기존 파일에 추가
  - `--minimal`: 최소 버전 생성
  - `--full`: 전체 분석 포함

## Workflow

### Step 1: Project Analysis
```bash
# 프로젝트 타입 감지
ls package.json pyproject.toml Cargo.toml go.mod pom.xml Gemfile 2>/dev/null

# 구조 파악
tree -I 'node_modules|dist|.git|__pycache__|venv|.venv' -L 2
```

### Step 2: Configuration Discovery
```bash
# 기존 설정 확인
ls -la .claude/ 2>/dev/null
cat .eslintrc* .prettierrc* tsconfig.json pyproject.toml 2>/dev/null | head -50
```

### Step 3: Session Analysis

이 세션에서 수행한 작업들을 분석:
- 사용된 명령어들
- 생성/수정된 파일들
- 발생한 이슈와 해결 방법
- 확립된 패턴과 규칙

### Step 4: Generate Documentation

## Output Sections

### Minimal (--minimal)
```markdown
# Project Name

## Quick Start
## Key Commands
## Tips
```

### Standard (default)
```markdown
# Project Name

## Quick Start
## Project Structure
## Development
### Commands
### Conventions
## Testing
## Tips for Claude
```

### Full (--full)
```markdown
# Project Name

## Overview
## Quick Start
## Project Structure
## Architecture
## Development Workflow
### Build & Run
### Testing
### Linting
## API Reference
## Database
## Deployment
## Troubleshooting
## Session Learnings
## Tips for Claude
```

## Session Learnings Template

세션에서 발견된 내용을 문서화:

```markdown
## Session Learnings

### What We Built
- [Feature/Fix description]

### Patterns Established
- [Pattern 1]
- [Pattern 2]

### Issues Resolved
| Issue | Solution |
|-------|----------|
| [Issue] | [How it was fixed] |

### Commands Discovered
- `command` - What it does
```

## Examples

### Basic Usage
```
/generate-claudemd
```

### Custom Output Path
```
/generate-claudemd --output docs/CLAUDE.md
```

### Append to Existing
```
/generate-claudemd --append
```

### Minimal Version
```
/generate-claudemd --minimal
```

### Full Analysis
```
/generate-claudemd --full
```

## Integration with Agent

이 스킬은 `claudemd-generator` 에이전트와 함께 사용할 수 있습니다:

```
Use the claudemd-generator agent to create a comprehensive CLAUDE.md
based on our session, then run /generate-claudemd --full
```

## Auto-Detection Features

자동으로 감지하는 항목:
- 프로젝트 언어/프레임워크
- 패키지 매니저 (npm, yarn, pnpm, pip, cargo, go)
- 테스트 프레임워크
- 린터/포매터 설정
- CI/CD 설정
- Docker 설정
- 환경 변수 구조

## Post-Generation

생성 후 권장 작업:
1. 내용 검토 및 수정
2. 프로젝트 특화 정보 추가
3. 팀 컨벤션 반영
4. `.claude/` 폴더에 복사 (필요시)
