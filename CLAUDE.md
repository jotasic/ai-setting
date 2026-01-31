# AI Settings Project

Claude Code 커스텀 에이전트와 스킬을 관리하는 프로젝트입니다.

## Project Overview

- **Purpose**: Claude Code의 커스텀 에이전트와 스킬(커맨드) 정의
- **Format**: Markdown + YAML frontmatter
- **Structure**: `.claude/` 폴더 기반 구성

## Key Directories

```
.claude/
├── agents/                    # 커스텀 서브에이전트 정의
│   ├── architect.md
│   ├── code-reviewer.md
│   ├── debugger.md
│   ├── doc-writer.md
│   ├── refactorer.md
│   ├── security-auditor.md
│   └── test-writer.md
└── skills/                    # 커스텀 스킬 (슬래시 커맨드) 정의
    ├── build/
    ├── commit/
    ├── explain-code/
    ├── fix-issue/
    ├── lint/
    ├── new-feature/
    ├── review-pr/
    ├── run-tests/
    └── search-code/
```

## Available Agents

| Agent | Description | Use Case |
|-------|-------------|----------|
| `architect` | 시스템 설계와 아키텍처 전문가 | 새 기능 설계, 구조적 결정 |
| `code-reviewer` | 코드 품질, 보안, 베스트 프랙티스 리뷰 | 코드 변경 후 리뷰 |
| `debugger` | 에러 및 테스트 실패 디버깅 전문가 | 버그 수정, 에러 분석 |
| `doc-writer` | 문서화 전문가 | README, API 문서 작성 |
| `refactorer` | 코드 리팩토링 전문가 | 코드 품질 개선 |
| `security-auditor` | 보안 취약점 검사 전문가 | 보안 검토, 취약점 분석 |
| `test-writer` | 테스트 코드 작성 전문가 | 테스트 추가, 커버리지 개선 |

## Available Skills (Commands)

### Development Workflow

| Skill | Usage | Description |
|-------|-------|-------------|
| `build` | `/build [target]` | 프로젝트 빌드 |
| `run-tests` | `/run-tests [path]` | 테스트 실행 |
| `lint` | `/lint [--fix]` | 린트 및 포맷팅 |
| `commit` | `/commit [message]` | Conventional Commits로 커밋 |

### Feature Development

| Skill | Usage | Description |
|-------|-------|-------------|
| `new-feature` | `/new-feature [description]` | 새 기능 구현 |
| `fix-issue` | `/fix-issue [issue-number]` | GitHub 이슈 수정 |
| `review-pr` | `/review-pr` | Pull Request 리뷰 |

### Code Understanding

| Skill | Usage | Description |
|-------|-------|-------------|
| `explain-code` | `/explain-code [file]` | 코드 설명 및 다이어그램 |
| `search-code` | `/search-code [query]` | 코드베이스 검색 |

## Usage Examples

### Using Agents

```
Use the architect agent to design the authentication system
Have the test-writer agent add tests for the user service
Ask the security-auditor agent to review this PR
```

### Using Skills

```
/build production
/run-tests src/services
/lint --fix
/commit feat(auth): add login endpoint
/new-feature user profile page
/explain-code src/utils/parser.ts
```

## Guidelines

- 에이전트는 명확한 description을 포함해야 함
- 스킬은 user-invocable 여부를 명시해야 함
- 민감한 작업은 `disable-model-invocation: true` 설정 권장
- 에이전트/스킬 추가 시 이 문서도 함께 업데이트
