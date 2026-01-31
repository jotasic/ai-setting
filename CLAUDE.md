# AI Settings Project

Claude Code 커스텀 에이전트와 스킬을 관리하는 프로젝트입니다.

## Project Overview

- **Purpose**: Claude Code의 커스텀 에이전트와 스킬(커맨드) 정의
- **Format**: Markdown + YAML frontmatter
- **Structure**: `.claude/` 폴더 기반 구성

## Key Directories

```
.claude/
├── agents/           # 커스텀 서브에이전트 정의
│   ├── code-reviewer.md
│   └── debugger.md
└── skills/           # 커스텀 스킬 (슬래시 커맨드) 정의
    ├── explain-code/
    ├── fix-issue/
    └── review-pr/
```

## Available Agents

| Agent | Description |
|-------|-------------|
| `code-reviewer` | 코드 품질, 보안, 베스트 프랙티스 리뷰 |
| `debugger` | 에러 및 테스트 실패 디버깅 전문가 |

## Available Skills (Commands)

| Skill | Usage | Description |
|-------|-------|-------------|
| `explain-code` | `/explain-code [file]` | 코드 설명 및 다이어그램 |
| `fix-issue` | `/fix-issue [issue-number]` | GitHub 이슈 분석 및 수정 |
| `review-pr` | `/review-pr` | Pull Request 리뷰 |

## Guidelines

- 에이전트는 명확한 description을 포함해야 함
- 스킬은 user-invocable 여부를 명시해야 함
- 민감한 작업은 `disable-model-invocation: true` 설정 권장
