# Claude Code Reference Project

Claude Code를 효과적으로 사용하기 위한 참고 프로젝트입니다.

## Quick Start

이 프로젝트의 `.claude` 폴더를 프로젝트에 복사하여 사용하세요:

```bash
cp -r .claude /your/project/
```

## Project Structure

```
.
├── .claude/
│   ├── agents/              # 커스텀 서브에이전트
│   ├── skills/              # 커스텀 스킬 (슬래시 커맨드)
│   ├── settings.json        # 권한 및 환경 설정
│   ├── settings.local.json  # 로컬 설정 (gitignore)
│   ├── mcp.json             # MCP 서버 설정
│   └── hooks.json           # 훅 설정
├── examples/
│   ├── prompts.md           # 효과적인 프롬프트 예제
│   ├── workflows.md         # 개발 워크플로우 가이드
│   └── agent-selection-guide.md
├── CLAUDE.md                # 이 파일
└── .gitignore
```

---

## Agents (서브에이전트)

### 모델별 에이전트 분류

| Model | Agent | Description |
|-------|-------|-------------|
| **opus** | `architect` | 시스템 설계, 아키텍처 결정 |
| **opus** | `security-auditor` | 보안 취약점 검사, OWASP 체크 |
| **sonnet** | `code-reviewer` | 코드 품질, 베스트 프랙티스 리뷰 |
| **sonnet** | `debugger` | 에러 분석, 버그 수정 |
| **sonnet** | `test-writer` | 테스트 코드 작성, 커버리지 개선 |
| **sonnet** | `refactorer` | 코드 리팩토링, 구조 개선 |
| **haiku** | `doc-writer` | 문서화, README, API 문서 |

### 사용 예시

```
Use the architect agent to design the notification system
Have the security-auditor agent review the auth module
Ask the debugger agent to fix the failing tests
```

---

## Skills (슬래시 커맨드)

### Development Workflow

| Skill | Usage | Description |
|-------|-------|-------------|
| `/build` | `/build [target]` | 프로젝트 빌드 |
| `/run-tests` | `/run-tests [path]` | 테스트 실행 |
| `/lint` | `/lint [--fix]` | 린트 및 포맷팅 |
| `/commit` | `/commit [message]` | Conventional Commits |
| `/code-quality` | `/code-quality` | 전체 품질 파이프라인 |
| `/git-workflow` | `/git-workflow start\|finish\|sync` | Git 브랜치 워크플로우 |

### Feature Development

| Skill | Usage | Description |
|-------|-------|-------------|
| `/new-feature` | `/new-feature [desc]` | 새 기능 구현 |
| `/fix-issue` | `/fix-issue [number]` | GitHub 이슈 수정 |
| `/review-pr` | `/review-pr` | PR 리뷰 |

### Code Understanding

| Skill | Usage | Description |
|-------|-------|-------------|
| `/explain-code` | `/explain-code [file]` | 코드 설명 |
| `/search-code` | `/search-code [query]` | 코드 검색 |
| `/mcp-demo` | `/mcp-demo [server]` | MCP 사용법 가이드 |

---

## Configuration Files

### settings.json - 권한 설정

```json
{
  "permissions": {
    "allow": ["Bash(npm run *)", "Read", "Glob"],
    "deny": ["Bash(rm -rf /)"]
  }
}
```

### mcp.json - MCP 서버 설정

지원되는 MCP 서버:
- `filesystem`: 파일시스템 접근
- `github`: GitHub API
- `postgres`: PostgreSQL 쿼리
- `memory`: 지식 그래프 메모리
- `puppeteer`: 브라우저 자동화
- `slack`: Slack 연동
- `fetch`: HTTP 요청
- `brave-search`: 웹 검색
- `sequential-thinking`: 복잡한 추론

### hooks.json - 훅 설정

이벤트 기반 자동화:
- `PreToolUse`: 도구 사용 전 검증
- `PostToolUse`: 도구 사용 후 처리
- `Notification`: 알림
- `Stop`: 세션 종료 시 정리

---

## Model Selection Guide

### When to use Opus
- 아키텍처 설계
- 보안 분석
- 복잡한 트레이드오프 결정
- 장기적 영향 평가

### When to use Sonnet
- 일반 코딩 작업
- 코드 리뷰
- 테스트 작성
- 버그 수정
- 리팩토링

### When to use Haiku
- 문서화
- 간단한 설명
- 빠른 응답 필요
- 대량 처리

---

## Common Workflows

### 1. Feature Development
```
architect → /new-feature → test-writer → code-reviewer → doc-writer → /commit
```

### 2. Bug Fix
```
debugger → fix → /run-tests → /commit
```

### 3. Code Review
```
security-auditor → code-reviewer → /run-tests
```

### 4. Refactoring
```
architect (plan) → /run-tests (baseline) → refactorer → /run-tests (verify)
```

---

## Examples

자세한 예제는 `examples/` 폴더를 참고하세요:

- **prompts.md**: 효과적인 프롬프트 작성법
- **workflows.md**: 개발 워크플로우 상세 가이드
- **agent-selection-guide.md**: 에이전트 선택 가이드

---

## Customization

### 새 에이전트 추가

`.claude/agents/my-agent.md`:
```markdown
---
name: my-agent
description: 에이전트 설명
tools: Read, Edit, Write, Bash
model: sonnet
---

에이전트 프롬프트 내용...
```

### 새 스킬 추가

`.claude/skills/my-skill/SKILL.md`:
```markdown
---
name: my-skill
description: 스킬 설명
argument-hint: [arguments]
---

스킬 프롬프트 내용...
$ARGUMENTS
```

---

## Tips

1. **컨텍스트 활용**: 관련 파일과 제약 조건을 명시
2. **에이전트 조합**: 복잡한 작업은 여러 에이전트 순차 사용
3. **점진적 진행**: 큰 작업은 작은 단계로 분할
4. **검증 반복**: 각 단계마다 테스트로 확인
5. **모델 최적화**: 작업 복잡도에 맞는 모델 선택

---

## Resources

- [Claude Code Documentation](https://docs.anthropic.com/claude-code)
- [MCP Specification](https://modelcontextprotocol.io)
- [Anthropic API Reference](https://docs.anthropic.com/api)
