# AI Coding Assistant Reference Project

다양한 AI 코딩 어시스턴트를 효과적으로 사용하기 위한 참고 프로젝트입니다.

## Quick Start

이 프로젝트의 설정 폴더를 프로젝트에 복사하여 사용하세요:

```bash
# Claude Code
cp -r claude /your/project/.claude

# 또는 심볼릭 링크
ln -s /path/to/ai-setting/claude /your/project/.claude
```

## Project Structure

```
.
├── claude/                 # Claude Code 설정
│   ├── agents/            # 17개의 커스텀 서브에이전트
│   ├── skills/            # 24개의 커스텀 스킬
│   ├── settings.json      # 권한 및 환경 설정
│   ├── mcp.json           # MCP 서버 설정
│   └── hooks.json         # 훅 설정
├── gemini/                 # Google Gemini 설정 (예정)
├── chatgpt/                # OpenAI ChatGPT 설정 (예정)
├── cursor/                 # Cursor IDE 설정 (예정)
├── copilot/                # GitHub Copilot 설정 (예정)
├── windsurf/               # Windsurf 설정 (예정)
├── examples/               # 프롬프트 및 워크플로우 예제
│   ├── prompts.md         # 효과적인 프롬프트 예제
│   ├── workflows.md       # 12개의 개발 워크플로우
│   ├── agent-selection-guide.md
│   ├── language-specific-guide.md
│   ├── framework-specific-guide.md
│   ├── advanced-prompts.md
│   ├── testing-strategy.md
│   ├── security-hardening.md
│   └── common-mistakes.md
└── CLAUDE.md              # 이 파일
```

---

## Agents (서브에이전트) - 17개

### 모델별 에이전트 분류

| Model | Agent | Description |
|-------|-------|-------------|
| **opus** | `architect` | 시스템 설계, 아키텍처 결정 |
| **opus** | `security-auditor` | 보안 취약점 검사, OWASP 체크 |
| **sonnet** | `code-reviewer` | 코드 품질, 베스트 프랙티스 리뷰 |
| **sonnet** | `debugger` | 에러 분석, 버그 수정 |
| **sonnet** | `test-writer` | 테스트 코드 작성, 커버리지 개선 |
| **sonnet** | `frontend-developer` | 프론트엔드 구현 (UI, 상태관리, 스타일링) |
| **sonnet** | `backend-developer` | 백엔드 구현 (API, 비즈니스 로직, DB 연동) |
| **sonnet** | `general-developer` | 범용 개발 (스크립트, CLI, 봇, 유틸리티) |
| **sonnet** | `refactorer` | 코드 리팩토링, 구조 개선 |
| **sonnet** | `performance-optimizer` | 성능 분석, 최적화 제안 |
| **sonnet** | `devops-specialist` | Docker, K8s, CI/CD |
| **sonnet** | `api-designer` | REST/GraphQL API 설계 |
| **sonnet** | `database-specialist` | 스키마 설계, 쿼리 최적화 |
| **sonnet** | `claudemd-generator` | 세션 기반 CLAUDE.md 자동 생성 |
| **sonnet** | `spec-writer` | 기획서(PRD) 작성 (기술 구현은 architect가 담당) |
| **haiku** | `doc-writer` | 문서화, README, API 문서 |
| **haiku** | `dependency-manager` | 의존성 관리, 보안 업데이트 |

### 사용 예시

```
Use the architect agent to design the notification system
Have the frontend-developer agent implement the UI components
Have the backend-developer agent implement the API endpoints
Have the general-developer agent create a data migration script
Have the security-auditor agent review the auth module
Ask the devops-specialist to set up CI/CD pipeline
```

---

## Skills (슬래시 커맨드) - 24개

### Development Workflow

| Skill | Usage | Description |
|-------|-------|-------------|
| `/build` | `/build [target]` | 프로젝트 빌드 |
| `/run-tests` | `/run-tests [path]` | 테스트 실행 |
| `/lint` | `/lint [--fix]` | 린트 및 포맷팅 |
| `/commit` | `/commit [message]` | Conventional Commits |
| `/code-quality` | `/code-quality` | 전체 품질 파이프라인 |
| `/git-workflow` | `/git-workflow start\|finish\|sync` | Git 브랜치 워크플로우 |
| `/setup-env` | `/setup-env [--clean]` | 개발 환경 설정 |

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
| `/architecture-review` | `/architecture-review [area]` | 아키텍처 분석 |
| `/estimate-effort` | `/estimate-effort [desc]` | 작업 규모 추정 |

### DevOps & Infrastructure

| Skill | Usage | Description |
|-------|-------|-------------|
| `/performance-profile` | `/performance-profile [target]` | 성능 프로파일링 |
| `/dependency-audit` | `/dependency-audit [--fix]` | 의존성 보안 검사 |
| `/db-migrate` | `/db-migrate create\|run\|rollback` | DB 마이그레이션 |
| `/changelog` | `/changelog [version]` | CHANGELOG 생성 |

### Documentation

| Skill | Usage | Description |
|-------|-------|-------------|
| `/api-docs-generate` | `/api-docs-generate [path]` | OpenAPI 문서 생성 |
| `/type-check-improve` | `/type-check-improve [path]` | 타입 커버리지 개선 |
| `/create-testdata` | `/create-testdata <model> [count]` | 테스트 데이터 생성 |
| `/generate-claudemd` | `/generate-claudemd [--full]` | 세션 기반 CLAUDE.md 생성 |
| `/write-spec` | `/write-spec <feature>` | 기획서(PRD) 생성, architect가 기술 설계 |

---

## Configuration Files

### settings.json - 권한 및 출력 설정

```json
{
  "preferences": {
    "verbosity": "minimal",
    "codeOnly": true,
    "maxExplanationLines": 3
  },
  "permissions": {
    "allow": [
      "Bash(npm run *)",
      "Bash(docker ps*)",
      "Bash(kubectl get*)",
      "Read", "Glob", "Grep"
    ],
    "deny": [
      "Bash(rm -rf /)",
      "Bash(git push --force*)"
    ]
  }
}
```

#### preferences 옵션

| 옵션 | 값 | 설명 |
|------|-----|------|
| `verbosity` | `"minimal"` \| `"normal"` \| `"detailed"` | 출력 상세도 |
| `codeOnly` | `true` \| `false` | 코드만 출력, 설명 생략 |
| `maxExplanationLines` | `number` | 설명 최대 줄 수 |

**maxExplanationLines 권장값:**
- `3`: 토큰 절약 최적화 (현재 설정)
- `5`: 간결하면서 충분한 설명
- `10`: 일반적인 사용
- `0`: 설명 생략 (codeOnly: true와 동일 효과)

### mcp.json - MCP 서버 설정 (16개)

지원되는 MCP 서버:
- `filesystem`: 파일시스템 접근
- `github`: GitHub API
- `git`: Git 히스토리
- `postgres`, `sqlite`: 데이터베이스
- `memory`: 지식 그래프 메모리
- `puppeteer`: 브라우저 자동화
- `docker`: 컨테이너 관리
- `slack`, `notion`, `linear`: 협업 도구
- `sentry`: 에러 모니터링
- `fetch`, `brave-search`: 웹 요청
- `sequential-thinking`: 복잡한 추론
- `time`: 시간 정보

### hooks.json - 훅 설정

이벤트 기반 자동화:
- `PreToolUse`: 도구 사용 전 검증 (위험 명령어 차단)
- `PostToolUse`: 도구 사용 후 처리 (자동 린트, 타입 체크)
- `Notification`: 알림
- `Stop`: 세션 종료 시 정리

---

## Model Selection Guide

### When to use Opus
- 아키텍처 설계
- 보안 분석
- 복잡한 트레이드오프 결정

### When to use Sonnet
- 일반 코딩 작업
- 코드 리뷰
- 테스트 작성
- 버그 수정

### When to use Haiku
- 문서화
- 간단한 설명
- 빠른 응답 필요

---

## Workflows (12개)

1. **Feature Development**: 설계 → 구현 → 테스트 → 리뷰 → 문서화
2. **Bug Fix**: 재현 → 분석 → 수정 → 검증
3. **Code Review**: 보안 → 품질 → 테스트 커버리지
4. **Refactoring**: 식별 → 계획 → 테스트 → 리팩토링 → 검증
5. **Security Audit**: 의존성 → 코드 스캔 → OWASP 체크
6. **Documentation**: 대상 파악 → 생성 → 예제 → 검토
7. **Onboarding**: 탐색 → 이해 → 설정 → 첫 기여
8. **Performance Optimization**: 측정 → 분석 → 최적화 → 검증
9. **Dependency Update**: 감사 → 계획 → 업데이트 → 테스트
10. **API Design**: 설계 → 스펙 → 구현 → 문서
11. **Database Migration**: 설계 → 계획 → 생성 → 테스트 → 실행
12. **Release Management**: 동결 → 테스트 → 변경로그 → 태그 → 배포

---

## Examples

자세한 예제는 `examples/` 폴더를 참고하세요:

- **prompts.md**: 효과적인 프롬프트 작성법
- **workflows.md**: 12개 개발 워크플로우 상세 가이드
- **agent-selection-guide.md**: 에이전트 선택 가이드
- **language-specific-guide.md**: 언어별 가이드 (TS, Python, Go, Rust)
- **framework-specific-guide.md**: 프레임워크별 가이드 (React, Vue, Express, FastAPI)
- **advanced-prompts.md**: 복잡한 작업을 위한 고급 프롬프트
- **testing-strategy.md**: 테스트 전략 수립 가이드
- **security-hardening.md**: 보안 강화 체크리스트
- **common-mistakes.md**: 흔한 실수와 해결법

---

## Customization

### 새 에이전트 추가

`claude/agents/my-agent.md`:
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

`claude/skills/my-skill/SKILL.md`:
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

## Output Rules

settings.json의 preferences를 따르세요:

- **verbosity: minimal** - 간결하게 출력
- **codeOnly: true** - 코드 위주, 불필요한 설명 생략
- **maxExplanationLines: 3** - 설명은 최대 3줄

```
# 좋은 예
Fix applied.

# 나쁜 예
이 버그는 상태 관리 문제로 인해 발생했습니다.
React의 useState 훅이 비동기적으로 동작하기 때문에...
(장황한 설명 계속)
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
