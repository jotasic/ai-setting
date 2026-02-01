---
name: generate-claudemd
description: 세션 대화 기록과 프로젝트 분석을 통해 CLAUDE.md 자동 생성
argument-hint: [--full] [--minimal] [--append]
allowed-tools: Bash, Read, Write, Grep, Glob
model: sonnet
category: documentation
---

# Generate CLAUDE.md

현재 세션의 대화 기록과 프로젝트 구조를 분석하여 CLAUDE.md를 생성합니다.

## Triggers (사용 조건)

- "CLAUDE.md 만들어줘", "generate claudemd"
- "프로젝트 문서화", "setup claude config"
- 새 프로젝트 설정시

## Arguments

- `--full`: 전체 분석 포함
- `--minimal`: 최소 버전
- `--append`: 기존 파일에 추가
- `--output <path>`: 출력 경로

## Workflow

```
┌─────────────────────────────────────┐
│  1. Analyze project structure       │
│  2. Detect configurations           │
│  3. Extract session learnings       │
│  4. Generate documentation          │
└─────────────────────────────────────┘
```

## Output Modes

| Mode | Content |
|------|---------|
| `--minimal` | Quick Start, Commands, Tips |
| default | + Structure, Conventions |
| `--full` | + Architecture, API, Deployment |

## Agent Integration

**상세 분석:**
```
Use the claudemd-generator agent to create comprehensive documentation
```

**아키텍처 문서:**
```
Use the architect agent to document system architecture
```

## Auto-Detection

- 프로젝트 언어/프레임워크
- 패키지 매니저
- 테스트 프레임워크
- 린터/포매터
- CI/CD 설정

## Output Format

```
CLAUDE.md Generated
═══════════════════════════════════════
Project: [name]
Type: [framework]

Sections:
  ✓ Quick Start
  ✓ Project Structure
  ✓ Development Commands
  ✓ Testing
  ✓ Tips for Claude

Output: ./CLAUDE.md
═══════════════════════════════════════
```

## Examples

```bash
/generate-claudemd                 # 기본
/generate-claudemd --full          # 전체 분석
/generate-claudemd --minimal       # 최소 버전
/generate-claudemd --append        # 기존에 추가
```

## Related Skills

- `/architecture-review`: 구조 분석
- `/setup-env`: 환경 설정
