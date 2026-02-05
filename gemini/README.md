# Gemini Code 설정 가이드

이 디렉토리는 Google Gemini Code 에이전트의 잠재력을 100% 활용하기 위한 **고급 설정 모음(Configuration Suite)**입니다.
단순한 코드 생성을 넘어, 기획부터 설계, 구현, 테스트, 배포까지 전 과정을 자동화하고 체계화하는 것을 목표로 합니다.

## 🚀 시작하기 (Setup)

이 설정을 적용하려면 다음 두 가지 방법 중 하나를 선택하세요:

### 1. 글로벌 설정 (권장)
모든 프로젝트에 공통으로 적용됩니다.
```bash
# 설정 디렉토리로 이동 (없으면 생성)
mkdir -p ~/.gemini

# 파일 복사
cp -r ai-setting/gemini/* ~/.gemini/
```

### 2. 프로젝트별 설정
특정 프로젝트에만 적용됩니다.
```bash
# 프로젝트 루트에서
mkdir -p .gemini
cp -r /path/to/ai-setting/gemini/* .gemini/
```

---

## 🤖 에이전트 (Agents)

전문적인 역할을 수행하는 페르소나들입니다. `@agentname`으로 호출하여 사용합니다.

### 핵심 에이전트
- **`@general-developer`**: 범용 개발자. 간단한 스크립트 작성이나 일반적인 코딩 업무를 수행합니다.
- **`@architect`**: 소프트웨어 아키텍트. 시스템 구조를 설계하고 기술 스택을 결정합니다.
- **`@spec-writer`**: 기획자(PM). 모호한 요구사항을 구체적인 기획서(PRD)로 변환합니다.
- **`@code-reviewer`**: 리뷰어. 코드 품질, 보안, 성능 관점에서 코드를 리뷰합니다.

### 전문 에이전트
- **Backend/Frontend**: `backend-developer`, `frontend-developer`
- **Quality & Security**: `test-writer`, `security-auditor`, `perfomance-optimizer`
- **Ops**: `devops-specialist`, `database-specialist`
- **Docs**: `doc-writer`

---

## 🛠️ 스킬 (Skills)

에이전트가 수행할 수 있는 구체적인 작업 절차(SOP)입니다. 자연어로 요청하면 에이전트가 적절한 스킬을 찾아 실행합니다.

### 기획 & 설계
- **`write-spec`**: "이 기능 기획해줘" → 5W1H 질문을 통해 PRD 작성
- **`new-feature`**: "로그인 기능 만들어줘" → 요구사항 분석 > 설계 > 구현 > 테스트 절차 수행
- **`estimate-effort`**: 작업의 규모와 영향 범위를 분석

### 개발 & 품질
- **`git-workflow`**: 브랜치 생성, 커밋, PR 생성 자동화
- **`code-quality`**: 린트, 테스트, 타입 체크, 보안 검사를 한 번에 실행
- **`lint` / `run-tests`**: 린터 및 테스트 실행기 자동 감지 및 실행
- **`type-check-improve`**: 타입 커버리지 분석 및 개선

### 유지보수
- **`dependency-audit`**: 의존성 취약점 분석 및 업데이트
- **`explain-code`**: 복잡한 코드를 다이어그램과 비유로 설명
- **`generate-geminimd`**: 현재 프로젝트 분석 문서를 생성 (`GEMINI.md`)

---

## 🔄 워크플로우 (Workflows)

자주 반복되는 작업 흐름을 슬래시 커맨드(`/`)로 정의했습니다. `.agent/workflows/`에 위치합니다.

- **`/boot`**: 새 프로젝트 초기화 (Git init, .gitignore, README, 기본 구조 생성)
- **`/review`**: 변경된 코드에 대한 심층 리뷰 수행
- **`/docs`**: 코드베이스 변경 사항을 문서에 반영

---

## 🔌 MCP 서버 (Mcp.json)

Gemini가 외부 시스템과 소통하기 위한 도구들입니다.
- **Filesystem**: 로컬 파일 읽기/쓰기
- **GitHub**: 이슈 및 PR 관리
- **Postgres**: 데이터베이스 쿼리 및 관리
- **Memory**: 장기 기억 저장소
- **Puppeteer**: 웹 브라우징 및 테스트

## 팁 (Tips)

1. **기획부터 시작하세요**: 바로 코딩하기보다 `@spec-writer`에게 "이런 거 만들고 싶어"라고 말해보세요.
2. **리뷰를 습관화하세요**: 구현이 끝나면 반드시 `/review` 명령어나 `@code-reviewer`를 통해 피드백을 받으세요.
3. **설명서를 만드세요**: 프로젝트에 들어오면 가장 먼저 `/generate-geminimd`를 실행하여 Gemini가 프로젝트를 이해하도록 도우세요.
