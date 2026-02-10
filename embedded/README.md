# Embedded Development - Claude Code Configuration

STM32F103VCT6 레거시 펌웨어 리팩토링을 위한 Claude Code 에이전트/스킬/MCP 설정입니다.

## Target Environment

| Item | Value |
|------|-------|
| MCU | STM32F103VCT6 (Cortex-M3, 72MHz) |
| Flash | 256 KB |
| RAM | 48 KB SRAM |
| Compiler | IAR Embedded Workbench for ARM (EWARM) |
| Library | STM32 Standard Peripheral Library (SPL) |
| Language | C99 |

---

## Quick Start

```bash
# 프로젝트에 임베디드 설정 복사
cp -r claude/embedded /your/firmware-project/.claude

# 또는 심볼릭 링크
ln -s /path/to/ai-setting/claude/embedded /your/firmware-project/.claude
```

---

## Agents (8개)

| Model | Agent | Description |
|-------|-------|-------------|
| **opus** | `embedded-architect` | 시스템 아키텍처, 메모리 레이아웃, 모듈 구조 설계 |
| **sonnet** | `firmware-developer` | SPL 기반 C 펌웨어 구현 |
| **sonnet** | `embedded-refactorer` | 레거시 코드 리팩토링 (전역변수 제거, 모듈 분리) |
| **sonnet** | `peripheral-specialist` | 페리페럴 설정 (GPIO/UART/SPI/I2C/TIM/ADC/DMA) |
| **sonnet** | `embedded-debugger` | HardFault 분석, 페리페럴 디버깅, 타이밍 이슈 |
| **sonnet** | `embedded-code-reviewer` | MISRA-C 리뷰, ISR 안전성, volatile 검사 |
| **sonnet** | `embedded-test-writer` | Unity/CMock 단위 테스트 작성 |
| **sonnet** | `memory-optimizer` | Flash/RAM 최적화, .map 파일 분석 |

### Usage Examples

```
Use the embedded-architect agent to design the module structure for the motor control subsystem

Use the embedded-refactorer agent to extract UART driver from main.c

Use the peripheral-specialist agent to configure SPI1 with DMA for flash memory communication

Use the embedded-debugger agent to analyze HardFault occurring during SPI transfer

Use the embedded-code-reviewer agent to review the timer module for MISRA-C compliance

Use the memory-optimizer agent to analyze the .map file and find optimization opportunities
```

---

## Skills (10개)

### Development Workflow

| Skill | Usage | Description |
|-------|-------|-------------|
| `/iar-build` | `/iar-build Project.ewp --config Release` | IAR 프로젝트 빌드 |
| `/static-analysis` | `/static-analysis src/ --misra` | MISRA-C 정적 분석 |
| `/memory-map` | `/memory-map Release/List/Project.map` | Flash/RAM 사용량 분석 |
| `/embedded-commit` | `/embedded-commit refactor(uart): extract driver` | 임베디드 Conventional Commits |

### Refactoring

| Skill | Usage | Description |
|-------|-------|-------------|
| `/refactor-legacy` | `/refactor-legacy main.c` | 5단계 레거시 리팩토링 워크플로우 |
| `/module-extract` | `/module-extract main.c bsp_uart` | 모놀리식 파일에서 모듈 분리 |
| `/register-audit` | `/register-audit src/` | 직접 레지스터 접근 → SPL 변환 감사 |
| `/dead-code` | `/dead-code --all` | 미사용 코드/변수/함수 탐지 |

### Analysis

| Skill | Usage | Description |
|-------|-------|-------------|
| `/analyze-isr` | `/analyze-isr --all` | ISR 안전성, 우선순위 분석 |
| `/peripheral-init` | `/peripheral-init USART1 115200` | 페리페럴 초기화 코드 생성 |

---

## Refactoring Workflow

레거시 코드 리팩토링의 전체 흐름:

```
1. 분석     /analyze-isr --all
            /dead-code --all
            /memory-map Project.map
            /register-audit src/

2. 계획     Use the embedded-architect agent to analyze main.c dependencies

3. 실행     /refactor-legacy main.c
            또는
            /module-extract main.c bsp_uart
            /module-extract main.c bsp_spi

4. 검증     /iar-build Project.ewp
            /static-analysis src/ --misra
            /memory-map Release/List/Project.map

5. 커밋     /embedded-commit refactor(uart): extract UART driver from main.c
```

---

## Directory Structure

```
claude/embedded/
├── agents/
│   ├── embedded-architect.md
│   ├── firmware-developer.md
│   ├── embedded-refactorer.md
│   ├── peripheral-specialist.md
│   ├── embedded-debugger.md
│   ├── embedded-code-reviewer.md
│   ├── embedded-test-writer.md
│   ├── memory-optimizer.md
│   └── resources/
│       ├── stm32f103-reference.md      # MCU 스펙 요약
│       ├── spl-api-patterns.md         # SPL 패턴 & 안티패턴
│       ├── iar-conventions.md          # IAR 확장 문법 레퍼런스
│       ├── misra-c-checklist.md        # MISRA-C:2012 체크리스트
│       ├── refactoring-patterns.md     # 리팩토링 패턴 카탈로그
│       └── isr-safety-guide.md         # ISR 안전성 가이드
├── skills/
│   ├── iar-build/SKILL.md
│   ├── static-analysis/SKILL.md
│   ├── memory-map/SKILL.md
│   ├── refactor-legacy/
│   │   ├── SKILL.md
│   │   └── resources/
│   │       ├── refactor-protocol.md
│   │       └── safety-checklist.md
│   ├── peripheral-init/SKILL.md
│   ├── analyze-isr/SKILL.md
│   ├── dead-code/SKILL.md
│   ├── module-extract/SKILL.md
│   ├── register-audit/SKILL.md
│   ├── embedded-commit/SKILL.md
│   └── _shared/
│       ├── stm32f103-constraints.md    # MCU 제약 조건
│       ├── iar-build-flags.md          # IAR 컴파일러 옵션
│       └── spl-to-module-map.md        # SPL 함수 → 모듈 매핑
├── settings.json                       # 권한 설정
├── mcp.json                            # MCP 서버 설정
├── hooks.json                          # 자동화 훅
└── README.md                           # 이 파일
```

---

## Configuration

### settings.json
- IAR 빌드 도구 (`iarbuild`), ARM 툴체인 허용
- 디버거/프로그래머 도구 (`openocd`, `st-flash`, `JLinkExe`) 허용
- 정적 분석 도구 (`cppcheck`, `pc-lint`, `splint`) 허용
- 위험 명령어 (전체 Flash erase, force push 등) 차단

### mcp.json
- `filesystem`: 프로젝트 소스/빌드 파일 접근
- `github`: 이슈, PR 관리
- `git`: 변경 이력 추적
- `memory`: 페리페럴 할당, 핀맵, 의존성 기록
- `sequential-thinking`: 복잡한 리팩토링 계획
- `fetch`: ST 데이터시트 참조

### hooks.json
- **PreToolUse**: 위험 명령어 차단, .icf 수정 경고, ISR 코드 수정 경고
- **PostToolUse**: volatile 누락 힌트, 커밋 후 확인
