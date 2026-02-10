---
name: embedded-commit
description: Commit with embedded-specific Conventional Commits scope
argument-hint: [message] [--amend]
allowed-tools: Bash, Read, Grep
disable-model-invocation: true
model: haiku
category: embedded-workflow
---

# Embedded Commit

임베디드 프로젝트에 맞는 Conventional Commits 형식으로 커밋합니다.

## Arguments

- `$ARGUMENTS`: 커밋 메시지
- `--amend`: 이전 커밋 수정

## Conventional Commits Format

```
<type>(<scope>): <description>

[optional body]
```

### Types

| Type | Description |
|------|-------------|
| feat | 새로운 기능/페리페럴 드라이버 |
| fix | 버그 수정 |
| refactor | 리팩토링 (동작 변경 없음) |
| perf | 성능/메모리 최적화 |
| test | 테스트 추가/수정 |
| docs | 문서 변경 |
| chore | 빌드 설정, IAR 프로젝트 설정 |
| style | 코드 스타일 (포맷팅) |

### Embedded Scopes

| Scope | Description |
|-------|-------------|
| `uart` | USART 관련 |
| `spi` | SPI 관련 |
| `i2c` | I2C 관련 |
| `gpio` | GPIO 관련 |
| `timer` | Timer 관련 |
| `adc` | ADC 관련 |
| `dma` | DMA 관련 |
| `isr` | 인터럽트 관련 |
| `bsp` | Board Support Package |
| `app` | 어플리케이션 로직 |
| `drv` | 드라이버 레이어 |
| `protocol` | 통신 프로토콜 |
| `config` | 설정/클럭/핀맵 |
| `boot` | 부트로더 |
| `iar` | IAR 프로젝트 설정 |
| `linker` | 링커 설정 (.icf) |

## Examples

```bash
/embedded-commit refactor(uart): extract UART driver from main.c
/embedded-commit fix(isr): add volatile to shared timer flag
/embedded-commit perf(dma): enable DMA for SPI1 transfers
/embedded-commit feat(protocol): add CRC16 validation
/embedded-commit chore(iar): update optimization to size for Release
/embedded-commit refactor(bsp): encapsulate GPIO globals into module
```

## Workflow

```
┌─────────────────────────────────────┐
│  1. git status + git diff           │
│  2. Stage relevant files            │
│  3. Create commit                   │
│  4. Verify commit                   │
└─────────────────────────────────────┘
```

## Guidelines

- 제목 50자 이내
- scope는 반드시 위 목록에서 선택
- 리팩토링은 반드시 `refactor` 타입 사용
- 메모리 최적화는 `perf` 타입 사용

## Related Skills

- `/iar-build`: 커밋 전 빌드 확인
- `/static-analysis`: 커밋 전 정적 분석
