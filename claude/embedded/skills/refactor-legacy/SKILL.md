---
name: refactor-legacy
description: Legacy embedded code refactoring workflow (5-phase)
argument-hint: <target-file-or-module>
allowed-tools: Bash, Read, Edit, Write, Grep, Glob, Task
model: sonnet
category: embedded-feature
---

# Refactor Legacy Code

대규모 레거시 임베디드 C 코드를 안전하게 리팩토링합니다.

## Arguments

- `$ARGUMENTS`: 리팩토링 대상 파일 또는 모듈명

## Workflow (5 Phases)

```
Phase 1: ANALYZE ── 코드 분석, 의존성 맵핑
    │
Phase 2: PLAN ──── 리팩토링 계획, 위험도 평가
    │
Phase 3: EXTRACT ── 모듈 분리, 인터페이스 정의
    │
Phase 4: VERIFY ─── 빌드 확인, MISRA 검사
    │
Phase 5: DOCUMENT ─ 변경 요약, 의존성 다이어그램
```

상세 프로토콜은 리소스를 참조하세요.

## Agent Assignment

| Phase | Agent | Role |
|-------|-------|------|
| ANALYZE | `embedded-architect` | 의존성 그래프, 전역변수 맵 |
| PLAN | `embedded-refactorer` | 리팩토링 계획, 위험도 평가 |
| EXTRACT | `firmware-developer` + `peripheral-specialist` | 코드 분리 |
| VERIFY | `embedded-code-reviewer` + `memory-optimizer` | 검증 |
| DOCUMENT | (inline) | 변경 요약 생성 |

## Safety Rules

1. 한 번에 하나의 모듈만 리팩토링
2. 매 변경 후 빌드 확인
3. ISR 타이밍 변경 금지
4. volatile 제거 금지
5. .icf 링커 설정 변경 시 architect 확인 필요

## References
- [Refactor Protocol](resources/refactor-protocol.md)
- [Safety Checklist](resources/safety-checklist.md)

## Related Skills

- `/static-analysis`: 리팩토링 전후 정적 분석
- `/memory-map`: 메모리 변화 확인
- `/dead-code`: 미사용 코드 제거
