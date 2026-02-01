---
name: dependency-audit
description: 의존성 보안 및 호환성 검사
argument-hint: [--fix] [--report]
allowed-tools: Bash, Read, Grep
model: haiku
category: infrastructure
---

# Dependency Audit

프로젝트 의존성의 보안 취약점과 업데이트 상태를 검사합니다.

## Triggers (사용 조건)

- "보안 검사해줘", "audit dependencies"
- "취약점 확인", "outdated packages"
- 배포 전 보안 점검시

## Arguments

- `--fix`: 자동 수정 시도
- `--report`: 상세 리포트 생성

## Workflow

```
┌─────────────────────────────────────┐
│  1. Security audit (npm/pip audit)  │
│  2. Check outdated packages         │
│  3. License check                   │
│  4. Bundle impact analysis          │
└─────────────────────────────────────┘
```

## Audit Commands

| Tool | Command |
|------|---------|
| npm | `npm audit` |
| yarn | `yarn audit` |
| pip | `pip-audit` |
| cargo | `cargo audit` |

## Agent Integration

**취약점 수정:**
```
Use the security-auditor agent to review and fix vulnerability in [package]
```

**의존성 업데이트:**
```
Use the dependency-manager agent to safely update outdated packages
```

## Output Format

```
Dependency Audit Report
═══════════════════════════════════════
Security:
  Critical: 0
  High: 1
  Medium: 3
  Low: 5

Vulnerabilities:
  - lodash@4.17.20 (High) → Update to 4.17.21

Outdated:
  - react 17.0.2 → 18.2.0 (Major)
  - axios 0.27.0 → 1.4.0 (Major)

Licenses: ✓ All compatible (MIT, Apache-2.0)

Recommendations:
  1. [Critical] Fix high severity issues
  2. [High] Update major versions
═══════════════════════════════════════
```

## Examples

```bash
/dependency-audit            # 검사만
/dependency-audit --fix      # 자동 수정
/dependency-audit --report   # 상세 리포트
```

## Related Skills

- `/setup-env`: 환경 설정
- `/code-quality`: 품질 검사
