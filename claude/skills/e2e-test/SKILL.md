---
name: e2e-test
description: E2E 테스트 실행 및 브라우저 상태 확인
argument-hint: <url-or-test-description>
allowed-tools: Task, Bash, Read, Write
model: sonnet
category: testing
---

# E2E Test

## ⚡ 즉시 실행

**아래 에이전트를 즉시 호출하세요:**

```
Use the e2e-tester agent to test: $ARGUMENTS
```

## 에이전트가 수행하는 작업

1. **브라우저 실행** - Playwright MCP로 브라우저 제어
2. **페이지 탐색** - URL 이동 및 상호작용
3. **스크린샷 캡처** - 시각적 상태 확인
4. **결과 분석** - 에러, 콘솔 로그, 네트워크 상태 확인
5. **버그 수정** - 발견된 문제 자동 수정 시도

## Playwright MCP 도구

| 도구 | 설명 |
|------|------|
| `playwright_navigate` | URL로 이동 |
| `playwright_screenshot` | 스크린샷 캡처 |
| `playwright_click` | 요소 클릭 |
| `playwright_fill` | 입력 필드 작성 |
| `playwright_evaluate` | JavaScript 실행 |
| `playwright_get_visible_text` | 화면 텍스트 추출 |

## 사용 예시

```bash
/e2e-test http://localhost:3000 로그인 기능 테스트
/e2e-test 회원가입 폼 유효성 검사 확인
/e2e-test 장바구니 추가/삭제 플로우
```

## Related Skills

- `/run-tests`: 유닛/통합 테스트
- `/fix-issue`: 버그 수정
- `/code-quality`: 전체 품질 검사
