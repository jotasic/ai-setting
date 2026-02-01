---
name: review-pr
description: GitHub Pull Request를 분석하고 리뷰합니다
argument-hint: [pr-number]
disable-model-invocation: true
allowed-tools: Bash, Read, Grep, Glob
model: sonnet
category: development
context: fork
agent: Explore
---

# Pull Request Review

GitHub PR을 분석하고 코드 리뷰를 수행합니다.

## Triggers (사용 조건)

- "PR 리뷰해줘", "review PR"
- "코드 리뷰", "PR 분석"
- PR 머지 전 검토 필요시

## Arguments

- `$ARGUMENTS`: PR 번호 (없으면 현재 브랜치 PR)

## Workflow

```
┌─────────────────────────────────────┐
│  1. Fetch PR info (gh)              │
│  2. Analyze diff                    │
│  3. Security check                  │
│  4. Code quality check              │
│  5. Provide feedback                │
└─────────────────────────────────────┘
```

## Review Checklist

- [ ] Code follows project conventions
- [ ] No security vulnerabilities
- [ ] Error handling appropriate
- [ ] Tests cover new functionality
- [ ] Documentation updated

## Agent Integration

**보안 검토:**
```
Use the security-auditor agent to review security implications of PR
```

**코드 품질:**
```
Use the code-reviewer agent to do detailed code review
```

**테스트 검토:**
```
Use the test-writer agent to verify test coverage for changes
```

## Output Format

```
PR Review: #[number]
═══════════════════════════════════════
Title: [title]
Author: [author]
Changes: +[additions] -[deletions] ([files] files)

Summary:
  [what this PR does]

Analysis:
  ✓ Code quality: Good
  ✓ Tests: Adequate
  ⚠ Security: Review needed
  ○ Documentation: Missing

Issues:
  - [file:line] [issue description]

Recommendation: [Approve/Request Changes/Comment]

Feedback:
  [detailed feedback]
═══════════════════════════════════════
```

## Examples

```bash
/review-pr           # 현재 브랜치 PR
/review-pr 123       # PR #123 리뷰
```

## Related Skills

- `/code-quality`: 품질 검사
- `/git-workflow`: 브랜치 워크플로우
- `/fix-issue`: 이슈 수정
