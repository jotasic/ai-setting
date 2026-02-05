---
name: review-pr
description: Analyze and review GitHub Pull Requests.
disable-model-invocation: true
allowed-tools: Bash, Read, Grep, Glob
context: fork
agent: Explore
---

# Pull Request Review

## PR Information

Get PR details using:
`gh pr view --json number,title,body,author,additions,deletions,changedFiles`

## PR Diff

Get PR diff using:
`gh pr diff`

## PR Comments

Get existing comments:
`gh pr view --comments`

---

## Review Process

### 1. Summary

Summarize the purpose and scope of this PR:
- What problem does it solve?
- What approach was taken?
- How many files changed?

### 2. Code Analysis

For each changed file:
- Purpose of changes
- Potential issues
- Suggestions for improvement

### 3. Checklist

- [ ] Code follows project conventions
- [ ] No security vulnerabilities introduced
- [ ] Error handling is appropriate
- [ ] Tests cover new functionality
- [ ] Documentation updated if needed

### 4. Feedback

Provide actionable feedback:

**Approve** if:
- Changes are correct and complete
- No blocking issues

**Request Changes** if:
- Security issues
- Breaking changes
- Missing critical functionality

**Comment** for:
- Suggestions
- Questions
- Minor improvements
