---
name: search-code
description: 코드베이스에서 특정 패턴이나 기능을 검색합니다
argument-hint: [search query]
allowed-tools: Read, Grep, Glob, Bash
agent: Explore
---

# Search Code

코드베이스에서 원하는 코드를 효과적으로 찾습니다.

## Arguments

- `$ARGUMENTS`: 검색할 패턴 또는 설명

## Search Strategies

### 1. Text Search

```bash
# Exact match
grep -rn "searchTerm" --include="*.{js,ts,py}"

# Regex
grep -rn "pattern.*match" --include="*.{js,ts,py}"

# Case insensitive
grep -rni "searchterm" --include="*.{js,ts,py}"
```

### 2. File Search

```bash
# By name
find . -name "*.test.js"

# By pattern
find . -path "**/components/*.tsx"
```

### 3. Symbol Search

- 함수 정의: `function functionName` or `def functionName`
- 클래스 정의: `class ClassName`
- 변수 할당: `const/let/var varName =`

### 4. Git History Search

```bash
# Commit messages
git log --oneline --grep="keyword"

# Code changes
git log -S "code_snippet" --oneline

# By author
git log --author="name" --oneline
```

## Output Format

```
Search: "$ARGUMENTS"

Found X matches:

[file:line] context...
[file:line] context...

Related:
  - [related files or functions]
```

## Tips

- 구체적인 검색어 사용
- 파일 타입 필터링 활용
- 정규표현식으로 유연한 검색
- 결과가 많으면 범위 좁히기
