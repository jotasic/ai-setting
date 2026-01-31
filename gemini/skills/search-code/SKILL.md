---
name: search-code
description: Search codebase for patterns or functions.
argument-hint: [search query]
allowed-tools: Read, Grep, Glob, Bash
agent: Explore
---

# Search Code

Find code efficiently.

## Arguments

- `$ARGUMENTS`: Search query

## Search Strategies

### 1. Text Search
`grep -rn "term"`, `grep -rn "regex"`

### 2. File Search
`find . -name "pattern"`

### 3. Symbol Search
Functions, Classes, Variables

### 4. Git History
`git log --grep`, `git log -S`

## Output Format
```
Search: "..."
Found X matches:
[file:line] context
...
Related: ...
```
