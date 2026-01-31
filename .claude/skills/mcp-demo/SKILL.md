---
name: mcp-demo
description: MCP 서버 사용법 데모 및 가이드
argument-hint: [server-name]
---

# MCP (Model Context Protocol) Demo

MCP 서버 연동 방법과 예제를 설명합니다.

## Arguments

- `$ARGUMENTS`: 특정 MCP 서버 이름 (선택)

## Available MCP Servers

### 1. Filesystem Server
파일시스템 접근을 위한 서버

```json
{
  "filesystem": {
    "command": "npx",
    "args": ["-y", "@anthropic/mcp-server-filesystem", "/allowed/path"]
  }
}
```

**사용 예:**
- 파일 읽기/쓰기
- 디렉토리 탐색
- 파일 검색

### 2. GitHub Server
GitHub API 연동

```json
{
  "github": {
    "command": "npx",
    "args": ["-y", "@anthropic/mcp-server-github"],
    "env": { "GITHUB_TOKEN": "..." }
  }
}
```

**사용 예:**
- 이슈/PR 관리
- 코드 검색
- 리포지토리 정보

### 3. PostgreSQL Server
데이터베이스 쿼리

```json
{
  "postgres": {
    "command": "npx",
    "args": ["-y", "@anthropic/mcp-server-postgres"],
    "env": { "DATABASE_URL": "..." }
  }
}
```

**사용 예:**
- SQL 쿼리 실행
- 스키마 탐색
- 데이터 분석

### 4. Memory Server
지식 그래프 기반 메모리

```json
{
  "memory": {
    "command": "npx",
    "args": ["-y", "@anthropic/mcp-server-memory"]
  }
}
```

**사용 예:**
- 세션 간 정보 유지
- 관계형 데이터 저장
- 컨텍스트 누적

### 5. Puppeteer Server
브라우저 자동화

```json
{
  "puppeteer": {
    "command": "npx",
    "args": ["-y", "@anthropic/mcp-server-puppeteer"]
  }
}
```

**사용 예:**
- 웹 스크래핑
- UI 테스트
- 스크린샷 캡처

### 6. Sequential Thinking Server
복잡한 추론을 위한 단계별 사고

```json
{
  "sequential-thinking": {
    "command": "npx",
    "args": ["-y", "@anthropic/mcp-server-sequential-thinking"]
  }
}
```

**사용 예:**
- 복잡한 문제 분해
- 단계별 추론
- 의사결정 트리

## MCP 설정 방법

### 1. 프로젝트 레벨 (권장)
`.claude/mcp.json` 파일 생성

### 2. 글로벌 레벨
`~/.claude/mcp.json` 파일 생성

### 3. 확인
Claude Code에서 `/mcp` 명령으로 연결된 서버 확인

## 커스텀 MCP 서버 만들기

```typescript
// server.ts
import { Server } from "@modelcontextprotocol/sdk/server";

const server = new Server({
  name: "my-custom-server",
  version: "1.0.0"
});

server.setRequestHandler("tools/list", async () => ({
  tools: [{
    name: "my_tool",
    description: "My custom tool",
    inputSchema: { type: "object", properties: {} }
  }]
}));

server.setRequestHandler("tools/call", async (request) => {
  // 도구 실행 로직
});
```
