---
name: api-docs-generate
description: OpenAPI/Swagger API 문서 자동 생성
argument-hint: [path] [--format=<yaml|json>]
allowed-tools: Bash, Read, Write, Grep, Glob
model: sonnet
category: documentation
---

# API Documentation Generator

프로젝트의 API 엔드포인트를 스캔하여 OpenAPI 문서를 생성합니다.

## Triggers (사용 조건)

- "API 문서 만들어줘", "generate API docs"
- "Swagger 생성", "OpenAPI spec"
- API 문서화 필요시

## Arguments

- `$ARGUMENTS`: 소스 경로 (default: `./src`)
- `--format=<yaml|json>`: 출력 포맷

## Workflow

```
┌─────────────────────────────────────┐
│  1. Detect API framework            │
│  2. Scan endpoints                  │
│  3. Generate OpenAPI spec           │
│  4. Create documentation            │
└─────────────────────────────────────┘
```

## Supported Frameworks

| Framework | Detection |
|-----------|-----------|
| Express.js | `express()` |
| NestJS | `@Controller()` |
| FastAPI | `@app.get()` |
| Django REST | `@api_view` |
| Go Gin | `gin.Default()` |

## Agent Integration

**API 설계 리뷰:**
```
Use the api-designer agent to review and improve API design
```

**문서 작성:**
```
Use the doc-writer agent to enhance API documentation with examples
```

## Output Format

```
API Docs Generated
═══════════════════════════════════════
Endpoints: 24
Methods: GET(12), POST(8), PUT(3), DELETE(1)

Output:
  docs/api/openapi.yaml
  docs/api/openapi.json
  docs/api/index.html (Swagger UI)

Coverage:
  ✓ All endpoints documented
  ⚠ 3 missing descriptions
  ○ 5 missing examples
═══════════════════════════════════════
```

## Examples

```bash
/api-docs-generate                    # 기본
/api-docs-generate src/api            # 특정 경로
/api-docs-generate --format=json      # JSON 포맷
```

## Related Skills

- `/architecture-review`: API 구조 분석
- `/full-dev`: 전체 개발 플로우
