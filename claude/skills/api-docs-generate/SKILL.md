---
name: api-docs-generate
description: OpenAPI/Swagger API 문서 자동 생성
argument-hint: [output-path]
---

# API Documentation Generator

프로젝트의 API 엔드포인트를 스캔하여 OpenAPI 문서를 생성합니다.

## Arguments

- `$ARGUMENTS`: 출력 경로 (default: `./docs/api`)

## Workflow

1. **Detect API Framework**
   - Express.js
   - Fastify
   - NestJS
   - FastAPI
   - Django REST Framework
   - Go Gin/Echo

2. **Scan Endpoints**
   - Route definitions
   - Request/response schemas
   - Authentication requirements
   - Query parameters

3. **Generate OpenAPI Spec**
   ```yaml
   openapi: 3.0.3
   info:
     title: API Name
     version: 1.0.0
   paths:
     /users:
       get:
         summary: List users
         responses:
           '200':
             description: Success
   ```

4. **Generate Documentation**
   - Swagger UI HTML
   - Redoc HTML
   - Markdown docs

## Commands

```bash
# Express + swagger-jsdoc
npx swagger-jsdoc -d swaggerDef.js -o swagger.json

# NestJS
npx @nestjs/swagger generate

# FastAPI (auto-generated at /docs)

# Go Swag
swag init
```

## Output Structure

```
docs/api/
├── openapi.yaml      # OpenAPI specification
├── openapi.json      # JSON format
├── swagger-ui/       # Interactive docs
│   └── index.html
└── README.md         # API overview
```

## Annotations Example

```javascript
/**
 * @openapi
 * /users:
 *   get:
 *     summary: Get all users
 *     tags: [Users]
 *     parameters:
 *       - name: limit
 *         in: query
 *         schema:
 *           type: integer
 *     responses:
 *       200:
 *         description: List of users
 *         content:
 *           application/json:
 *             schema:
 *               type: array
 *               items:
 *                 $ref: '#/components/schemas/User'
 */
```

## Validation

After generation:
- Validate OpenAPI spec syntax
- Check for missing descriptions
- Verify example values
- Test with Swagger UI
