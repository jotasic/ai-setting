---
name: api-docs-generate
description: OpenAPI/Swagger API documentation generator.
argument-hint: [output-path]
---

# API Documentation Generator

Scans project API endpoints and generates OpenAPI documentation.

## Arguments

- `$ARGUMENTS`: Output path (default: `./docs/api`)

## Workflow

1. **Detect API Framwork**
   - Express.js, NestJS, FastAPI, etc.

2. **Scan Endpoints**
   - Routes, schemas, auth, parameters

3. **Generate OpenAPI Spec**
   ```yaml
   openapi: 3.0.3
   info:
     title: API Name
     version: 1.0.0
   paths: ...
   ```

4. **Generate Documentation**
   - Swagger UI / Redoc / Markdown

## Commands

```bash
# Express + swagger-jsdoc
npx swagger-jsdoc -d swaggerDef.js -o swagger.json

# NestJS
npx @nestjs/swagger generate

# Go Swag
swag init
```

## Validation

- Validate syntax
- Check descriptions and examples
- Test with Swagger UI
