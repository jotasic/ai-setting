---
name: type-check-improve
description: Improve type safety (TypeScript/Python).
argument-hint: [path]
---

# Type Check & Improve

Analyze and improve type coverage.

## Arguments

- `$ARGUMENTS`: Path (default: `./src`)

## Workflow

1. **Run Type Check**: `tsc`, `mypy/pyright`
2. **Analyze Coverage**: `type-coverage`
3. **Identify Issues**: `any`, missing returns, null checks
4. **Generate Improvements**: Add types, guards, fix errors

## Checklist
- Strict mode (tsconfig, mypy.ini)
- No implicit any
- Strict null checks

## Output
- Coverage report
- Issues found
- Improvements applied
