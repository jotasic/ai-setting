---
name: generate-geminimd
description: Auto-generate GEMINI.md from session history and project analysis.
argument-hint: [--output <path>] [--append] [--minimal]
---

# Generate GEMINI.md

Analyzes current session and project to generate GEMINI.md.

## Arguments

- `--output <path>`: Output path (default: `./GEMINI.md`)
- `--append`: Append to existing
- `--minimal`: Minimal version
- `--full`: Full analysis

## Workflow

1. **Project Analysis**: Language, framework, structure.
2. **Configuration Discovery**: Existing config files.
3. **Session Analysis**: Commands used, patterns established.
4. **Generate Documentation**

## Output Sections

- Project Name
- Quick Start
- Project Structure
- Development
- Tips for Gemini

## Integration

Use with `gemini-doc-generator` (or general) agent.
