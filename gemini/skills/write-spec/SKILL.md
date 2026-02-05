---
name: write-spec
description: Transform user requirements into PRD (Product Requirements Document).
argument-hint: <feature-description>
---

# Write Specification (PRD)

Create a PRD from user requirements. Technical implementation is delegated to `architect`.

## Arguments

- `$ARGUMENTS`: Feature description

## Workflow

1. **Analyze**: Understand requirements
2. **Clarify**: Ask questions
3. **Write PRD**: Create document
4. **Handoff**: Guide to `architect`

## Output
`docs/specs/{feature-name}-prd.md`

## Content
- Overview (Background, Goal, User)
- Functional Requirements (P0, P1, P2)
- User Scenarios
- Non-functional Requirements
- Constraints
- Success Metrics
- Out of Scope

## Next Steps
Use `architect` agent to design system based on PRD.
