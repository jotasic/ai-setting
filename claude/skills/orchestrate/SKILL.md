---
name: orchestrate
description: Automatically orchestrate multiple specialized agents for complex features
argument-hint: <feature-description>
allowed-tools: Task, TodoWrite, Bash
model: sonnet
category: workflow
---

# Orchestrator

Automatically coordinate multiple specialized agents for complex features.

## When to Use

- Full-stack implementation (backend + frontend + DB)
- User requests "auto execute" or "parallel processing"
- Complex features requiring multiple domains

**Don't use when:**
- Single domain tasks (use domain agent directly)
- Want manual step-by-step control (use `/workflow-guide`)
- Minor bug fixes (use `/fix-issue`)

## Execution Phases

```
┌─────────────────────────────────────────────────────────────┐
│  PHASE 1: Analysis & Planning                                │
│  ├─ Analyze requirements: $ARGUMENTS                         │
│  ├─ Decompose into tasks                                     │
│  └─ Create task board with TodoWrite                         │
├─────────────────────────────────────────────────────────────┤
│  PHASE 2: Task Board Setup                                   │
│  ├─ Record all tasks in TodoWrite                            │
│  ├─ Set priorities and dependencies                          │
│  └─ Assign agents to each task                               │
├─────────────────────────────────────────────────────────────┤
│  PHASE 3: Parallel Execution                                 │
│  ├─ Execute tasks by priority order                          │
│  ├─ Run same-priority tasks in parallel (max 3)              │
│  └─ Update TodoWrite as tasks complete                       │
├─────────────────────────────────────────────────────────────┤
│  PHASE 4: Verification                                       │
│  ├─ Run build/lint/type-check after each phase               │
│  ├─ If verification fails → retry with different approach    │
│  └─ Max 2 retries per task                                   │
├─────────────────────────────────────────────────────────────┤
│  PHASE 5: Summary & Review                                   │
│  ├─ Collect all results                                      │
│  ├─ Run code-reviewer for final review                       │
│  └─ Generate completion summary                              │
└─────────────────────────────────────────────────────────────┘
```

## ⚡ Immediate Execution

Start orchestration immediately:

**Step 1: Plan**
```
Use the pm-agent to create task board for: $ARGUMENTS
```

**Step 2: Execute tasks by priority**

For P0 tasks (execute first):
```
Use the [assigned-agent] agent to [task-description]
```

For same-priority tasks (execute in parallel):
```
Use the Task tool to launch multiple agents in parallel
```

**Step 3: Verify after each phase**
```
/code-quality
```

**Step 4: Final review**
```
Use the code-reviewer agent to review all changes
```

## Rules

| Rule | Description |
|------|-------------|
| **Max Parallel** | Maximum 3 concurrent agent tasks |
| **Verification Gate** | Must verify after each phase |
| **Retry Logic** | Max 2 retries (30s, 60s delay), then try different approach |
| **Task Ownership** | Each agent manages its own task progress |

## Output Format

```
Orchestration Complete
═══════════════════════════════════════

Feature: [feature name]
Status: ✅ SUCCESS / ❌ PARTIAL / ❌ FAILED

Tasks Completed: X/Y
├─ ✅ T-001: [task] (backend-developer)
├─ ✅ T-002: [task] (frontend-developer)
└─ ❌ T-003: [task] (test-writer) - [reason]

Verification:
├─ Build: ✅ PASS
├─ Lint: ✅ PASS
├─ Type Check: ✅ PASS
└─ Tests: ✅ PASS

Next Steps:
- [any remaining work]
- [recommended follow-up]
═══════════════════════════════════════
```

## Related Skills

- `/workflow-guide`: Manual step-by-step coordination
- `/full-dev`: Full development flow
- `/plan`: Planning only (no execution)
