---
description: Implement a feature or fix with test-driven development (write the failing test first).
---

# TDD Command

Implement with test-driven development via the **tdd-guide** agent.

## Workflow

1. Write the failing test (RED).
2. Run — confirm it fails.
3. Minimal implementation (GREEN).
4. Refactor (IMPROVE).
5. Verify coverage (`CLAUDE.md` tiers: domain 80%, application 60%).

## Context

- pytest + fixtures + `tmp_path`; inject fake `Camera`/`Sensor`/`Actuator`.
- Never require hardware or model downloads in tests.
- Name tests by behavior; reference invariants where relevant.
