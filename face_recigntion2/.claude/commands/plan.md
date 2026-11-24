---
description: Restate requirements, assess risks, and create a step-by-step implementation plan. WAIT for operator CONFIRM before touching code.
---

# Plan Command

Invoke the **planner** agent to produce an implementation plan before writing
any code.

## What it does

1. Restates the requirement in the project's terms.
2. Identifies which invariants (INV-1…INV-6) and docs the change touches.
3. Breaks the work into phases with concrete steps.
4. Lists dependencies, risks, and test implications.
5. **WAITs for explicit confirmation** before any code is written.

## When to use

- New feature, architectural change, or complex refactor.
- Anything that touches the access decision or biometric handling.
- Any task that may need an ADR first.

## Notes

- Follow the ADR-first policy (`CLAUDE.md`) for new technology/versions.
- Keep scope tight to the survey (`docs/00-survey/`).
