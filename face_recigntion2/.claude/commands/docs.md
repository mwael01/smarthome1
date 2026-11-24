---
description: Update or generate documentation to match the current code and tree.
---

# Docs Command

Sync documentation with reality via **doc-updater**.

## Scope

- `docs/` SDLC artifacts (requirements, design, implementation, testing, deployment).
- `docs/03-implementation/setup.md` — the team environment guide.
- `README.md`, `CLAUDE.md`, `AGENTS.md` — layout and registry accuracy.

## Rule

Verify against the current code/tree before updating — no stale instructions
(INV-6: reproducible from a clean clone).
