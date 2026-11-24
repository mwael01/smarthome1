---
description: Review the recent change for correctness, quality, and invariants. Reports findings; does not edit.
---

# Code Review Command

Run a review over the current change using **code-reviewer** (and
**python-reviewer** / **security-reviewer** when the change touches the access
decision, liveness, biometric data, or the access log).

## Focus

- Access-decision correctness (INV-1, INV-2).
- Python rules (`.claude/rules/python/`).
- Error handling — no bare `except:`, no swallowed exceptions.
- Biometric/log invariants (INV-3, INV-4).

Reports findings with `file:line` refs, ranked by severity.
