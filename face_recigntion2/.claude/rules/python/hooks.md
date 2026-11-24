# Python Hooks

> This file extends [common/hooks.md](../common/hooks.md) with Python-specific
> content.

## Formatters / linters on save

Recommended PostToolUse (or editor on-save) hooks once the codebase exists:

- `ruff check --fix` and `ruff format` on `.py` changes.
- `mypy` on `.py` changes (once typing is enabled).

Wire these as project-local hooks in `.claude/settings.json` — do not assume a
global environment. Keep hook commands dependency-light so a fresh clone does
not break (INV-6).

## Pre-commit guard rails

Before committing Python that touches the access decision or biometric
handling, run the domain test suite (`pytest tests/domain`) — a red domain test
means the change violates an invariant (INV-1…INV-5).

## Observation hook

`.claude/skills/continuous-learning-v2` registers `observe.sh` for
PreToolUse/PostToolUse. It is defensive: it no-ops when Python is missing,
when the session is non-interactive, or when disabled. Do not delete it — it is
the learning substrate for the harness.
