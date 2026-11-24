# Python Coding Style

> This file extends [common/coding-style.md](../common/coding-style.md) with
> Python-specific content.

## Tooling

- **Formatting / linting:** `ruff` (fast, single tool). Configure once in
  `pyproject.toml`; do not mix with Black/flake8 configs.
- **Type checking:** `mypy` in strict mode where the codebase allows.
- **Imports:** stdlib → third-party → local, each group separated by a blank
  line. Sort with `ruff` (isort rules enabled).

## Idioms

- Prefer `pathlib.Path` over `os.path` and raw string paths.
- Use `dataclasses` or `attrs` for data carriers; avoid dicts-of-dicts for
  domain objects.
- Use type hints on public function signatures and return types.
- Prefer `functools`/`itertools`/`collections` helpers over hand-rolled loops.
- Context managers (`with`) for file handles, captures, and locks.
- Read-only collections: prefer `tuple`/`frozenset`/`Mapping` for data that must
  not mutate (see common/coding-style.md immutability).

## Error handling

- Raise specific exceptions; never a bare `except:` or a swallowed `except:
  pass` (see common/coding-style.md error handling).
- Let exceptions propagate to a single top-level handler that logs and exits
  cleanly — especially in the camera/recognition loop, where a silent failure
  could leave the gate logic in a bad state.

## Structure

- Small modules (200–400 lines typical, 800 max) organized by domain:
  `face/`, `liveness/`, `sensors/`, `gate/`, `logging/`, `simulation/`.
- One responsibility per module; extract utilities rather than growing files.

## Quality checklist

- [ ] Typed public APIs
- [ ] No bare `except:`
- [ ] `pathlib` over `os.path`
- [ ] Ruff clean, mypy clean where configured
- [ ] No hardcoded paths/secrets (use config + env)
