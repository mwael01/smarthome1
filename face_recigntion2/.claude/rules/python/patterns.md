# Python Patterns

> This file extends [common/patterns.md](../common/patterns.md) with
> Python-specific content.

## Pipeline structure (face recognition)

Model the recognition flow as a sequence of small, testable stages rather than
one monolithic function:

```
capture -> detect -> align -> embed -> compare -> liveness -> decision -> log
```

Each stage is a function/class with a single responsibility and a clear
input/output contract, so stages can be swapped (e.g., a different detector or
embedding model) without touching the decision logic.

## Interface over hardware

- Define protocols/ABCs for `Camera`, `Sensor`, and `Actuator` (gate).
- Domain logic depends on these interfaces, never on `cv2` or hardware directly.
- A `SimulatedCamera` / `SimulatedGate` implement the same interfaces for the
  demo and for tests (enables software-only development — INV-6).

## Configuration

- Centralize configuration in one place (e.g., `config.py` / a `.yaml` /
  `.env`), loaded at startup.
- No hardcoded paths, thresholds, or model names scattered in modules.
- Secrets (if any) come from environment variables only.

## Result types

- Prefer explicit result objects or enums over sentinel `None` / `-1` returns,
  especially for the access decision (e.g., `Decision.GRANT`,
  `Decision.DENY(reason=...)`). A denied decision must carry *why* (INV-4).

## Reuse patterns

- Use the standard library where it suffices (`collections`, `functools`,
  `dataclasses`, `pathlib`) before reaching for third-party deps.
- Keep the dependency list small — the team must install it (INV-6).
