# Python Testing

> This file extends [common/testing.md](../common/testing.md) with
> Python-specific content.

## Framework

- **pytest** is the only test runner. No `unittest`-style boilerplate.
- Use `pytest` fixtures for shared setup; `tmp_path`/`tmp_path_factory` for
  filesystem isolation.
- Use `pytest.mark.parametrize` for edge cases (e.g., grant/deny paths,
  liveness-fail-closed, spoof detection).

## Coverage

Tiered (see `CLAUDE.md`):

| Tier | Coverage |
|---|---|
| Domain (identity check, liveness gate, actuation logic, access log) | 80% |
| Application (camera pipeline, sensor glue, dashboard) | 60% |
| Infrastructure (configs, docs, scripts) | verify by run, not test |

Measure with `pytest --cov`.

## Mocking external hardware

- The camera and any sensor are **interfaces**, not direct `cv2.VideoCapture`
  calls in domain code. Tests inject a fake camera/sensor.
- Never require a physical camera or model download to run the test suite.
  Model-heavy inference is tested against small fixture images.

## TDD workflow (mandatory)

1. Write the test first (RED).
2. Run — it fails.
3. Minimal implementation (GREEN).
4. Refactor (IMPROVE).
5. Verify coverage.

## Naming

- Test functions describe behavior: `test_gate_stays_closed_when_liveness_times_out`.
- Reference invariants in names where relevant: `test_inv_1_no_actuation_without_liveness`.
