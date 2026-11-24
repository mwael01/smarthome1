---
description: Verify a feature is complete and consistent with the requirements and invariants.
---

# Verify Command

Verify a completed feature against the requirements and invariants.

## Checklist

- [ ] Behavior matches the spec/docs it was built from.
- [ ] Invariants INV-1…INV-6 hold (gate gated, fail-closed liveness, no
      biometric data in repo/logs, append-only log, independent alerts,
      reproducible).
- [ ] Tests pass and coverage meets the tier (`CLAUDE.md`).
- [ ] No stale docs (see `/docs`).
- [ ] No dependency added/bumped without approval.

Use **silent-failure-hunter** for the camera/recognition loop and
**security-reviewer** for access-decision/biometric changes.
