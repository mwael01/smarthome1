---
description: Fix a build, import, or dependency error with a minimal diff.
---

# Build Fix Command

Resolve a run/import/dependency failure via **build-error-resolver**.

## Process

1. Reproduce the error.
2. Isolate the cause.
3. Apply the smallest fix.
4. Verify by re-running.

## Constraint

Respect the version policy: do not add/bump a dependency version without
context7 + operator approval — surface the change instead of silently pinning.
