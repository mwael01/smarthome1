# Python Security

> This file extends [common/security.md](../common/security.md) with
> Python-specific content.

## Biometric data (INV-3)

- Face images, embeddings, and templates are **never committed** and **never
  written to the access log**.
- Enrolled identities live in a gitignored, documented, revocable store
  (`known_faces/`, `captures/`, etc. — see `.gitignore`).
- Only the team's own consented images are used for enrollment.

## Liveness / anti-spoofing (INV-1, INV-2)

- The access decision is `GRANT` only when recognition **and** liveness both
  pass. Any other combination is `DENY` (or flagged), never auto-grant.
- There is no code path that bypasses liveness. A liveness check that fails to
  produce a confident result must fail closed.

## Access log (INV-4)

- Append-only: write a full record (timestamp, identity, decision, reason) per
  event. No in-place edits or deletes.
- Log in UTC. Never log biometric material or secrets.

## Dependency hygiene

- Pin exact versions via the version policy (context7 → approve → pin).
- Prefer well-maintained, widely-used packages; review any package that touches
  the filesystem, network, or subprocess.

## Secrets & input

- Secrets via environment variables only; never hardcoded (common/security.md).
- Validate all external input (camera frames, config, filenames) at the
  boundary; never trust a filename or path from an external source.
- Avoid `subprocess`/`eval`/`pickle` on untrusted input.
