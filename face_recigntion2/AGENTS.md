# Smart Home Security — Universal Agent Operating Guide & Project Context

> **Agent-agnostic master instructions.** Designed for any AI coding assistant
> or multi-agent tool (Claude Code, Codex, Gemini CLI, Cursor, etc.).
> The Claude-Code-specific asset registry lives in `CLAUDE.md`; the principles
> below are tool-agnostic.

## 1. Stack (locked where ADRs exist)

| Layer | Choice |
|---|---|
| Language | Python 3 *(version line TBD via ADR)* |
| Face detection / recognition | TBD via ADR |
| Liveness / anti-spoofing | TBD via ADR |
| Camera capture | OpenCV (VideoCapture) — confirm via ADR |
| Environmental sensing | TBD via ADR |
| Simulation / 3D render | TBD via ADR |
| Logging / dashboard | TBD via ADR |
| Testing | pytest |
| Package / environment | venv + requirements.txt (or uv/poetry — TBD via ADR) |

## 2. Authoritative docs (READ FIRST)

- `docs/00-survey/` — survey foundation (1 overview + 12 papers).
- `docs/01-requirements/` · `docs/02-design/` · `docs/03-implementation/` ·
  `docs/04-testing/` · `docs/05-deployment/` — SDLC artifacts.

## 3. Source of truth (NON-NEGOTIABLE)

- The survey paper is the scope contract.
- The access log is append-only and the source of truth for access decisions.
- Biometric material is never committed and never logged.

## 4. Absolute NEVERs (red lines)

1. Never actuate the gate without face recognition **and** liveness (INV-1).
2. Never fail **open** on liveness — deny/flag, never auto-grant (INV-2).
3. Never commit or log biometric data (INV-3).
4. Never edit/delete the access log in place (INV-4).
5. Never let a recognition failure suppress a smoke/flame/motion alert (INV-5).
6. Never auto-pin a dependency version without explicit approval.

## 5. Version policy & documentation discovery

- Never assume versions from memory — check official docs before recommending
  or installing any framework/library/tool.
- Prefer the latest stable version compatible with the Python line.
- Never pin a version without explicit approval.

## 6. Decision-first policy

For any unresolved stack area (face-recognition library, liveness approach,
simulation tool, sensing, logging, Python line, package manager), write an ADR
under `docs/02-design/adr/NNNN-<title>.md` and wait for approval before
pinning a choice.

## 7. Commits / PRs / branches

- Conventional commits: `<type>(<scope>): <description>`.
- `<scope>` is a module name or a decision reference (`INV-N`, `ADR-NNNN`).
- Branches: `<type>/<short-kebab>`.

## 8. Invariants

See `CLAUDE.md` for the full INV-1…INV-6 list. In short: gate is gated (1),
fail closed on liveness (2), no biometric data in repo/logs (3), append-only
access log (4), environmental alerts independent (5), reproducible from a
clean clone (6).
