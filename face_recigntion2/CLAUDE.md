# Smart Home Security — Project Context

> Read from `docs/` only when a change alters documented behavior — not for
> trivial edits. Use section references and decision IDs (`INV-N`, `ADR-NNNN`)
> to navigate, not full-file reads.

## Stack (locked where ADRs exist)

| Layer | Choice |
|---|---|
| Language | Python 3.11 *(team target — version-line ADR pending)* |
| Face detection / recognition | **TBD via ADR** (OpenCV + face_recognition/dlib, DeepFace, or InsightFace) |
| Liveness / anti-spoofing | **TBD via ADR** (heuristic / rPPG / depth) |
| Camera capture | OpenCV `VideoCapture` *(confirmed via ADR)* |
| Environmental sensing (motion / smoke / flame) | **TBD via ADR** |
| Simulation / 3D render (for demo) | **TBD via ADR** |
| Logging / dashboard / notifications | **TBD via ADR** |
| Testing | pytest |
| Package / environment | `venv` + `requirements.txt` on native Windows (see `docs/03-implementation/setup.md`) |

## Authoritative docs (READ FIRST)

- `docs/00-survey/` — the literature survey foundation: 1 overview paper + 12
  companion papers (markdown = working copies, `docx/` = submission originals).
- `docs/01-requirements/` — SRS, use cases, user stories (to be created).
- `docs/02-design/` — architecture, ADRs, data models (to be created).
- `docs/03-implementation/` — environment/setup guide (`setup.md` added), coding standards (to be created).
- `docs/04-testing/` — test plan and cases (to be created).
- `docs/05-deployment/` — run/demo guide (`demo-guide.md` added).

## Source of truth (NON-NEGOTIABLE)

- **The survey paper is the scope contract.** Features outside the survey's
  proposed system are out of scope unless an ADR or explicit operator decision
  adds them.
- **The access log is the source of truth for access decisions.** It is
  append-only and never edited in place (INV-4).
- **Biometric material is not system of record for anything other than the
  identity check it powers** — never committed, never logged (INV-3).

## Absolute NEVERs (red lines)

1. Never actuate the gate/door without a successful face recognition **and** a
   passing liveness check for an enrolled identity (INV-1).
2. Never fail **open** on liveness — if anti-spoofing is unavailable or
   indeterminate, deny or flag; never auto-grant (INV-2).
3. Never commit face images, embeddings, or templates to git, and never write
   them to the access log (INV-3).
4. Never edit or delete the access log in place — append only (INV-4).
5. Never let a face-recognition failure suppress a smoke/flame/motion alert
   (INV-5).
6. Never auto-pin a dependency version without explicit operator approval
   (Version policy).

## Version policy

- **Never assume versions from memory.** Use the `context7` MCP
  (`resolve-library-id` → `query-docs`) before recommending or installing any
  framework, library, SDK, or tool.
- Prefer the latest stable version compatible with the project's Python line.
- **Never pin a version without explicit operator approval.**

## Commits / PRs / branches

- **Commit format:** `<type>(<scope>): <description>` (conventional commits).
  - `<type>` ∈ {feat, fix, refactor, perf, test, docs, chore, ci}.
  - `<scope>` is a module name (`face-recognition`, `liveness`, `simulation`,
    `logging`, `sensors`, `docs`, …) or a decision reference (`INV-N`, `ADR-NNNN`).
  - Examples: `feat(face-recognition): enroll + recognize pipeline`,
    `fix(INV-2): deny instead of grant on liveness timeout`,
    `docs(03-implementation): add Windows setup guide`.
- **Branch:** `<type>/<short-kebab>` (e.g., `feat/face-enrollment`,
  `fix/liveness-fail-closed`).

## Test expectations

Tiered minimums by code role:

| Tier | Examples | Coverage | Required scenarios |
|---|---|---|---|
| Domain | identity verification, liveness gate, gate actuation logic, access-log append | **80%** | grant/deny paths, liveness-fail-closed, no-gate-without-liveness, log append-only |
| Application | camera pipeline, sensor integrations, dashboard/notification glue | **60%** | happy path + error path per component |
| Infrastructure | configs, docs, scripts | no minimum | run/verify appropriate to the change |

Tests are pytest. Pure configs/docs require verification appropriate to the
change (e.g., run the setup guide from a clean clone), not artificial tests.

## ADR-first decision policy

If a task **introduces, replaces, or performs a major version upgrade** of a
technology in an unresolved decision area, produce an ADR and wait for approval.

Never choose automatically for unresolved decisions in:

> Face recognition library · liveness/anti-spoofing approach · simulation/3D-render
> tool · environmental sensing approach · logging/dashboard approach · Python
> version line · package/environment manager · camera capture details

Each ADR must cover: recommended option, alternatives, fit to the survey scope,
risks, complexity, and what needs operator approval.
ADRs → `docs/02-design/adr/NNNN-<title>.md`.

## Critical invariants (do not violate)

1. **INV-1 — Gate actuation is gated.** The gate/door actuates only after a
   successful face recognition **and** a passing liveness check for an enrolled
   identity. Motion, an environmental event, or a recognition match without
   liveness must never open the gate.
2. **INV-2 — Fail closed on liveness.** If anti-spoofing is unavailable or
   indeterminate, the system denies (or flags for manual review). No code path
   skips liveness.
3. **INV-3 — No biometric data in repo or logs.** Face images, embeddings, and
   templates are never committed and never written to the access log. Enrolled
   identities live in a documented, gitignored, revocable store using only the
   team's own consented images.
4. **INV-4 — Append-only access log.** Every access decision (GRANT/DENY +
   reason) and every environmental alert is appended with a UTC timestamp to a
   tamper-evident log. The log is never edited or deleted in place.
5. **INV-5 — Environmental alerts are independent.** Smoke/flame/motion detection
   and alerting run independently of face recognition. A recognition failure
   must never suppress a fire or intrusion alert.
6. **INV-6 — Reproducible from a clean clone.** A fresh clone + the documented
   setup runs the whole system with zero undocumented manual steps. Setup for a
   low-experience team is a first-class deliverable.

## Skills, Agents, Plugins & Rules

### MCP servers (`.mcp.json`)

- **`context7`** — mandatory for the version policy. Always
  `resolve-library-id` → `query-docs` before recommending any dependency.

### Project-local agents (`.claude/agents/`)

| Agent | When to use |
|---|---|
| architect | System design, architectural decisions |
| planner | Complex multi-step features, planning before code |
| code-reviewer | General code review after any change |
| python-reviewer | Python-specific review (idioms, typing, safety) |
| security-reviewer | Auth, liveness, biometric handling, input handling |
| tdd-guide | Write-tests-first for features and bug fixes |
| build-error-resolver | Build/import/dependency failures |
| code-explorer | Tracing execution paths, mapping architecture |
| silent-failure-hunter | Swallowed exceptions, bad fallbacks, error propagation |
| doc-updater | Updating docs, codemaps, guides |
| performance-optimizer | Bottlenecks in the CV pipeline / inference |
| refactor-cleaner | Dead code removal and consolidation |

### Project-local commands (`.claude/commands/`)

`/plan` · `/tdd` · `/code-review` · `/build-fix` · `/verify` · `/docs`

### Project-local skills (`.claude/skills/`)

- `continuous-learning-v2` — hook-based session observation and instinct learning.

### Concern → asset cheatsheet

| Concern | Primary | Backup |
|---|---|---|
| Face detection/recognition pipeline | architect | code-explorer, python-reviewer |
| Liveness / anti-spoofing (INV-2) | security-reviewer | architect, tdd-guide |
| Gate actuation gating (INV-1) | security-reviewer | silent-failure-hunter |
| Camera capture / OpenCV | build-error-resolver | code-explorer |
| Simulation / 3D render | architect | code-explorer |
| Append-only access log (INV-4) | security-reviewer | silent-failure-hunter |
| Environmental sensing (INV-5) | silent-failure-hunter | tdd-guide |
| Testing / coverage | tdd-guide | python-reviewer |
| Reproducible setup (INV-6) | doc-updater | build-error-resolver |
| Architecture decision (ADR) | planner | architect |

All assets are project-local under `.claude/` and `.mcp.json`. The repo is
self-contained.

> **IMPORTANT:** Only use skills, agents, commands, and rules defined in this
> repository (`.claude/`). Do not rely on global user-level Claude assets from
> `~/.claude/` or any other location outside this project. If a required asset
> is not present here, stop and ask before reaching for anything global.
