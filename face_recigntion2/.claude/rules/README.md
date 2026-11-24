# Rules

## Structure

Rules are organized into a **common** layer plus **language-specific** directories:

```
rules/
├── common/          # Language-agnostic principles (always apply)
│   ├── coding-style.md
│   ├── git-workflow.md
│   ├── testing.md
│   ├── performance.md
│   ├── patterns.md
│   ├── hooks.md
│   ├── agents.md
│   ├── security.md
│   ├── code-review.md
│   └── development-workflow.md
└── python/          # Python-specific (this project's stack)
    ├── coding-style.md
    ├── testing.md
    ├── patterns.md
    ├── security.md
    └── hooks.md
```

- **common/** contains universal principles — no language-specific code.
- **python/** extends the common rules with Python idioms, tools, and examples.
  Each file references its common counterpart.

## Rules vs Skills

- **Rules** define standards, conventions, and checklists that apply broadly
  (e.g., "no hardcoded secrets", "80% domain coverage").
- **Skills** (`skills/`) provide deep, actionable reference for specific tasks.

Rules tell you *what* to do; skills tell you *how*.

## Rule priority

When language-specific rules and common rules conflict, **language-specific
rules take precedence** (specific overrides general).

## Adding a language

To add support for a new language (e.g., `typescript/` if a web simulation
frontend is added):

1. Create `rules/<lang>/`.
2. Add files that extend the common rules (`coding-style.md`, `testing.md`,
   `patterns.md`, `hooks.md`, `security.md`).
3. Each file starts with:
   `> This file extends [common/xxx.md](../common/xxx.md) with <Language> specifics.`
