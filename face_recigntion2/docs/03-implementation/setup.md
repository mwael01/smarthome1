# Team Setup Guide

> Target: **native Windows 10/11**, **Python 3.11**, `venv` + `requirements.txt`.
> Philosophy: **the team does almost nothing by hand.** Install two tools, set a
> few environment variables, clone, and Claude Code does the rest from A to Z
> (INV-6). The only genuinely manual, credential-bearing step is git auth.

## Why native Windows (not WSL)

The demo's headline feature is **live webcam recognition**. WSL2 cannot see the
webcam (`/dev/video0` does not exist there), so the camera path would not run.
Native Windows sees the webcam directly. The cost — `dlib` (the face-recognition
engine) is source-only on PyPI and must be compiled — is handled automatically by
Claude (it installs the compiler and builds dlib).

## 1. The two things YOU install (5 min)

Open **PowerShell** and run:

```powershell
# Git — to clone the repo
winget install Git.Git

# Claude Code — the AI that does the rest (native installer, no Node.js/admin)
irm https://claude.ai/install.ps1 | iex
```

> If `winget` is missing (older Windows 10), update **App Installer** from the
> Microsoft Store, or install Git from git-scm.com instead.

Re-open the terminal afterwards so the new commands are on `PATH`.

## 2. Set the environment (the DeepSeek block)

Claude Code is pointed at DeepSeek with a few environment variables. In
**PowerShell**, paste this block (the Operator will give you the real token):

```powershell
$env:ANTHROPIC_BASE_URL = "https://api.deepseek.com/anthropic"
$env:ANTHROPIC_AUTH_TOKEN = "sk-1db6dd86f9d94d1586628383af52dd0c"
$env:API_TIMEOUT_MS = "600000"
$env:ANTHROPIC_MODEL = "DeepSeek-V4-Pro"
$env:ANTHROPIC_DEFAULT_HAIKU_MODEL = "DeepSeek-V4-Pro"
$env:CLAUDE_CODE_DISABLE_NONESSENTIAL_TRAFFIC = "1"
```

In **Git Bash / WSL**, the same block uses `export NAME=value` instead of
`$env:NAME = "..."`.

> **Never put the token in any file in this repo.** It is a secret. Set it in your
> terminal, or in your own machine's `$PROFILE` — not in the repository. To avoid
> retyping it, add these lines to your PowerShell `$PROFILE` (a local, per-user
> file that is never committed).

## 3. Authenticate git, clone, cd

```powershell
# one-time: log in to the host (GitHub/GitLab) — this needs YOUR account
gh auth login          # or configure SSH, or use HTTPS with a personal access token

git clone https://github.com/a7mada8a98/graduation-project-smart-home.git
cd graduation-project-smart-home
```

## 4. Run Claude and give the go

```powershell
claude
```

(With the environment block set, Claude Code connects straight to DeepSeek.) Then
paste:

> Set up this project on my machine from scratch: install Python 3.11 and the
> Visual Studio C++ Build Tools, create the Python virtual environment, install
> `cmake`, install the dependencies from `requirements.txt`, then run
> `python face_demo.py --list` to verify everything works. If a step needs my
> approval or fails, tell me plainly.

## What Claude does (A to Z, ~20 min)

1. Installs Python 3.11 (`winget install Python.Python.3.11`).
2. Installs VS 2022 C++ Build Tools (the compiler `dlib` needs).
3. Creates the venv and installs `cmake`.
4. Installs `requirements.txt` (compiles `dlib`, ~5 min).
5. Runs `python face_demo.py --list` to verify.

## Two things you still touch during setup

1. **A UAC prompt** when the C++ Build Tools install runs — click **Yes**. (Windows
   requires admin for this; Claude cannot click it for you.)
2. **Permission prompts** — Claude asks before running install commands; approve
   them (type `y` / click allow). This is expected and keeps you in control.

> Only if the Operator explicitly tells you to, you may instead run
> `claude --dangerously-skip-permissions`, which approves everything automatically.
> Understand that removes **all** safety prompts — use it only for this one-time
> setup, on the Operator's instruction.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `python` / `pip` not recognized | re-open the terminal after install, or ask Claude to reinstall |
| `dlib` build fails with a compiler error | VS Build Tools missing — ask Claude to reinstall the C++ workload |
| `cmake` not found | `pip install cmake` inside the venv |
| `ImportError: pkg_resources` (Python 3.12 only) | `pip install "setuptools<81"` |
| `--camera` can't open the webcam | another app holds the camera, or you are in WSL/Linux |
| Claude can't reach the model | re-check the environment block (token + base URL) |

## Next

Enroll faces and run the demo → `docs/05-deployment/demo-guide.md`.
