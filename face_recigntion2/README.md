# Smart Home Security System — Graduation Project

A lightweight multi-sensor smart-home security system centered on face
recognition as the access-control mechanism, with liveness/anti-spoofing,
motion detection, smoke/flame detection, dual-camera coverage, and gate
actuation — implemented primarily in software with a high-quality simulation
and 3D render for demonstration.

This repository is **documentation-driven**: the project follows the SDLC as a
sequence of living documents, then implementation follows the docs.

## Repository layout

```
docs/
  00-survey/          # Literature survey — 1 overview + 12 companion papers
  01-requirements/    # SRS, use cases, user stories (SDLC)
  02-design/          # Architecture, ADRs, data models
  03-implementation/  # Environment/setup guide, coding standards
  04-testing/         # Test plan and cases
  05-deployment/      # Run/demo guide
.claude/              # Project-local Claude harness (agents, commands, rules, skills)
CLAUDE.md             # Claude Code project instructions
AGENTS.md             # Agent-agnostic project instructions
```

## Team

| Member | Track |
|---|---|
| Khaled Niazy Yassein | Cyber Security |
| Khaled Naser Mansour | Cyber Security |
| Pola Lazem Shafek | Data Science |

Academic supervisor: Dr. Ayman Mohamed
Egyptian Chinese University — Faculty of Computers and Information Systems

## Status

- [x] Survey papers (overview + 12) — `docs/00-survey/`
- [ ] Requirements (SRS)
- [ ] Design / ADRs
- [ ] Implementation
- [ ] Testing
- [ ] Deployment / demo

## Getting started

- **Team setup** (install tools, clone, give Claude the go):
  `docs/03-implementation/setup.md`
- **Running the face-recognition demo** (camera + photo paths, enrollment):
  `docs/05-deployment/demo-guide.md`
