---
name: girdap-ida-algi-github-upload
description: DONE — girdap-ida-algi ROS2 package pushed to private GitHub repo github.com/EyupEker1/girdap-ida-algi
metadata: 
  node_type: memory
  type: project
  originSessionId: ce0708f0-5728-4a09-a7c8-d68bf546f586
---

**Completed 2026-07-09.** The `girdap-ida-algi` ROS2 package (extracted from `girdap-ida-algi(4).zip` to `/home/eyup/girdap-ida-algi`) was pushed to a new **private** repo: **https://github.com/EyupEker1/girdap-ida-algi** — branch `main`, 2 commits, remote `origin` over HTTPS.

Durable facts learned along the way:
- User's **GitHub username: `EyupEker1`** (logged in via `gh auth`, token in keyring, scopes: repo, workflow, gist, read:org).
- `gh` CLI **2.96.0 is installed at `/home/eyup/.local/bin/gh`** (direct binary download, NOT apt). Installed this way because **passwordless sudo is unavailable** on this box and `sudo apt install gh` failed. `~/.local/bin` is on PATH.
- Repo `.gitignore` excludes `build/`, `__pycache__/`, `*.egg-info/`, model files — only source is tracked. In-repo commit author is placeholder `Team GIRDAP <girdap@example.com>`.

**How to apply:** Future pushes for this project are just `git push` from `/home/eyup/girdap-ida-algi`. For other GitHub work, gh is already authenticated as EyupEker1 — no re-login needed. If sudo-requiring installs come up again, prefer the no-sudo `~/.local/bin` binary-download approach.
