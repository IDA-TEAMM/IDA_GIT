---
name: feedback-ida-scan-git-while-coding
description: "While actively coding on the ida_topics/ros2_ws repo, regularly fetch/check git for teammate updates"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 36e83169-4e5a-4bf2-bf8c-46b1cbbe394d
---

When working on code changes in this repo (not just at session start), periodically run `git fetch` / check `git status` and `git log` against `origin` to catch updates teammates (especially the captain, Yahya Seha Danış) push — don't only check once at the start of a session.

**Why:** On 2026-07-14, the captain pushed a large `son_kod/` addition directly to `origin/main` mid-session without announcing it separately — it was only discovered because the user happened to paste the GitHub repo URL and a fetch was run. Given the team pushes to the shared `IDA_GIT` repo somewhat unpredictably (see [[project_ida_captain_decision_repo]]), and the video deadline (2026-07-21) means multiple people are actively landing changes, stale local state risks working on outdated code or missing conflicting changes.

**How to apply:** During any coding session in `~/ros2_ws` (or a clone of `IDA_GIT`), run `git fetch origin` at reasonable intervals — e.g., before starting a new chunk of work, or every so often during a longer session — and surface anything new to the user rather than waiting to be asked. Don't do this so often it's noisy; a fetch before major steps (starting a new task, before a commit/push) is the right cadence, not on every single file edit.
