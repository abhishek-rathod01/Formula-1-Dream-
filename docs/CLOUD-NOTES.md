# Cloud environment setup for Formula-1-Dream-

The Claude Code cloud **Setup script** cannot live in the repo - it is attached
to the environment, not the clone. Paste the block below into the environment's
"Setup script" field at claude.ai/code (environment selector -> settings icon).

Everything that *can* live in the repo already does: `.claude/settings.json`
(permissions + SessionStart hook) and `scripts/claude-setup.sh` (dependency
install, gated on `CLAUDE_CODE_REMOTE`), so local and cloud behave the same.

Recommended environment name: `Formula-1-Dream-`
Network access: Trusted (default)

```bash
#!/bin/bash
set -u   # deliberately NOT -e: a non-zero exit blocks the session from starting

# Python 3.x, pip, pytest, ruff are preinstalled. Project deps are installed by
# the repo's SessionStart hook (scripts/claude-setup.sh) so they work locally too.
python3 --version

exit 0
```

Keep total setup-script runtime under about five minutes so the environment
cache can build. Append `|| true` to anything non-critical - a non-zero exit
stops the session from starting at all.
