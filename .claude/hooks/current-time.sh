#!/bin/bash
cat <<EOF
{
  "suppressOutput": true,
  "hookSpecificOutput": {
    "hookEventName": "SessionStart",
    "additionalContext": "Current time and date: $(date '+%H:%M:%S %Y-%m-%d')"
  }
}
EOF