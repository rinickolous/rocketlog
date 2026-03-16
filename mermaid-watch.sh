#!/usr/bin/env bash
set -euo pipefail

# Usage: ./mermaid-watch.sh <input.md> [output.svg]
#
# Watches a markdown file for changes, extracts the first mermaid code block,
# and re-renders it to SVG using mmdc on every save.
#
# Requires: inotifywait (inotify-tools), mmdc (@mermaid-js/mermaid-cli)

INPUT="${1:-}"
OUTPUT="${2:-}"

if [[ -z "$INPUT" ]]; then
  echo "Usage: $0 <input.md> [output.svg]" >&2
  exit 1
fi

if [[ ! -f "$INPUT" ]]; then
  echo "Error: file not found: $INPUT" >&2
  exit 1
fi

if [[ -z "$OUTPUT" ]]; then
  OUTPUT="${INPUT%.md}.svg"
fi

if ! command -v inotifywait &>/dev/null; then
  echo "Error: inotifywait not found. Install with: sudo pacman -S inotify-tools" >&2
  exit 1
fi

if ! command -v mmdc &>/dev/null; then
  echo "Error: mmdc not found. Install with: npm install -g @mermaid-js/mermaid-cli" >&2
  exit 1
fi

TMPFILE="$(mktemp /tmp/mermaid-XXXXXX.mmd)"
trap 'rm -f "$TMPFILE"' EXIT

extract_and_render() {
  # Extract content between first ```mermaid ... ``` block
  awk '/^```mermaid/{found=1; next} found && /^```/{exit} found{print}' "$INPUT" > "$TMPFILE"

  if [[ ! -s "$TMPFILE" ]]; then
    echo "  Warning: no mermaid block found in $INPUT" >&2
    return
  fi

	if 	mmdc -i "$TMPFILE" -o "$OUTPUT" -c mermaid.config.json --quiet 2>/dev/null; then
    echo "  Rendered -> $OUTPUT"
  else
    echo "  Render failed (syntax error?)" >&2
  fi
}

echo "Watching $INPUT -> $OUTPUT"
echo "Press Ctrl+C to stop."
echo ""

# Render once immediately on start
extract_and_render

# Then watch for writes
LAST_MOD=""
while true; do
  MOD="$(stat -c %Y "$INPUT")"
  if [[ "$MOD" != "$LAST_MOD" ]]; then
    LAST_MOD="$MOD"
    echo "[$(date +%H:%M:%S)] Change detected"
    extract_and_render
  fi
  sleep 0.5
done
