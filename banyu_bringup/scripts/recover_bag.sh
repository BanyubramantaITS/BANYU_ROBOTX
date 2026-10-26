#!/usr/bin/env bash
#
# Make a mission bag readable again after the AUV lost power (or the recorder was
# killed) before it could close the bag.
#
# Usage:
#   recover_bag.sh <run_dir>/bag
#
# A hard stop leaves no metadata.yaml and a truncated last .mcap file, so `ros2 bag`
# refuses to open the folder. This moves truncated files into <bag>/partial/ and
# rebuilds metadata.yaml from the complete ones. Files in partial/ can still be opened
# on their own in Foxglove, or repaired with `mcap recover` from the Foxglove mcap CLI.
set -euo pipefail

BAG="${1:?Usage: $0 <bag_dir>}"
BAG="${BAG%/}"

if [ -f "$BAG/metadata.yaml" ]; then
  echo "$BAG already has metadata.yaml; nothing to recover."
  exit 0
fi

# A complete MCAP file ends with the same magic bytes it starts with.
MAGIC="$(printf '\x89MCAP0\r\n' | od -An -tx1)"
shopt -s nullglob
for f in "$BAG"/*.mcap; do
  if [ "$(tail -c 8 "$f" | od -An -tx1)" != "$MAGIC" ]; then
    mkdir -p "$BAG/partial"
    mv "$f" "$BAG/partial/"
    echo "truncated, moved to partial/: $(basename "$f")"
  fi
done

ros2 bag reindex "$BAG" -s mcap
ros2 bag info "$BAG"
