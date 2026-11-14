#!/bin/bash
# Split one tmux window into an even grid of panes (default 2 rows x 4 columns), all in the
# directory it is called from. Inside tmux: adds a new window to the current session. Outside:
# creates/attaches the session.
START_DIR="$PWD"
ROWS=2
COLS=4
SESSION="banyu"

usage() {
  cat <<EOF
Usage: tools/scripts/run/tmux_panes.sh [OPTIONS]

Opens a tmux window split into an even grid of ROWS x COLS panes, each started in the current
directory ($START_DIR).
Inside tmux it adds the window to the current session; outside it creates (or reuses) a
session and attaches to it.

Options:
  -r, --rows N        Rows of panes (default $ROWS).
  -c, --cols N        Columns of panes (default $COLS).
  -s, --session NAME  Session to create/attach when run outside tmux (default $SESSION).
  -h, --help          Show this help message.

Example: ./src/BANYU_ROBOTX/tools/scripts/run/tmux_panes.sh -r 4 -c 4   # 16 panes
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    -r|--rows) ROWS="$2"; shift 2 ;;
    -c|--cols) COLS="$2"; shift 2 ;;
    -s|--session) SESSION="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
  esac
done

for n in "$ROWS" "$COLS"; do
  [[ "$n" =~ ^[1-9][0-9]*$ ]] || { echo "--rows/--cols must be positive integers" >&2; exit 1; }
done
command -v tmux > /dev/null || { echo "tmux is not installed" >&2; exit 1; }

if [ -n "$TMUX" ]; then
  WINDOW=$(tmux new-window -P -F "#{window_id}" -c "$START_DIR")
else
  tmux has-session -t "=$SESSION" 2> /dev/null || tmux new-session -d -s "$SESSION" -c "$START_DIR"
  WINDOW=$(tmux new-window -P -F "#{window_id}" -t "=$SESSION:" -c "$START_DIR")
fi

# Split off the remaining (n-1)/n each time, so N splits give N equal parts
split_even() {  # pane, direction flag (-v rows, -h cols), parts; prints the new pane ids
  local pane="$1" dir="$2" parts="$3" k
  for ((k = parts; k > 1; k--)); do
    pane=$(tmux split-window "$dir" -l "$(((k - 1) * 100 / k))%" -t "$pane" -c "$START_DIR" -P -F "#{pane_id}")
    echo "$pane"
  done
}

FIRST=$(tmux display -p -t "$WINDOW" "#{pane_id}")
ROW_PANES=("$FIRST" $(split_even "$FIRST" -v "$ROWS"))
for pane in "${ROW_PANES[@]}"; do
  split_even "$pane" -h "$COLS" > /dev/null
done
tmux select-pane -t "$FIRST"

[ -z "$TMUX" ] && exec tmux attach -t "=$SESSION"
exit 0
