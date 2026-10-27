#!/usr/bin/env bash
# SIMULATION ONLY: stop every process of a Task 2 sim run. `ros2 launch` leaves banyu_mission, gz,
# mavros_node and ardusub alive after SIGINT, and a stale banyu_mission holds the Groot2 port and kills
# the next run. Everything started for the sim carries ROS_DOMAIN_ID=40 in its environment (the UUV domain).
me=$$
mark='ROS_DOMAIN_ID=40'
sig() {
  for d in /proc/[0-9]*; do
    pid=${d#/proc/}
    [ "$pid" = "$me" ] || [ "$pid" = "$PPID" ] && continue
    if cat "$d/environ" 2>/dev/null | tr '\0' '\n' | grep -qx "$mark"; then
      case "$(cat "$d/comm" 2>/dev/null)" in bash|sh|task2_sim_e2e.) continue;; esac
      kill "$1" "$pid" 2>/dev/null
    fi
  done
}
sig -INT
sleep 12
sig -9
pkill -x ardusub 2>/dev/null
pkill -x gz 2>/dev/null
exit 0
