#!/usr/bin/env bash
# Inject the demo fault at a chosen instant, on a board that runs the demo
# under the S-CORE launch manager (LM). Four routes:
#   kill     SIGKILL VisionPilot (noticed by the LM and by heartbeat
#            staleness, 700 ms budget)
#   slow     SIGUSR1 VisionPilot, which then runs every frame 200 ms late; the
#            S-CORE health monitor fails it and the LM stops it (1300 ms budget)
#   freeze   stop the camera by pausing CARLA's sensor publisher: SIGSTOP the server process is too coarse,
#            so freeze = kill here as well until CARLA grows a per-sensor pause
#   channel  latch the fault on the rpmsg-si channel from the board's listener
#            (latches directly, no staleness detection, 200 ms budget)
#   si_fault.sh <kill|slow|freeze|channel> [<epoch-seconds-to-fire-at>]
#   si_fault.sh reset
# reset is openadkit's booth reset: it clears the CR52 latch and starts a new
# VisionPilot, then waits for its ready file. Prints SI_RESET_DONE.
# Prints SI_FAULT_INJECTED mode=<m> t=<epoch> fired=<epoch>. t= is the instant
# the gate measures from, which is the requested one, so it means the same
# thing here as in run-d6.sh's stand-in mode; fired= is when the injection
# actually landed, kept for diagnosis.
set -uo pipefail
mode="${1:-}"; at="${2:-}"
BOARD="${X5H_BOARD:-root@192.168.0.20}"
# A cold ssh handshake to the board costs 150-400 ms while the channel
# route's whole budget is 200 ms, so opening the connection at the fault
# instant made correct firmware read as cmd_late. Pay the handshake before
# the wait and let the injection reuse the master connection. ControlPersist
# has to outlive the wait; 600 s covers any DRIVE_S in use.
CTL="${TMPDIR:-/tmp}/si-fault-%C"
SSH=(ssh -o ControlMaster=auto -o ControlPath="$CTL" -o ControlPersist=600 -o ConnectTimeout=10)
case "$mode" in kill|slow|freeze|channel|reset) ;; *) echo "SI_FAULT_FAIL reason=bad_args"; exit 1 ;; esac
"${SSH[@]}" "$BOARD" true || { echo "SI_FAULT_FAIL reason=ssh_failed"; exit 1; }
if [ "$mode" = reset ]; then
  # Stop first, so a fallback the LM starts during the stop cannot latch the
  # CR52 after the clear. The LM removes the ready file before it starts
  # VisionPilot, which writes it again at its first frame, so this needs
  # camera frames: the LM falls back after 60 s without them.
  "${SSH[@]}" "$BOARD" 'systemctl stop score-lm.service && systemctl kill -s USR2 x5h-si-link.service && systemctl start score-lm.service' \
    || { echo "SI_FAULT_FAIL reason=reset_failed"; exit 1; }
  for _ in $(seq 1 60); do
    "${SSH[@]}" "$BOARD" 'test -e /run/score/vp.ready' && { echo "SI_RESET_DONE"; exit 0; }
    sleep 1
  done
  echo "SI_FAULT_FAIL reason=vp_not_ready"; exit 1
fi
if [ -n "$at" ]; then
  now=$(date +%s.%N); sleep "$(awk -v a="$at" -v n="$now" 'BEGIN { d = a - n; print (d > 0) ? d : 0 }')"
fi
case "$mode" in
  # The same commands as openadkit's booth `fault kill` and `fault slow`. The
  # LM never restarts VisionPilot, so nothing has to be stopped after it.
  kill|freeze) "${SSH[@]}" "$BOARD" 'pkill -KILL -x VisionPilot' ;;
  slow)        "${SSH[@]}" "$BOARD" 'pkill -USR1 -x VisionPilot' ;;
  channel)     "${SSH[@]}" "$BOARD" 'systemctl kill -s USR1 x5h-si-link.service' ;;
esac || { echo "SI_FAULT_FAIL reason=ssh_failed"; exit 1; }
fired=$(date +%s.%N)
echo "SI_FAULT_INJECTED mode=$mode t=${at:-$fired} fired=$fired"
