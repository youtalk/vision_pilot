#!/usr/bin/env bash
# Inject the demo fault at a chosen instant. Three routes, all seen by the
# CR52 watchdog the same way (spec section 5):
#   kill     stop VisionPilot's unit on board 1  (preferred; noticed via
#            heartbeat staleness, 700 ms budget)
#   freeze   stop the camera by pausing CARLA's sensor publisher: SIGSTOP the server process is too coarse,
#            so freeze = kill here as well until CARLA grows a per-sensor pause
#   channel  latch the fault on the rpmsg-si channel from the board's listener
#            (latches directly, no staleness detection, 200 ms budget)
#   si_fault.sh <kill|freeze|channel> [<epoch-seconds-to-fire-at>]
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
case "$mode" in kill|freeze|channel) ;; *) echo "SI_FAULT_FAIL reason=bad_args"; exit 1 ;; esac
"${SSH[@]}" "$BOARD" true || { echo "SI_FAULT_FAIL reason=ssh_failed"; exit 1; }
if [ -n "$at" ]; then
  now=$(date +%s.%N); sleep "$(awk -v a="$at" -v n="$now" 'BEGIN { d = a - n; print (d > 0) ? d : 0 }')"
fi
case "$mode" in
  kill|freeze) "${SSH[@]}" "$BOARD" 'systemctl stop x5h-vp.service' ;;
  channel)     "${SSH[@]}" "$BOARD" 'systemctl kill -s USR1 x5h-si-link.service' ;;
esac || { echo "SI_FAULT_FAIL reason=ssh_failed"; exit 1; }
fired=$(date +%s.%N)
echo "SI_FAULT_INJECTED mode=$mode t=${at:-$fired} fired=$fired"
