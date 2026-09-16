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
# Prints SI_FAULT_INJECTED mode=<m> t=<epoch>.
set -uo pipefail
mode="${1:-}"; at="${2:-}"
BOARD="${X5H_BOARD:-root@192.168.0.20}"
case "$mode" in kill|freeze|channel) ;; *) echo "SI_FAULT_FAIL reason=bad_args"; exit 1 ;; esac
if [ -n "$at" ]; then
  now=$(date +%s.%N); sleep "$(awk -v a="$at" -v n="$now" 'BEGIN { d = a - n; print (d > 0) ? d : 0 }')"
fi
case "$mode" in
  kill|freeze) ssh "$BOARD" 'systemctl stop x5h-vp.service' ;;
  channel)     ssh "$BOARD" 'systemctl kill -s USR1 x5h-si-link.service' ;;
esac
echo "SI_FAULT_INJECTED mode=$mode t=$(date +%s.%N)"
