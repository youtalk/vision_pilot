#!/usr/bin/env bash
# `loop_run.sh run` must end, and must not leave the scenario running, when
# the observer ends. Fakes docker and the Python, so no CARLA is needed.
set -u
here=$(cd "$(dirname "$0")" && pwd)
t=$(mktemp -d); trap 'rm -rf "$t"' EXIT
cat > "$t/docker" <<'EOT'
#!/usr/bin/env bash
# A long `docker logs` output, as a real bridge log has after the hero line.
[ "$1" = logs ] && { echo "hero_actor found"; exec seq 1 200000; }
exit 0
EOT
cat > "$t/py" <<'EOT'
#!/usr/bin/env bash
case "$*" in
  *config_carla.py*) echo "Running..."; exec -a fake-pov-scenario sleep 300 ;;
  *observer.py*) exit 0 ;;
  *) cat >/dev/null; echo "cleared 0" ;;
esac
EOT
chmod +x "$t/docker" "$t/py"
PATH="$t:$PATH" PY="$t/py" LOG_DIR="$t/logs" timeout 30 bash "$here/loop_run.sh" run test; rc=$?
sleep 0.5
if pgrep -f fake-pov-scenario >/dev/null; then pkill -f fake-pov-scenario; echo "FAIL scenario still running (rc=$rc)"; exit 1; fi
[ "$rc" -eq 0 ] || { echo "FAIL rc=$rc"; exit 1; }
echo LOOP_TEST_PASS
