#!/usr/bin/env bash
# run-carla-server.sh must refuse a package without the sha stamp or the launcher.
set -u
here=$(cd "$(dirname "$0")" && pwd); s="$here/run-carla-server.sh"
fail() { echo "TEST_FAIL test-run-carla-server reason=$1"; exit 1; }
[ -x "$s" ] || fail script_missing
tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
out=$(bash "$s" "$tmp" 2>&1) && fail empty_pkg_accepted
grep -q 'CARLA_SERVER_FAIL reason=no_sha_stamp' <<<"$out" || fail sha_reason
echo abc123 > "$tmp/ces2027-package-sha.txt"
out=$(bash "$s" "$tmp" 2>&1) && fail no_launcher_accepted
grep -q 'CARLA_SERVER_FAIL reason=no_launcher' <<<"$out" || fail launcher_reason

# ros2_port_bound replaced a log grep that a Shipping build made unsatisfiable.
# Drive it against a stub `ss` so the logic is checked without a real server.
eval "$(grep '^ros2_port_bound()' "$s")"
mkdir -p "$tmp/bin"
cat > "$tmp/bin/ss" <<'STUB'
#!/bin/sh
echo 'UNCONN 0 0 0.0.0.0:7650  0.0.0.0:* users:(("CarlaUnreal-Lin",pid=1,fd=97))'
echo 'UNCONN 0 0 0.0.0.0:7400  0.0.0.0:* users:(("other-daemon",pid=2,fd=3))'
STUB
chmod +x "$tmp/bin/ss"
PATH="$tmp/bin:$PATH"
ros2_port_bound 7650 || fail port_bound_missed
ros2_port_bound 7400 && fail foreign_process_accepted
ros2_port_bound 765 && fail port_prefix_matched
echo "TEST_PASS test-run-carla-server"
