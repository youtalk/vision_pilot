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
echo "TEST_PASS test-run-carla-server"
