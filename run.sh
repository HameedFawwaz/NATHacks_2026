#!/usr/bin/env bash
#
# Runs the whole stack for manual testing: ArduCopter SITL, core (the real GNSS
# detector + autonomy, on SITL's SERIAL1), and the dashboard in live mode (fed,
# with a console, by MAVProxy).
#
# THIS SCRIPT IS THE CANONICAL "HOW DO I RUN EVERYTHING" REFERENCE. If you
# change how core or the dashboard are built or run - binary names, flags,
# ports, Makefile targets, directory layout - update this script in the same
# change.
#
# Usage: ./run.sh [--instance N] [--core-port PORT] [--dash-port PORT]
#                  [--console-port PORT] [--web-addr HOST:PORT]
#
# Once everything is up, the dashboard's Fly tab can start a flight and inject
# faults (GPS jump, drift and jamming, a motor fault) through its threat
# simulator - see dashboard/README.md. This also drops you at MAVProxy's
# interactive console (a separate link from core's and the dashboard's, so it
# can't collide with either). From there you can fly manually and inject a GPS
# spoof by hand, e.g.:
#
#   param set SIM_GPS_GLITCH_X 0.001      # instant ~110 m jump
#
# For a slower, more realistic ramp (the kind the real detector takes longer
# to catch), see core/sitl/run_gnss_e2e_scenarios.py's RampSpoof - it drives
# the same parameter gradually rather than in one jump.
#
# Ctrl-D or `quit` at that console tears the whole stack down.

set -euo pipefail

# --- Configuration - override via environment or flags if your setup differs. ---
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/code/ardupilot}"
ARDUPILOT_VENV="${ARDUPILOT_VENV:-$HOME/venv-ardupilot}"
GO_BIN_DIR="${GO_BIN_DIR:-/usr/local/go/bin}"

INSTANCE=0
CORE_PORT=14551
DASH_PORT=14550
CONSOLE_PORT=14552
WEB_ADDR=127.0.0.1:8080

while [[ $# -gt 0 ]]; do
  case "$1" in
    --instance) INSTANCE="$2"; shift 2 ;;
    --core-port) CORE_PORT="$2"; shift 2 ;;
    --dash-port) DASH_PORT="$2"; shift 2 ;;
    --console-port) CONSOLE_PORT="$2"; shift 2 ;;
    --web-addr) WEB_ADDR="$2"; shift 2 ;;
    -h|--help) sed -n '2,28p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 1 ;;
  esac
done

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_DIR="$(mktemp -d /tmp/trust-issues-run.XXXXXX)"
echo "run directory (SITL state, logs): $RUN_DIR"

if [[ -f "$ARDUPILOT_VENV/bin/activate" ]]; then
  # shellcheck disable=SC1091
  source "$ARDUPILOT_VENV/bin/activate"
fi
[[ -d "$GO_BIN_DIR" ]] && export PATH="$PATH:$GO_BIN_DIR"
export PATH="$PATH:$ARDUPILOT_DIR/Tools/autotest"

SITL_PGID=""
CORE_PID=""
DASHBOARD_PID=""

cleanup() {
  echo
  echo "shutting down..."
  [[ -n "$DASHBOARD_PID" ]] && kill "$DASHBOARD_PID" 2>/dev/null || true
  [[ -n "$CORE_PID" ]] && kill "$CORE_PID" 2>/dev/null || true
  if [[ -n "$SITL_PGID" ]]; then
    kill -TERM "-$SITL_PGID" 2>/dev/null || true
  fi
  # Fallback net, in case anything detached from the group above (sim_vehicle.py
  # forks arducopter/MAVProxy/an xterm, and not all of them necessarily stay in it).
  pkill -f "arducopter.*-I${INSTANCE}( |$)" 2>/dev/null || true
  pkill -f "mavproxy\.py.*${DASH_PORT}.*${CONSOLE_PORT}" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# --- Build ---
echo "building core..."
cmake -B "$REPO_ROOT/core/build" -S "$REPO_ROOT/core" >/dev/null
cmake --build "$REPO_ROOT/core/build" -j"$(nproc)" >/dev/null

echo "building dashboard..."
( cd "$REPO_ROOT/dashboard" && make build >/dev/null )

# --- core: the real GNSS detector + autonomy. Started first so its port is bound
# before SITL's SERIAL1 starts sending to it. ---
echo "starting core_main on port $CORE_PORT..."
( cd "$RUN_DIR" && exec "$REPO_ROOT/core/build/core_main" "$CORE_PORT" ) > "$RUN_DIR/core_main.log" 2>&1 &
CORE_PID=$!
sleep 1

# --- SITL + MAVProxy. core gets SITL's SERIAL1 to itself, like a companion
# computer wired to a flight controller's telemetry port. It must not sit behind
# a MAVProxy --out: MAVProxy forwards what an --out sends only to the autopilot,
# and the autopilot never routes a message back out the link it came in on, so
# core's trust scores and autonomy STATUSTEXTs would never reach the dashboard.
# On its own serial port, ArduPilot forwards them to MAVProxy, which fans them out
# to every --out below. --no-extra-ports stops sim_vehicle.py adding its own
# 127.0.0.1:14550 output, which doubled every message to a dashboard on that port.
# The dashboard and this script's console each get a MAVProxy --out, so neither
# can collide with the other over a UDP port. ---
echo "starting SITL (instance $INSTANCE)..."
( cd "$RUN_DIR" && exec setsid sim_vehicle.py -v ArduCopter -I"$INSTANCE" --no-rebuild --no-extra-ports \
    -A "--serial1=udpclient:127.0.0.1:$CORE_PORT" \
    --out "udp:127.0.0.1:$DASH_PORT" \
    --out "udp:127.0.0.1:$CONSOLE_PORT" \
    -m "--daemon" ) \
    > "$RUN_DIR/sitl.log" 2>&1 &
SITL_PGID=$!

echo "waiting for SITL to come up..."
deadline=$((SECONDS + 90))
until grep -q "AP: ArduPilot Ready" "$RUN_DIR/sitl.log" 2>/dev/null; do
  if [[ $SECONDS -ge $deadline ]]; then
    echo "SITL didn't come up within 90s - check $RUN_DIR/sitl.log" >&2
    exit 1
  fi
  sleep 1
done

# --- dashboard, live mode ---
echo "starting dashboard on $WEB_ADDR, live mode..."
( cd "$REPO_ROOT/dashboard" && exec go run ./cmd/serve -addr "$WEB_ADDR" -mavlink "127.0.0.1:$DASH_PORT" ) \
    > "$RUN_DIR/dashboard.log" 2>&1 &
DASHBOARD_PID=$!

sleep 2
echo
echo "dashboard: http://$WEB_ADDR"
echo "logs:      $RUN_DIR"
echo
echo "dropping into MAVProxy's console on its own link (port $CONSOLE_PORT)."
echo "fault injection example:"
echo "  param set SIM_GPS_GLITCH_X 0.001"
echo "Ctrl-D or 'quit' here tears the whole stack down."
echo

cd "$RUN_DIR"
mavproxy.py --master "udp:127.0.0.1:$CONSOLE_PORT"
