# core

C implementation of the drone survivability layer: GNSS spoofing detection, lost-link
mitigation, autonomous failover decisions, and operator-link encryption. Runs on the
companion computer and talks to the flight controller over MAVLink.

## Layout

```
core/
  include/core/      public headers, one subdir per module
    mavlink/          MAVLink helper functions / connection handling (shared by all modules)
    nav/              GNSS spoofing detection
    link/             lost-link detection and failover
    autonomy/         decision layer consuming trust scores from nav/link/hardware
    crypto/           operator<->drone link encryption/signing
    common/           shared types (e.g. trust score struct) and utilities
  src/                implementation, mirrors include/core/
  third_party/        vendored deps (MAVLink C headers, see third_party/README.md)
  tests/              automated tests (no external dependencies, e.g. loopback UDP)
  tools/              manual dev executables that need something external running,
                      e.g. mavlink_monitor against a live ArduPilot SITL instance
  sitl/               SITL spoof/jam injection scripts and scenario configs
```

## Build

```
git submodule update --init --recursive   # first time, pulls in MAVLink headers
cmake -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

`mavlink/` and `common/` are implemented (connection handling, telemetry decode, send
helpers, shared trust-score struct — see `include/core/mavlink/mavlink.h` for the public
API). `nav/` (GNSS spoofing detection), `link/` (lost-link detection), `autonomy/` (decision
layer), and `crypto/` (link signing) are also implemented — see below.

## GNSS spoofing detection (`nav/`)

Maintains an independent position estimate (a Kalman filter fusing accelerometer, gyro,
and barometer — never GPS) and cross-checks incoming GPS against it. Six checks (jump,
short- and long-horizon KF divergence, raw-GPS-vs-EKF divergence, timestamp/replay, HDOP
abnormality) each produce a sub-score in `[0,1]`, fused via `min()` so one unambiguous red
flag can't be averaged away, then published as `nav_trust` with asymmetric slew limiting
(drops instantly, recovers slowly). See `include/core/nav/nav.h` for the public API,
`include/core/nav/gnss_anomaly.h` for the six checks, and `core/tests/test_gnss_anomaly.c`
/ `test_nav_kf.c` for fully synthetic (no SITL needed) scenario and filter-math tests.

`core_main` is the real integration loop: connect → detect → publish, against a live
flight controller.

**Known limitations**:

- The long-horizon KF divergence check was calibrated against real flight
  (`core/sitl/fly_square.py`, not just synthetic/stationary data) and can still
  occasionally false-positive for a few seconds during normal maneuvering. Root cause:
  this filter has no accel/gyro bias calibration, and in testing only received ~4Hz IMU
  samples (SITL/MAVProxy didn't honor our `MAV_CMD_SET_MESSAGE_INTERVAL` request to speed
  `SCALED_IMU2` up to 50Hz — see `core_mavlink_request_message_interval` in
  `core/include/core/mavlink/mavlink_send.h`), so integration error compounds faster than a
  properly-sampled filter would see. Follow-up: figure out why the rate request isn't
  taking effect (possibly a MAVProxy pass-through quirk, worth checking against a direct
  SITL connection without MAVProxy in between), or add basic bias estimation.
- **Minimum detectable sustained-drift rate**: re-anchoring to GPS is now gated on the
  filter's own divergence score staying at ceiling (see `gnss_anomaly.c`), so a slow spoof
  no longer gets silently absorbed every re-anchor interval the way it used to — but the
  detectable rate is still bounded by `warn_m / horizon_ms`. With the current long-horizon
  defaults (10m / 8s) that's **~1.25 m/s**: a spoof patient enough to drift slower than
  that, held indefinitely, remains undetectable by the KF-based checks alone. (The
  raw-GPS-vs-EKF check can still catch it — see the caveat on that check above — but only
  if the attacker fooled our telemetry relay without also fooling the flight controller's
  own GPS reception.) Closing this gap for good needs real IMU bias estimation, not just
  threshold tuning.

## Autonomy

`autonomy/` turns trust scores into a response. The table of behaviors is at the top of
`include/core/autonomy/autonomy.h`. Each transition goes to stderr, to a STATUSTEXT, and
to a CSV log.

The nav and link detectors aren't merged yet, so `tools/autonomy_runner` stands in for
them: it reads scores from stdin (`nav 0.1`, `link 0`, `hw 1`, `crypto 1`) and otherwise
runs the real decision loop. Once the detectors exist they should write through
`core_trust_score_set_component()` and call `core_autonomy_tick()` from the main loop.

`sitl/run_autonomy_scenario.py` runs this end to end against ArduPilot SITL. It flies a
copter, injects an attack, and checks the vehicle's response:

```
python3 sitl/run_autonomy_scenario.py --ardupilot /path/to/ardupilot --runner build/tools/autonomy_runner gnss
python3 sitl/run_autonomy_scenario.py --ardupilot /path/to/ardupilot --runner build/tools/autonomy_runner link
python3 sitl/run_autonomy_scenario.py --ardupilot /path/to/ardupilot --runner build/tools/autonomy_runner multi
```

`--ardupilot` is a checkout with `./waf configure --board sitl && ./waf copter` already
run. The script needs `pymavlink`. GNSS failover needs a non-GPS source set on the vehicle; see
`sitl/autonomy_failover.parm` for the optical-flow setup the scenario uses.

Building Copter-4.5 SITL on Apple Silicon needed three workarounds: `setuptools<81` in
the venv (`dronecan` imports `pkg_resources`, and without it the build hangs instead of
failing), `CC=/usr/bin/clang CXX=/usr/bin/clang++`, and `LDFLAGS=-Wl,-no_fixup_chains`
at `./waf configure`.

## Lost-link detection (`link/`)

`link/link_monitor` judges the operator's control link and publishes `link_trust`, which
autonomy turns into a hold and then a return home. `core_main` runs it every tick. See
`include/core/link/link_monitor.h` for the states and the full reasoning.

The companion computer isn't on the operator's link, so it measures it indirectly:

- **Operator heartbeats.** ArduPilot forwards the ground station's heartbeats to every
  other link. The monitor counts the fraction missing over 10 s and how long the link has
  been silent. The operator is `SYSID_MYGCS`'s default, sysid 255; change
  `operator_gcs_sysid` in the connection config if yours differs.
- **Radio data, to tell jamming from a clean loss.** ArduPilot never forwards
  `RADIO_STATUS`, but with `RSSI_TYPE=5` it puts the telemetry radio's RSSI in
  `RC_CHANNELS`, which `core_main` requests. Silent while the radio still hears strong RF
  is read as jamming; silent with the signal gone is a loss. If the radio is wired to the
  companion computer instead, its `RADIO_STATUS` noise floor is used, which is the
  stronger signal.

| State | When | `link_trust` |
|---|---|---|
| `UNKNOWN` | no operator heard yet, or the autopilot itself is silent | not reported |
| `NOMINAL` | heartbeats arriving as expected | about 1 |
| `DEGRADED` | heartbeats missing, weak signal, or interference while connected | below 0.7 |
| `LOST` | silent for 6 s, no sign of jamming | 0 |
| `JAMMED` | silent for 6 s while the radio shows interference | 0 |

Each state change goes out as a `LINK <STATE>: <reason>` STATUSTEXT, and the state and
loss are published as `LNK_STATE` and `LNK_LOSS` alongside `TRUST_LNK`.

Brief dropouts don't trip it: loss is measured over 10 s, the first 3 s of silence cost
nothing, trust drops at once but climbs back at 0.1/s, and the state only steps back down
after holding for 3 s. Steady loss doesn't make it bounce either: the link becomes
`DEGRADED` below 0.7 trust but is only `NOMINAL` again at 0.85 with a clean 30 s record. So
after an outage, trust (which autonomy acts on) recovers in about 10 s while the label
stays `DEGRADED` for about 30 s.

Limits:

- **Jamming versus loss is a heuristic.** With RSSI only, the assumption is that a jammed
  radio still reports strong RF energy. That holds in the SITL scenario because the
  scenario is built on it; check it against the real radio.
- **A GCS that slows its heartbeat mid-flight reads as loss.** Halving the rate looks
  exactly like losing half the heartbeats, so it's treated as loss rather than learned
  away. The rate is learned at startup and after an autopilot reconnect, and a GCS that
  speeds up is re-learned at once, since extra heartbeats can't be loss.
- **A very slow rise in noise can be absorbed.** The noise floor follows a healthy link
  with a time constant of about a minute, so a jammer that ramps up over tens of seconds
  is caught, but one that creeps up over many minutes may not be.
- **`RC_CHANNELS.rssi` of 0 is read as "no radio"**, which is what ArduPilot sends with
  `RSSI_TYPE=5` and no radio reporting. Other `RSSI_TYPE`s report the RC receiver's RSSI
  instead, which the monitor can't tell apart; use 5.
- **Forwarding needs an ordinary link.** ArduPilot only forwards heartbeats to active links
  that aren't marked private (`MAV_OPTIONS`/`SERIALn_OPTIONS`).
- **No operator ever heard means no verdict.** A drone that takes off without its ground
  station connected isn't monitored, the same as autonomy's rule for producers that have
  never reported.
- **No backup link.** The issue allows switching to one, but the hardware has none, so the
  response is left to autonomy.
- **`core_main` doesn't run autonomy yet**, so in SITL the vehicle doesn't act on the
  verdict. `tools/autonomy_runner` is still the way to exercise autonomy.

### Scripted jamming in SITL

`sitl/link_jam_proxy.py` stands in for the telemetry radio: it relays the operator's link,
drops packets, and reports `RADIO_STATUS` to the autopilot with chosen RSSI and noise.
`sitl/run_link_scenario.py` runs it against SITL and `core_main` and checks each case:

```
python3 sitl/run_link_scenario.py --ardupilot /path/to/ardupilot --core-main build/core_main
```

It checks that a steady link stays `NOMINAL`, that 2 s dropouts never reach `LOST` or
untrusted, that 30% loss is `DEGRADED`, that jamming is `JAMMED` and not `LOST`, that a
clean loss is `LOST` and not `JAMMED`, and that each recovers without bouncing. The vehicle
stays on the ground. Needs `pymavlink`.

To try it by hand with MAVProxy as the operator, start the proxy and type a profile
(`nominal`, `degraded`, `jam`, `lost`) to switch:

```
python3 sitl/link_jam_proxy.py
arducopter ... --defaults <copter.parm>,sitl/link_jam.parm \
  --serial1=udpclient:127.0.0.1:14551 --serial2=udpclient:127.0.0.1:14560
./build/core_main 14551
mavproxy.py --master=udpout:127.0.0.1:14562
```

## Testing against ArduPilot SITL

`tools/mavlink_monitor` is a standalone connectivity check: it opens a MAVLink
connection and prints the heartbeat plus whatever other telemetry has been decoded
(GPS, position, attitude, radio status) each time a new heartbeat arrives.

1. Start ArduPilot SITL with an extra MAVLink output pointed at this tool
   (14550 is conventionally already used by MAVProxy's own GCS link, so add a
   second `--out` for the companion computer):
   ```
   sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:14551
   ```
2. Run the monitor against that same port (14551 is the default if omitted):
   ```
   ./build/tools/mavlink_monitor 14551
   ```
   You should see a `[HEARTBEAT]` line (plus GPS/position/attitude once SITL has a
   fix) roughly once per second. Ctrl+C to stop.

### Running the real detector, and injecting spoofing scenarios

`core_main [port]` (default 14551) is the real GNSS spoofing detector: point it at SITL
the same way as `mavlink_monitor` above, and it prints `nav_trust` plus each sub-score
once a second. With no attack running, `nav_trust` should stay at/near 1.0.

To inject a scripted attack on top of a live SITL instance, run
`core/sitl/gnss_spoof_proxy` between SITL and `core_main` — it relays everything
transparently except `GPS_RAW_INT`, which it mutates per `--scenario`:

```
sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:14551
./build/sitl/gnss_spoof_proxy --scenario=jump --out-port=14552      # or drift / replay
./build/core_main 14552
```

- `--scenario=jump` (`--jump-at-sec`, `--jump-offset-m`, `--jump-heading-deg`): one
  fixed-offset teleport, held afterward. `nav_trust` should drop to 0 immediately.
- `--scenario=drift` (`--drift-rate-mps`, `--drift-heading-deg`, `--drift-start-sec`):
  self-consistent position+velocity drift. `nav_trust` decays smoothly as divergence
  accumulates.
- `--scenario=replay` (`--replay-at-sec`, `--replay-buffer-sec`, `--replay-mode`):
  loops a buffered window of real past fixes. `looped-live-timestamp` re-stamps time to
  look fresh (caught by KF divergence - needs the vehicle actually moving to show a
  signal); `stale-timestamp` leaves the original timestamps in place (caught by the
  replay/monotonicity check once the loop wraps - a smaller `--replay-buffer-sec` makes
  this wrap, and the detection, more frequent/visible in a live demo).

In every case, watch for the EKF-divergence sub-score (`ekf=`) - since the proxy only
mutates `GPS_RAW_INT`, the flight controller's own `GLOBAL_POSITION_INT` keeps reflecting
ground truth, so the EKF-divergence check reliably fires in this specific harness. **This
is a property of the harness, not a durability guarantee**: a real spoof that fools the
GPS receiver itself would make `GLOBAL_POSITION_INT` agree with the spoofed
`GPS_RAW_INT` too, and the EKF-divergence check would show nothing — in that case
detection rests entirely on the IMU-based KF checks (`dr=`/`drift=`), which never ingest
GPS at all.

### Flying the vehicle for a real (non-stationary) test

A stationary SITL instance doesn't exercise the IMU-based filter or the KF-divergence
checks (nothing is moving for them to track) — some scenarios above only show a real
signal with the vehicle actually flying. `core/sitl/fly_square.py` (Python + pymavlink)
arms, takes off, and flies an autonomous square via GUIDED-mode
`SET_POSITION_TARGET_LOCAL_NED` commands:

```
sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14553
python3 core/sitl/fly_square.py                     # defaults to udpin:0.0.0.0:14553
```

It talks directly to the vehicle (arm/mode/takeoff/position commands), separately from
`core_main`/`gnss_spoof_proxy`, which only observe telemetry — run it independently,
before or during a `core_main` session on a different port, with no conflict.

## Link signing (operator <-> drone authentication)

The link is protected with **MAVLink 2 message signing**: every frame carries a
SHA-256 signature over the key, header, payload, link id and a timestamp. With
signing on, `core_mavlink_conn_*` drops any inbound frame that is unsigned,
signed with the wrong key, tampered with, or replayed (old/repeated timestamp),
and signs everything it sends. Latency cost is a few microseconds per message
(`test_link_signing` prints the measurement).

Signing authenticates but does **not encrypt**: telemetry is still readable by
an eavesdropper, but commands and alerts cannot be forged or replayed.

### Provisioning a key

For MAVProxy/ArduPilot, derive the key from the **same nonempty ASCII
passphrase** used in MAVProxy's `signing setup <same passphrase>`. Only printable
ASCII bytes `0x21..0x7e` are accepted: no whitespace or control characters.
MAVProxy splits its command line on whitespace and uses only the first argument,
so a multiword passphrase would otherwise produce mismatched keys.

```sh
./build/tools/signing_keygen signing.key --from-passphrase
# Enter the passphrase on stdin (one line); do not pass it as a command argument.
```

This writes SHA-256 of the passphrase bytes, matching MAVProxy exactly. The
trailing line ending is stripped. Use a strong, unique passphrase and a private
terminal: stdin is not hidden when entered interactively.

Random keys are for links where **both ends are our own code**:

```sh
./build/tools/signing_keygen signing.key              # random key, hex file, mode 0600
./build/tools/signing_keygen other.key --print-hex     # also expose raw key on stdout
```

`--print-hex` is not a MAVProxy provisioning command: MAVProxy hashes its
passphrase argument and cannot take a raw hex key. It can be combined with
`--from-passphrase`, but avoid exposing key material unnecessarily.

`signing_keygen` refuses to overwrite an existing key. `*.key` is gitignored -
never commit a key. Copy the same file to the ground station and the companion
computer over a trusted channel (physical media or `scp`; not over the
unsigned link itself). Both ends must hold the same key.

### Enabling it in code

```c
core_mavlink_conn_config_t cfg = { /* ... as before ... */ };
/* CORE_SIGNING_KEY_FILE=signing.key; a bad configured key must fail closed. */
core_signing_env_result_t result = core_signing_key_load_from_env(cfg.signing_key);
if (result == CORE_SIGNING_ENV_ERROR) {
    return EXIT_FAILURE;  /* in main; include <stdlib.h> */
}
cfg.signing_enabled = (result == CORE_SIGNING_ENV_LOADED); /* UNSET leaves it off */
cfg.signing_link_id = 0;   /* unique per link if you run several */
```

or load a path directly with `core_signing_key_load_file()`. If signing is not
enabled (default for a zero-initialised config), behaviour is unchanged.

Rejections are counted in `core_telemetry_t.link_security`
(`rejected_unsigned`, `rejected_bad_sig`, `rejected_replay`, `last_reject_ms`)
so the dashboard and autonomy layer can treat a rising count as an attack
indicator.

### Rotation

1. Generate a new key with `signing_keygen` (use `--from-passphrase` for MAVProxy).
2. Stop the drone link, copy the new key to both ends, restart both.
3. Delete the old key file.

There is no live rekeying: both ends must switch together. Rotate after any
suspected key exposure and before each event/flight day.

### Flight-controller side

The autopilot must be given the same key so it accepts our signed commands
(and signs its telemetry). ArduPilot stores it via the `SETUP_SIGNING` MAVLink
message; MAVProxy's `signing setup <passphrase>` command does this, deriving
the 32-byte key as the SHA-256 of the ASCII passphrase. Use the passphrase route
above, and provision over a trusted local link, not an exposed unsigned link.
**Not yet verified against our SITL setup**: confirm the handshake and signature
verification before flight; do not silently fall back to an unsigned autopilot link.

## Dashboard

The operator dashboard is a separate project/framework and lives outside `core/`
(see `../dashboard`). It consumes trust scores and telemetry published by this
codebase over MAVLink; it does not share a build system with it.
