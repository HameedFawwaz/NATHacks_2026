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
API). `nav/`, `link/`, `autonomy/`, `crypto/` are still empty, scoped to their own issues.

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
