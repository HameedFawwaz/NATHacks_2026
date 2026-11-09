# About the Project

## Inspiration

Drone GPS spoofing keeps showing up outside of military contexts — delivery
drones, survey flights, even consumer GPS near conflict zones nudged miles off
course by signals that never claimed to be fake. What struck us wasn't that
spoofing is possible; it's how little a vehicle's own instruments protest
while it happens. A spoofed receiver doesn't throw an error. It reports a
clean, strong, perfectly plausible fix — just the wrong one. The failure mode
we wanted to attack wasn't "detect something is broken," it was "detect
something that insists it isn't." The same question applies to the link back
to the operator: a jammed radio and a radio that's simply out of range look
identical from one side of the connection, and a drone that can't tell them
apart will make the wrong call in both directions.

## What it does

`core` is a small C library that watches a vehicle's live MAVLink telemetry
and decides, continuously, how much to trust its navigation and its control
link. Six independent checks score incoming GPS against two dead-reckoned
estimates, the flight controller's own EKF, timestamp behavior, and GPS
quality, fused by taking the minimum rather than an average, so one
unambiguous red flag survives being outvoted by five green ones that simply
haven't noticed yet. A separate lost-link detector reads operator heartbeats
and radio RSSI to tell a clean loss of signal apart from active jamming — a
distinction that matters, because autonomy's correct response to each is
different. Autonomy turns both trust scores into an actual decision: switch
the EKF to a GPS-free source, hold position, return home, or land, commanded
over real MAVLink, not just logged. A browser dashboard shows an operator
what's trusted and why in plain language, can inject every one of those
attacks into a live ArduPilot SITL simulation to demonstrate the response,
and can arm, fly, and recover a test flight on its own. The same
detection-and-autonomy loop also runs as firmware on a bare ESP32, acting as
a real (if simulation-fed) companion computer wired to a flight controller's
telemetry port, not just a process on a laptop.

## How we built it

`core` and the dashboard are deliberately separate projects with no shared
build system — C with CMake for one, Go compiled to WebAssembly for the
other — talking only over MAVLink, the same protocol either would use against
real hardware. We built and validated everything against real ArduPilot
SITL from the start rather than hand-typing scores into the decision layer:
a scripted end-to-end harness flies real simulated missions, injects a fast
ramp, a slow ramp, and a believable random-route spoof (and separately, link
jamming and loss) against the real detector, and records what actually
happened — time to detect, lowest trust reached, true drift, which autonomy
behavior fired — rather than asserting it. The same harness runs against
either `core_main` on the laptop or the ESP32 over serial, so the two are
held to the same measured results instead of two different stories. The
dashboard's "threat simulator" drives the identical SITL-level fault
injection a terminal session would, just from a button, so a demo is never
showing browser-generated fake trust scores dressed up as a real response.

## What we learned

The most useful bugs we found were never in the detection math — they were
in what the detector was actually being fed. `core_main` originally only
asked the autopilot for IMU and barometer data, inheriting GPS and position
streams for free from whatever ground station happened to already be
requesting them. Wired directly to a flight controller with nothing else
listening — the exact deployment this project is for — that assumption
silently failed: no GPS, no position, ever, with no error message. We found
that only by running the detector genuinely standalone against SITL instead
of behind the GCS tooling that had quietly been making it work all along.
A second bug was subtler: a timestamp comparison that mixed two clocks
paced by different, independently-arriving message streams underflowed to a
near-infinite "age" for a single tick whenever a GPS sample happened to
decode slightly out of order relative to an IMU sample in the same network
read — crashing trust to zero, once a second, during a completely clean
flight. Neither bug was visible in unit tests built around synthetic,
perfectly-ordered telemetry; both only showed up against a real simulator
with real network timing. We came away far more skeptical of "it passed
behind the GCS we always test with" as a stand-in for "it works," and far
more inclined to ask what a detector is actually receiving before asking
whether its logic is right.

## Challenges we faced

Recovery turned out to be harder to get right than detection. It's easy to
make a detector that never misses an attack by making it also never trust
anything again afterward; it's much harder to let honest GPS back in once a
spoof clears without also reopening the door to a patient attacker who just
waits out the recovery window. We ended up requiring sustained agreement
between GPS and an independent, explicitly-GPS-free backup source — not just
a snapshot match, but agreement that holds steady for several seconds and
doesn't slide along with a continuing slow drift — before trust is allowed
to climb back up automatically, and we wrote down exactly where that
guarantee stops holding (a backup that's itself drifted too far, an attacker
patient enough to sit just inside both bounds) rather than claiming more
confidence than the mechanism earns. Telling jamming apart from a vehicle
that's simply out of range was its own problem, since both look like silence
from the companion computer's side of the link; we ended up needing a second
signal (radio RSSI) that the autopilot doesn't forward by default. And
getting the same core logic to run unmodified on both a laptop and a
resource-constrained microcontroller — one MAVLink transport becoming three
(UDP, POSIX serial, and the ESP32's own UART), with no filesystem and a
much tighter timing budget on the board — meant separating "the detection
and decision loop" cleanly from "how bytes reach it" earlier than we
otherwise would have, which paid for itself the moment real hardware entered
the picture.
