# Project Summary

Trust Issues is a drone survivability layer that keeps a vehicle flying — and
keeps its operator honestly informed — when something attacks its
navigation, its control link, or the link to its operator. GPS spoofing is a
solved-looking problem that isn't: a slow, patient drift doesn't trip a
sanity check the way a sudden jump does, and "the signal looks fine" is
exactly what a jammed radio link looks like too, for a while. `core` runs
six independent checks against incoming GPS — against two dead-reckoned
estimates, the flight controller's own EKF, timestamp behavior, and GPS
quality — fused so one clear red flag can't get averaged away by five green
ones. A matching lost-link detector tells a clean signal loss apart from
active jamming using radio RSSI the flight controller doesn't normally
expose. Autonomy turns both scores into an actual response — EKF failover
off GPS, a hold, a return home, or a landing — sent back as real MAVLink
commands, not a log line. A browser dashboard shows an operator what's
trusted and what isn't in plain language, lets them inject the same attacks
into a live ArduPilot simulation to watch the response happen, and can arm,
fly, and recover a test flight with no separate ground-control console. The
same detection-and-autonomy code also runs as firmware on a bare ESP32,
standing in for a real companion computer wired to a flight controller's
telemetry port — proof this isn't a simulation-only trick. Everything is
validated against real SITL flights, with exact detection times, drift, and
what still slips through, written down rather than hidden.
