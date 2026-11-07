# dashboard

Operator dashboard: map view (planned vs actual route) and hardware confidence
monitoring. Built as a separate project from `../core` — framework/stack TBD.

Consumes trust scores and telemetry published by `../core` over MAVLink (or a
bridge on top of it, e.g. MAVLink -> WebSocket/MQTT), not tied to its build system.
