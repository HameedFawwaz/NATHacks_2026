# Trust Issues

A drone survivability layer and mission-control dashboard. Detect spoofing, monitor
subsystem trust, and keep flying when the world stops being trustworthy.

## Structure

- `core/` — C implementation of the drone survivability layer (GNSS spoofing detection, lost-link mitigation, autonomy, link encryption). See `core/README.md`.
- `dashboard/` — operator dashboard, separate project/framework. See `dashboard/README.md`.
