# third_party

Vendored dependencies for the core C codebase.

## MAVLink v2 C headers

The core relies on the generated MAVLink v2 C headers (header-only, no build step),
vendored as a submodule at `third_party/mavlink/c_library_v2`
(https://github.com/mavlink/c_library_v2). On a fresh clone, fetch them with:

```
git submodule update --init --recursive
```

`CMakeLists.txt` already expects the headers at `third_party/mavlink/c_library_v2`.
