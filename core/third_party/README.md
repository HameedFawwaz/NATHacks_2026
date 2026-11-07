# third_party

Vendored dependencies for the core C codebase.

## MAVLink v2 C headers

The core relies on the generated MAVLink v2 C headers (header-only, no build step).
Fetch them with:

```
git submodule add https://github.com/mavlink/c_library_v2 third_party/mavlink/c_library_v2
git submodule update --init --recursive
```

`CMakeLists.txt` already expects the headers at `third_party/mavlink/c_library_v2`.
