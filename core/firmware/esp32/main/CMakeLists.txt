# Every core source except the host-only ones: main.c (core_main's UDP/tty
# wrapper), the POSIX socket/tty transports (*_posix.c) and the POSIX clock
# (common/time.c, replaced by esp32_time.c). The UART reaches core through the
# portable stream transport. Re-run `idf.py reconfigure` after adding a core
# source file (ESP-IDF evaluates this in script mode, where CONFIGURE_DEPENDS
# is not allowed).
get_filename_component(CORE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

file(GLOB_RECURSE CORE_SOURCES "${CORE_DIR}/src/*.c")
list(FILTER CORE_SOURCES EXCLUDE REGEX ".*/src/main\\.c$")
list(FILTER CORE_SOURCES EXCLUDE REGEX ".*_posix\\.c$")
list(FILTER CORE_SOURCES EXCLUDE REGEX ".*/src/common/time\\.c$")

idf_component_register(
  SRCS "app_main.c" "esp32_time.c" ${CORE_SOURCES}
  INCLUDE_DIRS "${CORE_DIR}/include" "${CORE_DIR}/third_party/mavlink/c_library_v2"
  REQUIRES esp_driver_uart esp_driver_gpio esp_timer
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE CORE_MAVLINK_POSIX_TRANSPORTS=0)
# Same as the host build: the vendored MAVLink headers trip these throughout.
# ESP-IDF also builds with -Werror; core's host build doesn't, so warnings that
# only this compiler raises in shared core code stay warnings here.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-address-of-packed-member
  -Wno-error=type-limits -Wno-error=format-truncation
  # MAVLink text fields are fixed-size and need no terminating NUL.
  -Wno-error=stringop-truncation)
