# Not linked against core_lib: like test_mavlink_loopback.c, this operates
# directly on vendored MAVLink types at the wire-relay boundary, rather than
# consuming core/mavlink/mavlink.h as a module would.
add_executable(gnss_spoof_proxy gnss_spoof_proxy.c)
target_link_libraries(gnss_spoof_proxy PRIVATE m)
