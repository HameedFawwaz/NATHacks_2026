add_executable(test_trust_score test_trust_score.c)
target_link_libraries(test_trust_score PRIVATE core_lib)
add_test(NAME test_trust_score COMMAND test_trust_score)

add_executable(test_mavlink_loopback test_mavlink_loopback.c)
target_link_libraries(test_mavlink_loopback PRIVATE core_lib)
add_test(NAME test_mavlink_loopback COMMAND test_mavlink_loopback)

add_executable(test_nav_kf test_nav_kf.c)
target_link_libraries(test_nav_kf PRIVATE core_lib)
add_test(NAME test_nav_kf COMMAND test_nav_kf)

add_executable(test_gnss_anomaly test_gnss_anomaly.c)
target_link_libraries(test_gnss_anomaly PRIVATE core_lib)
add_test(NAME test_gnss_anomaly COMMAND test_gnss_anomaly)

add_executable(test_autonomy test_autonomy.c)
target_link_libraries(test_autonomy PRIVATE core_lib)
add_test(NAME test_autonomy COMMAND test_autonomy)

add_executable(test_link_signing test_link_signing.c)
target_link_libraries(test_link_signing PRIVATE core_lib)
add_test(NAME test_link_signing COMMAND test_link_signing)

add_executable(test_nav_estimate test_nav_estimate.c)
target_link_libraries(test_nav_estimate PRIVATE core_lib)
add_test(NAME test_nav_estimate COMMAND test_nav_estimate)
