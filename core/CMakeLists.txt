cmake_minimum_required(VERSION 3.16)
project(drone_survivability_core C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

add_compile_options(-Wall -Wextra)
# The vendored MAVLink headers trigger -Waddress-of-packed-member throughout;
# that's generated third-party code, not ours, so it's silenced rather than fixed.
add_compile_options(-Wno-address-of-packed-member)

# MAVLink v2 C headers (header-only). Expected at:
#   core/third_party/mavlink/c_library_v2
# See core/third_party/README.md for how to fetch it.
include_directories(
  include
  third_party/mavlink/c_library_v2
)

file(GLOB_RECURSE CORE_LIB_SOURCES CONFIGURE_DEPENDS src/*.c)
list(REMOVE_ITEM CORE_LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c)

add_library(core_lib STATIC ${CORE_LIB_SOURCES})
target_link_libraries(core_lib PUBLIC m)

add_executable(core_main src/main.c)
target_link_libraries(core_main PRIVATE core_lib)

add_subdirectory(tools)
add_subdirectory(sitl)

enable_testing()
add_subdirectory(tests)
