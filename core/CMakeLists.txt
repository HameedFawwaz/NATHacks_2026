cmake_minimum_required(VERSION 3.16)
project(drone_survivability_core C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

add_compile_options(-Wall -Wextra)

# MAVLink v2 C headers (header-only). Expected at:
#   core/third_party/mavlink/c_library_v2
# See core/third_party/README.md for how to fetch it.
include_directories(
  include
  third_party/mavlink/c_library_v2
)

file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS src/*.c)

add_executable(core_main ${CORE_SOURCES})

enable_testing()
add_subdirectory(tests)
