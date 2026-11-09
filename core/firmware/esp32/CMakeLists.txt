# ESP-IDF project: core (GNSS spoofing + lost-link detection and autonomy) on
# an ESP32, speaking MAVLink over the board's USB serial port. Wired to SITL's
# (or a flight controller's) telemetry port, it replaces core_main as the
# companion computer. See core/README.md, "Running core on an ESP32".
#
#   . ~/esp/esp-idf/export.sh
#   idf.py -C core/firmware/esp32 -p /dev/cu.usbserial-0001 flash
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(core_esp32 C)
