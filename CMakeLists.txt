cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(drawer_sensor)

target_sources(app PRIVATE
  src/main.cpp
  src/adxl362.cpp
  src/bh1749_light.cpp
  src/ble_nus.cpp
)
