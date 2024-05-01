cmake_minimum_required(VERSION 3.20.0)


list(APPEND DTS_ROOT
  ${CMAKE_CURRENT_SOURCE_DIR}/dts
  )
  
set(BOARD rpi_pico)
# set(CMAKE_BUILD_TYPE Debug)
find_package(Zephyr)
project(app)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})