cmake_minimum_required(VERSION 3.13.1)

add_library(cmsis INTERFACE)

target_include_directories(cmsis
  INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/cmsis/CMSIS/Include
)
