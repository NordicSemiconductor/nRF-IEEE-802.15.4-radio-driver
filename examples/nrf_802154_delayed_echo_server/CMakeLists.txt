cmake_minimum_required(VERSION 3.13.1)

set(TARGET_SOC "nrf52840" CACHE STRING "Specify the target platform")

# In-tree examples assume that the driver root is two directories up the tree.
add_subdirectory(../.. driver)
project(nrf-802154-delayed-echo-client C ASM)

add_executable(nrf_802154_delayed_echo_server src/nrf_802154_delayed_echo_server.c)
target_link_libraries(nrf_802154_delayed_echo_server
  PRIVATE
    nrf_802154_platform
    nrf_802154_radio_driver
    mpsl_utils
)
set_target_properties(nrf_802154_delayed_echo_server PROPERTIES SUFFIX .elf)

add_hex_target(nrf_802154_delayed_echo_server)
add_flash_target(nrf_802154_delayed_echo_server)
