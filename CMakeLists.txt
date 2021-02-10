cmake_minimum_required(VERSION 3.13.1)

#
# Configurable build parameters
#
set(TARGET_SOC ""  CACHE STRING "Specify the target SoC")
set(TARGET_ABI ""  CACHE STRING "Specify the float ABI required for linking the target.")

# Function returns a list of supported SoCs. This is done by means of retrieving the list of
# directories inside the binary SL directory. This implies an assumption that only the SoCs
# with in-tree SL libraries are supported.
function(list_supported_soc platform_list)
  set(result "")
  set(sl_platforms_dir "${CMAKE_CURRENT_SOURCE_DIR}/nrf_802154_sl/lib")
  file(GLOB platforms RELATIVE "${sl_platforms_dir}" "${sl_platforms_dir}/*")
  foreach(platform IN LISTS platforms)
    if(IS_DIRECTORY "${sl_platforms_dir}/${platform}")
      list(APPEND result "${platform}")
    endif()
  endforeach()
  set(${platform_list} "${result}" PARENT_SCOPE)
endfunction()

# Toolchain must be set before the `project` command.
if (NOT TARGET nrf_802154_platform)
  include(cmake/cross_compilation_cfg.cmake)
endif ()

project (nrf-802154-radio-driver C ASM)

set(NRF_802154_DRIVER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" CACHE INTERNAL "")
include(nrf_802154_driver_sources.cmake)

list_supported_soc(platforms)

# Check platform
if (OPENSOURCE_SL)
  # Intentionally empty
elseif ("${TARGET_SOC}" STREQUAL "")
  string (REPLACE ";" ", " plat_str "${platforms}")
  message(FATAL_ERROR "Target SoC is unspecified. TARGET_SOC must be set to one of the following values: ${plat_str}")
elseif (NOT TARGET_SOC IN_LIST platforms)
  string (REPLACE ";" ", " plat_str "${platforms}")
  message(FATAL_ERROR "Target SoC is not supported. TARGET_SOC must be set to one of the following values: ${plat_str}")
endif ()

# Defining nrf_802154_platform allows for supplying out-of-tree platforms
# (for example platform with custom nrfx config or even whole Zephyr).
# If no out-of-tree platform is provided, the build system provides
# an in-tree defined platform for a given target SoC.
if (NOT TARGET nrf_802154_platform)
  add_library(nrf_802154_platform STATIC "")
  include(cmake/platform/${TARGET_SOC}_xxAA.cmake)
  target_compile_features(nrf_802154_platform PUBLIC c_std_99)
  target_compile_definitions(nrf_802154_platform PUBLIC NRF_802154_INTERNAL_SWI_IRQ_HANDLING=1)
  add_subdirectory(external)
  target_link_libraries(nrf_802154_platform
    PUBLIC
      nrfx
      mpsl
  )

  add_subdirectory(examples)
endif ()

target_include_directories(nrf_802154_platform PUBLIC ${NRF_802154_SL_INCLUDE_DIRS})

# Above if statement may set a default ABI for in-tree platform, so we check if it is set right after.
if (NOT OPENSOURCE_SL AND "${TARGET_ABI}" STREQUAL "")
  message(FATAL_ERROR "Target ABI is unspecified. TARGET_ABI must be set to one of the following values: hard, soft, softfp")
endif ()

message("-- nRF 802.15.4 radio driver target SoC: ${TARGET_SOC}")
message("-- nRF 802.15.4 radio driver target ABI: ${TARGET_ABI}")

# This conditional is provided only for Zephyr integration. Its use in any other context is not supported.
if (OPENSOURCE_SL)
  add_library(nrf_802154_sl STATIC "")
  target_link_libraries(nrf_802154_sl PUBLIC nrf_802154_platform)
  target_sources(nrf_802154_sl PRIVATE ${NRF_802154_SL_OPENSOURCE_SOURCES})
  target_compile_features(nrf_802154_sl PRIVATE c_std_99)
else ()
  add_library(nrf_802154_sl INTERFACE)
  target_link_libraries(nrf_802154_sl INTERFACE nrf_802154_platform)
  target_compile_definitions(nrf_802154_sl INTERFACE RAAL_REM=1)
  target_link_options(nrf_802154_sl INTERFACE -L${CMAKE_CURRENT_SOURCE_DIR}/nrf_802154_sl/lib/${TARGET_SOC}/${TARGET_ABI}-float)
  target_link_libraries(nrf_802154_sl
    INTERFACE
      # Binary SL and its platform have a circular dependency.
      -Wl,--start-group
      libnrf_802154_sl.a
      $<TARGET_FILE:nrf_802154_platform>
      -Wl,--end-group
  )
endif ()

add_library(nrf_802154_radio_driver STATIC "")
target_sources(nrf_802154_radio_driver
  PRIVATE
    ${NRF_802154_DRIVER_SOURCES_NRF52}
    ${NRF_802154_DRIVER_SOURCES_DIRECT}
)
target_include_directories(nrf_802154_radio_driver PUBLIC ${NRF_802154_DRIVER_INCLUDE_DIRS})
target_link_libraries(nrf_802154_radio_driver PUBLIC nrf_802154_sl)
target_compile_features(nrf_802154_radio_driver PRIVATE c_std_99)
