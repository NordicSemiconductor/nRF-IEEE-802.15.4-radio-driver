target_compile_definitions(nrf_802154_platform
  PUBLIC
    NRF52_SERIES
    NRF52811
    NRF52811_XXAA
)

target_compile_options(nrf_802154_platform
  PUBLIC
    -mcpu=cortex-m4
    -mthumb
    -mabi=aapcs
    -mfloat-abi=soft
)

target_link_options(nrf_802154_platform
  PUBLIC
    -Wl,--gc-sections
    --specs=nano.specs
    -mcpu=cortex-m4
    -mthumb
    -mabi=aapcs
    -mfloat-abi=soft
)

target_sources(nrf_802154_platform
  PRIVATE
    ${NRF_802154_DRIVER_SOURCES_PLATFORM_BAREMETAL}
    ${NRF_802154_SL_SOURCES_NRF52_PLATFORM_BAREMETAL}
)

# The compile and link options specified above require soft-float ABI.
set(TARGET_ABI "soft" CACHE STRING "" FORCE)
