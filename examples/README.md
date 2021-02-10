# nRF 802.15.4 radio driver - examples

Example bare-metal applications for nRF IEEE 802.15.4 radio driver are provided to familiarize you
with the radio driver API and reduce amount of work required to create your own IEEE 802.15.4
compliant application.

## Required hardware

Provided examples are compatible with the nRF52840 DK.

## Toolchain

Tools required to build the examples and program the board are as follows.

**CMake** - version 3.13.1 or above - is required to generate the build scripts for a build tool of
choice. This manual uses `make`.

**ARMGCC** - version 9-2019-q4-major - is required to build the examples. Provided CMake files assume
the ARM Toolchain is included in the path.

**nRF5x Command Line Tools** - version 9.4.0 - are required as they provide `nrfjprog` utility used
to program the target device.

## Creating an out-of-tree application

The top-level CMake description makes no assumptions about the project directory hierarchy using the nRF 802.15.4 radio driver, but expects the CMake description pulling the driver to meet certain requirements.

The CMake description that pulls the radio driver must set the following parameters prior to the directory inclusion:
* `TARGET_SOC` variable - specifies the SoC that the radio driver is meant to be built for.
* `TARGET_ABI` variable - specifies the float ABI that the radio driver is meant to use.
* `nrf_802154_platform` target - the target must provide all dependencies necessary to build the radio driver:
   - nrfx library
   - mpsl library
   - compiler ABI flags
   - must contain platform abstraction source files
   - must set the NRF_802154_INTERNAL_SWI_IRQ_HANDLING compile definition

   The target must be a static library.

Example integration with an application:
```cmake
cmake_minimum_required(VERSION 3.13.1)
project(application)

#
# Add nrfx drivers. The target can be named anything, as long as the headers and library
# is visible to the radio driver top-level CMake.
#
add_library(nrfx-custom STATIC "")

# Add any nrfx files, header paths, compiler options, defines, ABI settings etc. that are
# necessary to compile and link the application.
target_xyz(...)

#
# Include a CMake file which sets NRF_802154_SL_SOURCES_NRF52_PLATFORM_BAREMETAL
# and NRF_802154_SL_SOURCES_PLATFORM_BAREMETAL variables containing
# a list files providing nRF 802.15.4 radio driver platform abstraction.
#
# For the include to work properly, the NRF_802154_DRIVER_ROOT must be set to
# the radio driver directory root.
#
set(NRF_802154_DRIVER_ROOT "<radio-dir>")
include(${NRF_802154_DRIVER_ROOT}/nrf_802154_driver_sources.cmake)

#
# Create the nrf_802154_platform target that will be used by the radio driver
# top-level CMake to properly prepare the radio driver library targets.
# The target must be a static library.
#
add_library(nrf_802154_platform STATIC "")

#
# The nrf_802154_platform target must contain the platform abstraction source files.
#
target_sources(nrf_802154_platform
   PRIVATE
      ${NRF_802154_DRIVER_SOURCES_PLATFORM_BAREMETAL}
      ${NRF_802154_SL_SOURCES_NRF52_PLATFORM_BAREMETAL}
)
# Set the NRF_802154_INTERNAL_SWI_IRQ_HANDLING define for baremetal builds.
target_compile_definitions(nrf_802154_platform
   PUBLIC
      NRF_802154_INTERNAL_SWI_IRQ_HANDLING=1
)

#
# Provide the paths to the MPSL library and its header files.
#
# The MPSL library can be defined as a separate CMake target. In that case
# just use plain target_link_libraries(nrf_802154_platform PUBLIC <mpsl-target>)
#
# In the below demonstration we add the library using the paths.
#
target_link_libraries(nrf_802154_platform PUBLIC <mpsl-lib-path>.a)
target_include_directories(nrf_802154_platform PUBLIC <mpsl-header-path>)

#
# Make sure the nrfx drivers and compiler ABI flags are visible to the radio driver
#
target_link_libraries(nrf_802154_platform PUBLIC nrfx-custom)

#
# Set the target SoC and ABI variables required by the radio driver build description.
# These variables are necessary to select the correct nRF 802.5.4 Service Layer library
# variant.
#
# nRF52840 with hard-float ABI is used as an example.
#
set(TARGET_SOC "nrf52840")
set(TARGET_ABI "hard")

#
# Pull the radio driver build descriptions.
#
# The inclusion of this directory defines the nrf_802154_radio_driver target.
# This target contains all information necessary to properly build and link
# it with the final application.
#
add_subdirectory(${NRF_802154_DRIVER_ROOT})

#
# Finally create the application and link it with all dependencies.
#
add_executable(application <app-sources>)
target_link_libraries(application
   PRIVATE
      nrf_802154_radio_driver
      <other-dependencies>
)

```

> __Note:__ The platform definition in the radio driver top-level CMake is only intended for internal use by the in-tree examples and have very limited platform support. For this reason the use of that platform is not supported in any other context.
