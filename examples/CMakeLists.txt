set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/../output/${PLATFORM}")

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

add_subdirectory(common)

#
# Create a hex file from an executable target.
#
function (add_hex_target executable_target)
  add_custom_target(
    hex ALL
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${executable_target}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${executable_target}>
    $<TARGET_FILE_DIR:${executable_target}>/$<TARGET_FILE_BASE_NAME:${executable_target}>.hex
    DEPENDS ${executable_target}
  )
endfunction(add_hex_target)

#
# Add target for flashing the application hex to a device.
#
function (add_flash_target executable_target)
  add_custom_target(
    flash
    COMMAND  nrfjprog -f nrf52 -r --sectorerase
    --program $<TARGET_FILE_DIR:${executable_target}>/$<TARGET_FILE_BASE_NAME:${executable_target}>.hex
    DEPENDS hex
  )
endfunction(add_flash_target)

target_compile_options(nrf_802154_platform
  PUBLIC
    $<$<CONFIG:Debug>:-Os -g3>
    $<$<CONFIG:MinSizeRel>:-Os -DNDEBUG>
    $<$<CONFIG:RelWithDebInfo>:-Os -g3>
    $<$<CONFIG:Release>:-Os -DNDEBUG>
    --std=gnu99
    -ffunction-sections
    -fdata-sections
    -fno-strict-aliasing
    -fno-builtin
    --short-enums
    -Wall
    -Wno-attributes
    -Wno-format
)
