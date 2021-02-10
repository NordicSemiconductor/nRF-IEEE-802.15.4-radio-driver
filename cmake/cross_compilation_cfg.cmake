set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

if ("${CMAKE_HOST_UNIX}")
    set(toolchain_compiler arm-none-eabi-gcc)
    set(toolchain_size arm-none-eabi-size)
elseif("${CMAKE_HOST_WIN32}")
    set(toolchain_compiler arm-none-eabi-gcc.exe)
    set(toolchain_size arm-none-eabi-size.exe)
endif()

find_path(TOOLCHAIN_PATH ${toolchain_compiler})
set(CMAKE_C_COMPILER ${TOOLCHAIN_PATH}/${toolchain_compiler} CACHE INTERNAL "")
set(CMAKE_SIZE       ${TOOLCHAIN_PATH}/${toolchain_size}     CACHE INTERNAL "")

set(CMAKE_EXE_LINKER_FLAGS "--specs=nosys.specs" CACHE INTERNAL "")

set(CMAKE_FIND_ROOT_PATH ${CMAKE_SOURCE_DIR}/..)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
