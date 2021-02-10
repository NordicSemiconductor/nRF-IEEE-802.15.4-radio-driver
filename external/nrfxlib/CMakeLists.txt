add_library(mpsl STATIC IMPORTED GLOBAL)

set_target_properties(mpsl
  PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/sdk-nrfxlib/mpsl/lib/cortex-m4/${TARGET_ABI}-float/libmpsl.a"
)

target_include_directories(mpsl INTERFACE sdk-nrfxlib/mpsl/include)
