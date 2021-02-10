add_library(mpsl_utils STATIC src/mpsl_utils.c)
target_link_libraries(mpsl_utils PUBLIC nrf_802154_platform)
target_include_directories(mpsl_utils PUBLIC src)
