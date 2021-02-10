set(NRF_802154_SL_OPENSOURCE_SOURCES
    ${NRF_802154_SL_ROOT}/open/src/nrf_802154_sl_ant_div.c
    ${NRF_802154_SL_ROOT}/open/src/nrf_802154_sl_coex.c
    ${NRF_802154_SL_ROOT}/open/src/nrf_802154_sl_fem.c
    ${NRF_802154_SL_ROOT}/open/src/nrf_802154_sl_log.c
    ${NRF_802154_SL_ROOT}/open/src/nrf_802154_sl_rsch.c
    ${NRF_802154_SL_ROOT}/open/src/nrf_802154_sl_timer.c
    ${NRF_802154_SL_ROOT}/open/src/nrf_802154_sl_capabilities.c
    ${NRF_802154_SL_ROOT}/platform/clock/nrf_802154_clock.c
    ${NRF_802154_SL_ROOT}/platform/irq/nrf_802154_irq_baremetal.c
)

set(NRF_802154_SL_OPENSOURCE_INCLUDE_DIRS
    ${NRF_802154_SL_ROOT}/include
)

set(NRF_802154_SL_SOURCES_NRF52_PLATFORM_BAREMETAL
    ${NRF_802154_SL_ROOT}/platform/gpiote/nrf_802154_gpiote_none.c
    ${NRF_802154_SL_ROOT}/platform/gpiote/nrf_802154_gpiote_crit_sect.c
    ${NRF_802154_SL_ROOT}/platform/hp_timer/nrf_802154_hp_timer.c
    ${NRF_802154_SL_ROOT}/platform/lp_timer/nrf_802154_lp_timer.c
    ${NRF_802154_SL_ROOT}/platform/clock/nrf_802154_clock_mpsl.c
    ${NRF_802154_SL_ROOT}/platform/irq/nrf_802154_irq_baremetal.c
)

set(NRF_802154_SL_SOURCES_NRF53_PLATFORM_BAREMETAL
    ${NRF_802154_SL_ROOT}/platform/gpiote/nrf_802154_gpiote_none.c
    ${NRF_802154_SL_ROOT}/platform/gpiote/nrf_802154_gpiote_crit_sect.c
    ${NRF_802154_SL_ROOT}/platform/lp_timer/nrf_802154_lp_timer.c
    ${NRF_802154_SL_ROOT}/platform/clock/nrf_802154_clock_mpsl.c
    ${NRF_802154_SL_ROOT}/platform/irq/nrf_802154_irq_baremetal.c
)

set(NRF_802154_SL_SOURCES_NRF52_PLATFORM_ZEPHYR
    ${NRF_802154_SL_ROOT}/platform/gpiote/nrf_802154_gpiote_zephyr.c
    ${NRF_802154_SL_ROOT}/platform/gpiote/nrf_802154_gpiote_crit_sect.c
    ${NRF_802154_SL_ROOT}/platform/hp_timer/nrf_802154_hp_timer.c
    ${NRF_802154_SL_ROOT}/platform/lp_timer/nrf_802154_lp_timer.c
    ${NRF_802154_SL_ROOT}/platform/clock/nrf_802154_clock_zephyr.c
    ${NRF_802154_SL_ROOT}/platform/irq/nrf_802154_irq_zephyr.c
)

set(NRF_802154_SL_SOURCES_NRF53_PLATFORM_ZEPHYR
    ${NRF_802154_SL_ROOT}/platform/gpiote/nrf_802154_gpiote_zephyr.c
    ${NRF_802154_SL_ROOT}/platform/gpiote/nrf_802154_gpiote_crit_sect.c
    ${NRF_802154_SL_ROOT}/platform/lp_timer/nrf_802154_lp_timer.c
    ${NRF_802154_SL_ROOT}/platform/clock/nrf_802154_clock_zephyr.c
    ${NRF_802154_SL_ROOT}/platform/irq/nrf_802154_irq_zephyr.c
)

set(NRF_802154_SL_INCLUDE_DIRS
    ${NRF_802154_SL_ROOT}/include
)
