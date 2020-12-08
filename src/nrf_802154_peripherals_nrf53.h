/*
 * Copyright (c) 2019 - 2020, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @brief Module that defines the 802.15.4 driver peripheral usage for nRF53 family.
 *
 */

#ifndef NRF_802154_PERIPHERALS_NRF53_H__
#define NRF_802154_PERIPHERALS_NRF53_H__

#include <nrf.h>
#include <nrfx.h>
#include "nrf_802154_config.h"
#include "nrf_802154_debug.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @def NRF_802154_EGU_INSTANCE_NO
 *
 * Id of the EGU instance used by the driver to synchronize DPPIs and for requests and
 * notifications if SWI is in use.
 *
 */
#ifndef NRF_802154_EGU_INSTANCE_NO
#define NRF_802154_EGU_INSTANCE_NO 0
#endif

/**
 * @def NRF_802154_EGU_INSTANCE
 *
 * The EGU instance used by the driver to synchronize PPIs and for requests and notifications if
 * SWI is in use.
 *
 * @note This option is used by the core module regardless of the driver configuration.
 *
 */
#define NRF_802154_EGU_INSTANCE NRFX_CONCAT_2(NRF_EGU, NRF_802154_EGU_INSTANCE_NO)

/**
 * @def NRF_802154_EGU_IRQ_HANDLER
 *
 * The EGU IRQ handler used by the driver for requests and notifications if SWI is in use.
 *
 * @note This option is used when the driver uses SWI to process requests and notifications.
 *
 */
#define NRF_802154_EGU_IRQ_HANDLER \
    NRFX_CONCAT_3(EGU, NRF_802154_EGU_INSTANCE_NO, _IRQHandler)

/**
 * @def NRF_802154_EGU_IRQN
 *
 * The SWI EGU IRQ number used by the driver for requests and notifications if SWI is in use.
 *
 * @note This option is used when the driver uses SWI to process requests and notifications.
 *
 */
#define NRF_802154_EGU_IRQN \
    NRFX_CONCAT_3(EGU, NRF_802154_EGU_INSTANCE_NO, _IRQn)

/**
 * @def NRF_802154_EGU_USED_MASK
 *
 * Bit mask of instances of SWI/EGU peripherals used by the 802.15.4 driver.
 */
#ifndef NRF_802154_EGU_USED_MASK
#define NRF_802154_EGU_USED_MASK (1 << NRF_802154_EGU_INSTANCE_NO)
#endif

/**
 * @def NRF_802154_RTC_INSTANCE_NO
 *
 * Number of the RTC instance used in the standalone timer driver implementation.
 *
 */
#ifndef NRF_802154_RTC_INSTANCE_NO
#define NRF_802154_RTC_INSTANCE_NO 2
#endif

/**
 * @def NRF_802154_DPPI_RADIO_DISABLED_TO_EGU
 *
 * The DPPI channel that connects RADIO_DISABLED event to EGU task.
 *
 * @note This option is used by the core module regardless of the driver configuration.
 *
 */
#ifndef NRF_802154_DPPI_RADIO_DISABLED_TO_EGU
#define NRF_802154_DPPI_RADIO_DISABLED_TO_EGU 6U
#endif

/**
 * @def NRF_802154_DPPI_EGU_TO_RADIO_RAMP_UP
 *
 * The DPPI channel that connects EGU event to RADIO_TXEN or RADIO_RXEN task.
 *
 * @note This option is used by the core module regardless of the driver configuration.
 *       The peripheral is shared with @ref NRF_802154_DPPI_EGU_TO_RADIO_RAMP_UP.
 *
 */
#ifndef NRF_802154_DPPI_EGU_TO_RADIO_RAMP_UP
#define NRF_802154_DPPI_EGU_TO_RADIO_RAMP_UP 7U
#endif

/**
 * @def NRF_802154_DPPI_TIMER_COMPARE_TO_RADIO_TXEN
 *
 * The DPPI channel that connects TIMER_COMPARE event to RADIO_TXEN task.
 *
 * @note This option is used by the core module regardless of the driver configuration.
 *       The peripheral is shared with @ref NRF_802154_DPPI_EGU_TO_RADIO_RAMP_UP.
 *
 */
#ifndef NRF_802154_DPPI_TIMER_COMPARE_TO_RADIO_TXEN
#define NRF_802154_DPPI_TIMER_COMPARE_TO_RADIO_TXEN 7U
#endif

/**
 * @def NRF_802154_DPPI_RADIO_SYNC_TO_EGU_SYNC
 *
 * The DPPI channel that connects RADIO_SYNC event to EGU_SYNC task.
 * EGU_SYNC task belongs to one of EGU channels
 *
 */
#ifndef NRF_802154_DPPI_RADIO_SYNC_TO_EGU_SYNC
#define NRF_802154_DPPI_RADIO_SYNC_TO_EGU_SYNC 8U
#endif

#ifdef __cplusplus
}
#endif

#endif // NRF_802154_PERIPHERALS_NRF53_H__
