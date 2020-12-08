/*
 * Copyright (c) 2017 - 2020, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NRF_802154_RSCH_CRIT_SECT_H__
#define NRF_802154_RSCH_CRIT_SECT_H__

#include <stdbool.h>

#include "rsch/nrf_802154_rsch.h"

/**
 * @defgroup nrf_802154_rsch_crit_sect RSCH event queue used during critical sections
 * @{
 * @ingroup nrf_802154_rsch
 * @brief The critical section implementation for the RSCH module.
 */

/**
 * @brief Pointer to function used to enter critical section.
 */
typedef bool (* nrf_802154_sl_crit_sect_enter_t)(void);

/**
 * @brief Pointer to function used to exit critical section.
 */
typedef void (* nrf_802154_sl_crit_sect_exit_t)(void);

/**
 * @brief Structure that holds interface of Radio Scheduler critical sections.
 */
typedef struct
{
    nrf_802154_sl_crit_sect_enter_t enter;
    nrf_802154_sl_crit_sect_exit_t  exit;
} nrf_802154_sl_crit_sect_interface_t;

/**
 * @brief Initializes the RSCH critical section module.
 */
void nrf_802154_rsch_crit_sect_init(
    const nrf_802154_sl_crit_sect_interface_t * p_crit_sect_interface);

/**
 * @brief Requests the priority level from RSCH through the critical section module.
 *
 * @param[in]  prio  Requested priority level.
 */
void nrf_802154_rsch_crit_sect_prio_request(rsch_prio_t prio);

/**
 * @brief Notifies the core that the approved RSCH priority has changed.
 *
 * @note This function is called from the critical section context and does not preempt
 *       other critical sections.
 *
 * @param[in]  prio  Approved priority level.
 */
extern void nrf_802154_rsch_crit_sect_prio_changed(rsch_prio_t prio);

/**
 *@}
 **/

#endif // NRF_802154_RSCH_CRIT_SECT_H__
