/*
 * Copyright (c) 2020, Nordic Semiconductor ASA
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

#include "nrf_802154_sl_rsch.h"

#include <assert.h>
#include <stddef.h>
#include <string.h>
#include <nrf.h>

#include "rsch/nrf_802154_rsch.h"
#include "platform/clock/nrf_802154_clock.h"

static rsch_prio_t m_prev_prio;
static bool        m_ready;

/**
 * @brief Notifies the core that the approved RSCH priority has changed.
 *
 * @note This function is called from the critical section context and does not preempt
 *       other critical sections.
 *
 * @param[in]  prio  Approved priority level.
 */
extern void nrf_802154_rsch_crit_sect_prio_changed(rsch_prio_t prio);

/***************************************************************************************************
 * Public API
 **************************************************************************************************/

void nrf_802154_rsch_init(void)
{
    m_ready     = false;
    m_prev_prio = RSCH_PRIO_IDLE;
}

void nrf_802154_rsch_uninit(void)
{
    // Intenionally empty
}

void nrf_802154_rsch_continuous_ended(void)
{
    // Intentionally empty
}

bool nrf_802154_rsch_timeslot_request(uint32_t length_us)
{
    (void)length_us;

    assert(m_ready);

    return true;
}

bool nrf_802154_rsch_timeslot_is_requested(void)
{
    return false;
}

bool nrf_802154_rsch_prec_is_approved(rsch_prec_t prec, rsch_prio_t prio)
{
    return prio == RSCH_PRIO_IDLE ? true : m_ready;
}

uint32_t nrf_802154_rsch_timeslot_us_left_get(void)
{
    return UINT32_MAX;
}

void nrf_802154_clock_hfclk_ready(void)
{
    m_ready = true;
    nrf_802154_rsch_crit_sect_prio_changed(RSCH_PRIO_MAX);
}

void nrf_802154_rsch_crit_sect_prio_request(rsch_prio_t prio)
{
    if (m_prev_prio != prio)
    {
        if (prio == RSCH_PRIO_IDLE)
        {
            nrf_802154_clock_hfclk_stop();

            assert(m_ready);

            m_ready = false;

            nrf_802154_rsch_crit_sect_prio_changed(RSCH_PRIO_IDLE);
        }
        else if (m_prev_prio == RSCH_PRIO_IDLE)
        {
            assert(!m_ready);

            nrf_802154_clock_hfclk_start();
        }
        else
        {
            // Intentionally empty
        }

        m_prev_prio = prio;
    }
}

void nrf_802154_rsch_prio_drop_init(void)
{
    // Intentionally empty
}

void nrf_802154_rsch_crit_sect_init(void)
{
    // Intentionally empty
}

void nrf_802154_critical_section_rsch_enter(void)
{
    // Intentionally empty
}

void nrf_802154_critical_section_rsch_exit(void)
{
    // Intentionally empty
}

bool nrf_802154_critical_section_rsch_event_is_pending(void)
{
    return false;
}

bool nrf_802154_rsch_delayed_timeslot_request(const rsch_dly_ts_param_t * p_dly_ts_param)
{
    (void)p_dly_ts_param;

    return false;
}

bool nrf_802154_rsch_delayed_timeslot_cancel(rsch_dly_ts_id_t dly_ts_id)
{
    (void)dly_ts_id;

    return false;
}

bool nrf_802154_rsch_delayed_timeslot_priority_update(rsch_dly_ts_id_t dly_ts_id,
                                                      rsch_prio_t      dly_ts_prio)
{
    (void)dly_ts_id;
    (void)dly_ts_prio;

    return false;
}
