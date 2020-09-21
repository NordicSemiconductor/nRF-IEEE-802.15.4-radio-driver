/* Copyright (c) 2017 - 2018, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this
 *      list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file
 *   This file implements the nrf 802.15.4 simulated radio arbiter.
 *
 * This arbiter should be used for testing driver and tweaking other arbiters.
 *
 */

#include "nrf_raal_api.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "nrf_802154_debug.h"
#include "platform/clock/nrf_802154_clock.h"

static bool m_continuous_requested;
static bool m_continuous_granted;
static bool m_critical_section;
static enum {
    PENDING_EVENT_NONE,
    PENDING_EVENT_STARTED,
    PENDING_EVENT_ENDED,
} m_pending_event;
static uint16_t m_time_interval               = 250; // ms
static uint16_t m_ble_duty                    = 10;  // ms
static uint16_t m_pre_preemption_notification = 150; // us

static void continuous_grant(void)
{
    if (m_continuous_requested && !m_continuous_granted)
    {
        nrf_802154_pin_set(PIN_DBG_TIMESLOT_ACTIVE);
        m_continuous_granted = true;
        nrf_raal_timeslot_started();
    }
}

static void continuous_revoke(void)
{
    if (m_continuous_requested && m_continuous_granted)
    {
        nrf_802154_pin_clr(PIN_DBG_TIMESLOT_ACTIVE);
        m_continuous_granted = false;
        nrf_raal_timeslot_ended();
    }
}

void nrf_raal_init(void)
{

    NRF_MWU->PREGION[0].SUBS = 0x00000002;
    NRF_MWU->INTENSET        = MWU_INTENSET_PREGION0WA_Msk | MWU_INTENSET_PREGION0RA_Msk;

    NVIC_SetPriority(MWU_IRQn, 0);
    NVIC_ClearPendingIRQ(MWU_IRQn);
    NVIC_EnableIRQ(MWU_IRQn);

    NRF_TIMER0->MODE       = TIMER_MODE_MODE_Timer;
    NRF_TIMER0->BITMODE    = TIMER_BITMODE_BITMODE_24Bit;
    NRF_TIMER0->PRESCALER  = 4;
    NRF_TIMER0->INTENSET   = TIMER_INTENSET_COMPARE0_Msk |
                             TIMER_INTENSET_COMPARE1_Msk |
                             TIMER_INTENSET_COMPARE2_Msk;
    NRF_TIMER0->CC[0]      = m_time_interval * 1000UL;
    NRF_TIMER0->CC[1]      = m_ble_duty * 1000UL;
    NRF_TIMER0->CC[2]      = (m_time_interval * 1000UL) - m_pre_preemption_notification;

    NVIC_SetPriority(TIMER0_IRQn, 1);
    NVIC_ClearPendingIRQ(TIMER0_IRQn);
    NVIC_EnableIRQ(TIMER0_IRQn);

    m_continuous_requested = false;

    NRF_TIMER0->TASKS_START = 1;
}

void nrf_raal_uninit(void)
{
    // Intentionally empty.
}

void nrf_raal_continuous_mode_enter(void)
{
    nrf_802154_log(EVENT_TRACE_ENTER, FUNCTION_RAAL_CONTINUOUS_ENTER);

    assert(!m_continuous_requested);

    m_continuous_requested = true;
    m_pending_event = PENDING_EVENT_NONE;

    nrf_802154_clock_hfclk_start();

    nrf_802154_log(EVENT_TRACE_EXIT, FUNCTION_RAAL_CONTINUOUS_ENTER);
}

void nrf_raal_continuous_mode_exit(void)
{
    nrf_802154_log(EVENT_TRACE_ENTER, FUNCTION_RAAL_CONTINUOUS_EXIT);

    assert(m_continuous_requested);

    m_continuous_requested = false;
    m_continuous_granted   = false;

    nrf_802154_clock_hfclk_stop();

    nrf_802154_pin_clr(PIN_DBG_TIMESLOT_ACTIVE);

    nrf_802154_log(EVENT_TRACE_EXIT, FUNCTION_RAAL_CONTINUOUS_EXIT);
}

void nrf_raal_critical_section_enter(void)
{
    nrf_802154_log(EVENT_TRACE_ENTER, FUNCTION_RAAL_CRIT_SECT_ENTER);

    m_critical_section = true;
    nrf_802154_pin_set(PIN_DBG_RAAL_CRITICAL_SECTION);

    nrf_802154_log(EVENT_TRACE_EXIT, FUNCTION_RAAL_CRIT_SECT_ENTER);
}

void nrf_raal_critical_section_exit(void)
{
    NVIC_DisableIRQ(TIMER0_IRQn);
    __DSB();
    __ISB();

    nrf_802154_log(EVENT_TRACE_ENTER, FUNCTION_RAAL_CRIT_SECT_EXIT);

    m_critical_section = false;

    switch (m_pending_event)
    {
    case PENDING_EVENT_STARTED:
        continuous_grant();
        break;

    case PENDING_EVENT_ENDED:
        continuous_revoke();
        break;

    default:
        break;
    }

    m_pending_event = PENDING_EVENT_NONE;

    nrf_802154_pin_clr(PIN_DBG_RAAL_CRITICAL_SECTION);
    nrf_802154_log(EVENT_TRACE_EXIT, FUNCTION_RAAL_CRIT_SECT_EXIT);

    NVIC_EnableIRQ(TIMER0_IRQn);
}

bool nrf_raal_timeslot_request(uint32_t length_us)
{
    uint32_t timer;

    assert(m_continuous_requested);

    if (!m_continuous_granted)
    {
        return false;
    }

    NRF_TIMER0->TASKS_CAPTURE[3] = 1;
    timer = NRF_TIMER0->CC[3];

    return timer >= NRF_TIMER0->CC[1] && timer + length_us < NRF_TIMER0->CC[2];
}

bool nrf_raal_timeslot_is_granted(void)
{
    return m_continuous_granted;
}

uint32_t nrf_raal_timeslot_us_left_get(void)
{
    uint32_t timer;
    uint32_t timeslot_start = NRF_TIMER0->CC[1];
    uint32_t timeslot_end   = NRF_TIMER0->CC[2];

    NRF_TIMER0->TASKS_CAPTURE[3] = 1;
    timer = NRF_TIMER0->CC[3];

    return timer >= timeslot_start && timer < timeslot_end ? timeslot_end - timer : 0;
}

void nrf_802154_clock_hfclk_ready(void)
{
    // Just wait for next timeslot to report that timeslot is ready.
    // It is hard to report start of timeslot from this context due to races (i.e. different
    // priorities of TIMER0, CLOCK, RADIO).
}

void TIMER0_IRQHandler(void)
{
    nrf_802154_log(EVENT_TRACE_ENTER, FUNCTION_RAAL_SIG_HANDLER);

    if (NRF_TIMER0->EVENTS_COMPARE[1])
    {
        NRF_TIMER0->EVENTS_COMPARE[1] = 0;

        nrf_802154_log(EVENT_TRACE_ENTER, FUNCTION_RAAL_SIG_EVENT_START);

        NRF_MWU->REGIONENCLR = MWU_REGIONENCLR_PRGN0WA_Msk | MWU_REGIONENCLR_PRGN0RA_Msk;

        if (m_critical_section)
        {
            if (m_pending_event == PENDING_EVENT_ENDED)
            {
                m_pending_event = PENDING_EVENT_NONE;
            }
            else
            {
                m_pending_event = PENDING_EVENT_STARTED;
            }
        }
        else
        {
            continuous_grant();
        }

        nrf_802154_log(EVENT_TRACE_EXIT, FUNCTION_RAAL_SIG_EVENT_START);
    }

    if (NRF_TIMER0->EVENTS_COMPARE[2])
    {
        NRF_TIMER0->EVENTS_COMPARE[2] = 0;

        nrf_802154_log(EVENT_TRACE_ENTER, FUNCTION_RAAL_SIG_EVENT_MARGIN);

        if (m_critical_section)
        {
            if (m_pending_event == PENDING_EVENT_STARTED)
            {
                m_pending_event = PENDING_EVENT_NONE;
            }
            else
            {
                m_pending_event = PENDING_EVENT_ENDED;
            }
        }
        else
        {
            continuous_revoke();
        }

        nrf_802154_log(EVENT_TRACE_EXIT, FUNCTION_RAAL_SIG_EVENT_MARGIN);
    }

    if (NRF_TIMER0->EVENTS_COMPARE[0])
    {
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;

        nrf_802154_log(EVENT_TRACE_ENTER, FUNCTION_RAAL_SIG_EVENT_ENDED);

        NRF_MWU->REGIONENSET = MWU_REGIONENSET_PRGN0WA_Msk | MWU_REGIONENSET_PRGN0RA_Msk;

        NRF_TIMER0->TASKS_STOP  = 1;
        NRF_TIMER0->TASKS_CLEAR = 1;
        NRF_TIMER0->TASKS_START = 1;

        nrf_802154_log(EVENT_TRACE_EXIT, FUNCTION_RAAL_SIG_EVENT_ENDED);
    }

    nrf_802154_log(EVENT_TRACE_EXIT, FUNCTION_RAAL_SIG_HANDLER);
}

void MWU_IRQHandler(void)
{
    assert(false);
}
