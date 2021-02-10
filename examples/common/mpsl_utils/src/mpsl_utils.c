/*
 * Copyright (c) 2020 - 2021, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
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
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
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
 *
 */

/**
 * @file This file implements the MPSL utils API required to use the MPSL in baremetal applications.
 *
 */

#include "mpsl_utils.h"

#include <assert.h>
#include "mpsl.h"
#include "mpsl_clock.h"

/** @brief Low IRQ priority
 *
 *  This interrupt priority should be lower than MPSL_HIGH_IRQ_PRIORITY.
 */
#define MPSL_LOW_PRIO_IRQp (4)

static void mpsl_assert_handler(const char * const file, const uint32_t line)
{
    (void)file;
    (void)line;

    assert(false);
}

void mpsl_event_handler(uint32_t evt_id)
{
    (void)evt_id;

    assert(false);
}

void mpsl_utils_init(void)
{
#if ENABLE_LFRC
    mpsl_clock_lfclk_cfg_t clock_config =
    {
        .source       = MPSL_CLOCK_LF_SRC_RC,
        .rc_ctiv      = 16,
        .rc_temp_ctiv = 2,
    };

#else
    mpsl_clock_lfclk_cfg_t clock_config =
    {
        .source       = MPSL_CLOCK_LF_SRC_XTAL,
        .rc_ctiv      = 0,
        .rc_temp_ctiv = 0
    };

#endif

    NVIC_SetPriority(SWI5_IRQn, MPSL_LOW_PRIO_IRQp);
    NVIC_EnableIRQ(SWI5_IRQn);

    mpsl_init(&clock_config, SWI5_IRQn, mpsl_assert_handler);

    NVIC_SetPriority(POWER_CLOCK_IRQn, MPSL_LOW_PRIO_IRQp);
    NVIC_EnableIRQ(POWER_CLOCK_IRQn);
}

void mpsl_utils_uninit(void)
{
    mpsl_uninit();

    NVIC_DisableIRQ(POWER_CLOCK_IRQn);
}

void RTC0_IRQHandler(void)
{
    MPSL_IRQ_RTC0_Handler();
}

void TIMER0_IRQHandler(void)
{
    MPSL_IRQ_TIMER0_Handler();
}

void SWI5_IRQHandler(void)
{
    mpsl_low_priority_process();
}

void RADIO_IRQHandler(void)
{
    MPSL_IRQ_RADIO_Handler();
}

void POWER_CLOCK_IRQHandler(void)
{
    MPSL_IRQ_CLOCK_Handler();
}
