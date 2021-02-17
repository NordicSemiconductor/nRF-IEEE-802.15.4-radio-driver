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
 * @file
 *   This file implements the nrf 802.15.4 Interrupt Abstraction Layer with
 *   Zephyr API.
 */

#include <platform/irq/nrf_802154_irq.h>

#include <irq.h>
#include <nrfx.h>

void nrf_802154_irq_init(uint32_t irqn, uint32_t prio, nrf_802154_isr_t isr)
{
    irq_connect_dynamic(irqn, prio, isr, NULL, 0);
}

void nrf_802154_irq_enable(uint32_t irqn)
{
    irq_enable(irqn);
}

void nrf_802154_irq_disable(uint32_t irqn)
{
    irq_disable(irqn);
}

void nrf_802154_irq_set_pending(uint32_t irqn)
{
    /* Zephyr does not provide abstraction layer for setting pending IRQ */
    NVIC_SetPendingIRQ(irqn);
}

void nrf_802154_irq_clear_pending(uint32_t irqn)
{
    /* Zephyr does not provide abstraction layer for clearing pending IRQ */
    NVIC_ClearPendingIRQ(irqn);
}

bool nrf_802154_irq_is_enabled(uint32_t irqn)
{
    return irq_is_enabled(irqn);
}

uint32_t nrf_802154_irq_priority_get(uint32_t irqn)
{
    return NVIC_GetPriority(irqn);
}
