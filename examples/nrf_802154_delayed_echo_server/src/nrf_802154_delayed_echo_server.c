/*
 * Copyright (c) 2018 - 2021, Nordic Semiconductor ASA
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

#include "nrf_802154.h"
#include "nrf_802154_utils.h"
#include "nrf_802154_const.h"
#include "nrf_802154_sl_fault.h"
#include "hal/nrf_gpio.h"
#include "mpsl_utils.h"

/** @brief LED pin definitions. */
#define LED3                     15U

/** @brief Server address for this example. */
#define SERVER_ADDRESS_HIGH_BYTE 0xbe
#define SERVER_ADDRESS_LOW_BYTE  0xef
/** @brief Personal area network identificator for this example. */
#define PAN_ID_HIGH_BYTE         0xfa
#define PAN_ID_LOW_BYTE          0xce
/** @brief Default 802.15.4 channel. */
#define CHANNEL_802154           11U

/** @brief Delay of an echo transmission in microseconds. */
#define TX_DELAY_TIME_US         (1000UL * 1000UL)

/** @brief Initializes LEDs. */
static void led_init(void)
{
    nrf_gpio_cfg_output(LED3);
}

/** @brief Swaps bytes.
 *
 *  @param[in] p_a  Pointer to the first byte to be swapped.
 *  @parem[in] p_b  Pointer to the second byte to be swapped.
 */
static inline void swap_bytes(uint8_t * p_a, uint8_t * p_b)
{
    uint8_t temp;

    temp = *p_a;
    *p_a = *p_b;
    *p_b = temp;
}

/** @brief Swaps destination and source addresses.
 *
 *  @param[in] p_frame  Pointer to the frame with addresses to swap. It is assummed the first byte
 *                      contains frame length.
 */
static void frame_addresses_swap(uint8_t * p_frame)
{
    swap_bytes(&p_frame[DEST_ADDR_OFFSET], &p_frame[SRC_ADDR_OFFSET_SHORT_DST]);
    swap_bytes(&p_frame[DEST_ADDR_OFFSET + 1U], &p_frame[SRC_ADDR_OFFSET_SHORT_DST + 1U]);
}

void nrf_802154_sl_fault_handler(uint32_t module_id, int32_t line, const char * p_error)
{
    (void)module_id;
    (void)line;
    (void)p_error;

    assert(false);
}

void nrf_802154_received_raw(uint8_t * p_data, int8_t power, uint8_t lqi)
{
    (void)power;
    (void)lqi;

    nrf_gpio_pin_toggle(LED3);

    frame_addresses_swap(p_data);

    nrf_802154_transmit_raw_at(p_data,
                               true,
                               nrf_802154_time_get(),
                               TX_DELAY_TIME_US,
                               nrf_802154_channel_get());
}

void nrf_802154_transmitted_raw(const uint8_t * p_frame,
                                uint8_t       * p_ack,
                                int8_t          power,
                                uint8_t         lqi)
{
    (void)power;
    (void)lqi;

    nrf_802154_buffer_free_immediately_raw((uint8_t *)p_frame);

    if (p_ack != NULL)
    {
        nrf_802154_buffer_free_immediately_raw(p_ack);
    }

    /* There is no need to call nrf_802154_receive, due to driver automatically transitioning to
     * receiving state after a transmission.
     */
}

void nrf_802154_transmit_failed(const uint8_t       * p_frame,
                                nrf_802154_tx_error_t error)
{
    (void)error;

    nrf_802154_buffer_free_immediately_raw((uint8_t *)p_frame);

    /* There is no need to call nrf_802154_receive, due to driver automatically transitioning to
     * receiving state after a transmission.
     */
}

int main(void)
{
    static uint8_t pan_id[]        = {PAN_ID_LOW_BYTE, PAN_ID_HIGH_BYTE};
    static uint8_t short_address[] = {SERVER_ADDRESS_LOW_BYTE, SERVER_ADDRESS_HIGH_BYTE};

    led_init();

    mpsl_utils_init();

    nrf_802154_init();
    nrf_802154_pan_id_set(pan_id);
    nrf_802154_channel_set(CHANNEL_802154);
    nrf_802154_short_address_set(short_address);

    nrf_802154_receive();

    while (1)
    {
        __WFE();
    }
}
