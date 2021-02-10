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
#include <string.h>

/** @brief LED pin definitions. */
#define LED1                     13U
#define LED2                     14U

/** @brief Client address for this example. */
#define CLIENT_ADDRESS_HIGH_BYTE 0xca
#define CLIENT_ADDRESS_LOW_BYTE  0xfe
/** @brief Server address for this example. */
#define SERVER_ADDRESS_HIGH_BYTE 0xbe
#define SERVER_ADDRESS_LOW_BYTE  0xef
/** @brief Personal area network identificator for this example. */
#define PAN_ID_HIGH_BYTE         0xfa
#define PAN_ID_LOW_BYTE          0xce
/** @brief Default 802.15.4 channel. */
#define CHANNEL_802154           11U

/** @brief Delay of an echo reception in microseconds. */
#define RX_DELAY_TIME_US         (1000UL * 1000UL)
/** @brief Delay of the next transmission in microseconds. */
#define NEXT_TX_DELAY_TIME_US    (2000UL * 1000UL)
/** @brief Width of an echo reception in microseconds. */
#define WINDOW_WIDTH_US          (10UL * 1000UL)

/** @brief Size of MAC Header and MAC Footer.
 *
 *  @note Defined size is valid only for the MHR configuration presented in @ref frame_init.
 */
#define MHR_MFR_SIZE             11U

/** @brief Frame buffer. */
static uint8_t mp_frame[PHR_SIZE + MAX_PACKET_SIZE];
/** @brief Message to be sent. */
static char m_message[] = "Nordic Semiconductor";

/** @brief Current sequence number for frame numbering. */
static uint8_t m_current_sequence_number;

/** @brief Generate next sequence number.
 *
 *  @return Next sequence number.
 */
static uint8_t sequence_number_generate(void)
{
    return m_current_sequence_number++;
}

/** @brief Initialize LEDs. */
static void led_init(void)
{
    nrf_gpio_cfg_output(LED1);
    nrf_gpio_cfg_output(LED2);
}

/** @brief Initialize frame with fixed data. */
static void frame_init(void)
{
    /*
     * Example MHR
     *
     * Frame Control Field: Data (0x8861)
     *     .... .... .... .001 = Frame Type: Data (0x0001)
     *     .... .... .... 0... = Security Enabled: False
     *     .... .... ...0 .... = Frame Pending: False
     *     .... .... ..1. .... = Acknowledge Request: True
     *     .... .... .1.. .... = Intra-PAN: True
     *     .... 10.. .... .... = Destination Addressing Mode: Short/16-bit (0x02)
     *     ..00 .... .... .... = Frame Version: 0
     *     10.. .... .... .... = Source Addressing Mode: Short/16-bit (0x02)
     * Sequence Number: Filled in before transmit
     * Destination PAN: 0xface
     * Destination: 0xbeef
     * Source: 0xcafe
     */

    /* Prepare frame data */
    /* Frame length */
    mp_frame[PHR_OFFSET] = MHR_MFR_SIZE + sizeof(m_message) - 1U;
    /* Frame control */
    mp_frame[FRAME_TYPE_OFFSET]      = FRAME_TYPE_DATA | PAN_ID_COMPR_MASK | ACK_REQUEST_BIT;
    mp_frame[FRAME_TYPE_OFFSET + 1U] = DEST_ADDR_TYPE_SHORT | FRAME_VERSION_0 | SRC_ADDR_TYPE_SHORT;
    /* PAN ID */
    mp_frame[PAN_ID_OFFSET]      = PAN_ID_LOW_BYTE;
    mp_frame[PAN_ID_OFFSET + 1U] = PAN_ID_HIGH_BYTE;
    /* Destination address */
    mp_frame[DEST_ADDR_OFFSET]      = SERVER_ADDRESS_LOW_BYTE;
    mp_frame[DEST_ADDR_OFFSET + 1U] = SERVER_ADDRESS_HIGH_BYTE;
    /* Source address */
    mp_frame[SRC_ADDR_OFFSET_SHORT_DST]      = CLIENT_ADDRESS_LOW_BYTE;
    mp_frame[SRC_ADDR_OFFSET_SHORT_DST + 1U] = CLIENT_ADDRESS_HIGH_BYTE;
    /* Data */
    memcpy(&mp_frame[SRC_ADDR_OFFSET_SHORT_DST + SHORT_ADDRESS_SIZE], m_message, sizeof(m_message));
}

/** @brief Sets sequence number of a frame in the frame buffer.
 *
 *  @param[in] squence_number Sequence number to be set.
 */
static void frame_sequence_number_set(uint8_t sequence_number)
{
    mp_frame[DSN_OFFSET] = sequence_number;
}

void nrf_802154_sl_fault_handler(uint32_t module_id, int32_t line, const char * p_error)
{
    (void)module_id;
    (void)line;
    (void)p_error;

    assert(false);
}

void nrf_802154_transmitted_raw(const uint8_t * p_frame,
                                uint8_t       * p_ack,
                                int8_t          power,
                                uint8_t         lqi)
{
    (void)p_frame;
    (void)power;
    (void)lqi;

    nrf_gpio_pin_toggle(LED1);

    if (p_ack != NULL)
    {
        nrf_802154_buffer_free_immediately_raw(p_ack);
    }

    uint32_t current_time = nrf_802154_time_get();

    frame_sequence_number_set(sequence_number_generate());
    nrf_802154_transmit_raw_at(mp_frame,
                               true,
                               current_time,
                               NEXT_TX_DELAY_TIME_US,
                               nrf_802154_channel_get());

    nrf_802154_receive_at(current_time,
                          RX_DELAY_TIME_US - (WINDOW_WIDTH_US / 2U),
                          WINDOW_WIDTH_US,
                          nrf_802154_channel_get());

    nrf_802154_sleep();
}

void nrf_802154_received_raw(uint8_t * p_data, int8_t power, uint8_t lqi)
{
    (void)power;
    (void)lqi;

    nrf_gpio_pin_toggle(LED2);

    nrf_802154_buffer_free_immediately_raw(p_data);
    nrf_802154_sleep();
}

void nrf_802154_transmit_failed(const uint8_t * p_frame, nrf_802154_tx_error_t error)
{
    (void)p_frame;
    (void)error;

    nrf_802154_transmit_raw_at(mp_frame,
                               true,
                               nrf_802154_time_get(),
                               NEXT_TX_DELAY_TIME_US,
                               nrf_802154_channel_get());

    nrf_802154_sleep();
}

int main(void)
{
    static uint8_t pan_id[]        = {PAN_ID_LOW_BYTE, PAN_ID_HIGH_BYTE};
    static uint8_t short_address[] = {CLIENT_ADDRESS_LOW_BYTE, CLIENT_ADDRESS_HIGH_BYTE};

    led_init();

    mpsl_utils_init();

    nrf_802154_init();
    nrf_802154_pan_id_set(pan_id);
    nrf_802154_channel_set(CHANNEL_802154);
    nrf_802154_short_address_set(short_address);

    frame_init();

    frame_sequence_number_set(sequence_number_generate());
    nrf_802154_transmit_raw(mp_frame, true);

    while (1)
    {
        __WFE();
    }
}
