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

#include "unity.h"

#include "nrf_802154_config.h"
#include "nrf_802154_const.h"
#include "mock_nrf_radio.h"

#ifdef NRF_802154_PENDING_SHORT_ADDRESSES
    #undef NRF_802154_PENDING_SHORT_ADDRESSES
    #define NRF_802154_PENDING_SHORT_ADDRESSES    4
#endif
#ifdef NRF_802154_PENDING_EXTENDED_ADDRESSES
    #undef NRF_802154_PENDING_EXTENDED_ADDRESSES
    #define NRF_802154_PENDING_EXTENDED_ADDRESSES 4
#endif

#include "nrf_802154_ack_pending_bit.c"

/***********************************************************************************/
/***********************************************************************************/
/***********************************************************************************/

void setUp(void)
{

}

void tearDown(void)
{
    memset(m_pending_extended, 0, sizeof(m_pending_extended));
    memset(m_pending_short, 0xff, sizeof(m_pending_short));
}

/***********************************************************************************/
/****************************** ACK PENDING BIT TESTS ******************************/
/***********************************************************************************/

static uint8_t test_addr_extended_1[EXTENDED_ADDRESS_SIZE] = { 0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89 };
static uint8_t test_addr_extended_2[EXTENDED_ADDRESS_SIZE] = { 0x23, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89, 0x9a };
static uint8_t test_addr_extended_3[EXTENDED_ADDRESS_SIZE] = { 0x34, 0x45, 0x56, 0x67, 0x78, 0x89, 0x9a, 0xab };
static uint8_t test_addr_extended_4[EXTENDED_ADDRESS_SIZE] = { 0x45, 0x56, 0x67, 0x78, 0x89, 0x9a, 0xab, 0xbc };
static uint8_t test_addr_extended_5[EXTENDED_ADDRESS_SIZE] = { 0x56, 0x67, 0x78, 0x89, 0x9a, 0xab, 0xbc, 0xcd };

static uint8_t test_addr_short_1[SHORT_ADDRESS_SIZE] = { 0x12, 0x23 };
static uint8_t test_addr_short_2[SHORT_ADDRESS_SIZE] = { 0x23, 0x34 };
static uint8_t test_addr_short_3[SHORT_ADDRESS_SIZE] = { 0x34, 0x45 };
static uint8_t test_addr_short_4[SHORT_ADDRESS_SIZE] = { 0x45, 0x56 };
static uint8_t test_addr_short_5[SHORT_ADDRESS_SIZE] = { 0x56, 0x67 };

static uint8_t test_psdu_extended[EXTENDED_ADDRESS_SIZE + SRC_ADDR_OFFSET_EXTENDED_DST] = 
{
    0x00, 0x40, 0xcc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89 
};

static uint8_t test_psdu_short[SHORT_ADDRESS_SIZE + SRC_ADDR_OFFSET_SHORT_DST] = 
{
    0x00, 0x40, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x23 
};

void test_ShouldAddAddressToTheList(void)
{
     nrf_802154_ack_pending_bit_init();

    bool result;

    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    TEST_ASSERT_TRUE(result);
    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);
    TEST_ASSERT_TRUE(result);
    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);
    TEST_ASSERT_TRUE(result);
    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);
    TEST_ASSERT_TRUE(result);

    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    TEST_ASSERT_TRUE(result);
    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);
    TEST_ASSERT_TRUE(result);
    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);
    TEST_ASSERT_TRUE(result);
    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);
    TEST_ASSERT_TRUE(result);
}

void test_ShouldFailToAddAddressToTheListWhenListIsFull(void)
{
    nrf_802154_ack_pending_bit_init();

    bool result;

    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    TEST_ASSERT_TRUE(result);
    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);
    TEST_ASSERT_TRUE(result);
    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);
    TEST_ASSERT_TRUE(result);
    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);
    TEST_ASSERT_TRUE(result);
    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_5, true);
    TEST_ASSERT_FALSE(result);

    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    TEST_ASSERT_TRUE(result);
    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);
    TEST_ASSERT_TRUE(result);
    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);
    TEST_ASSERT_TRUE(result);
    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);
    TEST_ASSERT_TRUE(result);
    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_5, false);
    TEST_ASSERT_FALSE(result);
}

void test_ShouldKeepAscendingOrderAfterAddingExtendedAddressToTheList(void)
{
    nrf_802154_ack_pending_bit_init();

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_3, m_pending_extended[2], sizeof(test_addr_extended_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[3], sizeof(test_addr_extended_4));

    nrf_802154_ack_pending_bit_for_addr_reset(true);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_3, m_pending_extended[2], sizeof(test_addr_extended_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[3], sizeof(test_addr_extended_4));

    nrf_802154_ack_pending_bit_for_addr_reset(true);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_3, m_pending_extended[2], sizeof(test_addr_extended_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[3], sizeof(test_addr_extended_4));

    nrf_802154_ack_pending_bit_for_addr_reset(true);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_3, m_pending_extended[2], sizeof(test_addr_extended_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[3], sizeof(test_addr_extended_4));

    nrf_802154_ack_pending_bit_for_addr_reset(true);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_3, m_pending_extended[2], sizeof(test_addr_extended_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[3], sizeof(test_addr_extended_4));

    nrf_802154_ack_pending_bit_for_addr_reset(true);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_3, m_pending_extended[2], sizeof(test_addr_extended_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[3], sizeof(test_addr_extended_4));

    nrf_802154_ack_pending_bit_for_addr_reset(true);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_3, m_pending_extended[2], sizeof(test_addr_extended_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[3], sizeof(test_addr_extended_4));

    nrf_802154_ack_pending_bit_for_addr_reset(true);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_3, m_pending_extended[2], sizeof(test_addr_extended_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[3], sizeof(test_addr_extended_4));

    nrf_802154_ack_pending_bit_for_addr_reset(true);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_3, m_pending_extended[2], sizeof(test_addr_extended_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[3], sizeof(test_addr_extended_4));

    nrf_802154_ack_pending_bit_for_addr_reset(true);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_3, m_pending_extended[2], sizeof(test_addr_extended_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[3], sizeof(test_addr_extended_4));

    nrf_802154_ack_pending_bit_for_addr_reset(true);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_3, m_pending_extended[2], sizeof(test_addr_extended_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[3], sizeof(test_addr_extended_4));

    nrf_802154_ack_pending_bit_for_addr_reset(true);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_3, m_pending_extended[2], sizeof(test_addr_extended_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[3], sizeof(test_addr_extended_4));

    nrf_802154_ack_pending_bit_for_addr_reset(true);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_3, m_pending_extended[2], sizeof(test_addr_extended_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[3], sizeof(test_addr_extended_4));

    nrf_802154_ack_pending_bit_for_addr_reset(true);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_3, m_pending_extended[2], sizeof(test_addr_extended_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[3], sizeof(test_addr_extended_4));

    nrf_802154_ack_pending_bit_for_addr_reset(true);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_3, m_pending_extended[2], sizeof(test_addr_extended_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[3], sizeof(test_addr_extended_4));

    nrf_802154_ack_pending_bit_for_addr_reset(true);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_3, m_pending_extended[2], sizeof(test_addr_extended_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[3], sizeof(test_addr_extended_4));

    nrf_802154_ack_pending_bit_for_addr_reset(true);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_3, m_pending_extended[2], sizeof(test_addr_extended_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[3], sizeof(test_addr_extended_4));

    nrf_802154_ack_pending_bit_for_addr_reset(true);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_3, m_pending_extended[2], sizeof(test_addr_extended_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[3], sizeof(test_addr_extended_4));

    nrf_802154_ack_pending_bit_for_addr_reset(true);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_3, m_pending_extended[2], sizeof(test_addr_extended_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[3], sizeof(test_addr_extended_4));

    nrf_802154_ack_pending_bit_for_addr_reset(true);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_3, m_pending_extended[2], sizeof(test_addr_extended_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[3], sizeof(test_addr_extended_4));

    nrf_802154_ack_pending_bit_for_addr_reset(true);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_3, m_pending_extended[2], sizeof(test_addr_extended_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[3], sizeof(test_addr_extended_4));

    nrf_802154_ack_pending_bit_for_addr_reset(true);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_3, m_pending_extended[2], sizeof(test_addr_extended_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[3], sizeof(test_addr_extended_4));

    nrf_802154_ack_pending_bit_for_addr_reset(true);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_3, m_pending_extended[2], sizeof(test_addr_extended_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[3], sizeof(test_addr_extended_4));

    nrf_802154_ack_pending_bit_for_addr_reset(true);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_3, m_pending_extended[2], sizeof(test_addr_extended_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[3], sizeof(test_addr_extended_4));

    nrf_802154_ack_pending_bit_for_addr_reset(true);
}

void test_ShouldKeepAscendingOrderAfterAddingShortAddressToTheList(void)
{
    nrf_802154_ack_pending_bit_init();

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_3, m_pending_short[2], sizeof(test_addr_short_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[3], sizeof(test_addr_short_4));

    nrf_802154_ack_pending_bit_for_addr_reset(false);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_3, m_pending_short[2], sizeof(test_addr_short_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[3], sizeof(test_addr_short_4));

    nrf_802154_ack_pending_bit_for_addr_reset(false);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_3, m_pending_short[2], sizeof(test_addr_short_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[3], sizeof(test_addr_short_4));

    nrf_802154_ack_pending_bit_for_addr_reset(false);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_3, m_pending_short[2], sizeof(test_addr_short_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[3], sizeof(test_addr_short_4));

    nrf_802154_ack_pending_bit_for_addr_reset(false);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_3, m_pending_short[2], sizeof(test_addr_short_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[3], sizeof(test_addr_short_4));

    nrf_802154_ack_pending_bit_for_addr_reset(false);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_3, m_pending_short[2], sizeof(test_addr_short_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[3], sizeof(test_addr_short_4));

    nrf_802154_ack_pending_bit_for_addr_reset(false);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_3, m_pending_short[2], sizeof(test_addr_short_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[3], sizeof(test_addr_short_4));

    nrf_802154_ack_pending_bit_for_addr_reset(false);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_3, m_pending_short[2], sizeof(test_addr_short_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[3], sizeof(test_addr_short_4));

    nrf_802154_ack_pending_bit_for_addr_reset(false);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_3, m_pending_short[2], sizeof(test_addr_short_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[3], sizeof(test_addr_short_4));

    nrf_802154_ack_pending_bit_for_addr_reset(false);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_3, m_pending_short[2], sizeof(test_addr_short_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[3], sizeof(test_addr_short_4));

    nrf_802154_ack_pending_bit_for_addr_reset(false);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_3, m_pending_short[2], sizeof(test_addr_short_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[3], sizeof(test_addr_short_4));

    nrf_802154_ack_pending_bit_for_addr_reset(false);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_3, m_pending_short[2], sizeof(test_addr_short_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[3], sizeof(test_addr_short_4));

    nrf_802154_ack_pending_bit_for_addr_reset(false);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_3, m_pending_short[2], sizeof(test_addr_short_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[3], sizeof(test_addr_short_4));

    nrf_802154_ack_pending_bit_for_addr_reset(false);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_3, m_pending_short[2], sizeof(test_addr_short_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[3], sizeof(test_addr_short_4));

    nrf_802154_ack_pending_bit_for_addr_reset(false);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_3, m_pending_short[2], sizeof(test_addr_short_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[3], sizeof(test_addr_short_4));

    nrf_802154_ack_pending_bit_for_addr_reset(false);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_3, m_pending_short[2], sizeof(test_addr_short_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[3], sizeof(test_addr_short_4));

    nrf_802154_ack_pending_bit_for_addr_reset(false);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_3, m_pending_short[2], sizeof(test_addr_short_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[3], sizeof(test_addr_short_4));

    nrf_802154_ack_pending_bit_for_addr_reset(false);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_3, m_pending_short[2], sizeof(test_addr_short_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[3], sizeof(test_addr_short_4));

    nrf_802154_ack_pending_bit_for_addr_reset(false);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_3, m_pending_short[2], sizeof(test_addr_short_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[3], sizeof(test_addr_short_4));

    nrf_802154_ack_pending_bit_for_addr_reset(false);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_3, m_pending_short[2], sizeof(test_addr_short_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[3], sizeof(test_addr_short_4));

    nrf_802154_ack_pending_bit_for_addr_reset(false);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_3, m_pending_short[2], sizeof(test_addr_short_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[3], sizeof(test_addr_short_4));

    nrf_802154_ack_pending_bit_for_addr_reset(false);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_3, m_pending_short[2], sizeof(test_addr_short_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[3], sizeof(test_addr_short_4));

    nrf_802154_ack_pending_bit_for_addr_reset(false);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_3, m_pending_short[2], sizeof(test_addr_short_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[3], sizeof(test_addr_short_4));

    nrf_802154_ack_pending_bit_for_addr_reset(false);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_3, m_pending_short[2], sizeof(test_addr_short_3));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[3], sizeof(test_addr_short_4));

    nrf_802154_ack_pending_bit_for_addr_reset(false);
}

void test_ShouldNotAddAddressWhenAddressAlreadyOnTheList(void)
{
    nrf_802154_ack_pending_bit_init();

    bool result;

    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    TEST_ASSERT_TRUE(result);
    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(1, m_num_of_pending_extended);

    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    TEST_ASSERT_TRUE(result);
    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(1, m_num_of_pending_short);
}

void test_ShouldRemoveAddressFromTheList(void)
{
    nrf_802154_ack_pending_bit_init();

    bool result;

    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    TEST_ASSERT_TRUE(result);
    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);
    TEST_ASSERT_TRUE(result);
    result = nrf_802154_ack_pending_bit_for_addr_clear(test_addr_extended_1, true);
    TEST_ASSERT_TRUE(result);

    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    TEST_ASSERT_TRUE(result);
    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);
    TEST_ASSERT_TRUE(result);
    result = nrf_802154_ack_pending_bit_for_addr_clear(test_addr_short_2, false);
    TEST_ASSERT_TRUE(result);
}

void test_ShouldFailToRemoveAddressFromTheListWhenListIsEmpty(void)
{
    nrf_802154_ack_pending_bit_init();

    bool result;

    result = nrf_802154_ack_pending_bit_for_addr_clear(test_addr_extended_1, true);
    TEST_ASSERT_FALSE(result);

    result = nrf_802154_ack_pending_bit_for_addr_clear(test_addr_short_1, false);
    TEST_ASSERT_FALSE(result);
}

void test_ShouldKeepAscendingOrderAfterRemovingAddressFromTheList(void)
{
    nrf_802154_ack_pending_bit_init();

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_3, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_4, true);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);
    nrf_802154_ack_pending_bit_for_addr_clear(test_addr_extended_3, true);

    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_3, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_4, false);
    nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);
    nrf_802154_ack_pending_bit_for_addr_clear(test_addr_short_3, false);

    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_1, m_pending_extended[0], sizeof(test_addr_extended_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_2, m_pending_extended[1], sizeof(test_addr_extended_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_extended_4, m_pending_extended[2], sizeof(test_addr_extended_4));

    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_1, m_pending_short[0], sizeof(test_addr_short_1));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_2, m_pending_short[1], sizeof(test_addr_short_2));
    TEST_ASSERT_EQUAL_MEMORY(test_addr_short_4, m_pending_short[2], sizeof(test_addr_short_4));
}

void test_ShouldNotRemoveAddressWhenAddressNotOnTheList(void)
{
    nrf_802154_ack_pending_bit_init();

    bool result;

    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    TEST_ASSERT_TRUE(result);
    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_2, true);
    TEST_ASSERT_TRUE(result);
    result = nrf_802154_ack_pending_bit_for_addr_clear(test_addr_extended_3, true);
    TEST_ASSERT_FALSE(result);

    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    TEST_ASSERT_TRUE(result);
    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_2, false);
    TEST_ASSERT_TRUE(result);
    result = nrf_802154_ack_pending_bit_for_addr_clear(test_addr_short_3, false);
    TEST_ASSERT_FALSE(result);
}

void test_ShouldSetAckPendingBit(void)
{
    nrf_802154_ack_pending_bit_init();

    bool result;

    ///////////////////////////////////////////////////////////////////////////////////////////////

    result = nrf_802154_ack_pending_bit_should_be_set(test_psdu_extended);
    TEST_ASSERT_FALSE(result);

    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_extended_1, true);
    TEST_ASSERT_TRUE(result);

    result = nrf_802154_ack_pending_bit_should_be_set(test_psdu_extended);
    TEST_ASSERT_TRUE(result);

    ///////////////////////////////////////////////////////////////////////////////////////////////

    result = nrf_802154_ack_pending_bit_should_be_set(test_psdu_short);
    TEST_ASSERT_FALSE(result);

    result = nrf_802154_ack_pending_bit_for_addr_set(test_addr_short_1, false);
    TEST_ASSERT_TRUE(result);

    result = nrf_802154_ack_pending_bit_should_be_set(test_psdu_short);
    TEST_ASSERT_TRUE(result);
}
