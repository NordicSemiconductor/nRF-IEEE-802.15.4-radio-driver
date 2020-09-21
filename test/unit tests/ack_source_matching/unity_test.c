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

#include "nrf_802154_const.h"
#include "mock_nrf_802154.h"
#include "mock_nrf_802154_frame_parser.h"

#include "mac_features/ack_generator/nrf_802154_ack_data.c"

#define FCF_BYTE_0_DATA 0x41
#define FCF_BYTE_0_CMD  0x43

/***********************************************************************************/
/********************** ACK PENDING BIT SOURCE MATCHING TESTS **********************/
/***********************************************************************************/
static uint8_t test_psdu_extended[EXTENDED_ADDRESS_SIZE + SRC_ADDR_OFFSET_EXTENDED_DST + 1] = 
{
    0x00, FCF_BYTE_0_CMD, 0xcc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89,
    MAC_CMD_DATA_REQ
};

static uint8_t test_psdu_short[SHORT_ADDRESS_SIZE + SRC_ADDR_OFFSET_SHORT_DST + 1] = 
{
    0x00, FCF_BYTE_0_CMD, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x23, 
    MAC_CMD_DATA_REQ 
};

static uint8_t test_addr_extended_1[EXTENDED_ADDRESS_SIZE] = { 0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89 };
static uint8_t test_addr_short_1[SHORT_ADDRESS_SIZE]       = { 0x12, 0x23 };

static nrf_802154_frame_parser_mhr_data_t test_mhr_data_short    = 
{
    .p_src_addr = &(test_psdu_short[8]),
    .src_addr_size = SHORT_ADDRESS_SIZE,
    .addressing_end_offset = 10
};

static nrf_802154_frame_parser_mhr_data_t test_mhr_data_extended = 
{
    .p_src_addr = test_addr_extended_1,
    .src_addr_size = EXTENDED_ADDRESS_SIZE,
    .addressing_end_offset = 22
};

static bool test_extended_flag = false;
/***********************************************************************************/
/***********************************************************************************/
/***********************************************************************************/

// Default state before test case - correct payload, address not on list, checking enabled.
void setUp(void)
{    
    nrf_802154_ack_data_enable(true);
    
}

void tearDown(void)
{
    memset(m_pending_bit.extended_addr, 0, sizeof(m_pending_bit.extended_addr));
    memset(m_pending_bit.short_addr, 0xff, sizeof(m_pending_bit.short_addr));

    test_psdu_extended[test_mhr_data_extended.addressing_end_offset] = MAC_CMD_DATA_REQ;
    test_psdu_short[test_mhr_data_short.addressing_end_offset]       = MAC_CMD_DATA_REQ;
    test_psdu_extended[FRAME_TYPE_OFFSET]                            = FCF_BYTE_0_CMD;
    test_psdu_short[FRAME_TYPE_OFFSET]                               = FCF_BYTE_0_CMD;
}

/***********************************************************************************/
/***********************************************************************************/
/***********************************************************************************/

// Standard compliant - always returns true.
void test_StandardCompliantAlwaysTrue()
{   
    bool res;
    nrf_802154_ack_data_src_addr_matching_method_set(NRF_802154_SRC_ADDR_MATCH_ALWAYS_1);

    res = nrf_802154_ack_data_pending_bit_should_be_set(test_psdu_short);
    TEST_ASSERT_TRUE(res);
}

// Thread - if module disabled, always returns true.
void test_ThreadPendingDisabled()
{
    bool res;
    nrf_802154_ack_data_src_addr_matching_method_set(NRF_802154_SRC_ADDR_MATCH_THREAD);
    nrf_802154_ack_data_enable(false);
    test_extended_flag = false;

    nrf_802154_frame_parser_src_addr_get_ExpectAndReturn(test_psdu_short, &test_extended_flag, test_addr_short_1);
    nrf_802154_frame_parser_src_addr_get_IgnoreArg_p_src_addr_extended();
    nrf_802154_frame_parser_src_addr_get_ReturnThruPtr_p_src_addr_extended(&test_extended_flag);

    res = nrf_802154_ack_data_pending_bit_should_be_set(test_psdu_short);
    TEST_ASSERT_TRUE(res);
}

// Thread - if no source address found, always returns true.
void test_ThreadSourceAddressNULL()
{
    bool res;
    nrf_802154_ack_data_src_addr_matching_method_set(NRF_802154_SRC_ADDR_MATCH_THREAD);
    test_extended_flag = false;

    nrf_802154_frame_parser_src_addr_get_ExpectAndReturn(test_psdu_short, &test_extended_flag, NULL);
    nrf_802154_frame_parser_src_addr_get_IgnoreArg_p_src_addr_extended();
    nrf_802154_frame_parser_src_addr_get_ReturnThruPtr_p_src_addr_extended(&test_extended_flag);

    res = nrf_802154_ack_data_pending_bit_should_be_set(test_psdu_short);
    TEST_ASSERT_TRUE(res);
}

// Thread - if short address found on list, return true.
void test_ThreadShortAddressOnList()
{
    bool res;
    nrf_802154_ack_data_src_addr_matching_method_set(NRF_802154_SRC_ADDR_MATCH_THREAD);
    test_extended_flag = false;

    res = nrf_802154_ack_data_for_addr_set(test_addr_short_1, test_extended_flag, NRF_802154_ACK_DATA_PENDING_BIT, NULL, 0);
    TEST_ASSERT_TRUE(res);

    nrf_802154_frame_parser_src_addr_get_ExpectAndReturn(test_psdu_short, &test_extended_flag, test_addr_short_1);
    nrf_802154_frame_parser_src_addr_get_IgnoreArg_p_src_addr_extended();
    nrf_802154_frame_parser_src_addr_get_ReturnThruPtr_p_src_addr_extended(&test_extended_flag);

    res = nrf_802154_ack_data_pending_bit_should_be_set(test_psdu_short);
    TEST_ASSERT_TRUE(res);
}

// Thread - if extended address found on list, return true.
void test_ThreadExtAddressOnList()
{
    bool res;
    nrf_802154_ack_data_src_addr_matching_method_set(NRF_802154_SRC_ADDR_MATCH_THREAD);
    test_extended_flag = true;

    res = nrf_802154_ack_data_for_addr_set(test_addr_extended_1, test_extended_flag, NRF_802154_ACK_DATA_PENDING_BIT, NULL, 0);
    TEST_ASSERT_TRUE(res);

    nrf_802154_frame_parser_src_addr_get_ExpectAndReturn(test_psdu_extended, &test_extended_flag, test_addr_extended_1);
    nrf_802154_frame_parser_src_addr_get_IgnoreArg_p_src_addr_extended();
    nrf_802154_frame_parser_src_addr_get_ReturnThruPtr_p_src_addr_extended(&test_extended_flag);

    res = nrf_802154_ack_data_pending_bit_should_be_set(test_psdu_extended);
    TEST_ASSERT_TRUE(res);
}

// Thread - if short address not found on list, return false.
void test_ThreadShortAddressNotOnList()
{
    bool res;
    nrf_802154_ack_data_src_addr_matching_method_set(NRF_802154_SRC_ADDR_MATCH_THREAD);
    test_extended_flag = false;

    nrf_802154_frame_parser_src_addr_get_ExpectAndReturn(test_psdu_short, &test_extended_flag, test_addr_short_1);
    nrf_802154_frame_parser_src_addr_get_IgnoreArg_p_src_addr_extended();
    nrf_802154_frame_parser_src_addr_get_ReturnThruPtr_p_src_addr_extended(&test_extended_flag);

    res = nrf_802154_ack_data_pending_bit_should_be_set(test_psdu_short);
    TEST_ASSERT_FALSE(res);
}

// Thread - if extended address not found on list, return false.
void test_ThreadExtAddressNotOnList()
{
    bool res;
    nrf_802154_ack_data_src_addr_matching_method_set(NRF_802154_SRC_ADDR_MATCH_THREAD);
    test_extended_flag = true;

    nrf_802154_frame_parser_src_addr_get_ExpectAndReturn(test_psdu_extended, &test_extended_flag, test_addr_extended_1);
    nrf_802154_frame_parser_src_addr_get_IgnoreArg_p_src_addr_extended();
    nrf_802154_frame_parser_src_addr_get_ReturnThruPtr_p_src_addr_extended(&test_extended_flag);

    res = nrf_802154_ack_data_pending_bit_should_be_set(test_psdu_extended);
    TEST_ASSERT_FALSE(res);
}

// Zigbee - if module disabled - return true.
void test_ZigbeePendingDisabled()
{
    bool res;
    nrf_802154_ack_data_src_addr_matching_method_set(NRF_802154_SRC_ADDR_MATCH_ZIGBEE);
    nrf_802154_ack_data_enable(false);

    res = nrf_802154_ack_data_pending_bit_should_be_set(test_psdu_short);
    TEST_ASSERT_TRUE(res);
}

// Zigbee - if mhr parsing returns false, return true.
void test_ZigbeeParseFailed()
{
    bool res;
    nrf_802154_ack_data_src_addr_matching_method_set(NRF_802154_SRC_ADDR_MATCH_ZIGBEE);

    nrf_802154_frame_parser_mhr_parse_ExpectAndReturn(test_psdu_short, &test_mhr_data_short, false);
    nrf_802154_frame_parser_mhr_parse_IgnoreArg_p_fields();
    nrf_802154_frame_parser_mhr_parse_ReturnThruPtr_p_fields(&test_mhr_data_short);

    res = nrf_802154_ack_data_pending_bit_should_be_set(test_psdu_short);
    TEST_ASSERT_TRUE(res);
}

// Zigbee - if frame is not MAC command frame, return false.
void test_ZigbeeTypeNotCommand()
{
    bool res;
    nrf_802154_ack_data_src_addr_matching_method_set(NRF_802154_SRC_ADDR_MATCH_ZIGBEE);
    test_psdu_short[FRAME_TYPE_OFFSET] = FCF_BYTE_0_DATA;

    nrf_802154_frame_parser_mhr_parse_ExpectAndReturn(test_psdu_short, &test_mhr_data_short, true);
    nrf_802154_frame_parser_mhr_parse_IgnoreArg_p_fields();
    nrf_802154_frame_parser_mhr_parse_ReturnThruPtr_p_fields(&test_mhr_data_short);

    res = nrf_802154_ack_data_pending_bit_should_be_set(test_psdu_short);
    TEST_ASSERT_FALSE(res);
}

// Zigbee - if command is not data request, return false.
// Checked for both short and extended, as offsets differ.
void test_ZigbeeCommandNotRequest()
{
    bool res;
    nrf_802154_ack_data_src_addr_matching_method_set(NRF_802154_SRC_ADDR_MATCH_ZIGBEE);

    // Short
    test_psdu_short[test_mhr_data_short.addressing_end_offset] = MAC_CMD_BEACON_REQ;

    nrf_802154_frame_parser_mhr_parse_ExpectAndReturn(test_psdu_short, &test_mhr_data_short, true);
    nrf_802154_frame_parser_mhr_parse_IgnoreArg_p_fields();
    nrf_802154_frame_parser_mhr_parse_ReturnThruPtr_p_fields(&test_mhr_data_short);

    res = nrf_802154_ack_data_pending_bit_should_be_set(test_psdu_short);
    TEST_ASSERT_FALSE(res);
    
    // Extended
    test_psdu_extended[test_mhr_data_extended.addressing_end_offset] = MAC_CMD_BEACON_REQ;

    nrf_802154_frame_parser_mhr_parse_ExpectAndReturn(test_psdu_extended, &test_mhr_data_extended, true);
    nrf_802154_frame_parser_mhr_parse_IgnoreArg_p_fields();
    nrf_802154_frame_parser_mhr_parse_ReturnThruPtr_p_fields(&test_mhr_data_extended);

    res = nrf_802154_ack_data_pending_bit_should_be_set(test_psdu_extended);
    TEST_ASSERT_FALSE(res);
}

// Zigbee - if frame is on the list, return false if address is short address, true otherwise.
void test_ZigbeeAddressOnList()
{
    bool res;
    nrf_802154_ack_data_src_addr_matching_method_set(NRF_802154_SRC_ADDR_MATCH_ZIGBEE);

    // Short
    test_extended_flag = false;
    res = nrf_802154_ack_data_for_addr_set(test_addr_short_1, test_extended_flag, NRF_802154_ACK_DATA_PENDING_BIT, NULL, 0);
    TEST_ASSERT_TRUE(res);

    nrf_802154_frame_parser_mhr_parse_ExpectAndReturn(test_psdu_short, &test_mhr_data_short, true);
    nrf_802154_frame_parser_mhr_parse_IgnoreArg_p_fields();
    nrf_802154_frame_parser_mhr_parse_ReturnThruPtr_p_fields(&test_mhr_data_short);

    res = nrf_802154_ack_data_pending_bit_should_be_set(test_psdu_short);
    TEST_ASSERT_FALSE(res);

    // Extended
    test_extended_flag = true;
    res = nrf_802154_ack_data_for_addr_set(test_addr_extended_1, test_extended_flag, NRF_802154_ACK_DATA_PENDING_BIT, NULL, 0);
    TEST_ASSERT_TRUE(res);

    nrf_802154_frame_parser_mhr_parse_ExpectAndReturn(test_psdu_extended, &test_mhr_data_extended, true);
    nrf_802154_frame_parser_mhr_parse_IgnoreArg_p_fields();
    nrf_802154_frame_parser_mhr_parse_ReturnThruPtr_p_fields(&test_mhr_data_extended);

    res = nrf_802154_ack_data_pending_bit_should_be_set(test_psdu_extended);
    TEST_ASSERT_TRUE(res);
}

// Zigbee - if frame is not on the list, return true.
// Checked for both short and extended address, as decision path differs.
void test_ZigbeeAddressNotOnList()
{
    bool res;
    nrf_802154_ack_data_src_addr_matching_method_set(NRF_802154_SRC_ADDR_MATCH_ZIGBEE);

    // Short
    test_extended_flag = false;

    nrf_802154_frame_parser_mhr_parse_ExpectAndReturn(test_psdu_short, &test_mhr_data_short, true);
    nrf_802154_frame_parser_mhr_parse_IgnoreArg_p_fields();
    nrf_802154_frame_parser_mhr_parse_ReturnThruPtr_p_fields(&test_mhr_data_short);

    res = nrf_802154_ack_data_pending_bit_should_be_set(test_psdu_short);
    TEST_ASSERT_TRUE(res);

    // Extended
    test_extended_flag = true;

    nrf_802154_frame_parser_mhr_parse_ExpectAndReturn(test_psdu_extended, &test_mhr_data_extended, true);
    nrf_802154_frame_parser_mhr_parse_IgnoreArg_p_fields();
    nrf_802154_frame_parser_mhr_parse_ReturnThruPtr_p_fields(&test_mhr_data_extended);

    res = nrf_802154_ack_data_pending_bit_should_be_set(test_psdu_extended);
    TEST_ASSERT_TRUE(res);
}
