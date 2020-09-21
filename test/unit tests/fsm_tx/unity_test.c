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

#include <stdlib.h>

#include "unity.h"

#include "nrf_802154_config.h"
#include "nrf_802154_const.h"
#include "mock_nrf_802154.h"
#include "mock_nrf_802154_ack_data.h"
#include "mock_nrf_802154_core_hooks.h"
#include "mock_nrf_802154_critical_section.h"
#include "mock_nrf_802154_debug.h"
#include "mock_nrf_802154_frame_parser.h"
#include "mock_nrf_802154_notification.h"
#include "mock_nrf_802154_pib.h"
#include "mock_nrf_802154_priority_drop.h"
#include "mock_nrf_802154_procedures_duration.h"
#include "mock_nrf_802154_rsch.h"
#include "mock_nrf_802154_rssi.h"
#include "mock_nrf_802154_rx_buffer.h"
#include "mock_nrf_802154_timer_coord.h"
#include "mock_nrf_fem_protocol_api.h"
#include "mock_nrf_radio.h"
#include "mock_nrf_timer.h"
#include "mock_nrf_egu.h"
#include "mock_nrf_ppi.h"

#define __ISB()
#define __LDREXB(ptr)           0
#define __STREXB(value, ptr)    0

#include "nrf_802154_core.c"


/***********************************************************************************/
/***********************************************************************************/
/***********************************************************************************/

static rx_buffer_t m_rx_buffer;
static uint8_t     m_tx_buffer[MAX_PACKET_SIZE + 1];

static void insert_frame_with_noack_to_tx_buffer(void)
{
    mp_tx_data = m_tx_buffer;
    m_tx_buffer[0] = 16;
    m_tx_buffer[1] = 0x41;
    m_tx_buffer[2] = 0x98;
}

static void insert_frame_with_ack_request_to_tx_buffer(void)
{
    mp_tx_data = m_tx_buffer;
    m_tx_buffer[0] = 16;
    m_tx_buffer[1] = 0x61;
    m_tx_buffer[2] = 0x98;
}

static void insert_frame_2015_with_ack_request_to_tx_buffer(void)
{
    mp_tx_data = m_tx_buffer;
    m_tx_buffer[0] = 16;
    m_tx_buffer[1] = 0x61;
    m_tx_buffer[2] = 0xa8;
}

static void insert_ack_2015_to_rx_buffer(void)
{
    m_rx_buffer.data[0] = 5;
    m_rx_buffer.data[1] = 0x42;
    m_rx_buffer.data[2] = 0x28;
}

static void mark_rx_buffer_occupied(void)
{
    mp_current_rx_buffer = &m_rx_buffer;

    m_rx_buffer.free = false;
}

static void mark_rx_buffer_free(void)
{
    mp_current_rx_buffer = &m_rx_buffer;

    m_rx_buffer.free = true;
}

static void verify_setting_tx_packet_ptr(void)
{
    nrf_radio_packetptr_set_Expect(m_tx_buffer);
}

static void verify_setting_tx_power(void)
{
    int8_t tx_power = rand();
    nrf_802154_pib_tx_power_get_ExpectAndReturn(tx_power);
    nrf_radio_txpower_set_Expect(tx_power);
}

static void verify_transmitted_notification_noack(void)
{
    nrf_802154_critical_section_nesting_allow_Expect();

    nrf_802154_core_hooks_transmitted_Expect(m_tx_buffer);
    nrf_802154_notify_transmitted_Expect(m_tx_buffer, NULL, 0, 0);

    nrf_802154_critical_section_nesting_deny_Expect();
}

static void verify_transmit_failed_notification(nrf_802154_tx_error_t error)
{
    nrf_802154_critical_section_nesting_allow_Expect();

    nrf_802154_core_hooks_tx_failed_ExpectAndReturn(m_tx_buffer, error, true);
    nrf_802154_notify_transmit_failed_Expect(m_tx_buffer, error);

    nrf_802154_critical_section_nesting_deny_Expect();
}

static void verify_transmitted_notification_ack(void)
{
    // TODO: Verify LQI
    int8_t rssi = rand();
    int8_t corrected_rssi = rand();
    uint8_t corrected_lqi = rand();
    uint32_t expected_lqi = corrected_lqi * 4;

    if (expected_lqi > UINT8_MAX)
    {
        expected_lqi = UINT8_MAX;
    }

    nrf_radio_rssi_sample_get_ExpectAndReturn(rssi);
    nrf_802154_rssi_sample_corrected_get_ExpectAndReturn(rssi, corrected_rssi);
    nrf_802154_rssi_lqi_corrected_get_IgnoreAndReturn(corrected_lqi);

    nrf_802154_critical_section_nesting_allow_Expect();

    nrf_802154_core_hooks_transmitted_Expect(m_tx_buffer);
    nrf_802154_notify_transmitted_Expect(m_tx_buffer,
                                         m_rx_buffer.data,
                                         -corrected_rssi,
                                         expected_lqi);

    nrf_802154_critical_section_nesting_deny_Expect();
}

static void verify_receive_begin_setup(uint32_t shorts)
{
    uint32_t event_addr;
    uint32_t task_addr;
    uint32_t task2_addr;
    uint32_t lna_target_time;
    uint32_t pa_target_time;

    // RADIO setup
    nrf_radio_shorts_set_Expect(shorts);
    nrf_radio_bcc_set_Expect(24);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_CRCERROR);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_BCMATCH);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_CRCOK);
    nrf_radio_int_enable_Expect(NRF_RADIO_INT_CRCERROR_MASK |
                                NRF_RADIO_INT_BCMATCH_MASK  |
                                NRF_RADIO_INT_CRCOK_MASK);

    // FEM setup
    uint32_t delta_time = rand();

    nrf_timer_shorts_enable_Expect(NRF_802154_TIMER_INSTANCE,
                                   NRF_TIMER_SHORT_COMPARE0_STOP_MASK);
    nrf_802154_fal_lna_configuration_set_ExpectAndReturn(&m_activate_rx_cc0, NULL, NRF_SUCCESS);
    nrf_timer_cc_read_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL0, delta_time);
    nrf_timer_cc_write_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL1, delta_time + ACK_IFS - TXRU_TIME - EVENT_LAT);

    // Clear EVENTS that are checked later
    nrf_egu_event_clear_Expect(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT);

    // PPIs setup
    task2_addr = rand();
    nrf_ppi_task_address_get_ExpectAndReturn(PPI_CHGRP0_DIS_TASK,
                                             (uint32_t *)task2_addr);
    task_addr = rand();
    nrf_radio_task_address_get_ExpectAndReturn(NRF_RADIO_TASK_RXEN, task_addr);
    event_addr = rand();
    nrf_egu_event_address_get_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE,
                                              EGU_EVENT,
                                              (uint32_t *)event_addr);
    nrf_ppi_channel_and_fork_endpoint_setup_Expect(PPI_EGU_RAMP_UP, event_addr, task_addr, task2_addr);

    task_addr = rand();
    nrf_timer_task_address_get_ExpectAndReturn(NRF_802154_TIMER_INSTANCE,
                                               NRF_TIMER_TASK_START,
                                               (uint32_t *)task_addr);
    event_addr = rand();
    nrf_egu_event_address_get_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE,
                                              EGU_EVENT,
                                              (uint32_t *)event_addr);
    nrf_ppi_channel_endpoint_setup_Expect(PPI_EGU_TIMER_START, event_addr, task_addr);

    nrf_ppi_channel_include_in_group_Expect(PPI_EGU_RAMP_UP, PPI_CHGRP0);

    task_addr = rand();
    nrf_egu_task_address_get_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_TASK, (uint32_t *)task_addr);
    event_addr = rand();
    nrf_radio_event_address_get_ExpectAndReturn(NRF_RADIO_EVENT_DISABLED, event_addr);
    nrf_ppi_channel_endpoint_setup_Expect(PPI_DISABLED_EGU, event_addr, task_addr);

    nrf_ppi_channel_enable_Expect(PPI_EGU_RAMP_UP);
    nrf_ppi_channel_enable_Expect(PPI_EGU_TIMER_START);
    nrf_ppi_channel_enable_Expect(PPI_DISABLED_EGU);

    event_addr = rand();
    nrf_radio_event_address_get_ExpectAndReturn(NRF_RADIO_EVENT_CRCOK, event_addr);
    nrf_802154_timer_coord_timestamp_prepare_Expect(event_addr);
}

static void verify_receive_begin_finds_free_buffer(void)
{
    nrf_802154_rx_buffer_free_find_ExpectAndReturn(NULL);
}

static void verify_complete_receive_begin(bool free_buffer)
{
    uint32_t shorts = NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                      NRF_RADIO_SHORT_END_DISABLE_MASK       |
                      NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK;

    m_rsch_timeslot_is_granted = true;
    verify_setting_tx_power();

    if (free_buffer)
    {
        nrf_radio_packetptr_set_Expect(&m_rx_buffer.data);
        shorts |= NRF_RADIO_SHORT_RXREADY_START_MASK;
    }

    verify_receive_begin_setup(shorts);
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_TXDISABLE);

    if (!free_buffer)
    {
        verify_receive_begin_finds_free_buffer();
    }
}

static void verify_complete_ack_matching_enable(void)
{
    uint8_t sequence_number = rand();
    m_tx_buffer[DSN_OFFSET] = sequence_number;

    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_MHRMATCH);
    nrf_radio_mhmu_search_pattern_set_Expect(MHMU_PATTERN |
                                             ((uint32_t)sequence_number << MHMU_PATTERN_DSN_OFFSET));
}

static void verify_complete_ack_matching_disable(void)
{
    nrf_radio_mhmu_search_pattern_set_Expect(0);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_MHRMATCH);
}

static void verify_complete_ack_is_matched(void)
{
    nrf_radio_event_check_ExpectAndReturn(NRF_RADIO_EVENT_MHRMATCH, true);
    nrf_radio_crc_status_check_ExpectAndReturn(true);
}

static void verify_complete_ack_is_not_matched(void)
{
    nrf_radio_event_check_ExpectAndReturn(NRF_RADIO_EVENT_MHRMATCH, false);
}

static void verify_clearing_fem_config(void)
{
    nrf_802154_fal_lna_configuration_clear_ExpectAndReturn(NULL, &m_deactivate_on_disable, NRF_SUCCESS);
    nrf_802154_fal_lna_configuration_clear_ExpectAndReturn(NULL, &m_deactivate_on_disable, NRF_SUCCESS);
}

void setUp(void)
{
    m_rsch_timeslot_is_granted = true;
}

void tearDown(void)
{

}

/***************************************************************************************************
 * @section Notifications
 **************************************************************************************************/

void nrf_802154_rx_started(void){}
void nrf_802154_tx_started(const uint8_t * p_frame){}
void nrf_802154_rx_ack_started(void){}
void nrf_802154_tx_ack_started(const uint8_t * p_data){}

/***************************************************************************************************
 * @section Transmit begin function
 **************************************************************************************************/

static void verify_timeslot_request(bool cca, bool result)
{
    uint16_t duration = rand();
    bool     ack_req  = m_tx_buffer[ACK_REQUEST_OFFSET] & ACK_REQUEST_BIT;

    nrf_802154_frame_parser_ar_bit_is_set_ExpectAndReturn(m_tx_buffer, ack_req);

    nrf_802154_tx_duration_get_ExpectAndReturn(m_tx_buffer[0],
                                                        cca,
                                                        ack_req,
                                                        duration);
    nrf_802154_rsch_timeslot_request_ExpectAndReturn(duration, result);
}

// TODO: Instead request timeslot for transmission. If timeslot is not granted reset radio periph.
// Handle cases when timeslot is not granted

void test_transmit_begin_ShallDoNothingIfTimeslotIsNotGranted(void)
{
    verify_timeslot_request(false, false);

    tx_init(m_tx_buffer, false, false);
}

// Basic peripheral setup if CCA is not performed

static void verify_transmit_begin_periph_setup(bool cca)
{
    uint32_t event_addr;
    uint32_t task_addr1;
    uint32_t task_addr2;

    verify_timeslot_request(cca, true);

    verify_setting_tx_power();
    verify_setting_tx_packet_ptr();


    if (cca)
    {
        nrf_radio_shorts_set_Expect(NRF_RADIO_SHORT_RXREADY_CCASTART_MASK |
                                    NRF_RADIO_SHORT_CCABUSY_DISABLE_MASK  |
                                    NRF_RADIO_SHORT_CCAIDLE_TXEN_MASK     |
                                    NRF_RADIO_SHORT_TXREADY_START_MASK    |
                                    NRF_RADIO_SHORT_PHYEND_DISABLE_MASK);
    }
    else
    {
        nrf_radio_shorts_set_Expect(NRF_RADIO_SHORT_TXREADY_START_MASK |
                                    NRF_RADIO_SHORT_PHYEND_DISABLE_MASK);
    }

    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_PHYEND);
    if (cca)
    {
        nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_CCABUSY);
    }
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_ADDRESS);

    if (cca)
    {
        nrf_radio_int_enable_Expect(NRF_RADIO_INT_PHYEND_MASK  |
                                    NRF_RADIO_INT_ADDRESS_MASK |
                                    NRF_RADIO_INT_CCABUSY_MASK);
    }
    else
    {
        nrf_radio_int_enable_Expect(NRF_RADIO_INT_PHYEND_MASK | NRF_RADIO_INT_ADDRESS_MASK);
    }

    // Set timer and PPIs for FEM
    if (cca)
    {
        nrf_802154_fal_lna_configuration_set_ExpectAndReturn(&m_activate_rx_cc0, &m_ccaidle, NRF_SUCCESS);
        nrf_802154_fal_pa_configuration_set_ExpectAndReturn(&m_ccaidle, NULL, NRF_SUCCESS);
    }
    else
    {
        nrf_802154_fal_pa_configuration_set_ExpectAndReturn(&m_activate_tx_cc0, NULL, NRF_SUCCESS);
    }

    task_addr1 = rand();
    event_addr = rand();
    nrf_timer_shorts_enable_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_SHORT_COMPARE0_STOP_MASK);

    nrf_egu_event_address_get_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, (uint32_t *)event_addr);
    nrf_timer_task_address_get_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_START, (uint32_t *)task_addr1);

    nrf_ppi_channel_endpoint_setup_Expect(PPI_EGU_TIMER_START, event_addr, task_addr1);
    nrf_ppi_channel_enable_Expect(PPI_EGU_TIMER_START);

    nrf_egu_event_clear_Expect(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT);

    task_addr2 = rand();
    nrf_ppi_task_address_get_ExpectAndReturn(PPI_CHGRP0_DIS_TASK, (uint32_t *)task_addr2);
    task_addr1 = rand();
    nrf_radio_task_address_get_ExpectAndReturn(cca ? NRF_RADIO_TASK_RXEN : NRF_RADIO_TASK_TXEN,
                                               task_addr1);
    event_addr = rand();
    nrf_egu_event_address_get_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, (uint32_t *)event_addr);
    nrf_ppi_channel_and_fork_endpoint_setup_Expect(PPI_EGU_RAMP_UP, event_addr, task_addr1, task_addr2);

    task_addr1 = rand();
    nrf_egu_task_address_get_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_TASK, (uint32_t *)task_addr1);
    event_addr = rand();
    nrf_radio_event_address_get_ExpectAndReturn(NRF_RADIO_EVENT_DISABLED, event_addr);
    nrf_ppi_channel_endpoint_setup_Expect(PPI_DISABLED_EGU, event_addr, task_addr1);

    nrf_ppi_channel_include_in_group_Expect(PPI_EGU_RAMP_UP, PPI_CHGRP0);

    nrf_ppi_channel_enable_Expect(PPI_EGU_RAMP_UP);
    nrf_ppi_channel_enable_Expect(PPI_DISABLED_EGU);
}

void test_transmit_begin_ShallPrepareHardwareToTransmit(void)
{
    uint32_t event_addr;
    uint32_t task_addr1;
    uint32_t task_addr2;

    verify_timeslot_request(false, true);

    verify_setting_tx_power();
    verify_setting_tx_packet_ptr();

    nrf_radio_shorts_set_Expect(NRF_RADIO_SHORT_TXREADY_START_MASK |
                                NRF_RADIO_SHORT_PHYEND_DISABLE_MASK);

    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_PHYEND);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_ADDRESS);
    nrf_radio_int_enable_Expect(NRF_RADIO_INT_PHYEND_MASK | NRF_RADIO_INT_ADDRESS_MASK);

    // Set timer and PPIs for FEM
    task_addr1 = rand();
    event_addr = rand();

    nrf_802154_fal_pa_configuration_set_ExpectAndReturn(&m_activate_tx_cc0, NULL, NRF_SUCCESS);
    nrf_timer_shorts_enable_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_SHORT_COMPARE0_STOP_MASK);

    nrf_egu_event_address_get_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, (uint32_t *)event_addr);
    nrf_timer_task_address_get_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_START, (uint32_t *)task_addr1);
    nrf_ppi_channel_endpoint_setup_Expect(PPI_EGU_TIMER_START, event_addr, task_addr1);
    nrf_ppi_channel_enable_Expect(PPI_EGU_TIMER_START);

    nrf_egu_event_clear_Expect(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT);

    task_addr2 = rand();
    nrf_ppi_task_address_get_ExpectAndReturn(PPI_CHGRP0_DIS_TASK, (uint32_t *)task_addr2);
    task_addr1 = rand();
    nrf_radio_task_address_get_ExpectAndReturn(NRF_RADIO_TASK_TXEN, task_addr1);
    event_addr = rand();
    nrf_egu_event_address_get_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, (uint32_t *)event_addr);
    nrf_ppi_channel_and_fork_endpoint_setup_Expect(PPI_EGU_RAMP_UP, event_addr, task_addr1, task_addr2);

    task_addr1 = rand();
    nrf_egu_task_address_get_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_TASK, (uint32_t *)task_addr1);
    event_addr = rand();
    nrf_radio_event_address_get_ExpectAndReturn(NRF_RADIO_EVENT_DISABLED, event_addr);
    nrf_ppi_channel_endpoint_setup_Expect(PPI_DISABLED_EGU, event_addr, task_addr1);

    nrf_ppi_channel_include_in_group_Expect(PPI_EGU_RAMP_UP, PPI_CHGRP0);

    nrf_ppi_channel_enable_Expect(PPI_EGU_RAMP_UP);
    nrf_ppi_channel_enable_Expect(PPI_DISABLED_EGU);

    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    tx_init(m_tx_buffer, false, false);
}

// Basic peripheral setup if CCA is performed

void test_transmit_begin_ShallPrepareHardwareToCcaAndTransmit(void)
{
    verify_transmit_begin_periph_setup(true);

    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    tx_init(m_tx_buffer, true, false);
}

// Verify how detection of PPI status works

void test_transmit_begin_ShallTriggerDisableIfRequestedByArgument(void)
{
    verify_transmit_begin_periph_setup(false);

    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    tx_init(m_tx_buffer, false, false);
}

void test_transmit_begin_ShallNotTriggerDisableIfRadioIsRampingDown(void)
{
    verify_transmit_begin_periph_setup(false);

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_RXDISABLE);

    tx_init(m_tx_buffer, false, true);
}

void test_transmit_begin_ShallNotTriggerDisableIfEguEventIsSet(void)
{
    verify_transmit_begin_periph_setup(false);

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_DISABLED);
    nrf_egu_event_check_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, true);

    tx_init(m_tx_buffer, false, true);
}

void test_transmit_begin_ShallTriggerDisableIfRadioIsDisabledAndEguDidNotWork(void)
{
    verify_transmit_begin_periph_setup(false);

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_DISABLED);
    nrf_egu_event_check_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, false);
    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    tx_init(m_tx_buffer, false, true);
}

/***************************************************************************************************
 * @section Transmit terminate function
 **************************************************************************************************/

static void verify_tx_terminate_periph_reset(bool in_timeslot)
{
    m_state = RADIO_STATE_TX;

    nrf_ppi_channel_disable_Expect(PPI_DISABLED_EGU);
    nrf_ppi_channel_disable_Expect(PPI_EGU_RAMP_UP);

    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);
    nrf_timer_shorts_disable_Expect(NRF_802154_TIMER_INSTANCE,
                             NRF_TIMER_SHORT_COMPARE0_STOP_MASK |
                             NRF_TIMER_SHORT_COMPARE1_STOP_MASK);

    nrf_802154_fal_pa_configuration_clear_ExpectAndReturn(&m_activate_tx_cc0, NULL, NRF_SUCCESS);

    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);
    nrf_ppi_channel_disable_Expect(PPI_EGU_TIMER_START);

    nrf_ppi_channel_remove_from_group_Expect(PPI_EGU_RAMP_UP, PPI_CHGRP0);
    nrf_ppi_fork_endpoint_setup_Expect(PPI_EGU_RAMP_UP, 0);

    m_rsch_timeslot_is_granted = in_timeslot;

    if (in_timeslot)
    {
        nrf_radio_int_disable_Expect(NRF_RADIO_INT_CCABUSY_MASK |
                                     NRF_RADIO_INT_PHYEND_MASK  |
                                     NRF_RADIO_INT_ADDRESS_MASK);
        nrf_radio_shorts_set_Expect(0);
        nrf_fem_prepare_powerdown_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL0, PPI_EGU_TIMER_START, false);
        nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_CCASTOP);
        nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);
    }
}

void test_tx_terminate_ShallNotModifyRadioRegistersOutOfTimeslot(void)
{
    verify_tx_terminate_periph_reset(false);

    tx_terminate();
}

void test_tx_terminate_ShallResetPeriphAndTriggerDisableTask(void)
{
    verify_tx_terminate_periph_reset(true);

    tx_terminate();
}

/***************************************************************************************************
 * @section CCABUSY handler
 **************************************************************************************************/

void test_ccabusy_handler_ShallResetToRxStateAndNotifyFailure(void)
{
    insert_frame_with_noack_to_tx_buffer();

    verify_tx_terminate_periph_reset(true);
    verify_complete_receive_begin(false);

    verify_transmit_failed_notification(NRF_802154_TX_ERROR_BUSY_CHANNEL);

    irq_ccabusy_state_tx_frame();
}

/***************************************************************************************************
 * @section PHYEND handler
 **************************************************************************************************/

static void verify_phyend_ack_req_periph_setup(uint32_t shorts, bool buffer_free)
{
    uint32_t event_addr;
    uint32_t task_addr1;
    uint32_t task_addr2;

    m_state = RADIO_STATE_TX;
    bool ack_req = m_tx_buffer[ACK_REQUEST_OFFSET] & ACK_REQUEST_BIT;

    nrf_802154_frame_parser_ar_bit_is_set_ExpectAndReturn(m_tx_buffer, ack_req);

    nrf_ppi_channel_disable_Expect(PPI_DISABLED_EGU);

    nrf_radio_shorts_set_Expect(shorts);

    if (buffer_free)
    {
        nrf_radio_packetptr_set_Expect(m_rx_buffer.data);
    }

    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_END);
    nrf_radio_int_disable_Expect(NRF_RADIO_INT_CCABUSY_MASK |
                                 NRF_RADIO_INT_ADDRESS_MASK |
                                 NRF_RADIO_INT_PHYEND_MASK);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_ADDRESS);
    nrf_radio_int_enable_Expect(NRF_RADIO_INT_END_MASK |
                                NRF_RADIO_INT_ADDRESS_MASK);

    // Clear FEM configuration set at the beginning of the transmission
    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);
    nrf_timer_shorts_disable_Expect(NRF_802154_TIMER_INSTANCE,
                             NRF_TIMER_SHORT_COMPARE0_STOP_MASK |
                             NRF_TIMER_SHORT_COMPARE1_STOP_MASK);

    nrf_802154_fal_pa_configuration_clear_ExpectAndReturn(&m_activate_tx_cc0, NULL, NRF_SUCCESS);
    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);

    // Set PPIs necessary in rx_ack state
    nrf_802154_fal_lna_configuration_set_ExpectAndReturn(&m_activate_rx_cc0, NULL, NRF_SUCCESS);

    event_addr = rand();
    task_addr1 = rand();
    nrf_egu_event_address_get_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, (uint32_t *)event_addr);
    nrf_timer_task_address_get_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_START, (uint32_t *)task_addr1);

    nrf_timer_shorts_enable_Expect(m_activate_rx_cc0.event.timer.p_timer_instance,
                                   NRF_TIMER_SHORT_COMPARE0_STOP_MASK);
    nrf_ppi_channel_endpoint_setup_Expect(PPI_EGU_TIMER_START, event_addr, task_addr1);
    nrf_ppi_channel_enable_Expect(PPI_EGU_TIMER_START);

    task_addr2 = rand();
    nrf_ppi_task_address_get_ExpectAndReturn(PPI_CHGRP0_DIS_TASK, (uint32_t *)task_addr2);
    task_addr1 = rand();
    nrf_radio_task_address_get_ExpectAndReturn(NRF_RADIO_TASK_RXEN, task_addr1);
    event_addr = rand();
    nrf_egu_event_address_get_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, (uint32_t *)event_addr);
    nrf_ppi_channel_and_fork_endpoint_setup_Expect(PPI_EGU_RAMP_UP, event_addr, task_addr1, task_addr2);

    nrf_egu_event_clear_Expect(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT);

    nrf_ppi_channel_enable_Expect(PPI_EGU_RAMP_UP);

    nrf_ppi_channel_enable_Expect(PPI_DISABLED_EGU);
}

void test_phyend_handler_ShallDoNothingIfTransmissionHasNotStarted(void)
{
    m_flags.tx_started = false;

    irq_phyend_state_tx_frame();
}

void test_phyend_handler_ShallResetToRxStateAndNotifySuccessIfAckNotRequested(void)
{
    bool ack_req = m_tx_buffer[ACK_REQUEST_OFFSET] & ACK_REQUEST_BIT;

    m_flags.tx_started = true;
    insert_frame_with_noack_to_tx_buffer();

    nrf_802154_frame_parser_ar_bit_is_set_ExpectAndReturn(m_tx_buffer, ack_req);

    verify_tx_terminate_periph_reset(true);
    verify_complete_receive_begin(false);

    verify_transmitted_notification_noack();

    irq_phyend_state_tx_frame();
}

void test_phyend_handler_ShallSetPeriphToRxAckIfRequested(void)
{
    m_flags.tx_started = true;
    insert_frame_with_ack_request_to_tx_buffer();
    mark_rx_buffer_free();

    verify_phyend_ack_req_periph_setup(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                                       NRF_RADIO_SHORT_END_DISABLE_MASK |
                                       NRF_RADIO_SHORT_RXREADY_START_MASK,
                                       true);

    // Verify if PPI worked or is going to work
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_TXDISABLE);

    verify_complete_ack_matching_enable();

    // Trigger
    irq_phyend_state_tx_frame();
}

void test_phyend_handler_ShallNotTriggerDisableIfAckRequestedAndEguEventSet(void)
{
    m_flags.tx_started = true;
    insert_frame_with_ack_request_to_tx_buffer();
    mark_rx_buffer_free();

    verify_phyend_ack_req_periph_setup(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                                       NRF_RADIO_SHORT_END_DISABLE_MASK |
                                       NRF_RADIO_SHORT_RXREADY_START_MASK,
                                       true);

    // Verify if PPI worked or is going to work
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_DISABLED);
    nrf_egu_event_check_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, true);

    verify_complete_ack_matching_enable();

    // Trigger
    irq_phyend_state_tx_frame();
}

void test_phyend_handler_ShallTriggerDisableIfAckRequestedAndPpiDidNotWork(void)
{
    m_flags.tx_started = true;
    insert_frame_with_ack_request_to_tx_buffer();
    mark_rx_buffer_free();

    verify_phyend_ack_req_periph_setup(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                                       NRF_RADIO_SHORT_END_DISABLE_MASK |
                                       NRF_RADIO_SHORT_RXREADY_START_MASK,
                                       true);

    // Verify if PPI worked or is going to work
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_DISABLED);
    nrf_egu_event_check_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, false);
    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    verify_complete_ack_matching_enable();

    // Trigger
    irq_phyend_state_tx_frame();
}

void test_phyend_handler_ShallTryToFindNewBufferIfNotAvailable(void)
{
    m_flags.tx_started = true;
    insert_frame_with_ack_request_to_tx_buffer();
    mark_rx_buffer_occupied();

    verify_phyend_ack_req_periph_setup(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                                       NRF_RADIO_SHORT_END_DISABLE_MASK,
                                       false);

    // Verify if PPI worked or is going to work
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_TXDISABLE);

    // Find free buffer
    nrf_802154_rx_buffer_free_find_ExpectAndReturn(NULL);

    verify_complete_ack_matching_enable();

    // Trigger
    irq_phyend_state_tx_frame();
}

void test_phyend_handler_ShallUpdateShortsIfRxBufferIsFoundAfterPeriphIsSet(void)
{
    rx_buffer_t rx_buffer;
    memset(&rx_buffer, 0, sizeof(rx_buffer));
    rx_buffer.free = true;
    m_flags.tx_started = true;

    insert_frame_with_ack_request_to_tx_buffer();
    mark_rx_buffer_occupied();

    verify_phyend_ack_req_periph_setup(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                                       NRF_RADIO_SHORT_END_DISABLE_MASK,
                                       false);

    // Verify if PPI worked or is going to work
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_TXDISABLE);

    // Find free buffer
    nrf_802154_rx_buffer_free_find_ExpectAndReturn(&rx_buffer);
    nrf_radio_packetptr_set_Expect(&rx_buffer);
    nrf_radio_shorts_set_Expect(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                                NRF_RADIO_SHORT_END_DISABLE_MASK |
                                NRF_RADIO_SHORT_RXREADY_START_MASK);
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_RXRU);

    verify_complete_ack_matching_enable();

    // Trigger
    irq_phyend_state_tx_frame();
}

void test_phyend_handler_ShallTriggerStartIfRxBufferIsFoundTooLate(void)
{
    rx_buffer_t rx_buffer;
    memset(&rx_buffer, 0, sizeof(rx_buffer));
    rx_buffer.free = true;
    m_flags.tx_started = true;

    insert_frame_with_ack_request_to_tx_buffer();
    mark_rx_buffer_occupied();

    verify_phyend_ack_req_periph_setup(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                                       NRF_RADIO_SHORT_END_DISABLE_MASK,
                                       false);

    // Verify if PPI worked or is going to work
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_TXDISABLE);

    // Find free buffer
    nrf_802154_rx_buffer_free_find_ExpectAndReturn(&rx_buffer);
    nrf_radio_packetptr_set_Expect(&rx_buffer);
    nrf_radio_shorts_set_Expect(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                                NRF_RADIO_SHORT_END_DISABLE_MASK |
                                NRF_RADIO_SHORT_RXREADY_START_MASK);
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_RXIDLE);
    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_START);

    verify_complete_ack_matching_enable();

    // Trigger
    irq_phyend_state_tx_frame();
}

/***************************************************************************************************
 * @section rx_ack_terminate function
 **************************************************************************************************/

static void verify_rx_ack_terminate_hardware_reset(bool in_timeslot)
{
    nrf_ppi_channel_disable_Expect(PPI_DISABLED_EGU);
    nrf_ppi_channel_disable_Expect(PPI_EGU_RAMP_UP);

    nrf_802154_fal_lna_configuration_clear_ExpectAndReturn(&m_activate_rx_cc0, NULL, NRF_SUCCESS);
    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);
    nrf_timer_shorts_disable_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_SHORT_COMPARE0_STOP_MASK);

    nrf_ppi_channel_disable_Expect(PPI_EGU_TIMER_START);

    nrf_ppi_channel_remove_from_group_Expect(PPI_EGU_RAMP_UP, PPI_CHGRP0);
    nrf_ppi_fork_endpoint_setup_Expect(PPI_EGU_RAMP_UP, 0);

    m_rsch_timeslot_is_granted = in_timeslot;

    if (in_timeslot)
    {
        nrf_radio_int_disable_Expect(NRF_RADIO_INT_END_MASK |
                                     NRF_RADIO_INT_ADDRESS_MASK);
        nrf_radio_shorts_set_Expect(0);

        nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

        verify_complete_ack_matching_disable();
    }
}

void test_rx_ack_terminate_ShallNotModifyRadioRegistersOutOfTimeslot(void)
{
    verify_rx_ack_terminate_hardware_reset(false);

    rx_ack_terminate();
}

void test_rx_ack_terminate_ShallResetHardwareAndTriggerDisableTask(void)
{
    verify_rx_ack_terminate_hardware_reset(true);

    rx_ack_terminate();
}

/***************************************************************************************************
 * @section END handler in RX_ACK state
 **************************************************************************************************/

void test_end_handler_ShallResetRadioToStartReceivingAndNotifyTransmittedFrame(void)
{
    mark_rx_buffer_free();

    verify_complete_ack_is_matched();
    verify_rx_ack_terminate_hardware_reset(true);
    verify_complete_receive_begin(false);
    verify_transmitted_notification_ack();

    irq_end_state_rx_ack();
    TEST_ASSERT_EQUAL(RADIO_STATE_RX, m_state);
    TEST_ASSERT_EQUAL(false, m_rx_buffer.free);
}

void test_end_handler_ShallResetRadioToStartReceivingAndNotifyFailureOnOtherFrame(void)
{
    mark_rx_buffer_free();

    verify_complete_ack_is_not_matched();
    verify_rx_ack_terminate_hardware_reset(true);
    verify_complete_receive_begin(true);
    verify_transmit_failed_notification(NRF_802154_TX_ERROR_INVALID_ACK);

    irq_end_state_rx_ack();
    TEST_ASSERT_EQUAL(RADIO_STATE_RX, m_state);
    TEST_ASSERT_EQUAL(true, m_rx_buffer.free);
}

void test_end_handler_ShallResetRadioToStartReceivingAndNotifyTransmittedFrameType2015(void)
{
    nrf_802154_frame_parser_mhr_data_t tx_data;
    nrf_802154_frame_parser_mhr_data_t ack_data;
    uint16_t tx_src_addr = rand();
    uint16_t ack_dst_addr = tx_src_addr;

    memset(&tx_data, 0, sizeof(tx_data));
    memset(&ack_data, 0, sizeof(ack_data));

    tx_data.p_src_addr = (uint8_t *)&tx_src_addr;
    ack_data.p_dst_addr = (uint8_t *)&ack_dst_addr;

    tx_data.src_addr_size = SHORT_ADDRESS_SIZE;
    ack_data.dst_addr_size = SHORT_ADDRESS_SIZE;

    mark_rx_buffer_free();
    insert_ack_2015_to_rx_buffer();
    insert_frame_2015_with_ack_request_to_tx_buffer();

    verify_complete_ack_is_not_matched();
    nrf_radio_crc_status_check_ExpectAndReturn(true);

    nrf_802154_frame_parser_mhr_parse_ExpectAndReturn(m_tx_buffer, NULL, true);
    nrf_802154_frame_parser_mhr_parse_IgnoreArg_p_fields();
    nrf_802154_frame_parser_mhr_parse_ReturnThruPtr_p_fields(&tx_data);

    nrf_802154_frame_parser_mhr_parse_ExpectAndReturn(m_rx_buffer.data, NULL, true);
    nrf_802154_frame_parser_mhr_parse_IgnoreArg_p_fields();
    nrf_802154_frame_parser_mhr_parse_ReturnThruPtr_p_fields(&ack_data);

    verify_rx_ack_terminate_hardware_reset(true);
    verify_complete_receive_begin(false);
    verify_transmitted_notification_ack();

    irq_end_state_rx_ack();
    TEST_ASSERT_EQUAL(RADIO_STATE_RX, m_state);
    TEST_ASSERT_EQUAL(false, m_rx_buffer.free);
}

void test_end_handler_ShallResetRadioToStartReceivingAndNotifyTransmitFailedWhenAckDstAddrSizeDiffers(void)
{
    nrf_802154_frame_parser_mhr_data_t tx_data;
    nrf_802154_frame_parser_mhr_data_t ack_data;
    uint16_t tx_src_addr = rand();
    uint16_t ack_dst_addr = tx_src_addr;

    memset(&tx_data, 0, sizeof(tx_data));
    memset(&ack_data, 0, sizeof(ack_data));

    tx_data.p_src_addr = (uint8_t *)&tx_src_addr;
    ack_data.p_dst_addr = (uint8_t *)&ack_dst_addr;

    tx_data.src_addr_size = SHORT_ADDRESS_SIZE;
    ack_data.dst_addr_size = EXTENDED_ADDRESS_SIZE;

    mark_rx_buffer_free();
    insert_ack_2015_to_rx_buffer();
    insert_frame_2015_with_ack_request_to_tx_buffer();

    verify_complete_ack_is_not_matched();
    nrf_radio_crc_status_check_ExpectAndReturn(true);

    nrf_802154_frame_parser_mhr_parse_ExpectAndReturn(m_tx_buffer, NULL, true);
    nrf_802154_frame_parser_mhr_parse_IgnoreArg_p_fields();
    nrf_802154_frame_parser_mhr_parse_ReturnThruPtr_p_fields(&tx_data);

    nrf_802154_frame_parser_mhr_parse_ExpectAndReturn(m_rx_buffer.data, NULL, true);
    nrf_802154_frame_parser_mhr_parse_IgnoreArg_p_fields();
    nrf_802154_frame_parser_mhr_parse_ReturnThruPtr_p_fields(&ack_data);

    verify_rx_ack_terminate_hardware_reset(true);
    verify_complete_receive_begin(true);
    verify_transmit_failed_notification(NRF_802154_TX_ERROR_INVALID_ACK);

    irq_end_state_rx_ack();
    TEST_ASSERT_EQUAL(RADIO_STATE_RX, m_state);
    TEST_ASSERT_EQUAL(true, m_rx_buffer.free);
}

void test_end_handler_ShallResetRadioToStartReceivingAndNotifyTransmitFailedWhenAckDstAddrDiffers(void)
{
    nrf_802154_frame_parser_mhr_data_t tx_data;
    nrf_802154_frame_parser_mhr_data_t ack_data;
    uint16_t tx_src_addr = rand();
    uint16_t ack_dst_addr = tx_src_addr + 1;

    memset(&tx_data, 0, sizeof(tx_data));
    memset(&ack_data, 0, sizeof(ack_data));

    tx_data.p_src_addr = (uint8_t *)&tx_src_addr;
    ack_data.p_dst_addr = (uint8_t *)&ack_dst_addr;

    tx_data.src_addr_size = SHORT_ADDRESS_SIZE;
    ack_data.dst_addr_size = SHORT_ADDRESS_SIZE;

    mark_rx_buffer_free();
    insert_ack_2015_to_rx_buffer();
    insert_frame_2015_with_ack_request_to_tx_buffer();

    verify_complete_ack_is_not_matched();
    nrf_radio_crc_status_check_ExpectAndReturn(true);

    nrf_802154_frame_parser_mhr_parse_ExpectAndReturn(m_tx_buffer, NULL, true);
    nrf_802154_frame_parser_mhr_parse_IgnoreArg_p_fields();
    nrf_802154_frame_parser_mhr_parse_ReturnThruPtr_p_fields(&tx_data);

    nrf_802154_frame_parser_mhr_parse_ExpectAndReturn(m_rx_buffer.data, NULL, true);
    nrf_802154_frame_parser_mhr_parse_IgnoreArg_p_fields();
    nrf_802154_frame_parser_mhr_parse_ReturnThruPtr_p_fields(&ack_data);

    verify_rx_ack_terminate_hardware_reset(true);
    verify_complete_receive_begin(true);
    verify_transmit_failed_notification(NRF_802154_TX_ERROR_INVALID_ACK);

    irq_end_state_rx_ack();
    TEST_ASSERT_EQUAL(RADIO_STATE_RX, m_state);
    TEST_ASSERT_EQUAL(true, m_rx_buffer.free);
}

void test_end_handler_ShallResetRadioToStartReceivingAndNotifyTransmitFailedWhenAckDstAddrIsMissing(void)
{
    nrf_802154_frame_parser_mhr_data_t tx_data;
    nrf_802154_frame_parser_mhr_data_t ack_data;
    uint16_t tx_src_addr = rand();

    memset(&tx_data, 0, sizeof(tx_data));
    memset(&ack_data, 0, sizeof(ack_data));

    tx_data.p_src_addr = (uint8_t *)&tx_src_addr;
    ack_data.p_dst_addr = NULL;

    tx_data.src_addr_size = SHORT_ADDRESS_SIZE;
    ack_data.dst_addr_size = 0;

    mark_rx_buffer_free();
    insert_ack_2015_to_rx_buffer();
    insert_frame_2015_with_ack_request_to_tx_buffer();

    verify_complete_ack_is_not_matched();
    nrf_radio_crc_status_check_ExpectAndReturn(true);

    nrf_802154_frame_parser_mhr_parse_ExpectAndReturn(m_tx_buffer, NULL, true);
    nrf_802154_frame_parser_mhr_parse_IgnoreArg_p_fields();
    nrf_802154_frame_parser_mhr_parse_ReturnThruPtr_p_fields(&tx_data);

    nrf_802154_frame_parser_mhr_parse_ExpectAndReturn(m_rx_buffer.data, NULL, true);
    nrf_802154_frame_parser_mhr_parse_IgnoreArg_p_fields();
    nrf_802154_frame_parser_mhr_parse_ReturnThruPtr_p_fields(&ack_data);

    verify_rx_ack_terminate_hardware_reset(true);
    verify_complete_receive_begin(true);
    verify_transmit_failed_notification(NRF_802154_TX_ERROR_INVALID_ACK);

    irq_end_state_rx_ack();
    TEST_ASSERT_EQUAL(RADIO_STATE_RX, m_state);
    TEST_ASSERT_EQUAL(true, m_rx_buffer.free);
}


void test_end_handler_ShallResetRadioToStartReceivingAndNotifyTransmitFailedWhenAckIsInvalid(void)
{
    nrf_802154_frame_parser_mhr_data_t tx_data;
    nrf_802154_frame_parser_mhr_data_t ack_data;
    uint16_t tx_src_addr = rand();
    uint16_t ack_dst_addr = tx_src_addr + 1;

    memset(&tx_data, 0, sizeof(tx_data));
    memset(&ack_data, 0, sizeof(ack_data));

    tx_data.p_src_addr = (uint8_t *)&tx_src_addr;
    ack_data.p_dst_addr = (uint8_t *)&ack_dst_addr;

    tx_data.src_addr_size = SHORT_ADDRESS_SIZE;
    ack_data.dst_addr_size = SHORT_ADDRESS_SIZE;

    mark_rx_buffer_free();
    insert_ack_2015_to_rx_buffer();
    insert_frame_2015_with_ack_request_to_tx_buffer();

    verify_complete_ack_is_not_matched();
    nrf_radio_crc_status_check_ExpectAndReturn(true);

    nrf_802154_frame_parser_mhr_parse_ExpectAndReturn(m_tx_buffer, NULL, true);
    nrf_802154_frame_parser_mhr_parse_IgnoreArg_p_fields();
    nrf_802154_frame_parser_mhr_parse_ReturnThruPtr_p_fields(&tx_data);

    nrf_802154_frame_parser_mhr_parse_ExpectAndReturn(m_rx_buffer.data, NULL, false);
    nrf_802154_frame_parser_mhr_parse_IgnoreArg_p_fields();

    verify_rx_ack_terminate_hardware_reset(true);
    verify_complete_receive_begin(true);
    verify_transmit_failed_notification(NRF_802154_TX_ERROR_INVALID_ACK);

    irq_end_state_rx_ack();
    TEST_ASSERT_EQUAL(RADIO_STATE_RX, m_state);
    TEST_ASSERT_EQUAL(true, m_rx_buffer.free);
}

void test_end_handler_ShallResetRadioToStartReceivingAndNotifyTransmitFailedWhenAckCrcIsIsInvalid(void)
{
    nrf_802154_frame_parser_mhr_data_t tx_data;
    nrf_802154_frame_parser_mhr_data_t ack_data;
    uint16_t tx_src_addr = rand();
    uint16_t ack_dst_addr = tx_src_addr + 1;

    memset(&tx_data, 0, sizeof(tx_data));
    memset(&ack_data, 0, sizeof(ack_data));

    tx_data.p_src_addr = (uint8_t *)&tx_src_addr;
    ack_data.p_dst_addr = (uint8_t *)&ack_dst_addr;

    tx_data.src_addr_size = SHORT_ADDRESS_SIZE;
    ack_data.dst_addr_size = SHORT_ADDRESS_SIZE;

    mark_rx_buffer_free();
    insert_ack_2015_to_rx_buffer();
    insert_frame_2015_with_ack_request_to_tx_buffer();

    verify_complete_ack_is_not_matched();
    nrf_radio_crc_status_check_ExpectAndReturn(false);

    verify_rx_ack_terminate_hardware_reset(true);
    verify_complete_receive_begin(true);
    verify_transmit_failed_notification(NRF_802154_TX_ERROR_INVALID_ACK);

    irq_end_state_rx_ack();
    TEST_ASSERT_EQUAL(RADIO_STATE_RX, m_state);
    TEST_ASSERT_EQUAL(true, m_rx_buffer.free);
}

void test_end_handler_ShallResetRadioToStartReceivingAndNotifyTransmitFailedWhenAckTypeIsInvalid(void)
{
    mark_rx_buffer_free();
    insert_ack_2015_to_rx_buffer();
    insert_frame_2015_with_ack_request_to_tx_buffer();

    m_rx_buffer.data[FRAME_TYPE_OFFSET] |= FRAME_TYPE_MASK;

    verify_complete_ack_is_not_matched();

    verify_rx_ack_terminate_hardware_reset(true);
    verify_complete_receive_begin(true);
    verify_transmit_failed_notification(NRF_802154_TX_ERROR_INVALID_ACK);

    irq_end_state_rx_ack();
    TEST_ASSERT_EQUAL(RADIO_STATE_RX, m_state);
    TEST_ASSERT_EQUAL(true, m_rx_buffer.free);
}

void test_end_handler_ShallResetRadioToStartReceivingAndNotifyTransmitFailedWhenAckVersionIsInvalid(void)
{
    mark_rx_buffer_free();
    insert_ack_2015_to_rx_buffer();
    insert_frame_2015_with_ack_request_to_tx_buffer();

    m_rx_buffer.data[FRAME_VERSION_OFFSET] |= FRAME_VERSION_MASK;

    verify_complete_ack_is_not_matched();

    verify_rx_ack_terminate_hardware_reset(true);
    verify_complete_receive_begin(true);
    verify_transmit_failed_notification(NRF_802154_TX_ERROR_INVALID_ACK);

    irq_end_state_rx_ack();
    TEST_ASSERT_EQUAL(RADIO_STATE_RX, m_state);
    TEST_ASSERT_EQUAL(true, m_rx_buffer.free);
}

void test_end_handler_ShallResetRadioToStartReceivingAndNotifyTransmitFailedWhenSeqIsNotMatchedAndTransmittedFrameIsVersion2006(void)
{
    mark_rx_buffer_free();
    insert_ack_2015_to_rx_buffer();
    insert_frame_with_ack_request_to_tx_buffer();

    verify_complete_ack_is_not_matched();

    verify_rx_ack_terminate_hardware_reset(true);
    verify_complete_receive_begin(true);
    verify_transmit_failed_notification(NRF_802154_TX_ERROR_INVALID_ACK);

    irq_end_state_rx_ack();
    TEST_ASSERT_EQUAL(RADIO_STATE_RX, m_state);
    TEST_ASSERT_EQUAL(true, m_rx_buffer.free);
}

