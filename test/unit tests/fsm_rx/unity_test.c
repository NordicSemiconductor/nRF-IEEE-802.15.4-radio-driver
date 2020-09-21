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
#include "mock_nrf_802154_ack_generator.h"
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

static rx_buffer_t m_buffer;

static void mark_buffer_occupied(void)
{
    mp_current_rx_buffer = &m_buffer;

    m_buffer.free = false;
}

static void mark_buffer_free(void)
{
    mp_current_rx_buffer = &m_buffer;

    m_buffer.free = true;
}

static void insert_frame_with_noack_to_buffer(void)
{
    mp_current_rx_buffer = &m_buffer;

    m_buffer.free = true;

    m_buffer.data[0] = 16;
    m_buffer.data[1] = 0x41; // Data frame, no Ack Request

    m_flags.frame_filtered = true;
}

static void insert_frame_with_ack_request_to_buffer(void)
{
    mp_current_rx_buffer = &m_buffer;

    m_buffer.free = true;

    m_buffer.data[0] = 16;
    m_buffer.data[1] = 0x61; // Data frame, Ack Request

    m_flags.frame_filtered = true;
}

static void verify_setting_packet_ptr(void)
{
    nrf_radio_packetptr_set_Expect(m_buffer.data);
}

static void verify_setting_tx_power(void)
{
    int8_t tx_power = rand();
    nrf_802154_pib_tx_power_get_ExpectAndReturn(tx_power);
    nrf_radio_txpower_set_Expect(tx_power);
}

static void received_notification_verify(void)
{
    int8_t rssi = rand();
    int8_t corrected_rssi = rand();
    uint8_t corrected_lqi = rand();
    uint32_t expected_lqi = corrected_lqi * 4;

    if (expected_lqi > UINT8_MAX)
    {
        expected_lqi = UINT8_MAX;
    }

    nrf_802154_critical_section_nesting_allow_Expect();
    nrf_radio_rssi_sample_get_ExpectAndReturn(rssi);
    nrf_802154_rssi_sample_corrected_get_ExpectAndReturn(rssi, corrected_rssi);
    nrf_802154_rssi_lqi_corrected_get_IgnoreAndReturn(corrected_lqi);
    nrf_802154_notify_received_Expect(m_buffer.data, -corrected_rssi, expected_lqi);
    nrf_802154_critical_section_nesting_deny_Expect();
}

void setUp(void)
{

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
 * @section Receive begin function
 **************************************************************************************************/

// Basic peripheral setup

static void verify_receive_begin_setup(uint32_t shorts)
{
    uint32_t event_addr;
    uint32_t task_addr;
    uint32_t task2_addr;
    uint32_t delta_time;

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
    nrf_timer_shorts_enable_Expect(NRF_802154_TIMER_INSTANCE,
                            NRF_TIMER_SHORT_COMPARE0_STOP_MASK);

    delta_time = rand();
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

void test_receive_begin_ShallNotStartIfTimeslotIsNotGranted(void)
{

    m_rsch_timeslot_is_granted = false;
    rx_init(false);

    m_rsch_timeslot_is_granted = false;
    rx_init(true);
}

void test_recevie_begin_ShallSetCorrectShortsAndPpisIfBufferIsOccupied(void)
{
    mark_buffer_occupied();

    m_rsch_timeslot_is_granted = true;

    verify_setting_tx_power();
    verify_receive_begin_setup(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                               NRF_RADIO_SHORT_END_DISABLE_MASK |
                               NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);
    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);
    verify_receive_begin_finds_free_buffer();

    rx_init(false);
}

void test_receive_begin_ShallSetCorrectShortsAndPpisIfBufferIsAvailable(void)
{
    mark_buffer_free();

    m_rsch_timeslot_is_granted = true;

    verify_setting_tx_power();
    verify_setting_packet_ptr();
    verify_receive_begin_setup(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                               NRF_RADIO_SHORT_END_DISABLE_MASK |
                               NRF_RADIO_SHORT_RXREADY_START_MASK |
                               NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);
    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    rx_init(false);
}

// Triggering DISABLE if necessary

void test_receive_begin_ShallTriggerDisableIfRequestedByArgument(void)
{
    mark_buffer_occupied();

    m_rsch_timeslot_is_granted = true;

    verify_setting_tx_power();
    verify_receive_begin_setup(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                               NRF_RADIO_SHORT_END_DISABLE_MASK |
                               NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);

    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    verify_receive_begin_finds_free_buffer();

    rx_init(false);
}

void test_receive_begin_ShallNotTriggerDisableIfRampDownIsInProgress(void)
{
    mark_buffer_occupied();

    m_rsch_timeslot_is_granted = true;

    verify_setting_tx_power();
    verify_receive_begin_setup(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                               NRF_RADIO_SHORT_END_DISABLE_MASK |
                               NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_TXDISABLE);

    verify_receive_begin_finds_free_buffer();

    rx_init(true);
}

void test_receive_begin_ShallNotTriggerDisableIfEguEventWasTriggered(void)
{
    mark_buffer_occupied();

    m_rsch_timeslot_is_granted = true;

    verify_setting_tx_power();
    verify_receive_begin_setup(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                               NRF_RADIO_SHORT_END_DISABLE_MASK |
                               NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_DISABLED);
    nrf_egu_event_check_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, true);

    verify_receive_begin_finds_free_buffer();

    rx_init(true);
}

void test_receive_begin_ShallTriggerDisableIfEguEventWasNotTriggered(void)
{
    mark_buffer_occupied();

    m_rsch_timeslot_is_granted = true;

    verify_setting_tx_power();
    verify_receive_begin_setup(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                               NRF_RADIO_SHORT_END_DISABLE_MASK |
                               NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_DISABLED);
    nrf_egu_event_check_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, false);
    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    verify_receive_begin_finds_free_buffer();

    rx_init(true);
}

// Finding new buffer if none is available

void test_receive_begin_ShallSearchNewBufferIfCurrentlyNoneAvailable(void)
{
    mark_buffer_occupied();

    m_rsch_timeslot_is_granted = true;

    verify_setting_tx_power();
    verify_receive_begin_setup(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                               NRF_RADIO_SHORT_END_DISABLE_MASK |
                               NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);

    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    nrf_802154_rx_buffer_free_find_ExpectAndReturn(NULL);

    rx_init(false);
}

void test_receive_begin_ShallSetNewBufferIfFound(void)
{
    mark_buffer_occupied();

    m_rsch_timeslot_is_granted = true;

    verify_setting_tx_power();
    verify_receive_begin_setup(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                               NRF_RADIO_SHORT_END_DISABLE_MASK |
                               NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);

    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    rx_buffer_t new_buffer;
    memset(&new_buffer, 0, sizeof(new_buffer));
    new_buffer.free = true;
    nrf_802154_rx_buffer_free_find_ExpectAndReturn(&new_buffer);
    nrf_radio_packetptr_set_Expect(&new_buffer);
    nrf_radio_shorts_set_Expect(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                                NRF_RADIO_SHORT_END_DISABLE_MASK |
                                NRF_RADIO_SHORT_RXREADY_START_MASK |
                                NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_RXRU);

    rx_init(false);
}

void test_receive_begin_ShallTriggerStartIfNewBufferFoundAndShortDidNotWork(void)
{
    mark_buffer_occupied();

    m_rsch_timeslot_is_granted = true;

    verify_setting_tx_power();
    verify_receive_begin_setup(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                               NRF_RADIO_SHORT_END_DISABLE_MASK |
                               NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);

    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    rx_buffer_t new_buffer;
    memset(&new_buffer, 0, sizeof(new_buffer));
    new_buffer.free = true;
    nrf_802154_rx_buffer_free_find_ExpectAndReturn(&new_buffer);
    nrf_radio_packetptr_set_Expect(&new_buffer);
    nrf_radio_shorts_set_Expect(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                                NRF_RADIO_SHORT_END_DISABLE_MASK |
                                NRF_RADIO_SHORT_RXREADY_START_MASK |
                                NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_RXIDLE);
    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_START);

    rx_init(false);
}

/***************************************************************************************************
 * @section Rx terminate function
 **************************************************************************************************/

static inline void rx_terminate_periph_reset_verify(bool timeslot_granted)
{
    nrf_ppi_channel_disable_Expect(PPI_DISABLED_EGU);
    nrf_ppi_channel_disable_Expect(PPI_EGU_RAMP_UP);
    nrf_ppi_channel_disable_Expect(PPI_EGU_TIMER_START);

    nrf_802154_fal_lna_configuration_clear_ExpectAndReturn(&m_activate_rx_cc0, NULL, NRF_SUCCESS);
    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);

    nrf_ppi_channel_remove_from_group_Expect(PPI_EGU_RAMP_UP, PPI_CHGRP0);

    nrf_ppi_fork_endpoint_setup_Expect(PPI_EGU_RAMP_UP, 0);

    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);
    nrf_timer_shorts_disable_Expect(NRF_802154_TIMER_INSTANCE,
                                    NRF_TIMER_SHORT_COMPARE0_STOP_MASK);

    m_rsch_timeslot_is_granted = timeslot_granted;

    if (timeslot_granted)
    {
        nrf_radio_int_disable_Expect(NRF_RADIO_INT_CRCERROR_MASK |
                                     NRF_RADIO_INT_BCMATCH_MASK  |
                                     NRF_RADIO_INT_CRCOK_MASK);
        nrf_radio_shorts_set_Expect(0);
        nrf_fem_prepare_powerdown_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL0, PPI_EGU_TIMER_START, false);
        nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);
    }
}

void test_rx_terminate_ShallNotModifyRadioResigersOutOfTimeslot(void)
{
    rx_terminate_periph_reset_verify(false);

    rx_terminate();
}

void test_rx_terminate_ShallResetShortsAndPpisAndTriggerTasksToStopHardware(void)
{
    rx_terminate_periph_reset_verify(true);

    rx_terminate();
}

/***************************************************************************************************
 * @section CRCOK handler
 **************************************************************************************************/

// TODO: Filtering

void test_crcok_handler_ShallFilterFrame(void)
{
    // TODO: Implement filter testing (there is an example by @rolu)
}

// ACK requested

static void crcok_ack_periph_set_verify(void)
{
    uint8_t * p_ack = (uint8_t *)rand();
    uint32_t  event_addr;
    uint32_t  task_addr;
    uint32_t  time_to_pa;

    nrf_802154_frame_parser_ar_bit_is_set_ExpectAndReturn(m_buffer.data, true);

    nrf_802154_pib_auto_ack_get_ExpectAndReturn(true);

    nrf_802154_ack_generator_create_ExpectAndReturn(m_buffer.data, p_ack);
    nrf_radio_packetptr_set_Expect(p_ack);

    nrf_radio_shorts_set_Expect(NRF_RADIO_SHORT_TXREADY_START_MASK |
                                NRF_RADIO_SHORT_PHYEND_DISABLE_MASK);

    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_TXREADY);

    task_addr = rand();
    nrf_radio_task_address_get_ExpectAndReturn(NRF_RADIO_TASK_TXEN, task_addr);
    event_addr = rand();
    nrf_timer_event_address_get_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_EVENT_COMPARE1, (uint32_t *)event_addr);
    nrf_ppi_channel_endpoint_setup_Expect(PPI_TIMER_TX_ACK, event_addr, task_addr);

    nrf_ppi_channel_enable_Expect(PPI_TIMER_TX_ACK);

    nrf_timer_cc_read_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL1, time_to_pa);
    nrf_802154_fal_pa_configuration_set_IgnoreAndReturn(NRF_SUCCESS);
}

void test_crcok_handler_ShallPreparePeriphsToTransmitAckIfRequested(void)
{
    uint8_t * p_ack = (uint8_t *)rand();
    uint32_t  event_addr;
    uint32_t  task_addr;
    uint32_t  timer_cc1;
    uint32_t  timer_cc3;
    uint32_t  time_to_pa = rand();

    insert_frame_with_ack_request_to_buffer();

    nrf_802154_frame_parser_ar_bit_is_set_ExpectAndReturn(m_buffer.data, true);

    nrf_802154_pib_auto_ack_get_ExpectAndReturn(true);

    nrf_802154_ack_generator_create_ExpectAndReturn(m_buffer.data, p_ack);
    nrf_radio_packetptr_set_Expect(p_ack);

    nrf_radio_shorts_set_Expect(NRF_RADIO_SHORT_TXREADY_START_MASK |
                                NRF_RADIO_SHORT_PHYEND_DISABLE_MASK);

    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_TXREADY);

    task_addr = rand();
    nrf_radio_task_address_get_ExpectAndReturn(NRF_RADIO_TASK_TXEN, task_addr);
    event_addr = rand();
    nrf_timer_event_address_get_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_EVENT_COMPARE1, (uint32_t *)event_addr);
    nrf_ppi_channel_endpoint_setup_Expect(PPI_TIMER_TX_ACK, event_addr, task_addr);

    nrf_ppi_channel_enable_Expect(PPI_TIMER_TX_ACK);

    nrf_timer_cc_read_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL1, time_to_pa);

    nrf_802154_fal_pa_configuration_set_IgnoreAndReturn(NRF_SUCCESS);

    timer_cc1 = rand();
    timer_cc3 = timer_cc1 - 1;
    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_CAPTURE3);
    nrf_timer_cc_read_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL3, timer_cc3);
    nrf_timer_cc_read_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL1, timer_cc1);

    nrf_radio_int_disable_Expect(NRF_RADIO_INT_CRCERROR_MASK |
                                 NRF_RADIO_INT_BCMATCH_MASK  |
                                 NRF_RADIO_INT_CRCOK_MASK);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_PHYEND);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_ADDRESS);
    nrf_radio_int_enable_Expect(NRF_RADIO_INT_PHYEND_MASK |
                                NRF_RADIO_INT_ADDRESS_MASK);

    irq_crcok_state_rx();

    TEST_ASSERT_NOT_NULL(mp_ack);
}

void test_crcok_handler_ShallWaitForPhyendEventIfTimerIsTicking(void)
{
    uint32_t timer_cc1;
    uint32_t timer_cc3;

    insert_frame_with_ack_request_to_buffer();

    crcok_ack_periph_set_verify();

    timer_cc1 = rand();
    timer_cc3 = timer_cc1 - 1;
    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_CAPTURE3);
    nrf_timer_cc_read_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL3, timer_cc3);
    nrf_timer_cc_read_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL1, timer_cc1);

    nrf_radio_int_disable_Expect(NRF_RADIO_INT_CRCERROR_MASK |
                                 NRF_RADIO_INT_BCMATCH_MASK  |
                                 NRF_RADIO_INT_CRCOK_MASK);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_PHYEND);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_ADDRESS);
    nrf_radio_int_enable_Expect(NRF_RADIO_INT_PHYEND_MASK |
                                NRF_RADIO_INT_ADDRESS_MASK);

    irq_crcok_state_rx();
}

void test_crcok_handler_ShallWaitForPhyendEventIfTransmitterIsRampingUp(void)
{
    uint32_t timer_cc1;
    uint32_t timer_cc3;

    insert_frame_with_ack_request_to_buffer();

    crcok_ack_periph_set_verify();

    timer_cc1 = rand();
    timer_cc3 = timer_cc1;
    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_CAPTURE3);
    nrf_timer_cc_read_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL3, timer_cc3);
    nrf_timer_cc_read_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL1, timer_cc1);

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_TXRU);

    nrf_radio_int_disable_Expect(NRF_RADIO_INT_CRCERROR_MASK |
                                 NRF_RADIO_INT_BCMATCH_MASK  |
                                 NRF_RADIO_INT_CRCOK_MASK);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_PHYEND);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_ADDRESS);
    nrf_radio_int_enable_Expect(NRF_RADIO_INT_PHYEND_MASK |
                                NRF_RADIO_INT_ADDRESS_MASK);

    irq_crcok_state_rx();
}

void test_crcok_handler_ShallWaitForPhyendEventIfTransmitterEndedRampingUp(void)
{
    uint32_t timer_cc1;
    uint32_t timer_cc3;

    insert_frame_with_ack_request_to_buffer();

    crcok_ack_periph_set_verify();

    timer_cc1 = rand();
    timer_cc3 = timer_cc1;
    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_CAPTURE3);
    nrf_timer_cc_read_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL3, timer_cc3);
    nrf_timer_cc_read_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL1, timer_cc1);
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_TX);

    nrf_radio_event_check_ExpectAndReturn(NRF_RADIO_EVENT_TXREADY, true);

    nrf_radio_int_disable_Expect(NRF_RADIO_INT_CRCERROR_MASK |
                                 NRF_RADIO_INT_BCMATCH_MASK  |
                                 NRF_RADIO_INT_CRCOK_MASK);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_PHYEND);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_ADDRESS);
    nrf_radio_int_enable_Expect(NRF_RADIO_INT_PHYEND_MASK |
                                NRF_RADIO_INT_ADDRESS_MASK);

    irq_crcok_state_rx();
}

void test_crcok_handler_ShallNotifyReceivedFrameAndStartRxIfTransmitterDidNotRampUp(void)
{
    uint32_t timer_cc1;
    uint32_t timer_cc3;

    insert_frame_with_ack_request_to_buffer();
    m_buffer.free = false;

    crcok_ack_periph_set_verify();

    timer_cc1 = rand();
    timer_cc3 = timer_cc1;
    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_CAPTURE3);
    nrf_timer_cc_read_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL3, timer_cc3);
    nrf_timer_cc_read_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL1, timer_cc1);
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_TX);
    nrf_radio_event_check_ExpectAndReturn(NRF_RADIO_EVENT_TXREADY, false);

    nrf_ppi_channel_disable_Expect(PPI_TIMER_TX_ACK);

    rx_terminate_periph_reset_verify(true);

    m_rsch_timeslot_is_granted = true;
    verify_setting_tx_power();
    verify_receive_begin_setup(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                               NRF_RADIO_SHORT_END_DISABLE_MASK |
                               NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_TXDISABLE);
    verify_receive_begin_finds_free_buffer();

    received_notification_verify();

    irq_crcok_state_rx();
}

// No ACK

static void crcok_noack_periph_reset_verify(void)
{
    uint32_t event_addr;

    nrf_802154_frame_parser_ar_bit_is_set_ExpectAndReturn(m_buffer.data, false);

    nrf_ppi_channel_disable_Expect(PPI_DISABLED_EGU);

    nrf_radio_shorts_set_Expect(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                                NRF_RADIO_SHORT_END_DISABLE_MASK |
                                NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);

    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);

    nrf_ppi_channel_enable_Expect(PPI_EGU_RAMP_UP);

    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_BCMATCH);
    nrf_radio_bcc_set_Expect(24);

    nrf_egu_event_clear_Expect(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT);
    nrf_ppi_channel_enable_Expect(PPI_DISABLED_EGU);

    event_addr = rand();
    nrf_radio_event_address_get_ExpectAndReturn(NRF_RADIO_EVENT_CRCOK, event_addr);
    nrf_802154_timer_coord_timestamp_prepare_Expect(event_addr);

}

void test_crcok_handler_ShallResetReceiverAndNotifyIfAckNotRequested(void)
{
    int8_t rssi = rand();
    int8_t corrected_rssi = rand();
    uint8_t corrected_lqi = rand();
    uint32_t expected_lqi = corrected_lqi * 4;

    if (expected_lqi > UINT8_MAX)
    {
        expected_lqi = UINT8_MAX;
    }

    // TODO: Include LQI
    insert_frame_with_noack_to_buffer();

    crcok_noack_periph_reset_verify();

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_RXDISABLE);

    nrf_802154_rx_buffer_free_find_ExpectAndReturn(NULL);

    nrf_802154_critical_section_nesting_allow_Expect();
    nrf_radio_rssi_sample_get_ExpectAndReturn(rssi);
    nrf_802154_rssi_sample_corrected_get_ExpectAndReturn(rssi, corrected_rssi);
    nrf_802154_rssi_lqi_corrected_get_IgnoreAndReturn(corrected_lqi);
    nrf_802154_notify_received_Expect(m_buffer.data, -corrected_rssi, expected_lqi);
    nrf_802154_critical_section_nesting_deny_Expect();

    irq_crcok_state_rx();
}

void test_crcok_handler_ShallNotTriggerDisableIfEguEventIsSet(void)
{
    insert_frame_with_noack_to_buffer();

    crcok_noack_periph_reset_verify();

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_DISABLED);
    nrf_egu_event_check_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, true);

    nrf_802154_rx_buffer_free_find_ExpectAndReturn(NULL);

    received_notification_verify();

    irq_crcok_state_rx();
}

void test_crcok_handler_ShallTriggerDisableIfEguEventIsNotSet(void)
{
    insert_frame_with_noack_to_buffer();

    crcok_noack_periph_reset_verify();

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_DISABLED);
    nrf_egu_event_check_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, false);
    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    nrf_802154_rx_buffer_free_find_ExpectAndReturn(NULL);

    received_notification_verify();

    irq_crcok_state_rx();
}

void test_crcok_handler_ShallFindNewRxBufferAndUpdateShortsIfFound(void)
{
    static rx_buffer_t buffer;

    memset(&buffer, 0, sizeof(buffer));
    buffer.free = true;

    insert_frame_with_noack_to_buffer();

    crcok_noack_periph_reset_verify();
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_RXDISABLE);

    nrf_802154_rx_buffer_free_find_ExpectAndReturn(&buffer);
    nrf_radio_packetptr_set_Expect(&buffer);
    nrf_radio_shorts_set_Expect(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                                NRF_RADIO_SHORT_RXREADY_START_MASK     |
                                NRF_RADIO_SHORT_END_DISABLE_MASK       |
                                NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_RXRU);

    received_notification_verify();

    irq_crcok_state_rx();

}

void test_crcok_handler_ShallFindNewRxBufferAndTriggerStartIfShortDidNotWork(void)
{
    static rx_buffer_t buffer;

    memset(&buffer, 0, sizeof(buffer));
    buffer.free = true;

    insert_frame_with_noack_to_buffer();

    crcok_noack_periph_reset_verify();
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_RXDISABLE);

    nrf_802154_rx_buffer_free_find_ExpectAndReturn(&buffer);
    nrf_radio_packetptr_set_Expect(&buffer);
    nrf_radio_shorts_set_Expect(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                                NRF_RADIO_SHORT_RXREADY_START_MASK     |
                                NRF_RADIO_SHORT_END_DISABLE_MASK       |
                                NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_RXIDLE);
    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_START);

    received_notification_verify();

    irq_crcok_state_rx();
}

/***************************************************************************************************
 * @section PHYEND handler
 **************************************************************************************************/

static void verify_phyend_tx_ack_periph_reset(void)
{
    uint32_t task_addr;
    uint32_t event_addr;

    nrf_ppi_channel_disable_Expect(PPI_DISABLED_EGU);

    nrf_802154_fal_pa_configuration_clear_ExpectAndReturn(&m_activate_tx_cc0, NULL, NRF_SUCCESS);
    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);
    nrf_802154_fal_lna_configuration_set_ExpectAndReturn(&m_activate_rx_cc0, NULL, NRF_SUCCESS);

    nrf_radio_shorts_set_Expect(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                                NRF_RADIO_SHORT_END_DISABLE_MASK       |
                                NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);

    nrf_radio_bcc_set_Expect(24);

    nrf_radio_int_disable_Expect(NRF_RADIO_INT_PHYEND_MASK | NRF_RADIO_INT_ADDRESS_MASK);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_CRCERROR);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_BCMATCH);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_CRCOK);
    nrf_radio_int_enable_Expect(NRF_RADIO_INT_CRCERROR_MASK |
                                NRF_RADIO_INT_BCMATCH_MASK  |
                                NRF_RADIO_INT_CRCOK_MASK);

    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);

    nrf_ppi_channel_disable_Expect(PPI_TIMER_TX_ACK);
    nrf_ppi_channel_enable_Expect(PPI_EGU_RAMP_UP);

    nrf_egu_event_clear_Expect(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT);

    nrf_ppi_channel_enable_Expect(PPI_DISABLED_EGU);

    event_addr = rand();
    nrf_radio_event_address_get_ExpectAndReturn(NRF_RADIO_EVENT_CRCOK, event_addr);
    nrf_802154_timer_coord_timestamp_prepare_Expect(event_addr);
}

void test_phyend_handler_ShallResetPeripheralsForRxStateAndNotfiThatFrameWasReceived(void)
{
    uint32_t task_addr;
    uint32_t event_addr;

    insert_frame_with_ack_request_to_buffer();

    // Set hardware
    nrf_ppi_channel_disable_Expect(PPI_DISABLED_EGU);

    nrf_802154_fal_pa_configuration_clear_ExpectAndReturn(&m_activate_tx_cc0, NULL, NRF_SUCCESS);
    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);
    nrf_802154_fal_lna_configuration_set_ExpectAndReturn(&m_activate_rx_cc0, NULL, NRF_SUCCESS);

    nrf_radio_shorts_set_Expect(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                                NRF_RADIO_SHORT_END_DISABLE_MASK       |
                                NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);

    nrf_radio_bcc_set_Expect(24);

    nrf_radio_int_disable_Expect(NRF_RADIO_INT_PHYEND_MASK | NRF_RADIO_INT_ADDRESS_MASK);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_CRCERROR);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_BCMATCH);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_CRCOK);
    nrf_radio_int_enable_Expect(NRF_RADIO_INT_CRCERROR_MASK |
                                NRF_RADIO_INT_BCMATCH_MASK  |
                                NRF_RADIO_INT_CRCOK_MASK);

    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);

    nrf_ppi_channel_disable_Expect(PPI_TIMER_TX_ACK);
    nrf_ppi_channel_enable_Expect(PPI_EGU_RAMP_UP);

    nrf_egu_event_clear_Expect(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT);

    nrf_ppi_channel_enable_Expect(PPI_DISABLED_EGU);

    event_addr = rand();
    nrf_radio_event_address_get_ExpectAndReturn(NRF_RADIO_EVENT_CRCOK, event_addr);
    nrf_802154_timer_coord_timestamp_prepare_Expect(event_addr);

    // Detect if EGU worked or is going to work.
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_TXDISABLE);

    // Find new RX buffer
    nrf_802154_rx_buffer_free_find_ExpectAndReturn(NULL);

    received_notification_verify();

    // Trigger:
    irq_phyend_state_tx_ack();
}

void test_phyend_handler_ShallNotTriggerDisableIfStateIsNotDisabled(void)
{
    insert_frame_with_ack_request_to_buffer();

    verify_phyend_tx_ack_periph_reset();

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_TXDISABLE);

    nrf_802154_rx_buffer_free_find_ExpectAndReturn(NULL);

    received_notification_verify();

    // Trigger:
    irq_phyend_state_tx_ack();
}

void test_phyend_handler_ShallNotTriggerDisableIfEguEventIsSet(void)
{
    insert_frame_with_ack_request_to_buffer();

    verify_phyend_tx_ack_periph_reset();

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_DISABLED);
    nrf_egu_event_check_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, true);

    nrf_802154_rx_buffer_free_find_ExpectAndReturn(NULL);

    received_notification_verify();

    // Trigger:
    irq_phyend_state_tx_ack();
}

void test_phyend_handler_ShallTriggerDisableIfEguEventIsNotSet(void)
{
    insert_frame_with_ack_request_to_buffer();

    verify_phyend_tx_ack_periph_reset();

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_DISABLED);
    nrf_egu_event_check_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, false);
    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    nrf_802154_rx_buffer_free_find_ExpectAndReturn(NULL);

    received_notification_verify();

    // Trigger:
    irq_phyend_state_tx_ack();
}

void test_phyend_handler_ShallSetShortToStartReceiverIfBufferIsAvailable(void)
{
    rx_buffer_t buffer;
    memset (&buffer, 0, sizeof(buffer));
    buffer.free = true;

    insert_frame_with_ack_request_to_buffer();

    verify_phyend_tx_ack_periph_reset();

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_TXDISABLE);

    nrf_802154_rx_buffer_free_find_ExpectAndReturn(&buffer);
    nrf_radio_packetptr_set_Expect(&buffer);
    nrf_radio_shorts_set_Expect(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                                NRF_RADIO_SHORT_RXREADY_START_MASK     |
                                NRF_RADIO_SHORT_END_DISABLE_MASK       |
                                NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_RXRU);

    received_notification_verify();

    // Trigger:
    irq_phyend_state_tx_ack();
}

void test_phyend_handler_ShallStartReceiverIfBufferIsAvailableAndShortsDidNotWork(void)
{
    rx_buffer_t buffer;
    memset (&buffer, 0, sizeof(buffer));
    buffer.free = true;

    insert_frame_with_ack_request_to_buffer();

    verify_phyend_tx_ack_periph_reset();

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_TXDISABLE);

    nrf_802154_rx_buffer_free_find_ExpectAndReturn(&buffer);
    nrf_radio_packetptr_set_Expect(&buffer);
    nrf_radio_shorts_set_Expect(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                                NRF_RADIO_SHORT_RXREADY_START_MASK     |
                                NRF_RADIO_SHORT_END_DISABLE_MASK       |
                                NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_RXIDLE);
    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_START);

    received_notification_verify();

    // Trigger:
    irq_phyend_state_tx_ack();
}

/***************************************************************************************************
 * @section Tx Ack terminate function
 **************************************************************************************************/

void test_tx_ack_terminate_ShallNotModifyRadioRegistersOutOfTimeslot(void)
{
    nrf_ppi_channel_disable_Expect(PPI_DISABLED_EGU);
    nrf_ppi_channel_disable_Expect(PPI_EGU_RAMP_UP);
    nrf_ppi_channel_disable_Expect(PPI_EGU_TIMER_START);

    nrf_802154_fal_pa_configuration_clear_ExpectAndReturn(&m_activate_tx_cc0, NULL, NRF_SUCCESS);
    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);

    nrf_ppi_channel_remove_from_group_Expect(PPI_EGU_RAMP_UP, PPI_CHGRP0);
    nrf_ppi_fork_endpoint_setup_Expect(PPI_EGU_RAMP_UP, 0);

    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);
    nrf_timer_shorts_disable_Expect(NRF_802154_TIMER_INSTANCE,
                                    NRF_TIMER_SHORT_COMPARE0_STOP_MASK);

    m_rsch_timeslot_is_granted = false;

    tx_ack_terminate();
}

void test_tx_ack_terminate_ShallResetShortsAndPpisAndTriggerTasksToStopHardware(void)
{

    nrf_ppi_channel_disable_Expect(PPI_DISABLED_EGU);
    nrf_ppi_channel_disable_Expect(PPI_EGU_RAMP_UP);
    nrf_ppi_channel_disable_Expect(PPI_EGU_TIMER_START);

    nrf_802154_fal_pa_configuration_clear_ExpectAndReturn(&m_activate_tx_cc0, NULL, NRF_SUCCESS);
    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);

    nrf_ppi_channel_remove_from_group_Expect(PPI_EGU_RAMP_UP, PPI_CHGRP0);
    nrf_ppi_fork_endpoint_setup_Expect(PPI_EGU_RAMP_UP, 0);

    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);
    nrf_timer_shorts_disable_Expect(NRF_802154_TIMER_INSTANCE,
                                    NRF_TIMER_SHORT_COMPARE0_STOP_MASK);

    m_rsch_timeslot_is_granted = true;

    nrf_radio_int_disable_Expect(NRF_RADIO_INT_PHYEND_MASK | NRF_RADIO_INT_ADDRESS_MASK);
    nrf_radio_shorts_set_Expect(0);
    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    tx_ack_terminate();
}

void test_irq_crcerror_state_rx_ShallReserRadioForRxAndNotifyReceiveFailed(void)
{
    uint32_t event_addr;

    nrf_ppi_channel_disable_Expect(PPI_DISABLED_EGU);

    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);

    nrf_ppi_channel_enable_Expect(PPI_EGU_RAMP_UP);

    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_BCMATCH);
    nrf_radio_bcc_set_Expect(BCC_INIT);

    nrf_egu_event_clear_Expect(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT);
    nrf_ppi_channel_enable_Expect(PPI_DISABLED_EGU);

    event_addr = rand();
    nrf_radio_event_address_get_ExpectAndReturn(NRF_RADIO_EVENT_CRCOK, event_addr);
    nrf_802154_timer_coord_timestamp_prepare_Expect(event_addr);

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_RXDISABLE);

    nrf_802154_critical_section_nesting_allow_Expect();
    nrf_802154_notify_receive_failed_Expect(NRF_802154_RX_ERROR_INVALID_FCS);
    nrf_802154_critical_section_nesting_deny_Expect();

    irq_crcerror_state_rx();
}
