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
#include "mock_nrf_802154_notification.h"
#include "mock_nrf_802154_pib.h"
#include "mock_nrf_802154_priority_drop.h"
#include "mock_nrf_802154_procedures_duration.h"
#include "mock_nrf_802154_rsch.h"
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

static void verify_cca_notification(bool is_free)
{
    nrf_802154_critical_section_nesting_allow_Expect();

    nrf_802154_notify_cca_Expect(is_free);

    nrf_802154_critical_section_nesting_deny_Expect();
}

static void verify_setting_tx_power(void)
{
    int8_t tx_power = rand();
    nrf_802154_pib_tx_power_get_ExpectAndReturn(tx_power);
    nrf_radio_txpower_set_Expect(tx_power);
}

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

static void verify_complete_receive_begin(void)
{
    m_rsch_timeslot_is_granted = true;

    verify_setting_tx_power();
    verify_receive_begin_setup(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                               NRF_RADIO_SHORT_END_DISABLE_MASK       |
                               NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_RXDISABLE);
    verify_receive_begin_finds_free_buffer();
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

// Handle cases when timeslot is not granted

void test_cca_begin_ShallDoNothingIfNotEnoughTime(void)
{
    uint16_t duration = rand();
    nrf_802154_cca_duration_get_ExpectAndReturn(duration);
    nrf_802154_rsch_timeslot_request_ExpectAndReturn(duration, false);

    cca_init(true);
}

// Basic peripheral setup

static void verify_timeslot_request_test(void)
{
    uint16_t duration = rand();

    nrf_802154_cca_duration_get_ExpectAndReturn(duration);
    nrf_802154_rsch_timeslot_request_ExpectAndReturn(duration, true);
}

static void verify_cca_begin_periph_setup(void)
{
    uint32_t event_addr;
    uint32_t task_addr1;
    uint32_t task_addr2;

    verify_timeslot_request_test();

    nrf_radio_shorts_set_Expect(NRF_RADIO_SHORT_RXREADY_CCASTART_MASK |
                                NRF_RADIO_SHORT_CCABUSY_DISABLE_MASK);

    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_CCABUSY);
    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_CCAIDLE);
    nrf_radio_int_enable_Expect(NRF_RADIO_INT_CCABUSY_MASK | NRF_RADIO_INT_CCAIDLE_MASK);

    // Set timer and PPIs for FEM
    nrf_802154_fal_lna_configuration_set_ExpectAndReturn(&m_activate_rx_cc0, NULL, NRF_SUCCESS);
    
    task_addr1 = rand();
    event_addr = rand();
    nrf_egu_event_address_get_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, (uint32_t *)event_addr);
    nrf_timer_task_address_get_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_START, (uint32_t *)task_addr1);
    nrf_timer_shorts_enable_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_SHORT_COMPARE0_STOP_MASK);

    nrf_ppi_channel_endpoint_setup_Expect(PPI_EGU_TIMER_START, event_addr, task_addr1);
    nrf_ppi_channel_enable_Expect(PPI_EGU_TIMER_START);

    nrf_egu_event_clear_Expect(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT);

    task_addr2 = rand();
    nrf_ppi_task_address_get_ExpectAndReturn(PPI_CHGRP0_DIS_TASK, (uint32_t *)task_addr2);
    task_addr1 = rand();
    nrf_radio_task_address_get_ExpectAndReturn(NRF_RADIO_TASK_RXEN, task_addr1);
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

void test_cca_begin_ShallPrepareHardwareToPerformCca(void)
{
    verify_cca_begin_periph_setup();

    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    cca_init(false);
}


// Verify how detection of PPI status works

void test_cca_begin_ShallTriggerDisableIfRequestedByArgument(void)
{
    verify_cca_begin_periph_setup();

    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    cca_init(false);
}

void test_cca_begin_ShallNotTriggerDisableIfRadioIsRampingDown(void)
{
    verify_cca_begin_periph_setup();

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_RXDISABLE);

    cca_init(true);
}

void test_cca_begin_ShallNotTriggerDisableIfEguEventIsSet(void)
{
    verify_cca_begin_periph_setup();

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_DISABLED);
    nrf_egu_event_check_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, true);

    cca_init(true);
}

void test_cca_begin_ShallTriggerDisableIfRadioIsDisabledAndEguDidNotWork(void)
{
    verify_cca_begin_periph_setup();

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_DISABLED);
    nrf_egu_event_check_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, false);
    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    cca_init(true);
}

/***************************************************************************************************
 * @section CCA terminate function
 **************************************************************************************************/

static void verify_cca_terminate_periph_reset(bool in_timeslot)
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
        nrf_fem_prepare_powerdown_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL0, PPI_EGU_TIMER_START, false);
        nrf_radio_int_disable_Expect(NRF_RADIO_INT_CCAIDLE_MASK | NRF_RADIO_INT_CCABUSY_MASK);
        nrf_radio_shorts_set_Expect(0);
        nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_CCASTOP);
        nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);
    }
}

void test_cca_terminate_ShallNotModifyRadioRegistersOutOfTimeslot(void)
{
    verify_cca_terminate_periph_reset(false);

    cca_terminate();
}

void test_cca_terminate_ShallResetPeriphAndTriggerDisableTask(void)
{
    verify_cca_terminate_periph_reset(true);

    cca_terminate();
}

/***************************************************************************************************
 * @section CCABUSY and CCAIDLE handlers
 **************************************************************************************************/

void test_ccabusy_handler_ShallResetToRxStateAndNotifyBusyChannel(void)
{
    verify_cca_terminate_periph_reset(true);
    verify_complete_receive_begin();

    verify_cca_notification(false);

    irq_ccabusy_state_cca();
}

void test_ccaidle_handler_ShallResetToRxStateAndNotifyIdleChannel(void)
{
    verify_cca_terminate_periph_reset(true);
    verify_complete_receive_begin();

    verify_cca_notification(true);

    irq_ccaidle_state_cca();
}
