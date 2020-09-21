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
#include "mock_nrf_802154_ack_pending_bit.h"
#include "mock_nrf_802154_core_hooks.h"
#include "mock_nrf_802154_critical_section.h"
#include "mock_nrf_802154_debug.h"
#include "mock_nrf_802154_notification.h"
#include "mock_nrf_802154_pib.h"
#include "mock_nrf_802154_priority_drop.h"
#include "mock_nrf_802154_procedures_duration.h"
#include "mock_nrf_802154_revision.h"
#include "mock_nrf_802154_rssi.h"
#include "mock_nrf_802154_rx_buffer.h"
#include "mock_nrf_fem_control_api.h"
#include "mock_nrf_raal_api.h"
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

static void verify_energy_detected_notification(uint8_t result)
{
    nrf_802154_critical_section_nesting_allow_Expect();

    nrf_802154_notify_energy_detected_Expect(result);

    nrf_802154_critical_section_nesting_deny_Expect();
}

static void verify_setting_tx_power(void)
{
    int8_t tx_power = rand();
    nrf_802154_pib_tx_power_get_ExpectAndReturn(tx_power);
    nrf_radio_tx_power_set_Expect(tx_power);
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
    nrf_radio_int_enable_Expect(NRF_RADIO_INT_BCMATCH_MASK  |
                                NRF_RADIO_INT_CRCERROR_MASK |
                                NRF_RADIO_INT_CRCOK_MASK);

    // FEM setup
    lna_target_time = rand();
    pa_target_time = rand();
    nrf_fem_control_ppi_enable_Expect(NRF_FEM_CONTROL_LNA_PIN, NRF_TIMER_CC_CHANNEL0);
    nrf_fem_control_delay_get_ExpectAndReturn(NRF_FEM_CONTROL_LNA_PIN, lna_target_time);
    nrf_fem_control_delay_get_ExpectAndReturn(NRF_FEM_CONTROL_PA_PIN, pa_target_time);

    nrf_timer_shorts_enable_Expect(NRF_802154_TIMER_INSTANCE,
                                   NRF_TIMER_SHORT_COMPARE0_STOP_MASK |
                                   NRF_TIMER_SHORT_COMPARE2_STOP_MASK);
    nrf_timer_cc_write_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL0, lna_target_time);
    nrf_timer_cc_write_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL1, lna_target_time + 192 - 40 - 23);
    nrf_timer_cc_write_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL2, lna_target_time + 192 - 40 - 23 + pa_target_time);

    // Clear EVENTS that are checked later
    nrf_egu_event_clear_Expect(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT);

    // PPIs setup
    task2_addr = rand();
    nrf_ppi_task_address_get_ExpectAndReturn(PPI_CHGRP0_DIS_TASK,
                                             (uint32_t *)task2_addr);
    task_addr = rand();
    nrf_radio_task_address_get_ExpectAndReturn(NRF_RADIO_TASK_RXEN, (uint32_t *)task_addr);
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
    nrf_radio_event_address_get_ExpectAndReturn(NRF_RADIO_EVENT_DISABLED, (uint32_t *)event_addr);
    nrf_ppi_channel_endpoint_setup_Expect(PPI_DISABLED_EGU, event_addr, task_addr);

    nrf_ppi_channel_enable_Expect(PPI_EGU_RAMP_UP);
    nrf_ppi_channel_enable_Expect(PPI_EGU_TIMER_START);
    nrf_ppi_channel_enable_Expect(PPI_DISABLED_EGU);
}

static void verify_receive_begin_finds_free_buffer(void)
{
    nrf_802154_rx_buffer_free_find_ExpectAndReturn(NULL);
}

static void verify_complete_receive_begin(void)
{
    verify_setting_tx_power();
    verify_receive_begin_setup(NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK |
                               NRF_RADIO_SHORT_END_DISABLE_MASK |
                               NRF_RADIO_SHORT_ADDRESS_BCSTART_MASK);
    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_RX_DISABLE);
    verify_receive_begin_finds_free_buffer();
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
void nrf_802154_tx_ack_started(void){}

/***************************************************************************************************
 * @section Transmit begin function
 **************************************************************************************************/

// Handle cases when timeslot is not granted

void test_ed_begin_ShallDoNothingIfOutOfTimeslot(void)
{
    m_timeslot_is_granted = false;

    nrf_raal_timeslot_us_left_get_ExpectAndReturn(0);

    nrf_fem_control_ppi_disable_Expect(NRF_FEM_CONTROL_LNA_PIN);
    nrf_fem_control_pin_clear_Expect();

    ed_init(true);
}

void test_ed_begin_ShallResetRadioIfTimeslotIsTooShort(void)
{
    m_timeslot_is_granted = true;

    nrf_raal_timeslot_us_left_get_ExpectAndReturn(1);

    nrf_radio_power_set_Expect(false);
    nrf_radio_power_set_Expect(true);

    nrf_fem_control_ppi_disable_Expect(NRF_FEM_CONTROL_LNA_PIN);
    nrf_fem_control_pin_clear_Expect();

    ed_init(true);
}

// Basic peripheral setup

static void verify_ed_begin_periph_setup(void)
{
    uint32_t event_addr;
    uint32_t task_addr1;
    uint32_t task_addr2;
    uint32_t us;

    nrf_raal_timeslot_us_left_get_ExpectAndReturn(UINT32_MAX);

    us = rand();
    us = us ? us : 1;
    m_ed_time_left = us;
    nrf_radio_ed_loop_count_set_Expect((us - 1) / 128);

    nrf_radio_shorts_set_Expect(NRF_RADIO_SHORT_READY_EDSTART_MASK);

    nrf_radio_event_clear_Expect(NRF_RADIO_EVENT_EDEND);
    nrf_radio_int_enable_Expect(NRF_RADIO_INT_EDEND_MASK);


    nrf_fem_control_ppi_enable_Expect(NRF_FEM_CONTROL_LNA_PIN, NRF_TIMER_CC_CHANNEL0);
    nrf_fem_control_timer_set_Expect(NRF_FEM_CONTROL_LNA_PIN,
                                     NRF_TIMER_CC_CHANNEL0,
                                     NRF_TIMER_SHORT_COMPARE0_STOP_MASK);
    task_addr1 = rand();
    nrf_timer_task_address_get_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_START, (uint32_t *)task_addr1);
    event_addr = rand();
    nrf_egu_event_address_get_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, (uint32_t *)event_addr);
    nrf_fem_control_ppi_task_setup_Expect(NRF_FEM_CONTROL_LNA_PIN,
                                          PPI_EGU_TIMER_START,
                                          event_addr,
                                          task_addr1);

    nrf_egu_event_clear_Expect(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT);

    task_addr2 = rand();
    nrf_ppi_task_address_get_ExpectAndReturn(PPI_CHGRP0_DIS_TASK, (uint32_t *)task_addr2);
    task_addr1 = rand();
    nrf_radio_task_address_get_ExpectAndReturn(NRF_RADIO_TASK_RXEN, (uint32_t *)task_addr1);
    event_addr = rand();
    nrf_egu_event_address_get_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, (uint32_t *)event_addr);
    nrf_ppi_channel_and_fork_endpoint_setup_Expect(PPI_EGU_RAMP_UP, event_addr, task_addr1, task_addr2);

    task_addr1 = rand();
    nrf_egu_task_address_get_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_TASK, (uint32_t *)task_addr1);
    event_addr = rand();
    nrf_radio_event_address_get_ExpectAndReturn(NRF_RADIO_EVENT_DISABLED, (uint32_t *)event_addr);
    nrf_ppi_channel_endpoint_setup_Expect(PPI_DISABLED_EGU, event_addr, task_addr1);

    nrf_ppi_channel_include_in_group_Expect(PPI_EGU_RAMP_UP, PPI_CHGRP0);

    nrf_ppi_channel_enable_Expect(PPI_EGU_RAMP_UP);
    nrf_ppi_channel_enable_Expect(PPI_DISABLED_EGU);
}

void test_ed_begin_ShallPrepareHardwareToPerformEd(void)
{
    verify_ed_begin_periph_setup();

    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    ed_init(false);
}


// Verify how detection of PPI status works

void test_ed_begin_ShallTriggerDisableIfRequestedByArgument(void)
{
    verify_ed_begin_periph_setup();

    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    ed_init(false);
}

void test_ed_begin_ShallNotTriggerDisableIfRadioIsRampingDown(void)
{
    verify_ed_begin_periph_setup();

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_RX_DISABLE);

    ed_init(true);
}

void test_ed_begin_ShallNotTriggerDisableIfEguEventIsSet(void)
{
    verify_ed_begin_periph_setup();

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_DISABLED);
    nrf_egu_event_check_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, true);

    ed_init(true);
}

void test_ed_begin_ShallTriggerDisableIfRadioIsDisabledAndEguDidNotWork(void)
{
    verify_ed_begin_periph_setup();

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_DISABLED);
    nrf_egu_event_check_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, false);
    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    ed_init(true);
}

/***************************************************************************************************
 * @section ED terminate function
 **************************************************************************************************/

static void verify_ed_terminate_periph_reset(bool is_in_timeslot)
{
    nrf_ppi_channel_disable_Expect(PPI_DISABLED_EGU);
    nrf_ppi_channel_disable_Expect(PPI_EGU_RAMP_UP);

    nrf_fem_control_ppi_disable_Expect(NRF_FEM_CONTROL_LNA_PIN);
    nrf_fem_control_timer_reset_Expect(NRF_FEM_CONTROL_LNA_PIN, NRF_TIMER_SHORT_COMPARE0_STOP_MASK);
    nrf_ppi_channel_disable_Expect(PPI_EGU_TIMER_START);

    nrf_ppi_channel_remove_from_group_Expect(PPI_EGU_RAMP_UP, PPI_CHGRP0);
    nrf_ppi_fork_endpoint_setup_Expect(PPI_EGU_RAMP_UP, 0);

    if (is_in_timeslot)
    {
        nrf_radio_int_disable_Expect(NRF_RADIO_INT_EDEND_MASK);
        nrf_radio_shorts_set_Expect(0);
        nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);
    }
}

void test_ed_terminate_ShallNotModifyRadioRegistersIfTimslotIsNotGranted(void)
{
    m_timeslot_is_granted = false;
    verify_ed_terminate_periph_reset(false);

    ed_terminate();
}

void test_ed_terminate_ShallResetPeriphAndTriggerDisableTask(void)
{
    m_timeslot_is_granted = true;
    verify_ed_terminate_periph_reset(true);

    ed_terminate();
}

/***************************************************************************************************
 * @section EDEND handler
 **************************************************************************************************/

void test_edend_handler_ShallResetToRxStateAndNotifySuccessIfEdEnded(void)
{
    uint8_t result           = rand();
    uint8_t corrected_result = rand();
    uint8_t channel          = 11 + (rand() % 16);

    uint32_t expected_result = corrected_result * 4;

    if (expected_result > UINT8_MAX)
    {
        expected_result = UINT8_MAX;
    }

    m_ed_result    = 0;
    m_ed_time_left = 0;

    m_timeslot_is_granted = true;

    nrf_radio_ed_sample_get_ExpectAndReturn(result);

    nrf_802154_pib_channel_get_ExpectAndReturn(channel);
    nrf_radio_frequency_set_Expect((channel - 10) * 5);
    verify_ed_terminate_periph_reset(true);
    verify_complete_receive_begin();

    nrf_802154_rssi_ed_corrected_get_ExpectAndReturn(result, corrected_result);
    verify_energy_detected_notification(expected_result);

    irq_edend_state_ed();
}

void test_edend_handler_ShallStartNextIterationOfEdIfEdDidNotEnd(void)
{
    uint8_t result = rand();

    m_ed_time_left = rand();
    m_ed_time_left = m_ed_time_left ? m_ed_time_left : 1;

    nrf_radio_ed_sample_get_ExpectAndReturn(result);

    nrf_raal_timeslot_us_left_get_ExpectAndReturn(UINT32_MAX);
    nrf_radio_ed_loop_count_set_Expect((m_ed_time_left - 1) / 128);

    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_EDSTART);

    irq_edend_state_ed();
}

void test_edend_handler_ShallResetRadioAndWaitForNextTimeslotIfCannotStartNextIteration(void)
{
    uint8_t result = rand();

    m_ed_time_left = rand();
    m_ed_time_left = m_ed_time_left ? m_ed_time_left : 1;

    m_timeslot_is_granted = true;

    nrf_radio_ed_sample_get_ExpectAndReturn(result);

    nrf_raal_timeslot_us_left_get_ExpectAndReturn(0);

    nrf_radio_power_set_Expect(false);
    nrf_radio_power_set_Expect(true);

    nrf_fem_control_ppi_disable_Expect(NRF_FEM_CONTROL_LNA_PIN);
    nrf_fem_control_pin_clear_Expect();

    nrf_fem_control_ppi_disable_Expect(NRF_FEM_CONTROL_LNA_PIN);
    nrf_fem_control_timer_reset_Expect(NRF_FEM_CONTROL_LNA_PIN, NRF_TIMER_SHORT_COMPARE0_STOP_MASK);
    nrf_ppi_channel_disable_Expect(PPI_EGU_TIMER_START);

    irq_edend_state_ed();
}
