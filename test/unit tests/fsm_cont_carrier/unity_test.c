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
 * @section Transmit begin function
 **************************************************************************************************/

// Handle cases when timeslot is not granted

void test_continuous_carrier_begin_ShallDoNothingIfOutOfTimeslot(void)
{
    m_rsch_timeslot_is_granted = false;

    continuous_carrier_init(true);
}

// Basic peripheral setup

static void verify_continuous_carrier_begin_periph_setup(void)
{
    uint32_t event_addr;
    uint32_t task_addr;
    int8_t   tx_power;

    m_rsch_timeslot_is_granted = true;

    tx_power = rand();
    nrf_802154_pib_tx_power_get_ExpectAndReturn(tx_power);
    nrf_radio_txpower_set_Expect(tx_power);
    
    nrf_802154_fal_pa_configuration_set_ExpectAndReturn(&m_activate_tx_cc0, NULL, NRF_SUCCESS);
    event_addr = rand();
    task_addr = rand();
    nrf_egu_event_address_get_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, (uint32_t *)event_addr);
    nrf_timer_task_address_get_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_START, (uint32_t *)task_addr);
    nrf_timer_shorts_enable_Expect(m_activate_tx_cc0.event.timer.p_timer_instance,
                                   NRF_TIMER_SHORT_COMPARE0_STOP_MASK);
    nrf_ppi_channel_endpoint_setup_Expect(PPI_EGU_TIMER_START, event_addr, task_addr);
    nrf_ppi_channel_enable_Expect(PPI_EGU_TIMER_START);

    nrf_egu_event_clear_Expect(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT);

    task_addr = rand();
    nrf_radio_task_address_get_ExpectAndReturn(NRF_RADIO_TASK_TXEN, task_addr);
    event_addr = rand();
    nrf_egu_event_address_get_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, (uint32_t *)event_addr);
    nrf_ppi_channel_endpoint_setup_Expect(PPI_EGU_RAMP_UP, event_addr, task_addr);

    task_addr = rand();
    nrf_egu_task_address_get_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_TASK, (uint32_t *)task_addr);
    event_addr = rand();
    nrf_radio_event_address_get_ExpectAndReturn(NRF_RADIO_EVENT_DISABLED, event_addr);
    nrf_ppi_channel_endpoint_setup_Expect(PPI_DISABLED_EGU, event_addr, task_addr);

    nrf_ppi_channel_enable_Expect(PPI_EGU_RAMP_UP);
    nrf_ppi_channel_enable_Expect(PPI_DISABLED_EGU);
}

void test_continuous_carrier_begin_ShallPrepareHardwareToTransmitCarrier(void)
{
    verify_continuous_carrier_begin_periph_setup();

    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    continuous_carrier_init(false);
}

// Verify how detection of PPI status works

void test_continuous_carrier_begin_ShallTriggerDisableIfRequestedByArgument(void)
{
    verify_continuous_carrier_begin_periph_setup();

    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    continuous_carrier_init(false);
}

void test_continuous_carrier_begin_ShallNotTriggerDisableIfRadioIsRampingDown(void)
{
    verify_continuous_carrier_begin_periph_setup();

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_RXDISABLE);

    continuous_carrier_init(true);
}

void test_continuous_carrier_begin_ShallNotTriggerDisableIfEguEventIsSet(void)
{
    verify_continuous_carrier_begin_periph_setup();

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_DISABLED);
    nrf_egu_event_check_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, true);

    continuous_carrier_init(true);
}

void test_continuous_carrier_begin_ShallTriggerDisableIfRadioIsDisabledAndEguDidNotWork(void)
{
    verify_continuous_carrier_begin_periph_setup();

    nrf_radio_state_get_ExpectAndReturn(NRF_RADIO_STATE_DISABLED);
    nrf_egu_event_check_ExpectAndReturn(NRF_802154_SWI_EGU_INSTANCE, EGU_EVENT, false);
    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    continuous_carrier_init(true);
}

/***************************************************************************************************
 * @section Continuous Carrier terminate function
 **************************************************************************************************/

void test_continuous_carrier_terminate_ShallDoNothingOutOfTimeslot(void)
{
    nrf_ppi_channel_disable_Expect(PPI_DISABLED_EGU);
    nrf_ppi_channel_disable_Expect(PPI_EGU_RAMP_UP);

    nrf_802154_fal_pa_configuration_clear_ExpectAndReturn(&m_activate_tx_cc0, NULL, NRF_SUCCESS);
    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);
    nrf_ppi_channel_disable_Expect(PPI_EGU_TIMER_START);
    nrf_802154_fal_deactivate_now_Expect(NRF_802154_FAL_PA);

    m_rsch_timeslot_is_granted = false;

    continuous_carrier_terminate();
}

void test_continuous_carrier_terminate_ShallResetPeriphAndTriggerDisableTask(void)
{
    nrf_ppi_channel_disable_Expect(PPI_DISABLED_EGU);
    nrf_ppi_channel_disable_Expect(PPI_EGU_RAMP_UP);

    nrf_802154_fal_pa_configuration_clear_ExpectAndReturn(&m_activate_tx_cc0, NULL, NRF_SUCCESS);
    nrf_timer_task_trigger_Expect(NRF_802154_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);
    nrf_ppi_channel_disable_Expect(PPI_EGU_TIMER_START);
    nrf_802154_fal_deactivate_now_Expect(NRF_802154_FAL_PA);

    m_rsch_timeslot_is_granted = true;

    nrf_fem_prepare_powerdown_ExpectAndReturn(NRF_802154_TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL0, PPI_EGU_TIMER_START, false);
    nrf_radio_task_trigger_Expect(NRF_RADIO_TASK_DISABLE);

    continuous_carrier_terminate();
}
