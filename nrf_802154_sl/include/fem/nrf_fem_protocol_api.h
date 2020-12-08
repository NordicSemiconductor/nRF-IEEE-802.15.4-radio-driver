/*
 * Copyright (c) 2020, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @brief Protocol interface for Power Amplifier (PA) and Low Noise Amplifier (LNA).
 *
 * This module enables toggling of GPIO pins before and after the radio transmission and the radio reception
 * in order to control a Power Amplifier or a Low Noise Amplifier, or both.
 *
 * The application must first provide PA and LNA device-specific configuration parameters to this module.
 * The protocol must then provide PA and LNA protocol configuration parameters before it can use the functionality.
 *
 * When the PA/LNA module is configured, the stack can call the provided enable functions before radio activity
 * to enable the PA or LNA timer configurations for the upcoming radio activity.
 * By default, PA/LNA is automatically deactivated on the radio DISABLED event.
 */

#ifndef NRF_FEM_PROTOCOL_API_H__
#define NRF_FEM_PROTOCOL_API_H__

#include <nrf.h>

#if defined(NRF52_SERIES)
#include "hal/nrf_ppi.h"
#elif defined(NRF53_SERIES)
#include "hal/nrf_dppi.h"
#else
#error Unsupported chip
#endif

typedef enum
{
    NRF_802154_FAL_PA  = 1 << 0,
    NRF_802154_FAL_LNA = 1 << 1,
    NRF_802154_FAL_ALL = NRF_802154_FAL_PA | NRF_802154_FAL_LNA
} nrf_fal_functionality_t;

/**
 * @brief PA and LNA activation event types.
 */
typedef enum
{
    NRF_802154_FAL_EVENT_TYPE_TIMER,
    NRF_802154_FAL_EVENT_TYPE_GENERIC,
    NRF_802154_FAL_EVENT_TYPE_PPI,
} nrf_802154_fal_event_type_t;

/**
 * @brief Frontend Abstraction Layer event.
 *
 * The event can be a Timer Compare event or an Any event.
 * Register value is only used for the Timer Compare event and should only contain the timer value relative to the Timer Compare event.
 */
typedef struct
{
    union
    {
        struct
        {
            NRF_TIMER_Type * p_timer_instance; /* Pointer to a 1-us resolution timer instance. */

            struct
            {
                uint32_t start;           /* Timer value when the FEM can start preparing PA/LNA. */
                uint32_t end;             /* Timer value at which the PA/LNA have to be prepared. Radio operation shall start at this point. */
            }       counter_period;
            uint8_t compare_channel_mask; /* Mask of the compare channels that can be used by the FEM to schedule its own tasks. */
        } timer;
        struct
        {
            uint32_t register_address;        /* Address of event register. */
        } generic;
    }                           event;
    bool                        override_ppi; /* False to ignore the PPI channel below and use the one set by application. True to use the PPI channel below. */
    uint8_t                     ppi_ch_id;    /* PPI channel to be used for this event. */
    nrf_802154_fal_event_type_t type;         /* Type of event source. */
} nrf_802154_fal_event_t;

/**
 * @brief Sets up PA using the provided events for the upcoming radio transmission.
 *
 * Multiple configurations can be provided by repeating calls to this function (that is, you can set the activate and the deactivate events in multiple calls,
 * and the configuration is preserved between calls).
 *
 * The order of calls of this function and its `lna` counterpart must match the order of radio operations.
 * I.e. if you want to perform the CCA and then send the frame, you need first to issue @ref nrf_802154_fal_lna_configuration_set and only after @ref nrf_802154_fal_pa_configuration_set.
 *
 * If a NRF_802154_PA_LNA_EVENT_TYPE_TIMER timer event is provided, the PA will be configured to activate or deactivate at the application-configured time gap
 * before the timer instance reaches the given register_value. The time gap is set via @ref nrf_fem_interface_configuration_set.
 *
 * If a NRF_802154_PA_LNA_EVENT_TYPE_GENERIC event is provided, the PA will be configured to activate or deactivate when an event occurs.
 *
 * The function sets up the PPIs and the GPIOTE channel to activate PA for the upcoming radio transmission.
 * The PA pin will be active until deactivated, which can happen either by encountering a configured deactivation event or by using @ref nrf_802154_fal_deactivate_now.
 *
 * @param[in] p_activate_event   Pointer to the activation event structure.
 * @param[in] p_deactivate_event Pointer to the deactivation event structure.
 *
 * @pre To activate PA, nrf_fem_interface_configuration_set() must have been called first.
 *
 * @note If a timer event is provided, the caller of this function is responsible for starting the timer and configuring its shorts.
 *       Moreover, the caller is responsible for stopping the timer no earlier than the compare channel of the lowest ID among the provided ones does expire.
 *
 * @retval   ::NRFX_SUCCESS               PA activate setup is successful.
 * @retval   ::NRFX_ERROR_FORBIDDEN       PA is currently disabled.
 * @retval   ::NRFX_ERROR_INVALID_STATE   PA activate setup could not be performed due to invalid or missing configuration parameters
 *                                        in p_activate_event or p_deactivate_event, or both.
 */
int32_t nrf_802154_fal_pa_configuration_set(const nrf_802154_fal_event_t * const p_activate_event,
                                            const nrf_802154_fal_event_t * const p_deactivate_event);

/**
 * @brief Clears up the configuration provided by the @ref nrf_802154_fal_pa_configuration_set function.
 *
 * @retval   ::NRFX_SUCCESS               PA activate setup purge is successful.
 * @retval   ::NRFX_ERROR_FORBIDDEN       PA is currently disabled.
 * @retval   ::NRFX_ERROR_INVALID_STATE   PA activate setup purge could not be performed due to invalid or missing configuration parameters,
 *                                        that were previously used in @ref nrf_802154_fal_pa_configuration_set call.
 */
int32_t nrf_802154_fal_pa_configuration_clear(void);

/**
 * @brief Sets up LNA using the provided event for the upcoming radio reception.
 *
 * Multiple configurations can be provided by repeating calls to this function (that is, you can set the activate and the deactivate event in multiple calls,
 * and the configuration is preserved between calls).
 *
 * The order of calls of this function and its `pa` counterpart must match the order of radio operations.
 * I.e. if you want to perform the CCA and then send the frame, you need first to issue @ref nrf_802154_fal_lna_configuration_set and only after @ref nrf_802154_fal_pa_configuration_set.
 *
 * If a NRF_802154_PA_LNA_EVENT_TYPE_TIMER timer event is provided, the LNA will be configured to activate or deactivate at the application-configured time gap
 * before the timer instance reaches the given register_value. The time gap is set via @ref nrf_fem_interface_configuration_set.
 *
 * If a NRF_802154_PA_LNA_EVENT_TYPE_GENERIC event is provided, the LNA will be configured to activate or deactivate when an event occurs.
 *
 * The function sets up the PPIs and the GPIOTE channel to activate LNA for the upcoming radio transmission.
 * The LNA pin will be active until deactivated, which can happen either by encountering a configured deactivation event or by using @ref nrf_802154_fal_deactivate_now.
 *
 * @param[in] p_activate_event   Pointer to the activation event structure.
 * @param[in] p_deactivate_event Pointer to the deactivation event structure.
 *
 * @pre To activate LNA, nrf_fem_interface_configuration_set() must have been called first.
 *
 * @note If a timer event is provided, the caller of this function is responsible for starting the timer and configuring its shorts.
 *       Moreover, the caller is responsible for stopping the timer no earlier than the compare channel of the lowest ID among the provided ones does expire.
 *
 * @retval   ::NRFX_SUCCESS               LNA activate setup is successful.
 * @retval   ::NRFX_ERROR_FORBIDDEN       LNA is currently disabled.
 * @retval   ::NRFX_ERROR_INVALID_STATE   LNA activate setup could not be performed due to invalid or missing configuration parameters
 *                                        in p_activate_event or p_deactivate_event, or both.
 */
int32_t nrf_802154_fal_lna_configuration_set(const nrf_802154_fal_event_t * const p_activate_event,
                                             const nrf_802154_fal_event_t * const p_deactivate_event);

/**
 * @brief Clears up the configuration provided by the @ref nrf_802154_fal_lna_configuration_set function.
 *
 *
 * @retval   ::NRFX_SUCCESS               LNA activate setup purge is successful.
 * @retval   ::NRFX_ERROR_FORBIDDEN       LNA is currently disabled.
 * @retval   ::NRFX_ERROR_INVALID_STATE   LNA activate setup purge could not be performed due to invalid or missing configuration parameters,
 *                                        that were previously used in @ref nrf_802154_fal_lna_configuration_set call.
 */
int32_t nrf_802154_fal_lna_configuration_clear(void);

/**
 * @brief Deactivates PA/LNA pins with immediate effect.
 */
void nrf_802154_fal_deactivate_now(nrf_fal_functionality_t type);

#if defined(NRF52_SERIES)
/**
 * @brief Instruct FEM to disable PA and LNA as soon as possible using the group following the event.
 *
 * @param[in] event Address of the event which is triggered when the abort condition occurs.
 * @param[in] group PPI Group which shall be disabled when the abort event is triggered.
 *
 * @retval   ::NRFX_SUCCESS               Setting of the abort sequence path is successful.
 * @retval   ::NRFX_ERROR_FORBIDDEN       Setting of the abort sequence path could not be performed.
 */
int32_t nrf_802154_fal_abort_set(uint32_t event, nrf_ppi_channel_group_t group);

/**
 * @brief Adds one more PPI channel to the PPI Group prepared by the @ref nrf_802154_fal_abort_set function.
 *
 * @param[in] channel_to_add
 * @param[in] group
 *
 * @retval    ::NRFX_SUCCESS Setting of the abort sequence path is successful.
 */
int32_t nrf_802154_fal_abort_extend(nrf_ppi_channel_t       channel_to_add,
                                    nrf_ppi_channel_group_t group);

/**
 * @brief Removes one PPI channel from the PPI Group prepared by the @ref nrf_802154_fal_abort_set function.
 *
 * @param[in] channel_to_remove
 * @param[in] group
 *
 * @retval    ::NRFX_SUCCESS Setting of the abort sequence path is successful.
 */
int32_t nrf_802154_fal_abort_reduce(nrf_ppi_channel_t       channel_to_remove,
                                    nrf_ppi_channel_group_t group);

/**
 * @brief Clears up the configuration provided by the @ref nrf_802154_fal_abort_set function.
 *
 * @retval   ::NRFX_SUCCESS               Clearing of the abort sequence path is successful.
 * @retval   ::NRFX_ERROR_FORBIDDEN       Clearing was not done - the possible reason is that there was nothing to clear.
 */
int32_t nrf_802154_fal_abort_clear(void);
#endif // NRF52_SERIES

/**
 * @brief Cleans up the configured PA/LNA timer/radio instance and resources of PPI and GPIOTE.
 * The function resets the hardware that has been set up for the PA/LNA activation. The PA and LNA module control configuration parameters are not deleted.
 * The function is intended to be called after the radio disable signal.
 */
void nrf_802154_fal_cleanup(void);

/**
 * @brief Checks if the PA signaling is configured and enabled, and gets the configured gain in dB.
 *
 * @param[out] p_gain The configured gain in dB if PA is configured and enabled.
                      If there is no PA present or the PA does not affect the signal gain, returns 0 dB.
 *
 */
void nrf_802154_fal_pa_is_configured(int8_t * const p_gain);

#if defined(NRF52_SERIES)
/**
 * @brief Prepares the FEM module to switch to the Power Down state.
 *
 * @param[in] p_instance Timer instance that is used to schedule the transition to the Power Down state.
 * @param[in] compare_channel Compare channel to hold a value for the timer.
 * @param[in] ppi_id ID of the PPI channel used to switch to the Power Down state.
 *
 * @return bool Whether the scheduling of the transition was successful or not.
 *
 */
bool nrf_fem_prepare_powerdown(NRF_TIMER_Type  * p_instance,
                               uint32_t          compare_channel,
                               nrf_ppi_channel_t ppi_id);
#endif // NRF52_SERIES

#endif // NRF_FEM_PROTOCOL_API_H__
