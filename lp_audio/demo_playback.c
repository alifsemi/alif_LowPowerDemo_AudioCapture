/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 * @file     : demo_playback.c
 * @author   : Ahmad Rashed
 * @email    : ahmad.rashed@alifsemi.com
 * @version  : V1.0.0
 * @date     : 7-Jan-2026
 * @brief    : Use the WM8904 CODEC to playback the recorded audio for verification
 * @bug      : N/A
 * @Note     : None
 ******************************************************************************/

/* System Includes */
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

#include <RTE_Components.h>
#include <board_config.h>
#include <app_utils.h>
#include <pinconf.h>

#include <Driver_SAI.h>
extern ARM_DRIVER_SAI  ARM_Driver_SAI_(BOARD_DAC_OUTPUT_I2S_INSTANCE);
static ARM_DRIVER_SAI *i2s_dac = &ARM_Driver_SAI_(BOARD_DAC_OUTPUT_I2S_INSTANCE);

#if (BOARD_WM8904_CODEC_PRESENT) && defined(RTE_Drivers_WM8904_CODEC)
#include "WM8904_driver.h"
extern ARM_DRIVER_WM8904  WM8904;
static ARM_DRIVER_WM8904 *wm8904 = &WM8904;
#else
#error "WM8904 codec driver not configured in RTE_Components.h"
#endif

#if defined(ENSEMBLE_SOC_E1C)
/* demo_lppdm.c records 16-bit PCM at 16kHz */
static uint32_t wlen = 16;
static uint32_t sampling_rate = 16000;
#else
/* demo_lpi2s.c records 24-bit PCM at 48kHz */
static uint32_t wlen = 24;
static uint32_t sampling_rate = 48000;
#endif

static volatile int32_t call_back_event;

static void dac_callback(uint32_t event)
{
    if (event & ARM_SAI_EVENT_SEND_COMPLETE) {
        call_back_event |= ARM_SAI_EVENT_SEND_COMPLETE;
    }
}

/**
 * @fn      static int32_t board_i2s_dac_pins_config(void)
 * @brief   Configure I2S DAC pinmux which not
 *          handled by the board support library.
 * @retval  execution status.
 */
static int32_t board_i2s_dac_pins_config(void)
{
    int32_t status;

    /* Configure DAC I2S SDO */
    status = pinconf_set(PORT_(BOARD_DAC_OUTPUT_SDO_GPIO_PORT),
                         BOARD_DAC_OUTPUT_SDO_GPIO_PIN,
                         BOARD_DAC_OUTPUT_SDO_ALTERNATE_FUNCTION,
                         0);
    if (status) {
        return status;
    }

    /* Configure DAC I2S WS */
    status = pinconf_set(PORT_(BOARD_DAC_OUTPUT_WS_GPIO_PORT),
                         BOARD_DAC_OUTPUT_WS_GPIO_PIN,
                         BOARD_DAC_OUTPUT_WS_ALTERNATE_FUNCTION,
                         0);
    if (status) {
        return status;
    }

    /* Configure DAC I2S SCLK */
    status = pinconf_set(PORT_(BOARD_DAC_OUTPUT_SCLK_GPIO_PORT),
                         BOARD_DAC_OUTPUT_SCLK_GPIO_PIN,
                         BOARD_DAC_OUTPUT_SCLK_ALTERNATE_FUNCTION,
                         0);
    if (status) {
        return status;
    }

    return APP_SUCCESS;
}

#if BOARD_WM8904_CODEC_PRESENT
/**
  \fn          void board_wm8904_i2c_pins_config(void)
  \brief       Initialize the pinmux for I2C
  \return      status
*/
static int32_t board_wm8904_i2c_pins_config(void)
{
    int32_t status;

    /* I2C_SDA */
    status = pinconf_set(PORT_(BOARD_WM8904_CODEC_I2C_SDA_GPIO_PORT),
                         BOARD_WM8904_CODEC_I2C_SDA_GPIO_PIN,
                         BOARD_WM8904_CODEC_I2C_SDA_ALTERNATE_FUNCTION,
                         (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP));
    if (status) {
        return status;
    }

    /* I2C_SCL */
    status = pinconf_set(PORT_(BOARD_WM8904_CODEC_I2C_SCL_GPIO_PORT),
                         BOARD_WM8904_CODEC_I2C_SCL_GPIO_PIN,
                         BOARD_WM8904_CODEC_I2C_SCL_ALTERNATE_FUNCTION,
                         (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP));
    if (status) {
        return status;
    }

    return APP_SUCCESS;
}
#endif

/**
  \fn          void playback_init(void)
  \brief       routine for enabling WM8904 (I2C) and I2S for playback
  \param[in]   None
*/
int32_t playback_init(void)
{
    int32_t status;

    status = board_i2s_dac_pins_config();
    if (status != ARM_DRIVER_OK) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", status);
        return status;
    }

#if BOARD_WM8904_CODEC_PRESENT
    status = board_wm8904_i2c_pins_config();
    if (status != ARM_DRIVER_OK) {
        printf("I2C pinmux failed\n");
        return status;
    }

    /* Initialize WM8904 driver (i.e. I2C) */
    status = wm8904->Initialize();
    if (status != ARM_DRIVER_OK) {
        printf("WM8904 codec Init failed status = %" PRId32 "\n", status);
        goto error_codec_initialize;
    }

    /* Enable clock/power for peripheral */
    status = wm8904->PowerControl(ARM_POWER_FULL);
    if (status != ARM_DRIVER_OK) {
        printf("WM8904 codec Power up failed status = %" PRId32 "\n", status);
        goto error_codec_power;
    }
#endif

    /* Initialize I2S driver */
    status = i2s_dac->Initialize(dac_callback);
    if (status != ARM_DRIVER_OK) {
        printf("DAC Init failed status = %" PRId32 "\n", status);
        goto error_dac_initialize;
    }

    /* Enable clock/power for peripheral */
    status = i2s_dac->PowerControl(ARM_POWER_FULL);
    if (status != ARM_DRIVER_OK) {
        printf("DAC Power Failed status = %" PRId32 "\n", status);
        goto error_dac_power;
    }

    /* configure I2S Transmitter to Asynchronous Master */
    status = i2s_dac->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_MODE_MASTER | ARM_SAI_ASYNCHRONOUS |
                                  ARM_SAI_PROTOCOL_I2S | ARM_SAI_DATA_SIZE(wlen),
                              wlen * 2,
                              sampling_rate);
    if (status != ARM_DRIVER_OK) {
        printf("DAC Control status = %" PRId32 "\n", status);
        goto error_dac_control;
    }

    /* enable Transmitter */
    status = i2s_dac->Control(ARM_SAI_CONTROL_TX, 1, 0);
    if (status != ARM_DRIVER_OK) {
        printf("DAC TX status = %" PRId32 "\n", status);
        goto error_dac_control;
    }

    return APP_SUCCESS;

error_dac_control:
    i2s_dac->PowerControl(ARM_POWER_OFF);
error_dac_power:
    i2s_dac->Uninitialize();
error_dac_initialize:
#if BOARD_WM8904_CODEC_PRESENT
error_codec_power:
    wm8904->PowerControl(ARM_POWER_OFF);
    wm8904->Uninitialize();
error_codec_initialize:
#endif

    return APP_ERROR;
}

/**
  \fn          void playback_deinit(void)
  \brief       routine for sending audio to I2S
  \param[in]   None
*/
int32_t playback_audio(const void *buf, uint32_t len)
{
    int32_t status;

    /* Transmit the samples */
    status = i2s_dac->Send(buf, len);
    if (status != ARM_DRIVER_OK) {
        printf("DAC Send status = %" PRId32 "\n", status);
        return status;
    }

    /* Wait for the completion event */
    call_back_event = 0;
    while (call_back_event == 0) pm_core_enter_normal_sleep();

    return APP_SUCCESS;
}

/**
  \fn          void playback_deinit(void)
  \brief       routine for stopping playback
  \param[in]   None
*/
int32_t playback_deinit(void)
{
    int32_t status;

    /* Stop the TX */
    status = i2s_dac->Control(ARM_SAI_CONTROL_TX, 0, 0);
    if (status != ARM_DRIVER_OK) {
        printf("DAC TX status = %" PRId32 "\n", status);
    }

    i2s_dac->PowerControl(ARM_POWER_OFF);
    i2s_dac->Uninitialize();
#if BOARD_WM8904_CODEC_PRESENT
    wm8904->PowerControl(ARM_POWER_OFF);
    wm8904->Uninitialize();
#endif

    return APP_SUCCESS;
}
