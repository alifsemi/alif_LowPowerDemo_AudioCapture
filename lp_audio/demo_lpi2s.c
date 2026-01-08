/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     : demo_lpi2s.c
 * @author   : Ahmad Rashed
 * @email    : ahmad.rashed@alifsemi.com
 * @version  : V1.0.0
 * @date     : 11-Nov-2025
 * @brief    : HE-only sample demo for LPI2S for E8 Devkit
 * @bug      : N/A
 * @Note     : None
 ******************************************************************************/

/* System Includes */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

/* Project Includes */
#include "app_utils.h"
#include "board_config.h"
#include "clock_runtime.h"
#include "RTE_Components.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#endif

#include "pinconf.h"
#include "Driver_SAI.h"

/* optional, higher clock accuracy but adds ~1mW */
#define USE_HFXO                       0

#define NUM_SAMPLES 48000   /* 500 ms at 48kHz (stereo) */

/* Buffer for PCM samples */
static uint32_t sample_buf[NUM_SAMPLES];

/* ADC callback events */
#define ADC_RECEIVE_COMPLETE_EVENT (1U << 1)
#define ADC_RECEIVE_OVERFLOW_EVENT (1U << 2)

static uint32_t wlen                = 24;
static uint32_t sampling_rate       = 48000; /* 48Khz audio sampling rate */
static volatile uint32_t event_flag;

extern ARM_DRIVER_SAI  ARM_Driver_SAI_(BOARD_MIC_INPUT_I2S_INSTANCE);
static ARM_DRIVER_SAI *i2s_adc = &ARM_Driver_SAI_(BOARD_MIC_INPUT_I2S_INSTANCE);

/**
  \fn          void adc_callback(uint32_t event)
  \brief       Callback routine from the i2s driver
  \param[in]   event Event for which the callback has been called
*/
static void adc_callback(uint32_t event)
{
    if (event & ARM_SAI_EVENT_RECEIVE_COMPLETE) {
        /* Receive Success: Wake-up routine. */
        event_flag |= ADC_RECEIVE_COMPLETE_EVENT;
    }

    if (event & ARM_SAI_EVENT_RX_OVERFLOW) {
        /* Receive Error: fifo overflow occurred. */
        event_flag |= ADC_RECEIVE_OVERFLOW_EVENT;
    }
}

/**
 * @fn      static int32_t board_i2s_adc_pins_config(void)
 * @brief   Configure I2S ADC pinmux.
 * @retval  execution status.
 */
static int32_t board_i2s_adc_pins_config(void)
{
    int32_t status;

    /* Configure ADC I2S WS */
    status = pinconf_set(PORT_(BOARD_MIC_INPUT_WS_GPIO_PORT),
                         BOARD_MIC_INPUT_WS_GPIO_PIN,
                         BOARD_MIC_INPUT_WS_ALTERNATE_FUNCTION,
                         0);
    if (status) {
        return status;
    }

    /* Configure ADC I2S SCLK */
    status = pinconf_set(PORT_(BOARD_MIC_INPUT_SCLK_GPIO_PORT),
                         BOARD_MIC_INPUT_SCLK_GPIO_PIN,
                         BOARD_MIC_INPUT_SCLK_ALTERNATE_FUNCTION,
                         0);
    if (status) {
        return status;
    }

    /* Configure ADC I2S SDI */
    status = pinconf_set(PORT_(BOARD_MIC_INPUT_SDI_GPIO_PORT),
                         BOARD_MIC_INPUT_SDI_GPIO_PIN,
                         BOARD_MIC_INPUT_SDI_ALTERNATE_FUNCTION,
                         PADCTRL_READ_ENABLE);
    if (status) {
        return status;
    }

    return APP_SUCCESS;
}

/**
 * @fn      static int32_t demo_power_config(void)
 * @brief   Configure MCU clock and power for the demo
 * @retval  execution status.
 */
static int32_t demo_power_config(void)
{
    uint32_t error_code = SERVICES_REQ_SUCCESS;
    uint32_t service_error_code;

    run_profile_t runp = {0};
    runp.aon_clk_src = CLK_SRC_LFXO;        // change to LFRC if LFXO is not present
#if USE_HFXO
    runp.run_clk_src = CLK_SRC_HFXO;
    runp.cpu_clk_freq = CLOCK_FREQUENCY_76_8_XO_MHZ;
#else
    runp.run_clk_src = CLK_SRC_HFRC;
    runp.cpu_clk_freq = CLOCK_FREQUENCY_76_8_RC_MHZ;
#endif
    runp.dcdc_mode = DCDC_MODE_PFM_FORCED;  // PFM is used at low loads
    runp.dcdc_voltage = DCDC_VOUT_0800;
    runp.memory_blocks = MRAM_MASK | BACKUP4K_MASK;
    // runp.power_domains = PD_DBSS_MASK;   // uncomment this line to enable JTAG

    error_code = SERVICES_set_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        printf("SE: run profile error = %" PRId32 "\n", error_code);
    }

    error_code = system_update_clock_values();
    if (error_code) {
        printf("SE: update clock values error = %" PRId32 "\n", error_code);
    }

#if SOC_FEAT_CLK76P8M_CLK_ENABLE
    /* enable the HFOSCx2 clock used by I2S */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              /*clock_enable_t*/ CLKEN_HFOSCx2,
                                              /*bool enable   */ true,
                                              &service_error_code);
    if (error_code) {
        printf("SE: clk enable error = %" PRId32 "\n", error_code);
    }
#endif

    /* disable SE power domain */
    error_code = SERVICES_power_se_sleep_req(se_services_s_handle, 0, &service_error_code);
    if (error_code) {
        printf("SE: secure enclave power down error = %" PRId32 "\n", error_code);
    }

    /* clear the request for DEBUG and SYSTOP power domains from the M55 side */
    *(volatile uint32_t*)0x1A010400 = 0;
}

/**
 * @fn      static int32_t restore_power_config(void)
 * @brief   Restore MCU clock and power prior to running the demo
 * @retval  execution status.
 */
static int32_t restore_power_config(void)
{
    uint32_t error_code;
    uint32_t service_error_code;

    run_profile_t runp = {0};
    runp.aon_clk_src = CLK_SRC_LFXO;        // change to LFRC if LFXO is not present
    runp.run_clk_src = CLK_SRC_HFRC;
    runp.cpu_clk_freq = CLOCK_FREQUENCY_76_8_RC_MHZ;
    runp.dcdc_mode = DCDC_MODE_PWM; /* PWM is used at typical loads (field is ignored on E1C / B1) */
    runp.dcdc_voltage = DCDC_VOUT_0800;
    runp.memory_blocks = MRAM_MASK | BACKUP4K_MASK;
    runp.power_domains = PD_DBSS_MASK | PD_SYST_MASK;

    error_code = SERVICES_set_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        printf("SE: run profile error = %" PRId32 "\n", error_code);
    }

    error_code = system_update_clock_values();
    if (error_code) {
        printf("SE: update clock values error = %" PRId32 "\n", error_code);
    }

#if SOC_FEAT_CLK76P8M_CLK_ENABLE
    /* disable the HFOSCx2 clock used by I2S */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              /*clock_enable_t*/ CLKEN_HFOSCx2,
                                              /*bool enable   */ false,
                                              &service_error_code);
    if (error_code) {
        printf("SE: clk enable error = %" PRId32 "\n", error_code);
    }
#endif

    /* clear the request for DEBUG and SYSTOP power domains from the M55 side */
    *(volatile uint32_t*)0x1A010400 = 0;
}

/**
  \fn          int32_t ADC_Init(void)
  \brief       routine to initialize ADC
  \param[in]   None
  \return      status
*/
int32_t ADC_Init(void)
{
    int32_t status;

    status = board_i2s_adc_pins_config();
    if (status != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", status);
        return status;
    }

    /* Initialize I2S driver */
    status = i2s_adc->Initialize(adc_callback);
    if (status) {
        printf("ADC Init failed status = %" PRId32 "\n", status);
        return status;
    }

    /* Enable clock/power for peripheral */
    status = i2s_adc->PowerControl(ARM_POWER_FULL);
    if (status) {
        printf("ADC Power failed status = %" PRId32 "\n", status);
        return status;
    }

    /* configure I2S Receiver to Asynchronous Master */
    status = i2s_adc->Control(ARM_SAI_CONFIGURE_RX | ARM_SAI_MODE_MASTER | ARM_SAI_ASYNCHRONOUS |
                                  ARM_SAI_PROTOCOL_I2S | ARM_SAI_DATA_SIZE(wlen),
                              wlen * 2,
                              sampling_rate);
    if (status) {
        printf("ADC Control status = %" PRId32 "\n", status);
        return status;
    }

    return APP_SUCCESS;
}

/**
  \fn          int32_t Receiver(void)
  \brief       Function performing reception from mic
  \param[in]   None
  \return      status
*/
int32_t Receiver(void)
{
    int32_t status;

    /* Receive data */
    status = i2s_adc->Receive((uint32_t *) sample_buf, NUM_SAMPLES);
    if (status) {
        printf("ADC Receive status = %" PRId32 "\n", status);
        return status;
    }

    /* Wait for the completion event */
    while (1) {
        pm_core_enter_normal_sleep();

        if (event_flag & ADC_RECEIVE_COMPLETE_EVENT) {
            event_flag &= ~ADC_RECEIVE_COMPLETE_EVENT;
            break;
        }

        if (event_flag & ADC_RECEIVE_OVERFLOW_EVENT) {
            event_flag &= ~ADC_RECEIVE_OVERFLOW_EVENT;
        }
    }

    return APP_SUCCESS;
}

/**
  \fn          int main(void)
  \brief       Application Main
  \return      int application exit status
*/
int main(void)
{
    sys_busy_loop_us(100000);

    /* UART driver uses SystemCoreClock variable to calculate baud rate divider */
    SystemCoreClock = 76800000;
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    if (stdout_init() != ARM_DRIVER_OK) {
        WAIT_FOREVER_LOOP;
    }
#endif

    printf("Low Power Demo: Audio Capture using LP-I2S\n");
    printf("MCU is being placed in a low power state\n");
    printf("Secure Enclave and JTAG will be powered down\n");

    /* Initialize the SE services */
    se_services_port_init();
    demo_power_config();

    /* Begin the demo Application */
    int32_t status;
    status = ADC_Init();
    if (status) {
        printf("ADC Init failed status = %" PRId32 "\n", status);
        WAIT_FOREVER_LOOP;
    }

    /* enable Receiver */
    status = i2s_adc->Control(ARM_SAI_CONTROL_RX, 1, 0);
    if (status) {
        printf("ADC RX Enable status = %" PRId32 "\n", status);
        WAIT_FOREVER_LOOP;
    }

    int32_t count = 0;
    do {
        status  = Receiver();
        if (status) {
            printf("ADC Receive failed status = %" PRId32 "\n", status);
            WAIT_FOREVER_LOOP;
        }
        printf("ADC Receive loop count = %" PRId32 "\n", ++count);
    } while ((count < 100) && (status == 0));

    /* disable Receiver */
    status = i2s_adc->Control(ARM_SAI_CONTROL_RX, 0, 0);
    if (status) {
        printf("ADC RX status = %" PRId32 "\n", status);
        WAIT_FOREVER_LOOP;
    }

    i2s_adc->PowerControl(ARM_POWER_OFF);
    i2s_adc->Uninitialize();

    restore_power_config();
    printf("MCU power state and JTAG are restored\n");
    printf("Secure Enclave is running\n");

    /* Application will be waiting here upon JTAG "attach" or "pause" */
    WAIT_FOREVER_LOOP;
    return 0;
}
