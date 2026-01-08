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
 * @file     : demo_lppdm.c
 * @author   : Ahmad Rashed
 * @email    : ahmad.rashed@alifsemi.com
 * @version  : V1.0.0
 * @date     : 11-Nov-2025
 * @brief    : HE-only sample demo for LPPDM for B1 DevKit or E1C Devkit
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
#include "RTE_Components.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#endif

#include "pinconf.h"
#include "Driver_PDM.h"

#define NUM_SAMPLES 16000   /* 500 ms at 16kHz (stereo) */

#define CHANNEL_0                      4
#define CHANNEL_1                      5
#define CHANNEL_MASKS                  (ARM_PDM_MASK_CHANNEL_4 | ARM_PDM_MASK_CHANNEL_5)

/* PDM Channel 0 configurations */
#define CH0_PHASE                      0x00000003
#define CH0_GAIN                       0x00000013
#define CH0_PEAK_DETECT_TH             0x00060002
#define CH0_PEAK_DETECT_ITV            0x00020027

/* PDM Channel 1 configurations */
#define CH1_PHASE                      0x0000001F
#define CH1_GAIN                       0x0000000D
#define CH1_PEAK_DETECT_TH             0x00060002
#define CH1_PEAK_DETECT_ITV            0x0004002D

/* PDM driver instance */
extern ARM_DRIVER_PDM  Driver_LPPDM;
static ARM_DRIVER_PDM *PDMdrv = &Driver_LPPDM;

PDM_CH_CONFIG pdm_coef_reg;

/* For Demo purpose use channel 0  and channel 1 */
/* To store the PCM samples for Channel 0 and channel 1 */
static volatile uint16_t sample_buf[NUM_SAMPLES];

/* Channel 0 FIR coefficient */
uint32_t ch0_fir[18] = {0x00000000,
                        0x000007FF,
                        0x00000000,
                        0x00000004,
                        0x00000004,
                        0x000007FC,
                        0x00000000,
                        0x000007FB,
                        0x000007E4,
                        0x00000000,
                        0x0000002B,
                        0x00000009,
                        0x00000016,
                        0x00000049,
                        0x00000793,
                        0x000006F8,
                        0x00000045,
                        0x00000178};

/* Channel 1 FIR coefficient */
uint32_t ch1_fir[18] = {0x00000001,
                        0x00000003,
                        0x00000003,
                        0x000007F4,
                        0x00000004,
                        0x000007ED,
                        0x000007F5,
                        0x000007F4,
                        0x000007D3,
                        0x000007FE,
                        0x000007BC,
                        0x000007E5,
                        0x000007D9,
                        0x00000793,
                        0x00000029,
                        0x0000072C,
                        0x00000072,
                        0x000002FD};

/* PDM callback events */
typedef enum {
    PDM_CALLBACK_ERROR_EVENT           = (1 << 0),
    PDM_CALLBACK_WARNING_EVENT         = (1 << 1),
    PDM_CALLBACK_AUDIO_DETECTION_EVENT = (1 << 2)
} PDM_CB_EVENTS;

volatile int32_t call_back_event;

static void PDM_fifo_callback(uint32_t event)
{
    if (event & ARM_PDM_EVENT_ERROR) {
        call_back_event |= PDM_CALLBACK_ERROR_EVENT;
    }

    if (event & ARM_PDM_EVENT_CAPTURE_COMPLETE) {
        call_back_event |= PDM_CALLBACK_WARNING_EVENT;
    }

    if (event & ARM_PDM_EVENT_AUDIO_DETECTION) {
        call_back_event |= PDM_CALLBACK_AUDIO_DETECTION_EVENT;
    }
}

/**
 * @fn      static int32_t board_lppdm_pins_config(void)
 * @brief   Configure LPPDM pinmux.
 * @retval  execution status.
 */
static int32_t board_lppdm_pins_config(void)
{
    int32_t status;

    /* channel 0_1 data line */
    status = pinconf_set(PORT_(BOARD_LPPDM_D0_GPIO_PORT),
                         BOARD_LPPDM_D0_GPIO_PIN,
                         BOARD_LPPDM_D0_ALTERNATE_FUNCTION,
                         PADCTRL_READ_ENABLE);
    if (status) {
        return status;
    }

    /* channel 2_3 data line */
    status = pinconf_set(PORT_(BOARD_LPPDM_D1_GPIO_PORT),
                         BOARD_LPPDM_D1_GPIO_PIN,
                         BOARD_LPPDM_D1_ALTERNATE_FUNCTION,
                         PADCTRL_READ_ENABLE);
    if (status) {
        return status;
    }

    /* channel 4_5 data line */
    status = pinconf_set(PORT_(BOARD_LPPDM_D2_GPIO_PORT),
                         BOARD_LPPDM_D2_GPIO_PIN,
                         BOARD_LPPDM_D2_ALTERNATE_FUNCTION,
                         PADCTRL_READ_ENABLE);
    if (status) {
        return status;
    }

    /* channel 6_7 data line */
    status = pinconf_set(PORT_(BOARD_LPPDM_D3_GPIO_PORT),
                         BOARD_LPPDM_D3_GPIO_PIN,
                         BOARD_LPPDM_D3_ALTERNATE_FUNCTION,
                         PADCTRL_READ_ENABLE);
    if (status) {
        return status;
    }

    /* Channel 0_1 clock line */
    status = pinconf_set(PORT_(BOARD_LPPDM_C0_GPIO_PORT),
                         BOARD_LPPDM_C0_GPIO_PIN,
                         BOARD_LPPDM_C0_ALTERNATE_FUNCTION,
                         PADCTRL_DRIVER_DISABLED_HIGH_Z);
    if (status) {
        return status;
    }

    /* Channel 2_3 clock line */
    status = pinconf_set(PORT_(BOARD_LPPDM_C1_GPIO_PORT),
                         BOARD_LPPDM_C1_GPIO_PIN,
                         BOARD_LPPDM_C1_ALTERNATE_FUNCTION,
                         PADCTRL_DRIVER_DISABLED_HIGH_Z);
    if (status) {
        return status;
    }

    /* Channel 4_5 clock line */
    status = pinconf_set(PORT_(BOARD_LPPDM_C2_GPIO_PORT),
                         BOARD_LPPDM_C2_GPIO_PIN,
                         BOARD_LPPDM_C2_ALTERNATE_FUNCTION,
                         PADCTRL_DRIVER_DISABLED_HIGH_Z);
    if (status) {
        return status;
    }

    /* Channel 6_7 clock line */
    status = pinconf_set(PORT_(BOARD_LPPDM_C3_GPIO_PORT),
                         BOARD_LPPDM_C3_GPIO_PIN,
                         BOARD_LPPDM_C3_ALTERNATE_FUNCTION,
                         PADCTRL_DRIVER_DISABLED_HIGH_Z);
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
    runp.run_clk_src = CLK_SRC_HFXO;
    runp.cpu_clk_freq = CLOCK_FREQUENCY_76_8_XO_MHZ;
    runp.dcdc_voltage = DCDC_VOUT_0800;
    runp.memory_blocks = MRAM_MASK | BACKUP4K_MASK;
    // runp.power_domains = PD_DBSS_MASK;   // uncomment this line to enable JTAG

    error_code = SERVICES_set_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        printf("SE: run profile error = %" PRId32 "\n", error_code);
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
 * @fn         : void pdm_demo()
 * @brief      : PDM demo application
 *               -> Initialize the LPPDM module.
 *               -> Enable the Power for the LPPDM module
 *               -> Select the mode of operation in Control API.The
 *                    mode which user has selected will be applies to
 *                    all the channel which user has selected.
 *               -> Select the Bypass DC blocking IIR filter for reference.
 *               -> Select the LPPDM channel and use the selected channel
 *                    configuration and status register values.
 *               -> Play some audio and start capturing the data.
 *               -> Once all data has stored in the particular buffer ,
 *                  call back event will be set and it will stop capturing
 *                  data.
 *               -> Once all the data capture is done , go to the particular
 *                  memory location which user has given for storing PCM data
 *                  samples.
 *               -> Then export the memory and give the total size of
 *                  the buffer memory and select the particular bin file and export
 *                  the memory.
 *               -> Play the PCM sample file using ffplay command.
 * @return     : none
 */
void pdm_demo()
{
    int32_t status;

    status = board_lppdm_pins_config();
    if (status != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", status);
        return;
    }

    /* Initialize PDM driver */
    status = PDMdrv->Initialize(PDM_fifo_callback);
    if (status != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM init failed\n");
        return;
    }

    /* Enable clock/power for peripheral */
    status = PDMdrv->PowerControl(ARM_POWER_FULL);
    if (status != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Power up failed\n");
        goto error_uninitialize;
    }

    /* PDM Channel Selection:
     * This code selects PDM channel 0 and channel 1 for operation.
     * To select different channels (e.g., channel 2 and channel 3), update the macro parameter
     * in the PDMdrv->Control function as follows:
     * (ARM_PDM_MASK_CHANNEL_2 | ARM_PDM_MASK_CHANNEL_3)
     * Note: These macros are defined in Driver_PDM.h.
     */
    status = PDMdrv->Control(ARM_PDM_SELECT_CHANNEL,
                          (CHANNEL_MASKS),
                          0);
    if (status != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM channel select control failed\n");
        goto error_poweroff;
    }

    /* Select Standard voice PDM mode */
    status = PDMdrv->Control(ARM_PDM_MODE, ARM_PDM_MODE_AUDIOFREQ_16K_DECM_48, 0);
    if (status != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Standard voice control mode failed\n");
        goto error_poweroff;
    }

    /* Select the DC blocking IIR filter */
    status = PDMdrv->Control(ARM_PDM_BYPASS_IIR_FILTER, ENABLE, 0);
    if (status != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM DC blocking IIR control failed\n");
        goto error_poweroff;
    }

    /* Set Channel 0 Phase value */
    status = PDMdrv->Control(ARM_PDM_CHANNEL_PHASE, CHANNEL_0, CH0_PHASE);
    if (status != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    /* Set Channel 0 Gain value */
    status = PDMdrv->Control(ARM_PDM_CHANNEL_GAIN, CHANNEL_0, CH0_GAIN);
    if (status != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    /* Set Channel 0 Peak detect threshold value */
    status = PDMdrv->Control(ARM_PDM_CHANNEL_PEAK_DETECT_TH, CHANNEL_0, CH0_PEAK_DETECT_TH);
    if (status != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    /* Set Channel 0 Peak detect ITV value */
    status = PDMdrv->Control(ARM_PDM_CHANNEL_PEAK_DETECT_ITV, CHANNEL_0, CH0_PEAK_DETECT_ITV);
    if (status != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    pdm_coef_reg.ch_num = CHANNEL_0;
    memcpy(pdm_coef_reg.ch_fir_coef,
           ch0_fir,
           sizeof(pdm_coef_reg.ch_fir_coef)); /* Channel 0 fir coefficient */
    pdm_coef_reg.ch_iir_coef = 0x00000004;    /* Channel IIR Filter Coefficient */

    status = PDMdrv->Config(&pdm_coef_reg);
    if (status != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    /* Set Channel 1 Phase value */
    status = PDMdrv->Control(ARM_PDM_CHANNEL_PHASE, CHANNEL_1, CH1_PHASE);
    if (status != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    /* Set Channel 1 Gain value */
    status = PDMdrv->Control(ARM_PDM_CHANNEL_GAIN, CHANNEL_1, CH1_GAIN);
    if (status != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    /* Set Channel 1 Peak detect threshold value */
    status = PDMdrv->Control(ARM_PDM_CHANNEL_PEAK_DETECT_TH, CHANNEL_1, CH1_PEAK_DETECT_TH);
    if (status != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    /* Set Channel 1 Peak detect ITV value */
    status = PDMdrv->Control(ARM_PDM_CHANNEL_PEAK_DETECT_ITV, CHANNEL_1, CH1_PEAK_DETECT_ITV);
    if (status != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    /* Channel 1 configuration values */
    pdm_coef_reg.ch_num = CHANNEL_1; /* Channel 1 */
    memcpy(pdm_coef_reg.ch_fir_coef,
           ch1_fir,
           sizeof(pdm_coef_reg.ch_fir_coef)); /* Channel 1 fir coefficient*/
    pdm_coef_reg.ch_iir_coef = 0x00000004;    /* Channel IIR Filter Coefficient */

    status = PDMdrv->Config(&pdm_coef_reg);
    if (status != ARM_DRIVER_OK) {
        printf("\r\n Error: PDM Channel_Config failed\n");
        goto error_uninitialize;
    }

    printf("\n------> Start Speaking or Play some Audio!------> \n");

    int32_t count = 20;
    while(count-- > 0) {
        /* Receive the audio samples */
        call_back_event = 0;
        status = PDMdrv->Receive((uint16_t *) sample_buf, NUM_SAMPLES);
        if (status != ARM_DRIVER_OK) {
            printf("\r\n Error: PDM Receive failed\n");
            goto error_capture;
        }

        /* wait for the call back event */
        while (call_back_event == 0) pm_core_enter_normal_sleep();

        /* PDM fifo overflow error event */
        if (call_back_event == PDM_CALLBACK_ERROR_EVENT) {
            printf("\n PDM error event: Fifo overflow \n");
        }

        /* PDM fifo alomost full warning event */
        if (call_back_event == PDM_CALLBACK_WARNING_EVENT) {
            printf("\n PDM warning event : Fifo almost full\n");
        }

        /* PDM channel audio detection event */
        if (call_back_event == PDM_CALLBACK_AUDIO_DETECTION_EVENT) {
            printf("\n PDM audio detect event: data in the audio channel");
        }
    }

    call_back_event = 0;

    printf("\n------> Stop recording ------> \n");
    printf("\n--> PCM samples will be stored in 0x%" PRIxPTR " "
           "address and size of buffer is %" PRIu32 "\n",
           (uintptr_t) sample_buf,
           sizeof(sample_buf));

error_capture:
error_poweroff:
    status = PDMdrv->PowerControl(ARM_POWER_OFF);
    if (status != ARM_DRIVER_OK) {
        printf("\n Error: PDM power off failed\n");
    }

error_uninitialize:
    status = PDMdrv->Uninitialize();
    if (status != ARM_DRIVER_OK) {
        printf("\n Error: PDM Uninitialize failed\n");
    }
}

int main()
{
    sys_busy_loop_us(100000);

    /* UART driver uses SystemCoreClock variable to calculate baud rate divider */
    SystemCoreClock = 76800000;
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    if (stdout_init() != ARM_DRIVER_OK) {
        WAIT_FOREVER_LOOP;
    }
#endif

    printf("Low Power Demo: Audio Capture using LP-PDM\n");
    printf("MCU is being placed in a low power state\n");
    printf("Secure Enclave and JTAG will be powered down\n");

    /* Initialize the SE services */
    se_services_port_init();
    demo_power_config();

    /* Begin the demo Application */
    pdm_demo();

    restore_power_config();
    printf("MCU power state and JTAG are restored\n");
    printf("Secure Enclave is running\n");

    /* Application will be waiting here upon JTAG "attach" */
    WAIT_FOREVER_LOOP;
    return 0;
}
