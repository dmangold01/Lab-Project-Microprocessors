#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_lpadc.h"
#include "fsl_inputmux.h"
#include "fsl_power.h"
#include "fsl_anactrl.h"
#include "fsl_gpio.h" // Include GPIO library

#define DEMO_LPADC_BASE                  ADC0
#define DEMO_LPADC_USER_CHANNEL          0U /* PIO0_10 is ADC Channel 0 */
#define DEMO_LPADC_USER_CMDID            1U /* CMD1 */
#define DEMO_LPADC_VREF_SOURCE           kLPADC_ReferenceVoltageAlt2
#define DEMO_LPADC_DO_OFFSET_CALIBRATION false
#define DEMO_LPADC_OFFSET_VALUE_A        0x10U
#define DEMO_LPADC_OFFSET_VALUE_B        0x10U

#define LED_PORT                         1U  // Port for LEDs
#define LED1_PIN                         9U  // PIO1_9 for launched LED
#define LED2_PIN                         10U // PIO1_10 for no presence detected LED
#define LED3_PIN                         5U  // PIO1_5 for launch sequence LED

/*******************************************************************************
 * Variables
 ******************************************************************************/
#if (defined(DEMO_LPADC_USE_HIGH_RESOLUTION) && DEMO_LPADC_USE_HIGH_RESOLUTION)
const uint32_t g_LpadcFullRange   = 65536U;
const uint32_t g_LpadcResultShift = 0U;
#else
const uint32_t g_LpadcFullRange   = 4096U;
const uint32_t g_LpadcResultShift = 3U;
#endif /* DEMO_LPADC_USE_HIGH_RESOLUTION */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void delay_ms(uint32_t ms);

/*******************************************************************************
 * Code
 ******************************************************************************/
void delay_ms(uint32_t ms)
{
    uint32_t i;
    for (i = 0; i < ms * 1000; i++)
    {
        __NOP(); /* Simple delay loop */
    }
}

int main(void)
{
    lpadc_config_t mLpadcConfigStruct;
    lpadc_conv_trigger_config_t mLpadcTriggerConfigStruct;
    lpadc_conv_command_config_t mLpadcCommandConfigStruct;
    lpadc_conv_result_t mLpadcResultConfigStruct;

    /* Set BOD VBAT level to 1.65V */
    POWER_SetBodVbatLevel(kPOWER_BodVbatLevel1650mv, kPOWER_BodHystLevel50mv, false);
    /* Attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    CLOCK_SetClkDiv(kCLOCK_DivAdcAsyncClk, 8U, true);
    CLOCK_AttachClk(kMAIN_CLK_to_ADC_CLK);

    /* Disable LDOGPADC power down */
    POWER_DisablePD(kPDRUNCFG_PD_LDOGPADC);

    ANACTRL_Init(ANACTRL);
    ANACTRL_EnableVref1V(ANACTRL, true);

    PRINTF("LPADC Continuous Polling Example\r\n");

    LPADC_GetDefaultConfig(&mLpadcConfigStruct);
    mLpadcConfigStruct.enableAnalogPreliminary = true;
#if defined(DEMO_LPADC_VREF_SOURCE)
    mLpadcConfigStruct.referenceVoltageSource = DEMO_LPADC_VREF_SOURCE;
#endif
    LPADC_Init(DEMO_LPADC_BASE, &mLpadcConfigStruct);

#if defined(FSL_FEATURE_LPADC_HAS_CTRL_CALOFS) && FSL_FEATURE_LPADC_HAS_CTRL_CALOFS
#if defined(DEMO_LPADC_DO_OFFSET_CALIBRATION) && DEMO_LPADC_DO_OFFSET_CALIBRATION
    LPADC_DoOffsetCalibration(DEMO_LPADC_BASE);
#else
    LPADC_SetOffsetValue(DEMO_LPADC_BASE, DEMO_LPADC_OFFSET_VALUE_A, DEMO_LPADC_OFFSET_VALUE_B);
#endif
#endif

#if (defined(FSL_FEATURE_LPADC_HAS_CFG_CALOFS) && FSL_FEATURE_LPADC_HAS_CFG_CALOFS)
    LPADC_DoAutoCalibration(DEMO_LPADC_BASE);
#endif

    /* Set conversion CMD configuration */
    LPADC_GetDefaultConvCommandConfig(&mLpadcCommandConfigStruct);
    mLpadcCommandConfigStruct.channelNumber = DEMO_LPADC_USER_CHANNEL;
    LPADC_SetConvCommandConfig(DEMO_LPADC_BASE, DEMO_LPADC_USER_CMDID, &mLpadcCommandConfigStruct);

    /* Set trigger configuration */
    LPADC_GetDefaultConvTriggerConfig(&mLpadcTriggerConfigStruct);
    mLpadcTriggerConfigStruct.targetCommandId       = DEMO_LPADC_USER_CMDID;
    mLpadcTriggerConfigStruct.enableHardwareTrigger = false;
    LPADC_SetConvTriggerConfig(DEMO_LPADC_BASE, 0U, &mLpadcTriggerConfigStruct); /* Configure trigger0 */

    PRINTF("ADC Full Range: %d\r\n", g_LpadcFullRange);

    /* Configure GPIO for LEDs */
    gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 0};  // Set as output, initial low
    GPIO_PinInit(GPIO, LED_PORT, LED1_PIN, &led_config);     // Launched LED
    GPIO_PinInit(GPIO, LED_PORT, LED2_PIN, &led_config);     // No presence detected LED
    GPIO_PinInit(GPIO, LED_PORT, LED3_PIN, &led_config);     // Launch sequence LED

    uint8_t presenceDetected = 0;

    while (1)
    {
        LPADC_DoSoftwareTrigger(DEMO_LPADC_BASE, 1U); /* Trigger ADC Conversion */

        /* Wait for Conversion Result */
#if (defined(FSL_FEATURE_LPADC_FIFO_COUNT) && (FSL_FEATURE_LPADC_FIFO_COUNT == 2U))
        while (!LPADC_GetConvResult(DEMO_LPADC_BASE, &mLpadcResultConfigStruct, 0U))
#else
        while (!LPADC_GetConvResult(DEMO_LPADC_BASE, &mLpadcResultConfigStruct))
#endif
        {
        }

        /* Display ADC Result */
        uint32_t adcRawValue = ((mLpadcResultConfigStruct.convValue) >> g_LpadcResultShift);
        PRINTF("ADC Raw Value: %d   ", adcRawValue);

        // Convert raw ADC value to voltage
        float voltage = (float)adcRawValue * 3.3f / g_LpadcFullRange; // Assuming 3.3V reference

        if (adcRawValue < 75) // Presence detected
        {
            if (!presenceDetected)
            {
                GPIO_PinWrite(GPIO, LED_PORT, LED2_PIN, 0); // LED2 OFF
                PRINTF("Presence detected\n\r");
                presenceDetected = 1;

                // Launch sequence: Flash LED3 for 3 seconds
                for (int i = 0; i < 6; i++)
                {
                    // Check for presence during the launch sequence
                    LPADC_DoSoftwareTrigger(DEMO_LPADC_BASE, 1U); /* Trigger ADC Conversion */

                    /* Wait for Conversion Result */
                    while (!LPADC_GetConvResult(DEMO_LPADC_BASE, &mLpadcResultConfigStruct, 0))
                    {
                    }

                    uint32_t adcRawValueDuringLaunch = ((mLpadcResultConfigStruct.convValue) >> g_LpadcResultShift);

                    if (adcRawValueDuringLaunch >= 75) // No presence detected
                    {
                        PRINTF("Launch canceled due to absence\n\r");
                        presenceDetected = 0;

                        // Turn off LEDs for the launch sequence
                        GPIO_PinWrite(GPIO, LED_PORT, LED3_PIN, 0); // LED3 OFF
                        GPIO_PinWrite(GPIO, LED_PORT, LED1_PIN, 0); // LED1 OFF
                        break; // Exit the launch sequence
                    }

                    GPIO_PinWrite(GPIO, LED_PORT, LED3_PIN, 1); // LED3 ON
                    delay_ms(10000); // Flash duration
                    GPIO_PinWrite(GPIO, LED_PORT, LED3_PIN, 0); // LED3 OFF
                    GPIO_PinWrite(GPIO, LED_PORT, LED1_PIN, 1); // LED1 OFF
                    delay_ms(10000); // Flash interval
                }

                if (presenceDetected) // If presence still detected after launch sequence
                {
                    // Final check before completing launch
                    LPADC_DoSoftwareTrigger(DEMO_LPADC_BASE, 1U); /* Trigger ADC Conversion */

                    /* Wait for Conversion Result */
                    while (!LPADC_GetConvResult(DEMO_LPADC_BASE, &mLpadcResultConfigStruct, 0))
                    {
                    }

                    uint32_t adcRawValueAfterLaunch = ((mLpadcResultConfigStruct.convValue) >> g_LpadcResultShift);

                    if (adcRawValueAfterLaunch < 75) // No presence detected after the launch sequence
                    {
                        PRINTF("Launch canceled due to absence after launch sequence\n\r");
                        presenceDetected = 0;

                        // Turn off LEDs for the launch sequence
                        GPIO_PinWrite(GPIO, LED_PORT, LED3_PIN, 0); // LED3 OFF
                        GPIO_PinWrite(GPIO, LED_PORT, LED1_PIN, 0); // LED1 OFF
                    }
                    else
                    {
                        // Presence is still detected, proceed to launch completion
                        GPIO_PinWrite(GPIO, LED_PORT, LED1_PIN, 1); // LED1 ON
                        PRINTF("Launch sequence completed successfully\n\r");
                    }
                }
            }
        }
        else // No presence detected
        {
            PRINTF("Awaiting presence\r\n");
            presenceDetected = 0;

            // Turn off Launched LED
            GPIO_PinWrite(GPIO, LED_PORT, LED1_PIN, 0); // LED1 OFF

            // Turn on No Presence LED
            GPIO_PinWrite(GPIO, LED_PORT, LED2_PIN, 1); // LED2 ON
        }
    }
}
