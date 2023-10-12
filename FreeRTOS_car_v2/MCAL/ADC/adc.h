/*******************************************************************************
 *                                                                             *
 * [FILE NAME]:   adc.h                                                        *
 *                                                                             *
 * [AUTHOR]:      Ahmed Saeed, Amr Gafar, Kareem Abd-elrasheed,                *
 *                Mohamed Abosreea, Ahmed Maher, and Mohamed Ramadan           *
 *                                                                             *
 * [Version]:     1.0.0                                                        *
 *                                                                             *
 * [DATE]:        09/09/2023                                                   *
 *                                                                             *
 * [DESCRIPTION]: header file for ADC driver                                   *
 *                                                                             *
 *******************************************************************************/

#ifndef ADC_H_
#define ADC_H_

/*******************************************************************************
 *                                 Includes                                    *
 *******************************************************************************/
#include "LIB/std_types.h"

/*******************************************************************************
 *                               Types Declaration                             *
 *******************************************************************************/
/*
 * Description:
 * Enum defining ADC modules (ADC_0, ADC_1).
 */
enum ADC
{
    ADC_0, ADC_1
};

/*
 * Description:
 * Enum defining ADC channels (CH_0 to CH_11).
 */
enum CHANNEL
{
    CH_0, CH_1, CH_2, CH_3, CH_4, CH_5, CH_6, CH_7, CH_8, CH_9, CH_10, CH_11
};

/*
 * Description:
 * Enum defining ADC sample sequencers (SS_0, SS_1, SS_2, SS_3).
 */
enum SAMPLE_SEQUENSER
{
    SS_0, SS_1, SS_2, SS_3
};

/*
 * Description:
 * Enum defining ADC samples (S_0 to S_7).
 */
enum SAMPLE
{
    S_0, S_1, S_2, S_3, S_4, S_5, S_6, S_7
};

enum TEMP_MODE
{
    NO=0,YES
};
/*
 * Description:
 * Structure defining the configuration for an ADC channel, including the ADC module,
 * port, pin, sequencer, channel, and sample settings.
 */
typedef struct
{
    uint8_t ADC_module;
    uint8_t ADC_port;
    uint8_t ADC_pin;
    uint8_t ADC_sequencer;
    uint8_t ADC_channel;
    uint8_t ADC_sample;
    uint8_t ADC_temp_mode;
}ADC_Config;

/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/
/*
 * Description:
 * This function configures the ADC module by enabling its clock,
 * configuring the GPIO pin, disabling the specified sequencer,
 * selecting the trigger option, and enabling the sequencer.
 * Parameters:
 * - Copy_ADC: A pointer to an ADC_Config structure containing ADC configuration.
 * Returns: None.
 */
void ADC_Configure(ADC_Config * Copy_ADC);

/*
 * Description:
 * This function performs an ADC conversion for the specified ADC module, sequencer, and channel.
 * It starts the conversion, waits until it's complete, and returns the converted value.
 * Parameters:
 * - Copy_ADC: A pointer to an ADC_Config structure containing ADC configuration.
 * Returns:
 * - Local_reading: The converted ADC value.
 */
uint32_t ADC_Get_Conversion(ADC_Config * Copy_ADC);

#endif /* ADC_H_ */
