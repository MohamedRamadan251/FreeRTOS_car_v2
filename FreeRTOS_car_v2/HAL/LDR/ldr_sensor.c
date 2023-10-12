/*******************************************************************************
 *                                                                             *
 * [FILE NAME]:   gpio.h                                                       *
 *                                                                             *
 * [AUTHOR]:      Ahmed Saeed, Amr Gafar, Kareem Abd-elrasheed,                *
 *                Mohamed Abosreea, Ahmed Maher, and Mohamed Ramadan           *
 *                                                                             *
 * [Version]:     1.0.0                                                        *
 *                                                                             *
 * [DATE]:        09/09/2023                                                   *
 *                                                                             *
 * [DESCRIPTION]: source file for ldr sensor driver                     *
 *                                                                             *
 *******************************************************************************/
#include <stdio.h>
#include <stdlib.h>

#include "LIB/bitwise_operation.h"

#include "../../MCAL/ADC/adc.h"
#include "../../MCAL/general_functions.h"

#include "ldr_sensor.h"

/*
 * Description:
 * This function initializes the ldr sensor pins on the specified port and pin number.
 * It configures the pin as output based on the provided direction,
 * and sets up the necessary GPIO settings including pull-up/pull-down resistors.
 * Parameters:
 * - sensor: pointer to the ldr sensor configuration (PORT_ID , PIN_ID , ADC_CHANNEL , SEQUENCER ).
 * Returns: None.
 */
void ldr_sensor_init(LDR_Sensor_t *sensor)
{
    ADC_Config ldr = {
                      .ADC_channel = sensor->adcChannel,
                      .ADC_module = sensor->adc_module,
                      .ADC_pin = sensor->ldrPin,
                      .ADC_port = sensor->ldrPort,
                      .ADC_sequencer = sensor->sequencer,
                      .ADC_sample = sensor->sample,
                      .ADC_temp_mode = sensor->temp_sensor_mode
    };
    /*Set configuration of the ldr sensor*/
    ADC_Configure(&ldr);
}

/*
 * Description:
 * This function initializes the ldr sensor pins on the specified port and pin number.
 * It configures the pin as output based on the provided direction,
 * and sets up the necessary GPIO settings including pull-up/pull-down resistors.
 * Parameters:
 * - sensor: pointer to the ldr sensor configuration (PORT_ID , PIN_ID , ADC_CHANNEL , SEQUENCER ).
 * Returns: ADC value ( 0 to  4095 ).
 */
uint16_t ldr_sensor_reading(LDR_Sensor_t *sensor)
{
    ADC_Config ldr = {
                          .ADC_channel = sensor->adcChannel,
                          .ADC_module = sensor->adc_module,
                          .ADC_pin = sensor->ldrPin,
                          .ADC_port = sensor->ldrPort,
                          .ADC_sequencer = sensor->sequencer,
                          .ADC_sample = sensor->sample,
                          .ADC_temp_mode = sensor->temp_sensor_mode
        };
    return ADC_Get_Conversion(&ldr);
}
