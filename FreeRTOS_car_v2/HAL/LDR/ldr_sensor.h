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
 * [DESCRIPTION]: header file for ldr sensor driver                     *
 *                                                                             *
 *******************************************************************************/
#ifndef HAL_LDR_LDR_INTERFACE_H_
#define HAL_LDR_LDR_INTERFACE_H_
/*******************************************************************************
 *                                 Includes                                    *
 *******************************************************************************/
#include "LIB/std_types.h"

#include "../../MCAL/ADC/adc.h"
/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/

/*******************************************************************************
 *                               Types Declaration                             *
 *******************************************************************************/
/*
 * Description:
 * Struct to specify configuration of ldr sensor sensor,
 * including GPIO and ADC.
 */
typedef struct {
    uint8_t ldrPort;
    uint8_t ldrPin;
    uint8_t adcChannel;
    uint8_t sequencer;
    uint8_t sample;
    uint8_t adc_module;
    uint8_t temp_sensor_mode;
}LDR_Sensor_t;

/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/

/*
 * Description:
 * This function initializes the ldr sensor pins on the specified port and pin number.
 * It configures the pin as output based on the provided direction,
 * and sets up the necessary GPIO settings including pull-up/pull-down resistors.
 * Parameters:
 * - sensor: pointer to the ldr sensor configuration (PORT_ID , PIN_ID , ADC_CHANNEL , SEQUENCER ).
 * Returns: None.
 */
void ldr_sensor_init(LDR_Sensor_t *sensor);

/*
 * Description:
 * This function initializes the ldr sensor pins on the specified port and pin number.
 * It configures the pin as output based on the provided direction,
 * and sets up the necessary GPIO settings including pull-up/pull-down resistors.
 * Parameters:
 * - sensor: pointer to the ldr sensor configuration (PORT_ID , PIN_ID , ADC_CHANNEL , SEQUENCER ).
 * Returns: ADC value ( 0 to  4095 ).
 */
uint16_t ldr_sensor_reading(LDR_Sensor_t *sensor);


#endif /* HAL_LDR_LDR_INTERFACE_H_ */
