/*
 * temperture_sensor.c
 *
 *  Created on: Sep 13, 2023
 *      Author: Kareem Abdelrasheed
 */

#include "LIB/bitwise_operation.h"
#include "tm4c123gh6pm.h"


#include <stdio.h>
#include <stdlib.h>


#include "../../MCAL/GPIO/gpio.h"
#include "../../MCAL/ADC/adc.h"


#include "temperture_sensor.h"

void temperature_sensor_init(Temp_Sensor_t * sensor)
{
    ADC_Config temp_sensor = {
                          .ADC_channel = sensor->adcChannel,
                          .ADC_module = sensor->adc_module,
                          .ADC_pin = sensor->sensorPin,
                          .ADC_port = sensor->sensorPort,
                          .ADC_sequencer = sensor->sequencer,
                          .ADC_sample = sensor->sample,
                          .ADC_temp_mode = sensor->temp_sensor_mode
        };

    ADC_Configure(&temp_sensor);

}

uint32_t temperature_sensor_reading(Temp_Sensor_t * sensor)
{
    uint32_t Local_reading=0U;

    ADC_Config temp_sensor = {
                          .ADC_channel = sensor->adcChannel,
                          .ADC_module = sensor->adc_module,
                          .ADC_pin = sensor->sensorPin,
                          .ADC_port = sensor->sensorPort,
                          .ADC_sequencer = sensor->sequencer,
                          .ADC_sample = sensor->sample,
                          .ADC_temp_mode = sensor->temp_sensor_mode
        };

    Local_reading = ADC_Get_Conversion(&temp_sensor);

    return Local_reading;
}
