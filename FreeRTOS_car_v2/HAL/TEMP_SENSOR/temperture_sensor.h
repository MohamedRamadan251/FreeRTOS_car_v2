/*
 * temperture_sensor.h
 *
 *  Created on: Sep 13, 2023
 *      Author: Kareem Abdelrasheed
 */

#ifndef MCAL_TEMPERTURE_SENSOR_H_
#define MCAL_TEMPERTURE_SENSOR_H_
/*
 * Description:
 * Struct to specify configuration of ldr sensor sensor,
 * including GPIO and ADC.
 */
typedef struct {
    uint8_t sensorPort;
    uint8_t sensorPin;
    uint8_t adcChannel;
    uint8_t sequencer;
    uint8_t sample;
    uint8_t adc_module;
    uint8_t temp_sensor_mode;
}Temp_Sensor_t;


void temperature_sensor_init(Temp_Sensor_t * sensor);

uint32_t temperature_sensor_reading(Temp_Sensor_t * sensor);



#endif /* MCAL_TEMPERTURE_SENSOR_H_ */
