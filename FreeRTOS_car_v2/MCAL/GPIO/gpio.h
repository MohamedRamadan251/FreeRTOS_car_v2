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
 * [DESCRIPTION]: header file for GPIO driver                                  *
 *                                                                             *
 *******************************************************************************/

#ifndef GPIO_H_
#define GPIO_H_

/*******************************************************************************
 *                                 Includes                                    *
 *******************************************************************************/
#include "LIB/std_types.h"

/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/
/* Total Number of GPIO Ports */
#define NUM_OF_PORTS           6

/*
 * GPIO Port Identifiers
 * These macros define identifiers for the different GPIO ports.
 */
#define PORTA_ID               0
#define PORTB_ID               1
#define PORTC_ID               2
#define PORTD_ID               3
#define PORTE_ID               4
#define PORTF_ID               5

/*
 * GPIO Pin Identifiers
 * These macros define the identifiers for individual GPIO pins (0 to 7).
 */
#define PIN0_ID                0
#define PIN1_ID                1
#define PIN2_ID                2
#define PIN3_ID                3
#define PIN4_ID                4
#define PIN5_ID                5
#define PIN6_ID                6
#define PIN7_ID                7

/*******************************************************************************
 *                               Types Declaration                             *
 *******************************************************************************/
/*
 * Description:
 * Enum to specify different GPIO pin configurations including
 * input with pull-up, input with pull-down, output with pull-up, and
 * output with pull-down.
 */
typedef enum
{
    PIN_INPUT_PULL_UP,PIN_INPUT_PULL_DOWN,PIN_OUTPUT_PULL_UP,PIN_OUTPUT_PULL_DOWN
}GPIO_PinDirectionType;

/*
 * Description:
 * Enum to specify different GPIO port direction configurations
 * including input (0x00) and output (0xFF).
 */
typedef enum
{
    PORT_INPUT = 0 , PORT_OUTPUT=0xFF
}GPIO_PortDirectionType;

/*
 * Description:
 * Enum to specify different GPIO alternative functions,
 * including ADC and TIMER.
 */
typedef enum
{
    ADC = 0 , PWM = 1
}GPIO_AlternativeFunction;

/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/
/*
 * Description:
 * This function initializes a GPIO pin on the specified port and pin number.
 * It configures the pin as input or output based on the provided direction,
 * and sets up the necessary GPIO settings including pull-up/pull-down resistors.
 * Parameters:
 * - port: The GPIO port identifier (PORTA_ID - PORTF_ID).
 * - pin: The GPIO pin number (0-7).
 * - direction: The desired GPIO pin direction (PIN_INPUT_PULL_UP, PIN_INPUT_PULL_DOWN, PIN_OUTPUT_PULL_UP, PIN_OUTPUT_PULL_DOWN).
 * Returns: None.
 */
void gpio_init(uint8_t port, uint8_t pin, GPIO_PinDirectionType direction);

/*
 * Description:
 * This function configures a GPIO pin on the specified port and pin number
 * for alternative function usage, such as ADC or TIMER. It enables the
 * appropriate alternate function, and configures the pin as analog
 * (for ADC) or input (for TIMER).
 * Parameters:
 * - port: The GPIO port identifier (PORTA_ID - PORTF_ID).
 * - pin: The GPIO pin number (0-7).
 * - alterfunc: The desired alternate function (ADC or TIMER).
 * Returns: None.
 */
void gpio_alternative_function(uint8_t port, uint8_t pin, GPIO_AlternativeFunction alterfunc);

/*
 * Description:
 * This function writes a digital value (0 or 1) to the specified GPIO pin
 * on the given port. It clears or sets the pin based on the 'value' parameter.
 * Parameters:
 * - port: The GPIO port identifier (PORTA_ID - PORTF_ID).
 * - pin: The GPIO pin number (0-7).
 * - value: The value to be written to the pin (0 for clear, 1 for set).
 * Returns: None.
 */
void gpio_write_pin(uint8_t port, uint8_t pin, uint8_t value);

/*
 * Description:
 * This function reads the digital value (0 or 1) from the specified GPIO pin
 * on the given port and returns the value.
 * Parameters:
 * - port: The GPIO port identifier (PORTA_ID - PORTF_ID).
 * - pin: The GPIO pin number (0-7).
 * Returns:
 * - The digital value read from the specified GPIO pin (0 or 1).
 */
uint8_t gpio_read_pin(uint8_t port, uint8_t pin);

/*
 * Description:
 * This function writes a digital value to all pins of the specified GPIO port.
 * Parameters:
 * - port: The GPIO port identifier (PORTA_ID - PORTF_ID).
 * - value: The digital value to write to all pins of the specified port (0 or 1).
 * Returns: None.
 */
void gpio_write_port(uint8_t port_num, uint8_t value);

#endif /* GPIO_H_ */
