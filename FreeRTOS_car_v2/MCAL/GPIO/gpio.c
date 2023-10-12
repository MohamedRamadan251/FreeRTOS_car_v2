/*******************************************************************************
 *                                                                             *
 * [FILE NAME]:   gpio.c                                                       *
 *                                                                             *
 * [AUTHOR]:      Ahmed Saeed, Amr Gafar, Kareem Abd-elrasheed,                *
 *                Mohamed Abosreea, Ahmed Maher, and Mohamed Ramadan           *
 *                                                                             *
 * [Version]:     1.0.0                                                        *
 *                                                                             *
 * [DATE]:        09/09/2023                                                   *
 *                                                                             *
 * [DESCRIPTION]: source file for gpio driver                                  *
 *                                                                             *
 *******************************************************************************/

/*******************************************************************************
 *                                 Includes                                    *
 *******************************************************************************/
#include "gpio.h"
#include "tm4c123gh6pm.h"
#include "LIB/bitwise_operation.h"

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
void gpio_init(uint8_t port, uint8_t pin, GPIO_PinDirectionType direction)
{
    switch(port)
    {
    case PORTA_ID:
        /* Enable clock for GPIO Port A */
        SET_BIT(SYSCTL_RCGCGPIO_R, 0);
        /* Wait until the clock is ready */
        while ((SYSCTL_PRGPIO_R & (1 << 0)) == 0);
        /* Unlock Port A if needed */
        GPIO_PORTA_LOCK_R = 0X4C4F434B;
        /* Allow changes to be made to the specified pins */
        SET_BIT(GPIO_PORTA_CR_R, pin);

        if (direction == PIN_INPUT_PULL_UP)
        {
            /* Set the pin as an input */
            CLEAR_BIT(GPIO_PORTA_DIR_R, pin);
            /* Enable the internal pull-up resistor */
            SET_BIT(GPIO_PORTA_PUR_R, pin);
            /* Enable the digital functionality of the pin */
            SET_BIT(GPIO_PORTA_DEN_R, pin);
        }

        else if (direction == PIN_INPUT_PULL_DOWN)
        {
            /* Set the pin as an input */
            CLEAR_BIT(GPIO_PORTA_DIR_R, pin);
            /* Disable the internal pull-up resistor */
            CLEAR_BIT(GPIO_PORTA_PUR_R, pin);
            /* Enable the digital functionality of the pin */
            SET_BIT(GPIO_PORTA_DEN_R, pin);
        }
        else if (direction == PIN_OUTPUT_PULL_UP)
        {
            /* Set the pin as an output */
            SET_BIT(GPIO_PORTA_DIR_R, pin);
            /* Enable the internal pull-up resistor */
            SET_BIT(GPIO_PORTA_PUR_R, pin);
            /* Enable the digital functionality of the pin */
            SET_BIT(GPIO_PORTA_DEN_R, pin);
        }

        else if (direction == PIN_OUTPUT_PULL_DOWN)
        {
            /* Set the pin as an output */
            SET_BIT(GPIO_PORTA_DIR_R, pin);
            /* Disable the internal pull-up resistor */
            CLEAR_BIT(GPIO_PORTA_PUR_R, pin);
            /* Enable the digital functionality of the pin */
            SET_BIT(GPIO_PORTA_DEN_R, pin);
        }

        break;

        case PORTB_ID:
            /* Enable clock for GPIO Port B */
            SET_BIT(SYSCTL_RCGCGPIO_R,1);
            /* Wait until the clock is ready */
            while((SYSCTL_PRGPIO_R &(1<<1))==0);
            /* Unlock Port B if needed */
            GPIO_PORTB_LOCK_R=0X4C4F434B;
            /* Allow changes to be made to the specified pins */
            SET_BIT(GPIO_PORTB_CR_R,pin);

            if(direction == PIN_INPUT_PULL_UP)
            {
                /* Set the pin as an input */
                CLEAR_BIT(GPIO_PORTB_DIR_R,pin);
                /* Enable the internal pull-up resistor */
                SET_BIT(GPIO_PORTB_PUR_R,pin);
                /* Enable the digital functionality of the pin */
                SET_BIT(GPIO_PORTB_DEN_R,pin);
            }

            else if(direction == PIN_INPUT_PULL_DOWN)
            {
                /* Set the pin as an input */
                CLEAR_BIT(GPIO_PORTB_DIR_R,pin);
                /* Disable the internal pull-up resistor */
                CLEAR_BIT(GPIO_PORTB_PUR_R,pin);
                /* Enable the digital functionality of the pin */
                SET_BIT(GPIO_PORTB_DEN_R,pin);
            }

            else if(direction == PIN_OUTPUT_PULL_UP)
            {
                /* Set the pin as an output */
                SET_BIT(GPIO_PORTB_DIR_R,pin);
                /* Enable the internal pull-up resistor */
                SET_BIT(GPIO_PORTB_PUR_R,pin);
                /* Enable the digital functionality of the pin */
                SET_BIT(GPIO_PORTB_DEN_R,pin);
            }

            else if(direction == PIN_OUTPUT_PULL_DOWN)
            {
                /* Set the pin as an output */
                SET_BIT(GPIO_PORTB_DIR_R,pin);
                /* Disable the internal pull-up resistor */
                CLEAR_BIT(GPIO_PORTB_PUR_R,pin);
                /* Enable the digital functionality of the pin */
                SET_BIT(GPIO_PORTB_DEN_R,pin);
            }

            break;

        case PORTC_ID:
            /* Enable clock for GPIO Port C */
            SET_BIT(SYSCTL_RCGCGPIO_R,2);
            /* Wait until the clock is ready */
            while((SYSCTL_PRGPIO_R &(1<<2))==0);
            /* Unlock Port C if needed */
            GPIO_PORTC_LOCK_R=0X4C4F434B;
            /* Allow changes to be made to the specified pins */
            SET_BIT(GPIO_PORTC_CR_R,pin);

            if(direction == PIN_INPUT_PULL_UP)
            {
                /* Set the pin as an input */
                CLEAR_BIT(GPIO_PORTC_DIR_R,pin);
                /* Enable the internal pull-up resistor */
                SET_BIT(GPIO_PORTC_PUR_R,pin);
                /* Enable the digital functionality of the pin */
                SET_BIT(GPIO_PORTC_DEN_R,pin);
            }

            else if(direction == PIN_INPUT_PULL_DOWN)
            {
                /* Set the pin as an input */
                CLEAR_BIT(GPIO_PORTC_DIR_R,pin);
                /* Disable the internal pull-up resistor */
                CLEAR_BIT(GPIO_PORTC_PUR_R,pin);
                /* Enable the digital functionality of the pin */
                SET_BIT(GPIO_PORTC_DEN_R,pin);
            }

            else if(direction == PIN_OUTPUT_PULL_UP)
            {
                /* Set the pin as an output */
                SET_BIT(GPIO_PORTC_DIR_R,pin);
                /* Enable the internal pull-up resistor */
                SET_BIT(GPIO_PORTC_PUR_R,pin);
                /* Enable the digital functionality of the pin */
                SET_BIT(GPIO_PORTC_DEN_R,pin);
            }

            else if(direction == PIN_OUTPUT_PULL_DOWN)
            {
                /* Set the pin as an output */
                SET_BIT(GPIO_PORTC_DIR_R,pin);
                /* Disable the internal pull-up resistor */
                CLEAR_BIT(GPIO_PORTC_PUR_R,pin);
                /* Enable the digital functionality of the pin */
                SET_BIT(GPIO_PORTC_DEN_R,pin);
            }

            break;

        case PORTD_ID:
            /* Enable clock for GPIO Port D */
            SET_BIT(SYSCTL_RCGCGPIO_R,3);
            /* Wait until the clock is ready */
            while((SYSCTL_PRGPIO_R &(1<<3))==0);
            /* Unlock Port D if needed */
            GPIO_PORTD_LOCK_R=0X4C4F434B;
            /* Allow changes to be made to the specified pins */
            SET_BIT(GPIO_PORTD_CR_R,pin);

            if(direction == PIN_INPUT_PULL_UP)
            {
                /* Set the pin as an input */
                CLEAR_BIT(GPIO_PORTD_DIR_R,pin);
                /* Enable the internal pull-up resistor */
                SET_BIT(GPIO_PORTD_PUR_R,pin);
                /* Enable the digital functionality of the pin */
                SET_BIT(GPIO_PORTD_DEN_R,pin);
            }

            else if(direction == PIN_INPUT_PULL_DOWN)
            {
                /* Set the pin as an input */
                CLEAR_BIT(GPIO_PORTD_DIR_R,pin);
                /* Disable the internal pull-up resistor */
                CLEAR_BIT(GPIO_PORTD_PUR_R,pin);
                /* Enable the digital functionality of the pin */
                SET_BIT(GPIO_PORTD_DEN_R,pin);
            }

            else if(direction == PIN_OUTPUT_PULL_UP)
            {
                /* Set the pin as an output */
                SET_BIT(GPIO_PORTD_DIR_R,pin);
                /* Enable the internal pull-up resistor */
                SET_BIT(GPIO_PORTD_PUR_R,pin);
                /* Enable the digital functionality of the pin */
                SET_BIT(GPIO_PORTD_DEN_R,pin);
            }

            else if(direction == PIN_OUTPUT_PULL_DOWN)
            {
                /* Set the pin as an output */
                SET_BIT(GPIO_PORTD_DIR_R,pin);
                /* Disable the internal pull-up resistor */
                CLEAR_BIT(GPIO_PORTD_PUR_R,pin);
                /* Enable the digital functionality of the pin */
                SET_BIT(GPIO_PORTD_DEN_R,pin);
            }

            break;

        case PORTE_ID:
            /* Enable clock for GPIO Port E */
            SET_BIT(SYSCTL_RCGCGPIO_R,4);
            /* Wait until the clock is ready */
            while((SYSCTL_PRGPIO_R &(1<<4))==0);
            /* Unlock Port E if needed */
            GPIO_PORTE_LOCK_R=0X4C4F434B;
            /* Allow changes to be made to the specified pins */
            SET_BIT(GPIO_PORTE_CR_R,pin);

            if(direction == PIN_INPUT_PULL_UP)
            {
                /* Set the pin as an input */
                CLEAR_BIT(GPIO_PORTE_DIR_R,pin);
                /* Enable the internal pull-up resistor */
                SET_BIT(GPIO_PORTE_PUR_R,pin);
                /* Enable the digital functionality of the pin */
                SET_BIT(GPIO_PORTE_DEN_R,pin);
            }

            else if(direction == PIN_INPUT_PULL_DOWN)
            {
                /* Set the pin as an input */
                CLEAR_BIT(GPIO_PORTE_DIR_R,pin);
                /* Disable the internal pull-up resistor */
                CLEAR_BIT(GPIO_PORTE_PUR_R,pin);
                /* Enable the digital functionality of the pin */
                SET_BIT(GPIO_PORTE_DEN_R,pin);
            }

            else if(direction == PIN_OUTPUT_PULL_UP)
            {
                /* Set the pin as an output */
                SET_BIT(GPIO_PORTE_DIR_R,pin);
                /* Enable the internal pull-up resistor */
                SET_BIT(GPIO_PORTE_PUR_R,pin);
                /* Enable the digital functionality of the pin */
                SET_BIT(GPIO_PORTE_DEN_R,pin);
            }

            else if(direction == PIN_OUTPUT_PULL_DOWN)
            {
                /* Set the pin as an output */
                SET_BIT(GPIO_PORTE_DIR_R,pin);
                /* Disable the internal pull-up resistor */
                CLEAR_BIT(GPIO_PORTE_PUR_R,pin);
                /* Enable the digital functionality of the pin */
                SET_BIT(GPIO_PORTE_DEN_R,pin);
            }

            break;

        case PORTF_ID:
            /* Enable clock for GPIO Port F */
            SET_BIT(SYSCTL_RCGCGPIO_R,5);
            /* Wait until the clock is ready */
            while((SYSCTL_PRGPIO_R &(1<<5))==0);
            /* Unlock Port F if needed */
            GPIO_PORTF_LOCK_R=0X4C4F434B;
            /* Allow changes to be made to the specified pins */
            SET_BIT(GPIO_PORTF_CR_R,pin);

            if(direction == PIN_INPUT_PULL_UP)
            {
                /* Set the pin as an input */
                CLEAR_BIT(GPIO_PORTF_DIR_R,pin);
                /* Enable the internal pull-up resistor */
                SET_BIT(GPIO_PORTF_PUR_R,pin);
                /* Enable the digital functionality of the pin */
                SET_BIT(GPIO_PORTF_DEN_R,pin);
            }

            else if(direction == PIN_INPUT_PULL_DOWN)
            {
                /* Set the pin as an input */
                CLEAR_BIT(GPIO_PORTF_DIR_R,pin);
                /* Disable the internal pull-up resistor */
                CLEAR_BIT(GPIO_PORTF_PUR_R,pin);
                /* Enable the digital functionality of the pin */
                SET_BIT(GPIO_PORTF_DEN_R,pin);
            }

            else if(direction == PIN_OUTPUT_PULL_UP)
            {
                /* Set the pin as an output */
                SET_BIT(GPIO_PORTF_DIR_R,pin);
                /* Enable the internal pull-up resistor */
                SET_BIT(GPIO_PORTF_PUR_R,pin);
                /* Enable the digital functionality of the pin */
                SET_BIT(GPIO_PORTF_DEN_R,pin);
            }

            else if(direction == PIN_OUTPUT_PULL_DOWN)
            {
                /* Set the pin as an output */
                SET_BIT(GPIO_PORTF_DIR_R,pin);
                /* Disable the internal pull-up resistor */
                CLEAR_BIT(GPIO_PORTF_PUR_R,pin);
                /* Enable the digital functionality of the pin */
                SET_BIT(GPIO_PORTF_DEN_R,pin);
            }

            break;

        default:
            break;
    }
}

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
void gpio_alternative_function(uint8_t port, uint8_t pin, GPIO_AlternativeFunction alterfunc)
{
    switch(port)
    {
        case PORTA_ID:
            /* Enable clock for GPIO Port A */
            SET_BIT(SYSCTL_RCGCGPIO_R,0);
            /* Wait until the clock is ready */
            while((SYSCTL_PRGPIO_R &(1<<0))==0);
            /* Unlock Port A if needed */
            GPIO_PORTA_LOCK_R=0X4C4F434B;
            /* Allow changes to be made to the specified pins */
            SET_BIT(GPIO_PORTA_CR_R,pin);

            if(alterfunc == ADC)
            {
                /* Set the Alternate Function Select for the pin in Port A */
                SET_BIT(GPIO_PORTA_AFSEL_R,pin);

                /* Clear the Digital Enable for the pin in Port A */
                CLEAR_BIT(GPIO_PORTA_DEN_R,pin);

                /* Set the Analog Mode Select for the pin in Port A */
                SET_BIT(GPIO_PORTA_AMSEL_R,pin);
            }

            else if(alterfunc == PWM)
            {
                /* Set the specific pin in GPIO port A as an output */
                SET_BIT(GPIO_PORTA_DIR_R, pin);

                /* Enable the digital function for the specific pin in GPIO port A */
                SET_BIT(GPIO_PORTA_DEN_R, pin);

                /* Enable the alternate function for the specific pin in GPIO port A */
                SET_BIT(GPIO_PORTA_AFSEL_R, pin);

                /* Configure the pin's alternate function using the PCTL register */
                GPIO_PORTA_PCTL_R = ((4 * pin) << 7);
            }

            break;

        case PORTB_ID:
            /* Enable clock for GPIO Port B */
            SET_BIT(SYSCTL_RCGCGPIO_R,1);
            /* Wait until the clock is ready */
            while((SYSCTL_PRGPIO_R &(1<<1))==0);
            /* Unlock Port B if needed */
            GPIO_PORTB_LOCK_R=0X4C4F434B;
            /* Allow changes to be made to the specified pins */
            SET_BIT(GPIO_PORTB_CR_R,pin);

            if(alterfunc == ADC)
            {
                /* Set the Alternate Function Select for the pin in Port B */
                SET_BIT(GPIO_PORTB_AFSEL_R,pin);

                /* Clear the Digital Enable for the pin in Port B */
                CLEAR_BIT(GPIO_PORTB_DEN_R,pin);

                /* Set the Analog Mode Select for the pin in Port B */
                SET_BIT(GPIO_PORTB_AMSEL_R,pin);
            }

            else if(alterfunc == PWM)
            {
                /* Set the specific pin in GPIO port B as an output */
                SET_BIT(GPIO_PORTB_DIR_R, pin);

                /* Enable the digital function for the specific pin in GPIO port B */
                SET_BIT(GPIO_PORTB_DEN_R, pin);

                /* Enable the alternate function for the specific pin in GPIO port B */
                SET_BIT(GPIO_PORTB_AFSEL_R, pin);

                /* Configure the pin's alternate function using the PCTL register */
                GPIO_PORTB_PCTL_R |= (7 << (4 * pin));
            }

            break;

        case PORTC_ID:
            /* Enable clock for GPIO Port C */
            SET_BIT(SYSCTL_RCGCGPIO_R,2);
            /* Wait until the clock is ready */
            while((SYSCTL_PRGPIO_R &(1<<2))==0);
            /* Unlock Port C if needed */
            GPIO_PORTC_LOCK_R=0X4C4F434B;
            /* Allow changes to be made to the specified pins */
            SET_BIT(GPIO_PORTC_CR_R,pin);

            if(alterfunc == ADC)
            {
                /* Set the Alternate Function Select for the pin in Port C */
                SET_BIT(GPIO_PORTC_AFSEL_R,pin);

                /* Clear the Digital Enable for the pin in Port C */
                CLEAR_BIT(GPIO_PORTC_DEN_R,pin);

                /* Set the Analog Mode Select for the pin in Port C */
                SET_BIT(GPIO_PORTC_AMSEL_R,pin);
            }

            else if(alterfunc == PWM)
            {
                /* Set the specific pin in GPIO port C as an output */
                SET_BIT(GPIO_PORTC_DIR_R, pin);

                /* Enable the digital function for the specific pin in GPIO port C */
                SET_BIT(GPIO_PORTC_DEN_R, pin);

                /* Enable the alternate function for the specific pin in GPIO port C */
                SET_BIT(GPIO_PORTC_AFSEL_R, pin);

                /* Configure the pin's alternate function using the PCTL register */
                GPIO_PORTC_PCTL_R = ((4 * pin) << 7);
            }

            break;

        case PORTD_ID:
            /* Enable clock for GPIO Port D */
            SET_BIT(SYSCTL_RCGCGPIO_R,3);
            /* Wait until the clock is ready */
            while((SYSCTL_PRGPIO_R &(1<<3))==0);
            /* Unlock Port D if needed */
            GPIO_PORTD_LOCK_R=0X4C4F434B;
            /* Allow changes to be made to the specified pins */
            SET_BIT(GPIO_PORTD_CR_R,pin);

            if(alterfunc == ADC)
            {
                /* Set the Alternate Function Select for the pin in Port D */
                SET_BIT(GPIO_PORTD_AFSEL_R,pin);

                /* Clear the Digital Enable for the pin in Port D */
                CLEAR_BIT(GPIO_PORTD_DEN_R,pin);

                /* Set the Analog Mode Select for the pin in Port D */
                SET_BIT(GPIO_PORTD_AMSEL_R,pin);
            }

            else if(alterfunc == PWM)
            {
                /* Set the specific pin in GPIO port D as an output */
                SET_BIT(GPIO_PORTD_DIR_R, pin);

                /* Enable the digital function for the specific pin in GPIO port D */
                SET_BIT(GPIO_PORTD_DEN_R, pin);

                /* Enable the alternate function for the specific pin in GPIO port D */
                SET_BIT(GPIO_PORTD_AFSEL_R, pin);

                /* Configure the pin's alternate function using the PCTL register */
                GPIO_PORTD_PCTL_R = ((4 * pin) << 7);
            }

            break;

        case PORTE_ID:
            /* Enable clock for GPIO Port E */
            SET_BIT(SYSCTL_RCGCGPIO_R,4);
            /* Wait until the clock is ready */
            while((SYSCTL_PRGPIO_R &(1<<4))==0);
            /* Unlock Port E if needed */
            GPIO_PORTE_LOCK_R=0X4C4F434B;
            /* Allow changes to be made to the specified pins */
            SET_BIT(GPIO_PORTE_CR_R,pin);

            if(alterfunc == ADC)
            {
                /* Set the Alternate Function Select for the pin in Port E */
                SET_BIT(GPIO_PORTE_AFSEL_R,pin);

                /* Clear the Digital Enable for the pin in Port E */
                CLEAR_BIT(GPIO_PORTE_DEN_R,pin);

                /* Set the Analog Mode Select for the pin in Port E */
                SET_BIT(GPIO_PORTE_AMSEL_R,pin);
            }

            else if(alterfunc == PWM)
            {
                /* Set the specific pin in GPIO port E as an output */
                SET_BIT(GPIO_PORTE_DIR_R, pin);

                /* Enable the digital function for the specific pin in GPIO port E */
                SET_BIT(GPIO_PORTE_DEN_R, pin);

                /* Enable the alternate function for the specific pin in GPIO port E */
                SET_BIT(GPIO_PORTE_AFSEL_R, pin);

                /* Configure the pin's alternate function using the PCTL register */
                GPIO_PORTE_PCTL_R = ((4 * pin) << 7);
            }

            break;

        case PORTF_ID:
            /* Enable clock for GPIO Port F */
            SET_BIT(SYSCTL_RCGCGPIO_R,5);
            /* Wait until the clock is ready */
            while((SYSCTL_PRGPIO_R &(1<<5))==0);
            /* Unlock Port F if needed */
            GPIO_PORTF_LOCK_R=0X4C4F434B;
            /* Allow changes to be made to the specified pins */
            SET_BIT(GPIO_PORTF_CR_R,pin);

            if (alterfunc == ADC)
            {
                /* Set the Alternate Function Select for the pin in Port F */
                SET_BIT(GPIO_PORTF_AFSEL_R, pin);

                /* Clear the Digital Enable for the pin in Port F */
                CLEAR_BIT(GPIO_PORTF_DEN_R, pin);

                /* Set the Analog Mode Select for the pin in Port F */
                SET_BIT(GPIO_PORTF_AMSEL_R, pin);
            }

            else if(alterfunc == PWM)
            {
                /* Set the specific pin in GPIO port F as an output */
                SET_BIT(GPIO_PORTF_DIR_R, pin);

                /* Enable the digital function for the specific pin in GPIO port F */
                SET_BIT(GPIO_PORTF_DEN_R, pin);

                /* Enable the alternate function for the specific pin in GPIO port F */
                SET_BIT(GPIO_PORTF_AFSEL_R, pin);

                /* Configure the pin's alternate function using the PCTL register */
                GPIO_PORTF_PCTL_R = ((4 * pin) << 7);
            }

            break;

        default:
            break;
    }
}

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
void gpio_write_pin(uint8_t port, uint8_t pin, uint8_t value)
{
    switch(port)
        {
            case PORTA_ID:
                /* Check if the value is 0 */
                if(value == 0)
                {
                    /* clear the specified pin */
                    CLEAR_BIT(GPIO_PORTA_DATA_R,pin);
                }

                /* Check if the value is 1 */
                else if(value == 1)
                {
                    /* set the specified pin */
                    SET_BIT(GPIO_PORTA_DATA_R,pin);
                }

                break;

            case PORTB_ID:
                /* Check if the value is 0 */
                if(value == 0)
                {
                    /* clear the specified pin */
                    CLEAR_BIT(GPIO_PORTB_DATA_R,pin);
                }

                /* Check if the value is 1 */
                else if(value == 1)
                {
                    /* set the specified pin */
                    SET_BIT(GPIO_PORTB_DATA_R,pin);
                }

                break;

            case PORTC_ID:
                /* Check if the value is 0 */
                if(value == 0)
                {
                    /* clear the specified pin */
                    CLEAR_BIT(GPIO_PORTC_DATA_R,pin);
                }

                /* Check if the value is 1 */
                else if(value == 1)
                {
                    /* set the specified pin */
                    SET_BIT(GPIO_PORTC_DATA_R,pin);
                }

                break;

            case PORTD_ID:
                /* Check if the value is 0 */
                if(value == 0)
                {
                    /* clear the specified pin */
                    CLEAR_BIT(GPIO_PORTD_DATA_R,pin);
                }

                /* Check if the value is 1 */
                else if(value == 1)
                {
                    /* set the specified pin */
                    SET_BIT(GPIO_PORTD_DATA_R,pin);
                }

                break;

            case PORTE_ID:
                /* Check if the value is 0 */
                if(value == 0)
                {
                    /* clear the specified pin */
                    CLEAR_BIT(GPIO_PORTE_DATA_R,pin);
                }

                /* Check if the value is 1 */
                else if(value == 1)
                {
                    /* set the specified pin */
                    SET_BIT(GPIO_PORTE_DATA_R,pin);
                }

                break;

            case PORTF_ID:
                /* Check if the value is 0 */
                if(value == 0)
                {
                    /* clear the specified pin */
                    CLEAR_BIT(GPIO_PORTF_DATA_R,pin);
                }

                /* Check if the value is 1 */
                else if(value == 1)
                {
                    /* set the specified pin */
                    SET_BIT(GPIO_PORTF_DATA_R,pin);
                }

                break;
        }
}

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
uint8_t gpio_read_pin(uint8_t port, uint8_t pin)
{
    /* Variable to store the read value */
    uint8_t value;

    switch(port)
    {
        case PORTA_ID:
            /* Read the value of the specified pin in Port A */
            value = GET_BIT(GPIO_PORTA_DATA_R,pin);
            break;

        case PORTB_ID:
            /* Read the value of the specified pin in Port B */
            value = GET_BIT(GPIO_PORTB_DATA_R,pin);
            break;

        case PORTC_ID:
            /* Read the value of the specified pin in Port C */
            value = GET_BIT(GPIO_PORTC_DATA_R,pin);
            break;

        case PORTD_ID:
            /* Read the value of the specified pin in Port D */
            value = GET_BIT(GPIO_PORTD_DATA_R,pin);
            break;

        case PORTE_ID:
            /* Read the value of the specified pin in Port E */
            value = GET_BIT(GPIO_PORTE_DATA_R,pin);
            break;

        case PORTF_ID:
            /* Read the value of the specified pin in Port F */
            value = GET_BIT(GPIO_PORTF_DATA_R,pin);
            break;
    }

    return value;
}

/*
 * Description:
 * This function writes a digital value to all pins of the specified GPIO port.
 * Parameters:
 * - port: The GPIO port identifier (PORTA_ID - PORTF_ID).
 * - value: The digital value to write to all pins of the specified port (0 or 1).
 * Returns: None.
 */
void gpio_write_port(uint8_t port, uint8_t value)
{
    switch(port)
    {
        case PORTA_ID:
            /* Write the 'value' to Port A */
            GPIO_PORTA_DATA_R = value;
            break;

        case PORTB_ID:
            /* Write the 'value' to Port B */
            GPIO_PORTB_DATA_R = value;
            break;

        case PORTC_ID:
            /* Write the 'value' to Port C */
            GPIO_PORTC_DATA_R = value;
            break;

        case PORTD_ID:
            /* Write the 'value' to Port D */
            GPIO_PORTD_DATA_R = value;
            break;

        case PORTE_ID:
            /* Write the 'value' to Port E */
            GPIO_PORTE_DATA_R = value;
            break;

        case PORTF_ID:
            /* Write the 'value' to Port F */
            GPIO_PORTF_DATA_R = value;
            break;
    }
}

