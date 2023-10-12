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
 * [DESCRIPTION]: source file for switch driver                                *
 *                                                                             *
 *******************************************************************************/
#include "../../MCAL/GPIO/gpio.h"

#include "switch.h"

/*******************************************************************************
 *                              Functions Definition                           *
 *******************************************************************************/

/*
 * Description:
 * This function initializes the switch pin on the specified port and pin number.
 * It configures the pin as output based on the provided direction,
 * and sets up the necessary GPIO settings including pull-up/pull-down resistors.
 * Parameters:
 * - sensor: pointer to the switch configuration (PORT_ID , PIN_ID , HARDWARE CONNECTION , INITIAL STATE ).
 * Returns: None.
 */

void button_init(button_t *btn) {

    if(((void*)0) == btn)
    {
        return;
    }
    else
    {
        gpio_init(btn->btnPort, btn->btnPin, PIN_INPUT_PULL_UP);
    }
}
/*
 * Description:
 * This function initializes the switch pin on the specified port and pin number.
 * It configures the pin as output based on the provided direction,
 * and sets up the necessary GPIO settings including pull-up/pull-down resistors.
 * Parameters:
 * - sensor: pointer to the switch configuration (PORT_ID , PIN_ID , HARDWARE CONNECTION , INITIAL STATE ).
 * - btn_state: pointer to return the current state of a switch (PRESSED , RELEASED).
 * Returns: None.
 */
void button_get_state(button_t *btn, buttonState_t *btn_state) {

    if(((void*)0) == btn || ((void*)0) == btn_state)
    {
        return;
    }
    else
    {
        uint8_t state ;
        state = gpio_read_pin(btn->btnPort, btn->btnPin);
        switch(btn-> btn_conn)
        {
        case PULL_UP :
            switch(state)
            {
            case LOW :
                *btn_state = PRESSED ;
                break;
            case HIGH :
                *btn_state = RELEASED ;
                break;
            default :
                ;
            }
            break;
            case PULL_DOWN :
                switch(state)
                {
                case LOW :
                    *btn_state = RELEASED ;
                    break;
                case HIGH :
                    *btn_state = PRESSED ;
                    break;
                default :
                    break;
                }
                break;
                default:
                    ;
        }
    }
}



