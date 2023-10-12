/*******************************************************************************
 *                                                                             *
 * [FILE NAME]:   timer.c                                                      *
 *                                                                             *
 * [AUTHOR]:      Ahmed Saeed, Amr Gafar, Kareem Abd-elrasheed,                *
 *                Mohamed Abosreea, Ahmed Maher, and Mohamed Ramadan           *
 *                                                                             *
 * [Version]:     1.0.0                                                        *
 *                                                                             *
 * [DATE]:        09/09/2023                                                   *
 *                                                                             *
 * [DESCRIPTION]: source file for timer driver                                 *
 *                                                                             *
 *******************************************************************************/

/*******************************************************************************
 *                                 Includes                                    *
 *******************************************************************************/
#include "tm4c123gh6pm.h"

#include "LIB/bitwise_operation.h"

#include "../../MCAL/GPIO/gpio.h"
#include "../../MCAL/TIMER/timer.h"

uint32_t seconds_count  = 0 ;
/* timer0A handler triggered at user-defined intervals */
void TIMER0A_Handler(void)
{
    if(GET_BIT(TIMER0_MIS_R,INDIVIDUAL_A))
    {

    }

    /* Timer0A timeout flag bit clears*/
    SET_BIT(TIMER0_ICR_R,INDIVIDUAL_A);
}

/* timer0B handler triggered at user-defined intervals */
void TIMER0B_Handler(void)
{
    if(GET_BIT(TIMER0_MIS_R,INDIVIDUAL_B))
    {

    }

    /* Timer0B timeout flag bit clears*/
    SET_BIT(TIMER0_ICR_R,INDIVIDUAL_B);
}

/* timer1A handler triggered at user-defined intervals */
void TIMER1A_Handler(void)
{
    if(GET_BIT(TIMER1_MIS_R,INDIVIDUAL_A))
    {

    }

    /* Timer1A timeout flag bit clears*/
    SET_BIT(TIMER1_ICR_R,INDIVIDUAL_A);
}

/* timer1B handler triggered at user-defined intervals */
void TIMER1B_Handler(void)
{
    if(GET_BIT(TIMER1_MIS_R,INDIVIDUAL_B))
    {

    }

    /* Timer1B timeout flag bit clears*/
    SET_BIT(TIMER1_ICR_R,INDIVIDUAL_B);
}

/* timer1A handler triggered at user-defined intervals */
void TIMER2A_Handler(void)
{
    if(GET_BIT(TIMER2_MIS_R,INDIVIDUAL_A))
    {

    }

    /* Timer2A timeout flag bit clears*/
    SET_BIT(TIMER2_ICR_R,INDIVIDUAL_A);
}

/* timer2B handler triggered at user-defined intervals */
void TIMER2B_Handler(void)
{
    if(GET_BIT(TIMER2_MIS_R,INDIVIDUAL_B))
    {

    }

    /* Timer2B timeout flag bit clears*/
    SET_BIT(TIMER2_ICR_R,INDIVIDUAL_B);
}

/* timer3A handler triggered at user-defined intervals */
void TIMER3A_Handler(void)
{
    if(GET_BIT(TIMER3_MIS_R,INDIVIDUAL_A) )
    {
        seconds_count++;
    }

    /* Timer3A timeout flag bit clears*/
    SET_BIT(TIMER3_ICR_R,INDIVIDUAL_A);
}

/* timer3B handler triggered at user-defined intervals */
void TIMER3B_Handler(void)
{
    if(GET_BIT(TIMER3_MIS_R,INDIVIDUAL_B))
    {

    }

    /* Timer3B timeout flag bit clears*/
    SET_BIT(TIMER3_ICR_R,INDIVIDUAL_B);
}

/* timer4A handler triggered at user-defined intervals */
void TIMER4A_Handler(void)
{
    if(GET_BIT(TIMER4_MIS_R,INDIVIDUAL_A))
    {

    }

    /* Timer4A timeout flag bit clears*/
    SET_BIT(TIMER4_ICR_R,INDIVIDUAL_A);
}

/* timer4B handler triggered at user-defined intervals */
void TIMER4B_Handler(void)
{
    if(GET_BIT(TIMER4_MIS_R,INDIVIDUAL_B))
    {

    }

    /* Timer4B timeout flag bit clears*/
    SET_BIT(TIMER4_ICR_R,INDIVIDUAL_B);
}

/* timer5A handler triggered at user-defined intervals */
void TIMER5A_Handler(void)
{
    if(GET_BIT(TIMER5_MIS_R,INDIVIDUAL_A))
    {

    }

    /* Timer5A timeout flag bit clears*/
    SET_BIT(TIMER5_ICR_R,INDIVIDUAL_A);
}

/* timer5B handler triggered at user-defined intervals */
void TIMER5B_Handler(void)
{
    if(GET_BIT(TIMER5_MIS_R,INDIVIDUAL_B))
    {

    }

    /* Timer5B timeout flag bit clears*/
    SET_BIT(TIMER5_ICR_R,INDIVIDUAL_B);
}

/* Wtimer0A handler triggered at user-defined intervals */
void WTIMER0A_Handler(void)
{
    if(GET_BIT(WTIMER0_MIS_R,INDIVIDUAL_A))
    {

    }

    /* Timer0A timeout flag bit clears*/
    SET_BIT(WTIMER0_ICR_R,INDIVIDUAL_A);
}

/* Wtimer0B handler triggered at user-defined intervals */
void WTIMER0B_Handler(void)
{
    if(GET_BIT(WTIMER0_MIS_R,INDIVIDUAL_B))
    {

    }

    /* WTimer0B timeout flag bit clears*/
    SET_BIT(WTIMER0_ICR_R,INDIVIDUAL_B);
}

/* Wtimer1A handler triggered at user-defined intervals */
void WTIMER1A_Handler(void)
{
    if(GET_BIT(WTIMER1_MIS_R,INDIVIDUAL_A))
    {

    }

    /* WTimer1A timeout flag bit clears*/
    SET_BIT(WTIMER1_ICR_R,INDIVIDUAL_A);
}

/* Wtimer1B handler triggered at user-defined intervals */
void WTIMER1B_Handler(void)
{
    if(GET_BIT(WTIMER1_MIS_R,INDIVIDUAL_B))
    {

    }

    /* WTimer1B timeout flag bit clears*/
    SET_BIT(WTIMER1_ICR_R,INDIVIDUAL_B);
}

/* Wtimer1A handler triggered at user-defined intervals */
void WTIMER2A_Handler(void)
{
    if(GET_BIT(WTIMER2_MIS_R,INDIVIDUAL_A))
    {

    }

    /* WTimer2A timeout flag bit clears*/
    SET_BIT(WTIMER2_ICR_R,INDIVIDUAL_A);
}

/* Wtimer2B handler triggered at user-defined intervals */
void WTIMER2B_Handler(void)
{
    if(GET_BIT(WTIMER2_MIS_R,INDIVIDUAL_B))
    {

    }

    /* WTimer2B timeout flag bit clears*/
    SET_BIT(WTIMER2_ICR_R,INDIVIDUAL_B);
}

/* Wtimer3A handler triggered at user-defined intervals */
void WTIMER3A_Handler(void)
{
    if(GET_BIT(WTIMER3_MIS_R,INDIVIDUAL_A))
    {

    }

    /* WTimer3A timeout flag bit clears*/
    SET_BIT(WTIMER3_ICR_R,INDIVIDUAL_A);
}

/* Wtimer3B handler triggered at user-defined intervals */
void WTIMER3B_Handler(void)
{
    if(GET_BIT(WTIMER3_MIS_R,INDIVIDUAL_B))
    {

    }

    /* WTimer3B timeout flag bit clears*/
    SET_BIT(WTIMER3_ICR_R,INDIVIDUAL_B);
}

/* Wtimer4A handler triggered at user-defined intervals */
void WTIMER4A_Handler(void)
{
    if(GET_BIT(WTIMER4_MIS_R,INDIVIDUAL_A))
    {

    }

    /* WTimer4A timeout flag bit clears*/
    SET_BIT(WTIMER4_ICR_R,INDIVIDUAL_A);
}

/* Wtimer4B handler triggered at user-defined intervals */
void WTIMER4B_Handler(void)
{
    if(GET_BIT(WTIMER4_MIS_R,INDIVIDUAL_B))
    {

    }

    /* WTimer4B timeout flag bit clears*/
    SET_BIT(WTIMER4_ICR_R,INDIVIDUAL_B);
}

/* Wtimer5A handler triggered at user-defined intervals */
void WTIMER5A_Handler(void)
{
    if(GET_BIT(WTIMER5_MIS_R,INDIVIDUAL_A))
    {

    }

    /* WTimer5A timeout flag bit clears*/
    SET_BIT(WTIMER5_ICR_R,INDIVIDUAL_A);
}

/* Wtimer5B handler triggered at user-defined intervals */
void WTIMER5B_Handler(void)
{
    if(GET_BIT(WTIMER5_MIS_R,INDIVIDUAL_B))
    {

    }

    /* WTimer5B timeout flag bit clears*/
    SET_BIT(WTIMER5_ICR_R,INDIVIDUAL_B);
}

/*
 * Description:
 * This function initializes a timer module, configures its settings,
 * and enables the timer for a specified duration in milliseconds.
 * Parameters:
 * - Config_Ptr: A pointer to a structure containing timer configuration settings.
 * - milliSeconds: The duration for which the timer should run in milliseconds.
 * Returns: None.
 */
void timer_init(const GPTM_ConfigType * Config_Ptr,uint32_t milliSeconds)
{
    uint32_t time;
    uint32_t prescaler;

    switch(Config_Ptr->timer_type)
    {
    case _16_OR_32_BIT_TIMER:
        /*enable clock Timer in run mode */
        SET_BIT(SYSCTL_RCGCTIMER_R,Config_Ptr->timer_number);
        switch(Config_Ptr->timer_number)
        {
        case TIMER0:

            /* disable timer0 output */
            CLEAR_BIT(TIMER0_CTL_R,Config_Ptr->timer_use);

            /*assign the global operation configuration option */
            TIMER0_CFG_R = Config_Ptr->timer_global_operation;

            switch(Config_Ptr->timer_use)
            {
            case INDIVIDUAL_A:
                /*assign the mode of timerA0 */
                TIMER0_TAMR_R |= Config_Ptr->timer_mode;

                /* TimerA0 prescaler value */
                TIMER0_TAPR_R = (int)((float)(milliSeconds - 1) / 4);

                /* TimerA0 counter starting count down value  */
                TIMER0_TAILR_R = (((SYSTEM_FREQUENCY - 1) / 1000) * 4);

                break;

            case INDIVIDUAL_B:
                /*assign the mode of timerB0 */
                TIMER0_TBMR_R |= Config_Ptr->timer_mode;

                /* TimerB0 prescaler value */
                TIMER0_TBPR_R = (int)((float)(milliSeconds - 1) / 4);

                /* TimerB0 counter starting count down value  */
                TIMER0_TBILR_R = (((SYSTEM_FREQUENCY - 1) / 1000) * 4);

                break;
            }

            /* Timer0 timeout flag bit clears*/
            SET_BIT(TIMER0_ICR_R,Config_Ptr->timer_use);

            /*enables Timer0 timeout  interrupt mask */
            SET_BIT(TIMER0_IMR_R,Config_Ptr->timer_use);

            /* Enable Timer0 module */
            SET_BIT(TIMER0_CTL_R,Config_Ptr->timer_use);

            switch(Config_Ptr->timer_use)
            {
            case INDIVIDUAL_A:
                /*enable IRQ19 */
                SET_BIT(NVIC_EN0_R,19);
                break;

            case INDIVIDUAL_B:
                /*enable IRQ20 */
                SET_BIT(NVIC_EN0_R,20);
                break;
            }

            break;

            case TIMER1:

                /* disable timer1 output */
                CLEAR_BIT(TIMER1_CTL_R,Config_Ptr->timer_use);

                /*assign the global operation configuration option */
                TIMER1_CFG_R = Config_Ptr->timer_global_operation;

                switch(Config_Ptr->timer_use)
                {
                case INDIVIDUAL_A:
                    /*assign the mode of timerA1 */
                    TIMER1_TAMR_R |= Config_Ptr->timer_mode;

                    /* TimerA1 prescaler value */
                    TIMER1_TAPR_R = (int)((float)(milliSeconds - 1) / 4);

                    /* TimerA1 counter starting count down value  */
                    TIMER1_TAILR_R = (((SYSTEM_FREQUENCY - 1) / 1000) * 4);

                    break;

                case INDIVIDUAL_B:
                    /*assign the mode of timerB1 */
                    TIMER1_TBMR_R |= Config_Ptr->timer_mode;

                    /* TimerB1 prescaler value */
                    TIMER1_TBPR_R = (int)((float)(milliSeconds - 1) / 4);

                    /* TimerB1 counter starting count down value  */
                    TIMER1_TBILR_R = (((SYSTEM_FREQUENCY - 1) / 1000) * 4);

                    break;
                }

                /* Timer1 timeout flag bit clears*/
                SET_BIT(TIMER1_ICR_R,Config_Ptr->timer_use);

                /*enables Timer1 timeout  interrupt mask */
                SET_BIT(TIMER1_IMR_R,Config_Ptr->timer_use);

                /* Enable Timer1 module */
                SET_BIT(TIMER1_CTL_R,Config_Ptr->timer_use);

                switch(Config_Ptr->timer_use)
                {
                case INDIVIDUAL_A:
                    /*enable IRQ21 */
                    SET_BIT(NVIC_EN0_R,21);
                    break;

                case INDIVIDUAL_B:
                    /*enable IRQ22 */
                    SET_BIT(NVIC_EN0_R,22);
                    break;
                }

                break;

                case TIMER2:

                    /* disable timer2 output */
                    CLEAR_BIT(TIMER2_CTL_R,Config_Ptr->timer_use);

                    /*assign the global operation configuration option */
                    TIMER2_CFG_R = Config_Ptr->timer_global_operation;

                    switch(Config_Ptr->timer_use)
                    {
                    case INDIVIDUAL_A:
                        /*assign the mode of timerA2 */
                        TIMER2_TAMR_R |= Config_Ptr->timer_mode;

                        /* TimerA2 prescaler value */
                        TIMER2_TAPR_R = (int)((float)(milliSeconds - 1) / 4);

                        /* TimerA2 counter starting count down value  */
                        TIMER2_TAILR_R = (((SYSTEM_FREQUENCY - 1) / 1000) * 4);

                        break;

                    case INDIVIDUAL_B:
                        /*assign the mode of timerB2 */
                        TIMER2_TBMR_R |= Config_Ptr->timer_mode;

                        /* TimerB2 prescaler value */
                        TIMER2_TBPR_R = (int)((float)(milliSeconds - 1) / 4);

                        /* TimerB2 counter starting count down value  */
                        TIMER2_TBILR_R = (((SYSTEM_FREQUENCY - 1) / 1000) * 4);

                        break;
                    }

                    /* Timer2 timeout flag bit clears*/
                    SET_BIT(TIMER2_ICR_R,Config_Ptr->timer_use);

                    /*enables Timer2 timeout  interrupt mask */
                    SET_BIT(TIMER2_IMR_R,Config_Ptr->timer_use);

                    /* Enable Timer2 module */
                    SET_BIT(TIMER2_CTL_R,Config_Ptr->timer_use);

                    switch(Config_Ptr->timer_use)
                    {
                    case INDIVIDUAL_A:
                        /*enable IRQ23 */
                        SET_BIT(NVIC_EN0_R,23);
                        break;

                    case INDIVIDUAL_B:
                        /*enable IRQ24 */
                        SET_BIT(NVIC_EN0_R,24);
                        break;
                    }

                    break;

                    case TIMER3:
                    {

                        /* disable timer3 output */
                        CLEAR_BIT(TIMER3_CTL_R,Config_Ptr->timer_use);

                        /*assign the global operation configuration option */
                        TIMER3_CFG_R = Config_Ptr->timer_global_operation;

                        switch(Config_Ptr->timer_use)
                        {
                        case INDIVIDUAL_A:
                            /*assign the mode of timerA3 */
                            TIMER3_TAMR_R |= Config_Ptr->timer_mode;

                            /* TimerA3 prescaler value */
                            TIMER3_TAPR_R = (int)((float)(milliSeconds - 1) / 4);

                            /* TimerA3 counter starting count down value  */
                            TIMER3_TAILR_R = (((SYSTEM_FREQUENCY - 1) / 1000) * 4);

                            break;

                        case INDIVIDUAL_B:
                            /*assign the mode of timerB3 */
                            TIMER3_TBMR_R |= Config_Ptr->timer_mode;

                            /* TimerB3 prescaler value */
                            TIMER3_TBPR_R = (int)((float)(milliSeconds - 1) / 4);

                            /* TimerB3 counter starting count down value  */
                            TIMER3_TBILR_R = (((SYSTEM_FREQUENCY - 1) / 1000) * 4);

                            break;
                        }

                        /* Timer3 timeout flag bit clears*/
                        SET_BIT(TIMER3_ICR_R,Config_Ptr->timer_use);

                        /*enables Timer3 time-out  interrupt mask */
                        SET_BIT(TIMER3_IMR_R,Config_Ptr->timer_use);

                        /* Enable Timer3 module */
                        SET_BIT(TIMER3_CTL_R,Config_Ptr->timer_use);

                        switch(Config_Ptr->timer_use)
                        {
                        case INDIVIDUAL_A:
                            /*enable IRQ35 */
                            SET_BIT(NVIC_EN1_R,3);
                            break;

                        case INDIVIDUAL_B:
                            /*enable IRQ36 */
                            SET_BIT(NVIC_EN1_R,4);
                            break;
                        }

                    } break;

                        case TIMER4:

                            /* disable timer4 output */
                            CLEAR_BIT(TIMER4_CTL_R,Config_Ptr->timer_use);

                            /*assign the global operation configuration option */
                            TIMER4_CFG_R = Config_Ptr->timer_global_operation;

                            switch(Config_Ptr->timer_use)
                            {
                            case INDIVIDUAL_A:
                                /*assign the mode of timerA4 */
                                TIMER4_TAMR_R |= Config_Ptr->timer_mode;

                                /* TimerA4 prescaler value */
                                TIMER4_TAPR_R = (int)((float)(milliSeconds - 1) / 4);

                                /* TimerA4 counter starting count down value  */
                                TIMER4_TAILR_R = (((SYSTEM_FREQUENCY - 1) / 1000) * 4);

                                break;

                            case INDIVIDUAL_B:
                                /*assign the mode of timerB4 */
                                TIMER4_TBMR_R |= Config_Ptr->timer_mode;

                                /* TimerB4 prescaler value */
                                TIMER4_TBPR_R = (int)((float)(milliSeconds - 1) / 4);

                                /* TimerB4 counter starting count down value  */
                                TIMER4_TBILR_R = (((SYSTEM_FREQUENCY - 1) / 1000) * 4);

                                break;
                            }

                            /* Timer4 timeout flag bit clears*/
                            SET_BIT(TIMER4_ICR_R,Config_Ptr->timer_use);

                            /*enables Timer4 time-out  interrupt mask */
                            SET_BIT(TIMER4_IMR_R,Config_Ptr->timer_use);

                            /* Enable Timer4 module */
                            SET_BIT(TIMER4_CTL_R,Config_Ptr->timer_use);

                            switch(Config_Ptr->timer_use)
                            {
                            case INDIVIDUAL_A:
                                /*enable IRQ70 */
                                SET_BIT(NVIC_EN2_R,6);
                                break;

                            case INDIVIDUAL_B:
                                /*enable IRQ71 */
                                SET_BIT(NVIC_EN2_R,7);
                                break;
                            }

                            break;

                            case TIMER5:

                                /* disable timer5 output */
                                CLEAR_BIT(TIMER5_CTL_R,Config_Ptr->timer_use);

                                /*assign the global operation configuration option */
                                TIMER5_CFG_R = Config_Ptr->timer_global_operation;

                                switch(Config_Ptr->timer_use)
                                {
                                case INDIVIDUAL_A:
                                    /*assign the mode of timerA5 */
                                    TIMER5_TAMR_R |= Config_Ptr->timer_mode;

                                    /* TimerA5 prescaler value */
                                    TIMER5_TAPR_R = (int)((float)(milliSeconds - 1) / 4);

                                    /* TimerA5 counter starting count down value  */
                                    TIMER5_TAILR_R = (((SYSTEM_FREQUENCY - 1) / 1000) * 4);

                                    break;

                                case INDIVIDUAL_B:
                                    /*assign the mode of timerB5 */
                                    TIMER5_TBMR_R |= Config_Ptr->timer_mode;

                                    /* TimerB5 prescaler value */
                                    TIMER5_TBPR_R = (int)((float)(milliSeconds - 1) / 4);

                                    /* TimerB5 counter starting count down value  */
                                    TIMER5_TBILR_R = (((SYSTEM_FREQUENCY - 1) / 1000) * 4);

                                    break;
                                }

                                /* Timer5 timeout flag bit clears*/
                                SET_BIT(TIMER5_ICR_R,Config_Ptr->timer_use);

                                /*enables Timer5 time-out  interrupt mask */
                                SET_BIT(TIMER5_IMR_R,Config_Ptr->timer_use);

                                /* Enable Timer5 module */
                                SET_BIT(TIMER5_CTL_R,Config_Ptr->timer_use);

                                switch(Config_Ptr->timer_use)
                                {
                                case INDIVIDUAL_A:
                                    /*enable IRQ92 */
                                    SET_BIT(NVIC_EN2_R,28);
                                    break;

                                case INDIVIDUAL_B:
                                    /*enable IRQ93 */
                                    SET_BIT(NVIC_EN2_R,29);
                                    break;
                                }

                                break;
        }
        break;

        case _32_OR_64_BIT_WIDE_TIMER:
            /*enable clock Timer in run mode */
            SET_BIT(SYSCTL_RCGCWTIMER_R,Config_Ptr->timer_number);
            switch(Config_Ptr->timer_number)
            {
            case TIMER0:

                /* disable timer0 output */
                CLEAR_BIT(WTIMER0_CTL_R,Config_Ptr->timer_use);

                /*assign the global operation configuration option */
                WTIMER0_CFG_R = Config_Ptr->timer_global_operation;

                switch(Config_Ptr->timer_use)
                {
                case INDIVIDUAL_A:
                    /*assign the mode of timerA0 */
                    WTIMER0_TAMR_R |= Config_Ptr->timer_mode;

                    time = (((SYSTEM_FREQUENCY - 1) / 1000) * milliSeconds);
                    prescaler = time / 4294967296;

                    /* TimerA0 prescaler value */
                    WTIMER0_TAPR_R = prescaler;

                    /* TimerA0 counter starting count down value  */
                    WTIMER0_TAILR_R = time - prescaler * 4294967296;

                    break;

                case INDIVIDUAL_B:
                    /*assign the mode of timerB0 */
                    WTIMER0_TBMR_R |= Config_Ptr->timer_mode;

                    time = (((SYSTEM_FREQUENCY - 1) / 1000) * milliSeconds);
                    prescaler = time / 4294967296;

                    /* TimerB0 prescaler value */
                    WTIMER0_TBPR_R = prescaler;

                    /* TimerB0 counter starting count down value  */
                    WTIMER0_TBILR_R = time - prescaler * 4294967296;

                    break;
                }

                /* Timer0 timeout flag bit clears*/
                SET_BIT(WTIMER0_ICR_R,Config_Ptr->timer_use);

                /*enables Timer0 timeout  interrupt mask */
                SET_BIT(WTIMER0_IMR_R,Config_Ptr->timer_use);

                /* Enable Timer0 module */
                SET_BIT(WTIMER0_CTL_R,Config_Ptr->timer_use);

                switch(Config_Ptr->timer_use)
                {
                case INDIVIDUAL_A:
                    /*enable IRQ94 */
                    SET_BIT(NVIC_EN2_R,30);
                    break;

                case INDIVIDUAL_B:
                    /*enable IRQ95 */
                    SET_BIT(NVIC_EN2_R,31);
                    break;
                }

                break;

                case TIMER1:

                    /* disable timer1 output */
                    CLEAR_BIT(WTIMER1_CTL_R,Config_Ptr->timer_use);

                    /*assign the global operation configuration option */
                    WTIMER1_CFG_R = Config_Ptr->timer_global_operation;

                    switch(Config_Ptr->timer_use)
                    {
                    case INDIVIDUAL_A:
                        /*assign the mode of timerA1 */
                        WTIMER1_TAMR_R |= Config_Ptr->timer_mode;

                        time = (((SYSTEM_FREQUENCY - 1) / 1000) * milliSeconds);
                        prescaler = time / 4294967296;

                        /* TimerA1 prescaler value */
                        WTIMER1_TAPR_R = prescaler;

                        /* TimerA1 counter starting count down value  */
                        WTIMER1_TAILR_R = time - prescaler * 4294967296;

                        break;

                    case INDIVIDUAL_B:
                        /*assign the mode of timerB1 */
                        WTIMER1_TBMR_R |= Config_Ptr->timer_mode;

                        time = (((SYSTEM_FREQUENCY - 1) / 1000) * milliSeconds);
                        prescaler = time / 4294967296;

                        /* TimerB1 prescaler value */
                        WTIMER1_TBPR_R = prescaler;

                        /* TimerB1 counter starting count down value  */
                        WTIMER1_TBILR_R = time - prescaler * 4294967296;

                        break;
                    }

                    /* Timer1 timeout flag bit clears*/
                    SET_BIT(WTIMER1_ICR_R,Config_Ptr->timer_use);

                    /*enables Timer1 timeout  interrupt mask */
                    SET_BIT(WTIMER1_IMR_R,Config_Ptr->timer_use);

                    /* Enable Timer1 module */
                    SET_BIT(WTIMER1_CTL_R,Config_Ptr->timer_use);

                    switch(Config_Ptr->timer_use)
                    {
                    case INDIVIDUAL_A:
                        /*enable IRQ96 */
                        SET_BIT(NVIC_EN3_R,0);
                        break;

                    case INDIVIDUAL_B:
                        /*enable IRQ97 */
                        SET_BIT(NVIC_EN3_R,1);
                        break;
                    }

                    break;

                    case TIMER2:

                        /* disable timer2 output */
                        CLEAR_BIT(WTIMER2_CTL_R,Config_Ptr->timer_use);

                        /*assign the global operation configuration option */
                        WTIMER2_CFG_R = Config_Ptr->timer_global_operation;

                        switch(Config_Ptr->timer_use)
                        {
                        case INDIVIDUAL_A:
                            /*assign the mode of timerA2 */
                            WTIMER2_TAMR_R |= Config_Ptr->timer_mode;

                            time = (((SYSTEM_FREQUENCY - 1) / 1000) * milliSeconds);
                            prescaler = time / 4294967296;

                            /* TimerA2 prescaler value */
                            WTIMER2_TAPR_R = prescaler;

                            /* TimerA2 counter starting count down value  */
                            WTIMER2_TAILR_R = time - prescaler * 4294967296;

                            break;

                        case INDIVIDUAL_B:
                            /*assign the mode of timerB2 */
                            WTIMER2_TBMR_R |= Config_Ptr->timer_mode;

                            time = (((SYSTEM_FREQUENCY - 1) / 1000) * milliSeconds);
                            prescaler = time / 4294967296;

                            /* TimerB2 prescaler value */
                            WTIMER2_TBPR_R = prescaler;

                            /* TimerB2 counter starting count down value  */
                            WTIMER2_TBILR_R = time - prescaler * 4294967296;

                            break;
                        }

                        /* Timer2 timeout flag bit clears*/
                        SET_BIT(WTIMER2_ICR_R,Config_Ptr->timer_use);

                        /*enables Timer2 timeout  interrupt mask */
                        SET_BIT(WTIMER2_IMR_R,Config_Ptr->timer_use);

                        /* Enable Timer2 module */
                        SET_BIT(WTIMER2_CTL_R,Config_Ptr->timer_use);

                        switch(Config_Ptr->timer_use)
                        {
                        case INDIVIDUAL_A:
                            /*enable IRQ98 */
                            SET_BIT(NVIC_EN3_R,2);
                            break;

                        case INDIVIDUAL_B:
                            /*enable IRQ99 */
                            SET_BIT(NVIC_EN3_R,3);
                            break;
                        }

                        break;

                        case TIMER3:

                            /* disable timer3 output */
                            CLEAR_BIT(WTIMER3_CTL_R,Config_Ptr->timer_use);

                            /*assign the global operation configuration option */
                            WTIMER3_CFG_R = Config_Ptr->timer_global_operation;

                            switch(Config_Ptr->timer_use)
                            {
                            case INDIVIDUAL_A:
                                /*assign the mode of timerA3 */
                                WTIMER3_TAMR_R |= Config_Ptr->timer_mode;

                                time = (((SYSTEM_FREQUENCY - 1) / 1000) * milliSeconds);
                                prescaler = time / 4294967296;

                                /* TimerA3 prescaler value */
                                WTIMER3_TAPR_R = prescaler;

                                /* TimerA3 counter starting count down value  */
                                WTIMER3_TAILR_R = time - prescaler * 4294967296;

                                break;

                            case INDIVIDUAL_B:
                                /*assign the mode of timerB3 */
                                WTIMER3_TBMR_R |= Config_Ptr->timer_mode;

                                time = (((SYSTEM_FREQUENCY - 1) / 1000) * milliSeconds);
                                prescaler = time / 4294967296;

                                /* TimerB3 prescaler value */
                                WTIMER3_TBPR_R = prescaler;

                                /* TimerB3 counter starting count down value  */
                                WTIMER3_TBILR_R = time - prescaler * 4294967296;

                                break;
                            }

                            /* Timer3 timeout flag bit clears*/
                            SET_BIT(WTIMER3_ICR_R,Config_Ptr->timer_use);

                            /*enables Timer3 time-out  interrupt mask */
                            SET_BIT(WTIMER3_IMR_R,Config_Ptr->timer_use);

                            /* Enable Timer3 module */
                            SET_BIT(WTIMER3_CTL_R,Config_Ptr->timer_use);

                            switch(Config_Ptr->timer_use)
                            {
                            case INDIVIDUAL_A:
                                /*enable IRQ100 */
                                SET_BIT(NVIC_EN3_R,4);
                                break;

                            case INDIVIDUAL_B:
                                /*enable IRQ101 */
                                SET_BIT(NVIC_EN3_R,5);
                                break;
                            }

                            break;

                            case TIMER4:

                                /* disable timer4 output */
                                CLEAR_BIT(WTIMER4_CTL_R,Config_Ptr->timer_use);

                                /*assign the global operation configuration option */
                                WTIMER4_CFG_R = Config_Ptr->timer_global_operation;

                                switch(Config_Ptr->timer_use)
                                {
                                case INDIVIDUAL_A:
                                    /*assign the mode of timerA4 */
                                    WTIMER4_TAMR_R |= Config_Ptr->timer_mode;

                                    time = (((SYSTEM_FREQUENCY - 1) / 1000) * milliSeconds);
                                    prescaler = time / 4294967296;

                                    /* TimerA4 prescaler value */
                                    WTIMER4_TAPR_R = prescaler;

                                    /* TimerA4 counter starting count down value  */
                                    WTIMER4_TAILR_R = time - prescaler * 4294967296;

                                    break;

                                case INDIVIDUAL_B:
                                    /*assign the mode of timerB4 */
                                    WTIMER4_TBMR_R |= Config_Ptr->timer_mode;

                                    time = (((SYSTEM_FREQUENCY - 1) / 1000) * milliSeconds);
                                    prescaler = time / 4294967296;

                                    /* TimerB4 prescaler value */
                                    WTIMER4_TBPR_R = prescaler;

                                    /* TimerB4 counter starting count down value  */
                                    WTIMER4_TBILR_R = time - prescaler * 4294967296;

                                    break;
                                }

                                /* Timer4 timeout flag bit clears*/
                                SET_BIT(WTIMER4_ICR_R,Config_Ptr->timer_use);

                                /*enables Timer4 time-out  interrupt mask */
                                SET_BIT(WTIMER4_IMR_R,Config_Ptr->timer_use);

                                /* Enable Timer4 module */
                                SET_BIT(WTIMER4_CTL_R,Config_Ptr->timer_use);

                                switch(Config_Ptr->timer_use)
                                {
                                case INDIVIDUAL_A:
                                    /*enable IRQ102 */
                                    SET_BIT(NVIC_EN3_R,6);
                                    break;

                                case INDIVIDUAL_B:
                                    /*enable IRQ103 */
                                    SET_BIT(NVIC_EN3_R,7);
                                    break;
                                }

                                break;

                                case TIMER5:

                                    /* disable timer5 output */
                                    CLEAR_BIT(WTIMER5_CTL_R,Config_Ptr->timer_use);

                                    /*assign the global operation configuration option */
                                    WTIMER5_CFG_R = Config_Ptr->timer_global_operation;

                                    switch(Config_Ptr->timer_use)
                                    {
                                    case INDIVIDUAL_A:
                                        /*assign the mode of timerA5 */
                                        WTIMER5_TAMR_R |= Config_Ptr->timer_mode;

                                        time = (((SYSTEM_FREQUENCY - 1) / 1000) * milliSeconds);
                                        prescaler = time / 4294967296;

                                        /* TimerA5 prescaler value */
                                        WTIMER5_TAPR_R = prescaler;

                                        /* TimerA5 counter starting count down value  */
                                        WTIMER5_TAILR_R = time - prescaler * 4294967296;

                                        break;

                                    case INDIVIDUAL_B:
                                        /*assign the mode of timerB5 */
                                        WTIMER5_TBMR_R |= Config_Ptr->timer_mode;

                                        time = (((SYSTEM_FREQUENCY - 1) / 1000) * milliSeconds);
                                        prescaler = time / 4294967296;

                                        /* TimerB5 prescaler value */
                                        WTIMER5_TBPR_R = prescaler;

                                        /* TimerB5 counter starting count down value  */
                                        WTIMER5_TBILR_R = time - prescaler * 4294967296;

                                        break;
                                    }

                                    /* Timer5 timeout flag bit clears*/
                                    SET_BIT(WTIMER5_ICR_R,Config_Ptr->timer_use);

                                    /*enables Timer5 time-out  interrupt mask */
                                    SET_BIT(WTIMER5_IMR_R,Config_Ptr->timer_use);

                                    /* Enable Timer5 module */
                                    SET_BIT(WTIMER5_CTL_R,Config_Ptr->timer_use);

                                    switch(Config_Ptr->timer_use)
                                    {
                                    case INDIVIDUAL_A:
                                        /*enable IRQ104 */
                                        SET_BIT(NVIC_EN3_R,8);
                                        break;

                                    case INDIVIDUAL_B:
                                        /*enable IRQ105 */
                                        SET_BIT(NVIC_EN3_R,9);
                                        break;
                                    }

                                    break;
            }
            break;
    }
}

/*
 * Description:
 * This function initializes the PWM module based on the provided configuration.
 * It enables the selected timer and configures its mode, output levels, and other settings.
 * Additionally, it sets the alternative function for the corresponding GPIO pins.
 * Parameters:
 * - Config_Ptr: A pointer to the PWM_ConfigType structure containing the configuration settings.
 * Returns: None.
 */
void pwm_init(const GPTM_ConfigType * Config_Ptr)
{
    switch(Config_Ptr->timer_type)
    {
    case _16_OR_32_BIT_TIMER:
        /*enable clock Timer in run mode */
        SET_BIT(SYSCTL_RCGCTIMER_R,Config_Ptr->timer_number);
        switch(Config_Ptr->timer_number)
        {
        case TIMER0:

            switch(Config_Ptr->timer_use)
            {
            case INDIVIDUAL_A:
                /* Setting alternative function of PORTB pin 6 to PWM */
                gpio_alternative_function(PORTB_ID, PIN6_ID, PWM);

                break;

            case INDIVIDUAL_B:
                /* Setting alternative function of PORTB pin 7 to PWM */
                gpio_alternative_function(PORTB_ID, PIN7_ID, PWM);

                break;
            }

            /* disable timer0 output */
            CLEAR_BIT(TIMER0_CTL_R,Config_Ptr->timer_use);

            /* the timer continues counting while the processor is halted by the debugger */
            CLEAR_BIT(TIMER0_CTL_R,(Config_Ptr->timer_use) + 1);

            /*assign the global operation configuration option */
            TIMER0_CFG_R = Config_Ptr->timer_global_operation;

            switch(Config_Ptr->timer_use)
            {
            case INDIVIDUAL_A:
                /* Set the Alternate Mode for Timer A to PWM mode*/
                SET_BIT(TIMER0_TAMR_R , TAAMS);

                /* Choose the edge-count Capture Mode for Timer A */
                CLEAR_BIT(TIMER0_TAMR_R , TACMR);

                /* Set Timer A's mode */
                TIMER0_TAMR_R |= Config_Ptr->timer_mode;

                /* Set the Timer A PWM output level to be inverted */
                SET_BIT(TIMER0_CTL_R , TAPWML);

                break;

            case INDIVIDUAL_B:
                /* Set the Alternate Mode for Timer B to PWM mode*/
                SET_BIT(TIMER0_TBMR_R , TBAMS);

                /* Choose the edge-count Capture Mode for Timer B */
                CLEAR_BIT(TIMER0_TBMR_R , TBCMR);

                /* Set Timer B's mode */
                TIMER0_TBMR_R |= Config_Ptr->timer_mode;

                /* Set the Timer B PWM output level to be inverted */
                SET_BIT(TIMER0_CTL_R , TBPWML);

                break;
            }

            break;

            case TIMER1:

                switch(Config_Ptr->timer_use)
                {
                case INDIVIDUAL_A:
                    /* Setting alternative function of PORTB pin 4 to PWM */
                    gpio_alternative_function(PORTB_ID, PIN4_ID, PWM);

                    break;

                case INDIVIDUAL_B:
                    /* Setting alternative function of PORTB pin 5 to PWM */
                    gpio_alternative_function(PORTB_ID, PIN5_ID, PWM);

                    break;
                }

                /* disable timer1 output */
                CLEAR_BIT(TIMER1_CTL_R,Config_Ptr->timer_use);

                /* the timer continues counting while the processor is halted by the debugger */
                CLEAR_BIT(TIMER1_CTL_R,(Config_Ptr->timer_use) + 1);

                /*assign the global operation configuration option */
                TIMER1_CFG_R = Config_Ptr->timer_global_operation;

                switch(Config_Ptr->timer_use)
                {
                case INDIVIDUAL_A:
                    /* Set the Alternate Mode for Timer A to PWM mode*/
                    SET_BIT(TIMER1_TAMR_R , TAAMS);

                    /* Choose the edge-count Capture Mode for Timer A */
                    CLEAR_BIT(TIMER1_TAMR_R , TACMR);

                    /* Set Timer A's mode */
                    TIMER1_TAMR_R |= Config_Ptr->timer_mode;

                    /* Set the Timer A PWM output level to be inverted */
                    SET_BIT(TIMER1_CTL_R , TAPWML);

                    break;

                case INDIVIDUAL_B:
                    /* Set the Alternate Mode for Timer B to PWM mode*/
                    SET_BIT(TIMER1_TBMR_R , TBAMS);

                    /* Choose the edge-count Capture Mode for Timer B */
                    CLEAR_BIT(TIMER1_TBMR_R , TBCMR);

                    /* Set Timer B's mode */
                    TIMER1_TBMR_R |= Config_Ptr->timer_mode;

                    /* Set the Timer B PWM output level to be inverted */
                    SET_BIT(TIMER1_CTL_R , TBPWML);

                    break;
                }

                break;

                case TIMER2:

                    switch(Config_Ptr->timer_use)
                    {
                    case INDIVIDUAL_A:
                        /* Setting alternative function of PORTB pin 0s to PWM */
                        gpio_alternative_function(PORTB_ID, PIN0_ID, PWM);

                        break;

                    case INDIVIDUAL_B:
                        /* Setting alternative function of PORTB pin 1 to PWM */
                        gpio_alternative_function(PORTB_ID, PIN1_ID, PWM);

                        break;
                    }

                    /* disable timer2 output */
                    CLEAR_BIT(TIMER2_CTL_R,Config_Ptr->timer_use);

                    /* the timer continues counting while the processor is halted by the debugger */
                    CLEAR_BIT(TIMER2_CTL_R,(Config_Ptr->timer_use) + 1);

                    /*assign the global operation configuration option */
                    TIMER2_CFG_R = Config_Ptr->timer_global_operation;

                    switch(Config_Ptr->timer_use)
                    {
                    case INDIVIDUAL_A:
                        /* Set the Alternate Mode for Timer A to PWM mode*/
                        SET_BIT(TIMER2_TAMR_R , TAAMS);

                        /* Choose the edge-count Capture Mode for Timer A */
                        CLEAR_BIT(TIMER2_TAMR_R , TACMR);

                        /* Set Timer A's mode */
                        TIMER2_TAMR_R |= Config_Ptr->timer_mode;

                        /* Set the Timer A PWM output level to be inverted */
                        SET_BIT(TIMER2_CTL_R , TAPWML);

                        break;

                    case INDIVIDUAL_B:
                        /* Set the Alternate Mode for Timer B to PWM mode*/
                        SET_BIT(TIMER2_TBMR_R , TBAMS);

                        /* Choose the edge-count Capture Mode for Timer B */
                        CLEAR_BIT(TIMER2_TBMR_R , TBCMR);

                        /* Set Timer B's mode */
                        TIMER2_TBMR_R |= Config_Ptr->timer_mode;

                        /* Set the Timer B PWM output level to be inverted */
                        SET_BIT(TIMER2_CTL_R , TBPWML);

                        break;
                    }

                    break;

                    case TIMER3:

                        switch(Config_Ptr->timer_use)
                        {
                        case INDIVIDUAL_A:
                            /* Setting alternative function of PORTB pin 2 to PWM */
                            gpio_alternative_function(PORTB_ID, PIN2_ID, PWM);

                            break;

                        case INDIVIDUAL_B:
                            /* Setting alternative function of PORTB pin 3 to PWM */
                            gpio_alternative_function(PORTB_ID, PIN3_ID, PWM);

                            break;
                        }

                        /* disable timer3 output */
                        CLEAR_BIT(TIMER3_CTL_R,Config_Ptr->timer_use);

                        /* the timer continues counting while the processor is halted by the debugger */
                        CLEAR_BIT(TIMER3_CTL_R,(Config_Ptr->timer_use) + 1);

                        /*assign the global operation configuration option */
                        TIMER3_CFG_R = Config_Ptr->timer_global_operation;

                        switch(Config_Ptr->timer_use)
                        {
                        case INDIVIDUAL_A:
                            /* Set the Alternate Mode for Timer A to PWM mode*/
                            SET_BIT(TIMER3_TAMR_R , TAAMS);

                            /* Choose the edge-count Capture Mode for Timer A */
                            CLEAR_BIT(TIMER3_TAMR_R , TACMR);

                            /* Set Timer A's mode */
                            TIMER3_TAMR_R |= Config_Ptr->timer_mode;

                            /* Set the Timer A PWM output level to be inverted */
                            SET_BIT(TIMER3_CTL_R , TAPWML);

                            break;

                        case INDIVIDUAL_B:
                            /* Set the Alternate Mode for Timer B to PWM mode*/
                            SET_BIT(TIMER3_TBMR_R , TBAMS);

                            /* Choose the edge-count Capture Mode for Timer B */
                            CLEAR_BIT(TIMER3_TBMR_R , TBCMR);

                            /* Set Timer B's mode */
                            TIMER3_TBMR_R |= Config_Ptr->timer_mode;

                            /* Set the Timer B PWM output level to be inverted */
                            SET_BIT(TIMER3_CTL_R , TBPWML);

                            break;
                        }

                        break;

                        case TIMER4:

                            switch(Config_Ptr->timer_use)
                            {
                            case INDIVIDUAL_A:
                                /* Setting alternative function of PORTC pin 0 to PWM */
                                gpio_alternative_function(PORTC_ID, PIN0_ID, PWM);

                                break;

                            case INDIVIDUAL_B:
                                /* Setting alternative function of PORTC pin 1 to PWM */
                                gpio_alternative_function(PORTC_ID, PIN1_ID, PWM);

                                break;
                            }

                            /* disable timer4 output */
                            CLEAR_BIT(TIMER4_CTL_R,Config_Ptr->timer_use);

                            /* the timer continues counting while the processor is halted by the debugger */
                            CLEAR_BIT(TIMER4_CTL_R,(Config_Ptr->timer_use) + 1);

                            /*assign the global operation configuration option */
                            TIMER4_CFG_R = Config_Ptr->timer_global_operation;

                            switch(Config_Ptr->timer_use)
                            {
                            case INDIVIDUAL_A:
                                /* Set the Alternate Mode for Timer A to PWM mode*/
                                SET_BIT(TIMER4_TAMR_R , TAAMS);

                                /* Choose the edge-count Capture Mode for Timer A */
                                CLEAR_BIT(TIMER4_TAMR_R , TACMR);

                                /* Set Timer A's mode */
                                TIMER4_TAMR_R |= Config_Ptr->timer_mode;

                                /* Set the Timer A PWM output level to be inverted */
                                SET_BIT(TIMER4_CTL_R , TAPWML);

                                break;

                            case INDIVIDUAL_B:
                                /* Set the Alternate Mode for Timer B to PWM mode*/
                                SET_BIT(TIMER4_TBMR_R , TBAMS);

                                /* Choose the edge-count Capture Mode for Timer B */
                                CLEAR_BIT(TIMER4_TBMR_R , TBCMR);

                                /* Set Timer B's mode */
                                TIMER4_TBMR_R |= Config_Ptr->timer_mode;

                                /* Set the Timer B PWM output level to be inverted */
                                SET_BIT(TIMER4_CTL_R , TBPWML);

                                break;
                            }

                            break;

                            case TIMER5:

                                switch(Config_Ptr->timer_use)
                                {
                                case INDIVIDUAL_A:
                                    /* Setting alternative function of PORTC pin 2 to PWM */
                                    gpio_alternative_function(PORTC_ID, PIN2_ID, PWM);

                                    break;

                                case INDIVIDUAL_B:
                                    /* Setting alternative function of PORTC pin 3 to PWM */
                                    gpio_alternative_function(PORTC_ID, PIN3_ID, PWM);

                                    break;
                                }

                                /* disable timer5 output */
                                CLEAR_BIT(TIMER5_CTL_R,Config_Ptr->timer_use);

                                /* the timer continues counting while the processor is halted by the debugger */
                                CLEAR_BIT(TIMER5_CTL_R,(Config_Ptr->timer_use) + 1);

                                /*assign the global operation configuration option */
                                TIMER5_CFG_R = Config_Ptr->timer_global_operation;

                                switch(Config_Ptr->timer_use)
                                {
                                case INDIVIDUAL_A:
                                    /* Set the Alternate Mode for Timer A to PWM mode*/
                                    SET_BIT(TIMER5_TAMR_R , TAAMS);

                                    /* Choose the edge-count Capture Mode for Timer A */
                                    CLEAR_BIT(TIMER5_TAMR_R , TACMR);

                                    /* Set Timer A's mode */
                                    TIMER5_TAMR_R |= Config_Ptr->timer_mode;

                                    /* Set the Timer A PWM output level to be inverted */
                                    SET_BIT(TIMER5_CTL_R , TAPWML);

                                    break;

                                case INDIVIDUAL_B:
                                    /* Set the Alternate Mode for Timer B to PWM mode*/
                                    SET_BIT(TIMER5_TBMR_R , TBAMS);

                                    /* Choose the edge-count Capture Mode for Timer B */
                                    CLEAR_BIT(TIMER5_TBMR_R , TBCMR);

                                    /* Set Timer B's mode */
                                    TIMER5_TBMR_R |= Config_Ptr->timer_mode;

                                    /* Set the Timer B PWM output level to be inverted */
                                    SET_BIT(TIMER5_CTL_R , TBPWML);

                                    break;
                                }

                                break;
        }
        break;

        case _32_OR_64_BIT_WIDE_TIMER:
            /*enable clock Timer in run mode */
            SET_BIT(SYSCTL_RCGCWTIMER_R,Config_Ptr->timer_number);
            switch(Config_Ptr->timer_number)
            {
            case TIMER0:

                switch(Config_Ptr->timer_use)
                {
                case INDIVIDUAL_A:
                    /* Setting alternative function of PORTC pin 4 to PWM */
                    gpio_alternative_function(PORTC_ID, PIN4_ID, PWM);

                    break;

                case INDIVIDUAL_B:
                    /* Setting alternative function of PORTC pin 5 to PWM */
                    gpio_alternative_function(PORTC_ID, PIN5_ID, PWM);

                    break;
                }

                /* disable timer0 output */
                CLEAR_BIT(WTIMER0_CTL_R,Config_Ptr->timer_use);

                /* the timer continues counting while the processor is halted by the debugger */
                CLEAR_BIT(WTIMER0_CTL_R,(Config_Ptr->timer_use) + 1);

                /*assign the global operation configuration option */
                WTIMER0_CFG_R = Config_Ptr->timer_global_operation;

                switch(Config_Ptr->timer_use)
                {
                case INDIVIDUAL_A:
                    /* Set the Alternate Mode for Timer A to PWM mode*/
                    SET_BIT(WTIMER0_TAMR_R , TAAMS);

                    /* Choose the edge-count Capture Mode for Timer A */
                    CLEAR_BIT(WTIMER0_TAMR_R , TACMR);

                    /* Set Timer A's mode */
                    WTIMER0_TAMR_R |= Config_Ptr->timer_mode;

                    /* Set the Timer A PWM output level to be inverted */
                    SET_BIT(WTIMER0_CTL_R , TAPWML);

                    break;

                case INDIVIDUAL_B:
                    /* Set the Alternate Mode for Timer B to PWM mode*/
                    SET_BIT(WTIMER0_TBMR_R , TBAMS);

                    /* Choose the edge-count Capture Mode for Timer B */
                    CLEAR_BIT(WTIMER0_TBMR_R , TBCMR);

                    /* Set Timer B's mode */
                    WTIMER0_TBMR_R |= Config_Ptr->timer_mode;

                    /* Set the Timer B PWM output level to be inverted */
                    SET_BIT(WTIMER0_CTL_R , TBPWML);

                    break;
                }

                break;

                case TIMER1:

                    switch(Config_Ptr->timer_use)
                    {
                    case INDIVIDUAL_A:
                        /* Setting alternative function of PORTC pin 6 to PWM */
                        gpio_alternative_function(PORTC_ID, PIN6_ID, PWM);

                        break;

                    case INDIVIDUAL_B:
                        /* Setting alternative function of PORTC pin 7 to PWM */
                        gpio_alternative_function(PORTC_ID, PIN7_ID, PWM);

                        break;
                    }

                    /* disable timer1 output */
                    CLEAR_BIT(WTIMER1_CTL_R,Config_Ptr->timer_use);

                    /* the timer continues counting while the processor is halted by the debugger */
                    CLEAR_BIT(WTIMER1_CTL_R,(Config_Ptr->timer_use) + 1);

                    /*assign the global operation configuration option */
                    WTIMER1_CFG_R = Config_Ptr->timer_global_operation;

                    switch(Config_Ptr->timer_use)
                    {
                    case INDIVIDUAL_A:
                        /* Set the Alternate Mode for Timer A to PWM mode*/
                        SET_BIT(WTIMER1_TAMR_R , TAAMS);

                        /* Choose the edge-count Capture Mode for Timer A */
                        CLEAR_BIT(WTIMER1_TAMR_R , TACMR);

                        /* Set Timer A's mode */
                        WTIMER1_TAMR_R |= Config_Ptr->timer_mode;

                        /* Set the Timer A PWM output level to be inverted */
                        SET_BIT(WTIMER1_CTL_R , TAPWML);

                        break;

                    case INDIVIDUAL_B:
                        /* Set the Alternate Mode for Timer B to PWM mode*/
                        SET_BIT(WTIMER1_TBMR_R , TBAMS);

                        /* Choose the edge-count Capture Mode for Timer B */
                        CLEAR_BIT(WTIMER1_TBMR_R , TBCMR);

                        /* Set Timer B's mode */
                        WTIMER1_TBMR_R |= Config_Ptr->timer_mode;

                        /* Set the Timer B PWM output level to be inverted */
                        SET_BIT(WTIMER1_CTL_R , TBPWML);

                        break;
                    }

                    break;

                    case TIMER2:

                        switch(Config_Ptr->timer_use)
                        {
                        case INDIVIDUAL_A:
                            /* Setting alternative function of PORTD pin 0 to PWM */
                            gpio_alternative_function(PORTD_ID, PIN0_ID, PWM);

                            break;

                        case INDIVIDUAL_B:
                            /* Setting alternative function of PORTD pin 1 to PWM */
                            gpio_alternative_function(PORTD_ID, PIN1_ID, PWM);

                            break;
                        }

                        /* disable timer2 output */
                        CLEAR_BIT(WTIMER2_CTL_R,Config_Ptr->timer_use);

                        /* the timer continues counting while the processor is halted by the debugger */
                        CLEAR_BIT(WTIMER2_CTL_R,(Config_Ptr->timer_use) + 1);

                        /*assign the global operation configuration option */
                        WTIMER2_CFG_R = Config_Ptr->timer_global_operation;

                        switch(Config_Ptr->timer_use)
                        {
                        case INDIVIDUAL_A:
                            /* Set the Alternate Mode for Timer A to PWM mode*/
                            SET_BIT(WTIMER2_TAMR_R , TAAMS);

                            /* Choose the edge-count Capture Mode for Timer A */
                            CLEAR_BIT(WTIMER2_TAMR_R , TACMR);

                            /* Set Timer A's mode */
                            WTIMER2_TAMR_R |= Config_Ptr->timer_mode;

                            /* Set the Timer A PWM output level to be inverted */
                            SET_BIT(WTIMER2_CTL_R , TAPWML);

                            break;

                        case INDIVIDUAL_B:
                            /* Set the Alternate Mode for Timer B to PWM mode*/
                            SET_BIT(WTIMER2_TBMR_R , TBAMS);

                            /* Choose the edge-count Capture Mode for Timer B */
                            CLEAR_BIT(WTIMER2_TBMR_R , TBCMR);

                            /* Set Timer B's mode */
                            WTIMER2_TBMR_R |= Config_Ptr->timer_mode;

                            /* Set the Timer B PWM output level to be inverted */
                            SET_BIT(WTIMER2_CTL_R , TBPWML);

                            break;
                        }

                        break;

                        case TIMER3:

                            switch(Config_Ptr->timer_use)
                            {
                            case INDIVIDUAL_A:
                                /* Setting alternative function of PORTD pin 2 to PWM */
                                gpio_alternative_function(PORTD_ID, PIN2_ID, PWM);

                                break;

                            case INDIVIDUAL_B:
                                /* Setting alternative function of PORTD pin 3 to PWM */
                                gpio_alternative_function(PORTD_ID, PIN3_ID, PWM);

                                break;
                            }

                            /* disable timer3 output */
                            CLEAR_BIT(WTIMER3_CTL_R,Config_Ptr->timer_use);

                            /* the timer continues counting while the processor is halted by the debugger */
                            CLEAR_BIT(WTIMER3_CTL_R,(Config_Ptr->timer_use) + 1);

                            /*assign the global operation configuration option */
                            WTIMER3_CFG_R = Config_Ptr->timer_global_operation;

                            switch(Config_Ptr->timer_use)
                            {
                            case INDIVIDUAL_A:
                                /* Set the Alternate Mode for Timer A to PWM mode*/
                                SET_BIT(WTIMER3_TAMR_R , TAAMS);

                                /* Choose the edge-count Capture Mode for Timer A */
                                CLEAR_BIT(WTIMER3_TAMR_R , TACMR);

                                /* Set Timer A's mode */
                                WTIMER3_TAMR_R |= Config_Ptr->timer_mode;

                                /* Set the Timer A PWM output level to be inverted */
                                SET_BIT(WTIMER3_CTL_R , TAPWML);

                                break;

                            case INDIVIDUAL_B:
                                /* Set the Alternate Mode for Timer B to PWM mode*/
                                SET_BIT(WTIMER3_TBMR_R , TBAMS);

                                /* Choose the edge-count Capture Mode for Timer B */
                                CLEAR_BIT(WTIMER3_TBMR_R , TBCMR);

                                /* Set Timer B's mode */
                                WTIMER3_TBMR_R |= Config_Ptr->timer_mode;

                                /* Set the Timer B PWM output level to be inverted */
                                SET_BIT(WTIMER3_CTL_R , TBPWML);

                                break;
                            }

                            break;

                            case TIMER4:

                                switch(Config_Ptr->timer_use)
                                {
                                case INDIVIDUAL_A:
                                    /* Setting alternative function of PORTD pin 4 to PWM */
                                    gpio_alternative_function(PORTD_ID, PIN4_ID, PWM);

                                    break;

                                case INDIVIDUAL_B:
                                    /* Setting alternative function of PORTD pin 5 to PWM */
                                    gpio_alternative_function(PORTD_ID, PIN5_ID, PWM);

                                    break;
                                }

                                /* disable timer4 output */
                                CLEAR_BIT(WTIMER4_CTL_R,Config_Ptr->timer_use);

                                /* the timer continues counting while the processor is halted by the debugger */
                                CLEAR_BIT(WTIMER4_CTL_R,(Config_Ptr->timer_use) + 1);

                                /*assign the global operation configuration option */
                                WTIMER4_CFG_R = Config_Ptr->timer_global_operation;

                                switch(Config_Ptr->timer_use)
                                {
                                case INDIVIDUAL_A:
                                    /* Set the Alternate Mode for Timer A to PWM mode*/
                                    SET_BIT(WTIMER4_TAMR_R , TAAMS);

                                    /* Choose the edge-count Capture Mode for Timer A */
                                    CLEAR_BIT(WTIMER4_TAMR_R , TACMR);

                                    /* Set Timer A's mode */
                                    WTIMER4_TAMR_R |= Config_Ptr->timer_mode;

                                    /* Set the Timer A PWM output level to be inverted */
                                    SET_BIT(WTIMER4_CTL_R , TAPWML);

                                    break;

                                case INDIVIDUAL_B:
                                    /* Set the Alternate Mode for Timer B to PWM mode*/
                                    SET_BIT(WTIMER4_TBMR_R , TBAMS);

                                    /* Choose the edge-count Capture Mode for Timer B */
                                    CLEAR_BIT(WTIMER4_TBMR_R , TBCMR);

                                    /* Set Timer B's mode */
                                    WTIMER4_TBMR_R |= Config_Ptr->timer_mode;

                                    /* Set the Timer B PWM output level to be inverted */
                                    SET_BIT(WTIMER4_CTL_R , TBPWML);

                                    break;
                                }

                                break;

                                case TIMER5:

                                    switch(Config_Ptr->timer_use)
                                    {
                                    case INDIVIDUAL_A:
                                        /* Setting alternative function of PORTD pin 6 to PWM */
                                        gpio_alternative_function(PORTD_ID, PIN6_ID, PWM);

                                        break;

                                    case INDIVIDUAL_B:
                                        /* Setting alternative function of PORTD pin 7 to PWM */
                                        gpio_alternative_function(PORTD_ID, PIN7_ID, PWM);

                                        break;
                                    }

                                    /* disable timer5 output */
                                    CLEAR_BIT(WTIMER5_CTL_R,Config_Ptr->timer_use);

                                    /* the timer continues counting while the processor is halted by the debugger */
                                    CLEAR_BIT(WTIMER5_CTL_R,(Config_Ptr->timer_use) + 1);

                                    /*assign the global operation configuration option */
                                    WTIMER5_CFG_R = Config_Ptr->timer_global_operation;

                                    switch(Config_Ptr->timer_use)
                                    {
                                    case INDIVIDUAL_A:
                                        /* Set the Alternate Mode for Timer A to PWM mode*/
                                        SET_BIT(WTIMER5_TAMR_R , TAAMS);

                                        /* Choose the edge-count Capture Mode for Timer A */
                                        CLEAR_BIT(WTIMER5_TAMR_R , TACMR);

                                        /* Set Timer A's mode */
                                        WTIMER5_TAMR_R |= Config_Ptr->timer_mode;

                                        /* Set the Timer A PWM output level to be inverted */
                                        SET_BIT(WTIMER5_CTL_R , TAPWML);

                                        break;

                                    case INDIVIDUAL_B:
                                        /* Set the Alternate Mode for Timer B to PWM mode*/
                                        SET_BIT(WTIMER5_TBMR_R , TBAMS);

                                        /* Choose the edge-count Capture Mode for Timer B */
                                        CLEAR_BIT(WTIMER5_TBMR_R , TBCMR);

                                        /* Set Timer B's mode */
                                        WTIMER5_TBMR_R |= Config_Ptr->timer_mode;

                                        /* Set the Timer B PWM output level to be inverted */
                                        SET_BIT(WTIMER5_CTL_R , TBPWML);

                                        break;
                                    }

                                    break;
            }
            break;
    }
}

/*
 * Description:
 * This function initializes and starts a PWM timer based on the provided configuration.
 * It sets the initial value of the timer interval load register, configures the match register,
 * and enables the specified PWM timer and channel.
 * Parameters:
 * - Config_Ptr: A pointer to a PWM_ConfigType structure containing timer configuration.
 * - compare_value: The value to set in the timer match register for PWM generation.
 * Returns: None.
 */
void pwm_start(const GPTM_ConfigType * Config_Ptr,uint32_t compare_value)
{
    switch(Config_Ptr->timer_type)
    {
    case _16_OR_32_BIT_TIMER:
        switch(Config_Ptr->timer_number)
        {
        case TIMER0:

            switch(Config_Ptr->timer_use)
            {
            case INDIVIDUAL_A:
                /* Set the initial value of TimerA Interval Load Register */
                TIMER0_TAILR_R = 0xFFFF;

                /* Set the TimerA Match Register to the desired value */
                TIMER0_TAMATCHR_R = compare_value;

                /* Enable Timer0A */
                SET_BIT(TIMER0_CTL_R , Config_Ptr->timer_use);

                break;

            case INDIVIDUAL_B:
                /* Set the initial value of TimerB Interval Load Register */
                TIMER0_TBILR_R = 0xFFFF;

                /* Set the TimerB Match Register to the desired value */
                TIMER0_TBMATCHR_R = compare_value;

                /* Enable Timer0B */
                SET_BIT(TIMER0_CTL_R , Config_Ptr->timer_use);

                break;
            }

            break;

            case TIMER1:

                switch(Config_Ptr->timer_use)
                {
                case INDIVIDUAL_A:
                    /* Set the initial value of TimerA Interval Load Register */
                    TIMER1_TAILR_R = 0xFFFF;

                    /* Set the TimerA Match Register to the desired value */
                    TIMER1_TAMATCHR_R = compare_value;

                    /* Enable Timer1A */
                    SET_BIT(TIMER1_CTL_R , Config_Ptr->timer_use);

                    break;

                case INDIVIDUAL_B:
                    /* Set the initial value of TimerB Interval Load Register */
                    TIMER1_TBILR_R = 0xFFFF;

                    /* Set the TimerB Match Register to the desired value */
                    TIMER1_TBMATCHR_R = compare_value;

                    /* Enable Timer1B */
                    SET_BIT(TIMER1_CTL_R , Config_Ptr->timer_use);

                    break;
                }

                break;

                case TIMER2:

                    switch(Config_Ptr->timer_use)
                    {
                    case INDIVIDUAL_A:
                        /* Set the initial value of TimerA Interval Load Register */
                        TIMER2_TAILR_R = 0xFFFF;

                        /* Set the TimerA Match Register to the desired value */
                        TIMER2_TAMATCHR_R = compare_value ;

                        /* Enable Timer2A */
                        SET_BIT(TIMER2_CTL_R , Config_Ptr->timer_use);

                        break;

                    case INDIVIDUAL_B:
                        /* Set the initial value of TimerB Interval Load Register */
                        TIMER2_TBILR_R = 0xFFFF;

                        /* Set the TimerB Match Register to the desired value */
                        TIMER2_TBMATCHR_R = compare_value ;

                        /* Enable Timer2B */
                        SET_BIT(TIMER2_CTL_R , Config_Ptr->timer_use);

                        break;
                    }

                    break;

                    case TIMER3:

                        switch(Config_Ptr->timer_use)
                        {
                        case INDIVIDUAL_A:
                            /* Set the initial value of TimerA Interval Load Register */
                            TIMER3_TAILR_R = 0xFFFF;

                            /* Set the TimerA Match Register to the desired value */
                            TIMER3_TAMATCHR_R = compare_value ;

                            /* Enable Timer3A */
                            SET_BIT(TIMER3_CTL_R , Config_Ptr->timer_use);

                            break;

                        case INDIVIDUAL_B:
                            /* Set the initial value of TimerB Interval Load Register */
                            TIMER3_TBILR_R = 0xFFFF;

                            /* Set the TimerB Match Register to the desired value */
                            TIMER3_TBMATCHR_R = compare_value ;

                            /* Enable Timer3B */
                            SET_BIT(TIMER3_CTL_R , Config_Ptr->timer_use);

                            break;
                        }

                        break;

                        case TIMER4:

                            switch(Config_Ptr->timer_use)
                            {
                            case INDIVIDUAL_A:
                                /* Set the initial value of TimerA Interval Load Register */
                                TIMER4_TAILR_R = 0xFFFF;

                                /* Set the TimerA Match Register to the desired value */
                                TIMER4_TAMATCHR_R = compare_value ;

                                /* Enable Timer4A */
                                SET_BIT(TIMER4_CTL_R , Config_Ptr->timer_use);

                                break;

                            case INDIVIDUAL_B:
                                /* Set the initial value of TimerB Interval Load Register */
                                TIMER4_TBILR_R = 0xFFFF;

                                /* Set the TimerB Match Register to the desired value */
                                TIMER4_TBMATCHR_R = compare_value ;

                                /* Enable Timer4B */
                                SET_BIT(TIMER4_CTL_R , Config_Ptr->timer_use);

                                break;
                            }

                            break;

                            case TIMER5:

                                switch(Config_Ptr->timer_use)
                                {
                                case INDIVIDUAL_A:
                                    /* Set the initial value of TimerA Interval Load Register */
                                    TIMER5_TAILR_R = 0xFFFF;

                                    /* Set the TimerA Match Register to the desired value */
                                    TIMER5_TAMATCHR_R = compare_value ;

                                    /* Enable Timer5A */
                                    SET_BIT(TIMER5_CTL_R , Config_Ptr->timer_use);

                                    break;

                                case INDIVIDUAL_B:
                                    /* Set the initial value of TimerB Interval Load Register */
                                    TIMER5_TBILR_R = 0xFFFF;

                                    /* Set the TimerB Match Register to the desired value */
                                    TIMER5_TBMATCHR_R = compare_value ;

                                    /* Enable Timer5B */
                                    SET_BIT(TIMER5_CTL_R , Config_Ptr->timer_use);

                                    break;
                                }

                                break;
        }
        break;

        case _32_OR_64_BIT_WIDE_TIMER:
            /*enable clock Timer in run mode */
            SET_BIT(SYSCTL_RCGCWTIMER_R,Config_Ptr->timer_number);
            switch(Config_Ptr->timer_number)
            {
            case TIMER0:

                switch(Config_Ptr->timer_use)
                {
                case INDIVIDUAL_A:
                    /* Set the initial value of TimerA Interval Load Register */
                    WTIMER0_TAILR_R = 0xFFFF;

                    /* Set the TimerA Match Register to the desired value */
                    WTIMER0_TAMATCHR_R = compare_value ;

                    /* Enable Wide Timer0A */
                    SET_BIT(WTIMER0_CTL_R , Config_Ptr->timer_use);

                    break;

                case INDIVIDUAL_B:
                    /* Set the initial value of TimerB Interval Load Register */
                    WTIMER0_TBILR_R = 0xFFFF;

                    /* Set the TimerB Match Register to the desired value */
                    WTIMER0_TBMATCHR_R = compare_value ;

                    /* Enable Wide Timer0B */
                    SET_BIT(WTIMER0_CTL_R , Config_Ptr->timer_use);

                    break;
                }

                break;

                case TIMER1:

                    switch(Config_Ptr->timer_use)
                    {
                    case INDIVIDUAL_A:
                        /* Set the initial value of TimerA Interval Load Register */
                        WTIMER1_TAILR_R = 0xFFFF;

                        /* Set the TimerA Match Register to the desired value */
                        WTIMER1_TAMATCHR_R = compare_value ;

                        /* Enable Wide Timer1A */
                        SET_BIT(WTIMER1_CTL_R , Config_Ptr->timer_use);

                        break;

                    case INDIVIDUAL_B:
                        /* Set the initial value of TimerB Interval Load Register */
                        WTIMER1_TBILR_R = 0xFFFF;

                        /* Set the TimerB Match Register to the desired value */
                        WTIMER1_TBMATCHR_R = compare_value ;

                        /* Enable Wide Timer1B */
                        SET_BIT(WTIMER1_CTL_R , Config_Ptr->timer_use);

                        break;
                    }
                    break;

                    case TIMER2:

                        switch(Config_Ptr->timer_use)
                        {
                        case INDIVIDUAL_A:
                            /* Set the initial value of TimerA Interval Load Register */
                            WTIMER2_TAILR_R = 0xFFFF;

                            /* Set the TimerA Match Register to the desired value */
                            WTIMER2_TAMATCHR_R = compare_value ;

                            /* Enable Wide Timer2A */
                            SET_BIT(WTIMER2_CTL_R , Config_Ptr->timer_use);

                            break;

                        case INDIVIDUAL_B:
                            /* Set the initial value of TimerB Interval Load Register */
                            WTIMER2_TBILR_R = 0xFFFF;

                            /* Set the TimerB Match Register to the desired value */
                            WTIMER2_TBMATCHR_R = compare_value ;

                            /* Enable Wide Timer2B */
                            SET_BIT(WTIMER2_CTL_R , Config_Ptr->timer_use);

                            break;
                        }
                        break;

                        case TIMER3:

                            switch(Config_Ptr->timer_use)
                            {
                            case INDIVIDUAL_A:
                                /* Set the initial value of TimerA Interval Load Register */
                                WTIMER3_TAILR_R = 0xFFFF;

                                /* Set the TimerA Match Register to the desired value */
                                WTIMER3_TAMATCHR_R = compare_value ;

                                /* Enable Wide Timer3A */
                                SET_BIT(WTIMER3_CTL_R , Config_Ptr->timer_use);

                                break;

                            case INDIVIDUAL_B:
                                /* Set the initial value of TimerB Interval Load Register */
                                WTIMER3_TBILR_R = 0xFFFF;

                                /* Set the TimerB Match Register to the desired value */
                                WTIMER3_TBMATCHR_R = compare_value ;

                                /* Enable Wide Timer3B */
                                SET_BIT(WTIMER3_CTL_R , Config_Ptr->timer_use);

                                break;
                            }
                            break;

                            case TIMER4:

                                switch(Config_Ptr->timer_use)
                                {
                                case INDIVIDUAL_A:
                                    /* Set the initial value of TimerA Interval Load Register */
                                    WTIMER4_TAILR_R = 0xFFFF;

                                    /* Set the TimerA Match Register to the desired value */
                                    WTIMER4_TAMATCHR_R = compare_value ;

                                    /* Enable Wide Timer4A */
                                    SET_BIT(WTIMER4_CTL_R , Config_Ptr->timer_use);

                                    break;

                                case INDIVIDUAL_B:
                                    /* Set the initial value of TimerB Interval Load Register */
                                    WTIMER4_TBILR_R = 0xFFFF;

                                    /* Set the TimerB Match Register to the desired value */
                                    WTIMER4_TBMATCHR_R = compare_value ;

                                    /* Enable Wide Timer4B */
                                    SET_BIT(WTIMER4_CTL_R , Config_Ptr->timer_use);

                                    break;
                                }
                                break;

                                case TIMER5:

                                    switch(Config_Ptr->timer_use)
                                    {
                                    case INDIVIDUAL_A:
                                        /* Set the initial value of TimerA Interval Load Register */
                                        WTIMER5_TAILR_R = 0xFFFF;

                                        /* Set the TimerA Match Register to the desired value */
                                        WTIMER5_TAMATCHR_R = compare_value ;

                                        /* Enable Wide Timer5A */
                                        SET_BIT(WTIMER5_CTL_R , Config_Ptr->timer_use);

                                        break;

                                    case INDIVIDUAL_B:
                                        /* Set the initial value of TimerB Interval Load Register */
                                        WTIMER5_TBILR_R = 0xFFFF;

                                        /* Set the TimerB Match Register to the desired value */
                                        WTIMER5_TBMATCHR_R = compare_value ;

                                        /* Enable Wide Timer5B */
                                        SET_BIT(WTIMER5_CTL_R , Config_Ptr->timer_use);

                                        break;
                                    }
                                    break;
            }
            break;
    }
}



