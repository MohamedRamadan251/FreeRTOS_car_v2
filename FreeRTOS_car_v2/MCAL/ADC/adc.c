/*******************************************************************************
 *                                                                             *
 * [FILE NAME]:   adc.c                                                        *
 *                                                                             *
 * [AUTHOR]:      Ahmed Saeed, Amr Gafar, Kareem Abd-elrasheed,                *
 *                Mohamed Abosreea, Ahmed Maher, and Mohamed Ramadan           *
 *                                                                             *
 * [Version]:     1.0.0                                                        *
 *                                                                             *
 * [DATE]:        09/09/2023                                                   *
 *                                                                             *
 * [DESCRIPTION]: source file for ADC driver                                   *
 *                                                                             *
 *******************************************************************************/

/*******************************************************************************
 *                                 Includes                                    *
 *******************************************************************************/
#include "LIB/bitwise_operation.h"

#include "tm4c123gh6pm.h"

#include "../../MCAL/GPIO/gpio.h"
#include "../../MCAL/ADC/adc.h"

/*
 * Description:
 * This function configures the ADC module by enabling its clock,
 * configuring the GPIO pin, disabling the specified sequencer,
 * selecting the trigger option, and enabling the sequencer.
 * Parameters:
 * - Copy_ADC: A pointer to an ADC_Config structure containing ADC configuration.
 * Returns: None.
 */
void ADC_Configure(ADC_Config * Copy_ADC)
{
    /*Generate ADC_module Clock*/
    switch (Copy_ADC->ADC_module)
    {
    case ADC_0:
        /*Generate ADC0_module CLK*/
        SYSCTL_RCGCADC_R |= (1<<ADC_0);
        break;

    case ADC_1:
        /*Generate ADC1_module CLK*/
        SYSCTL_RCGCADC_R |= (1<<ADC_1);
        break;

    default:
        break;
    }

    /*Configure GPIO pin*/
    gpio_alternative_function(Copy_ADC->ADC_port, Copy_ADC->ADC_pin, ADC);

    /*Disable sequencer*/
    switch (Copy_ADC->ADC_module)
    {
    case ADC_0:
        ADC0_ACTSS_R &= (~(1<<Copy_ADC->ADC_sequencer));
        break;

    case ADC_1:
        ADC1_ACTSS_R &= (~(1<<Copy_ADC->ADC_sequencer));
        break;

    default:
        break;
    }

    /*Select the trigger option to start the sampling*/
    /*By default: (processor)*/
    switch(Copy_ADC->ADC_sequencer)
    {
    case SS_0:
        if(ADC_0 == Copy_ADC->ADC_module)
        {
            /*Select sample and the channel*/
            ADC0_SSMUX0_R |= ((Copy_ADC->ADC_channel)<<(Copy_ADC->ADC_sample));
            /*Only one read*/
            if(NO == Copy_ADC->ADC_temp_mode)
            {
                ADC0_SSCTL0_R |= (2<<(Copy_ADC->ADC_sample));
            }
            else
            {
                ADC0_SSCTL0_R |= (10<<(Copy_ADC->ADC_sample));
            }
        }

        else
        {
            /*Select sample and the channel*/
            ADC1_SSMUX0_R |= ((Copy_ADC->ADC_channel)<<(Copy_ADC->ADC_sample));
            /*Only one read*/
            if(NO == Copy_ADC->ADC_temp_mode)
            {
                ADC1_SSCTL0_R |= (2<<(Copy_ADC->ADC_sample));
            }
            else
            {
                ADC1_SSCTL0_R |= (10<<(Copy_ADC->ADC_sample));
            }
        }

        break;

    case SS_1:
        if(ADC_0 == Copy_ADC->ADC_module)
        {
            /*Select sample and the channel*/
            ADC0_SSMUX1_R |= ((Copy_ADC->ADC_channel)<<(Copy_ADC->ADC_sample));
            /*Only one read*/
            if(NO == Copy_ADC->ADC_temp_mode)
            {
                ADC0_SSCTL1_R |= (2<<(Copy_ADC->ADC_sample));
            }
            else
            {
                ADC0_SSCTL1_R |= (10<<(Copy_ADC->ADC_sample));
            }
        }

        else
        {
            /*Select sample and the channel*/
            ADC1_SSMUX2_R |= ((Copy_ADC->ADC_channel)<<(Copy_ADC->ADC_sample));
            /*Only one read*/
            if(NO == Copy_ADC->ADC_temp_mode)
            {
                ADC1_SSCTL1_R |= (2<<(Copy_ADC->ADC_sample));
            }
            else
            {
                ADC1_SSCTL1_R |= (10<<(Copy_ADC->ADC_sample));
            }
        }

        break;

    case SS_2:
        if(ADC_0 == Copy_ADC->ADC_module)
        {
            /*Select sample and the channel*/
            ADC0_SSMUX2_R |= ((Copy_ADC->ADC_channel)<<(Copy_ADC->ADC_sample));
            /*Only one read*/
            if(NO == Copy_ADC->ADC_temp_mode)
            {
                ADC0_SSCTL2_R |= (2<<(Copy_ADC->ADC_sample));
            }
            else
            {
                ADC0_SSCTL2_R |= (10<<(Copy_ADC->ADC_sample));
            }
        }

        else
        {
            /*Select sample and the channel*/
            ADC1_SSMUX2_R |= ((Copy_ADC->ADC_channel)<<(Copy_ADC->ADC_sample));
            /*Only one read*/
            if(NO == Copy_ADC->ADC_temp_mode)
            {
                ADC1_SSCTL2_R |= (2<<(Copy_ADC->ADC_sample));
            }
            else
            {
                ADC1_SSCTL2_R |= (10<<(Copy_ADC->ADC_sample));
            }
        }

        break;

    case SS_3:
        if(ADC_0 == Copy_ADC->ADC_module)
        {
            /*Select sample and the channel*/
            ADC0_SSMUX3_R |= ((Copy_ADC->ADC_channel)<<(Copy_ADC->ADC_sample));
            /*Only one read*/
            if(NO == Copy_ADC->ADC_temp_mode)
            {
                ADC0_SSCTL3_R |= (2<<(Copy_ADC->ADC_sample));
            }
            else
            {
                ADC0_SSCTL3_R |= (10<<(Copy_ADC->ADC_sample));
            }

        }

        else
        {
            /*Select sample and the channel*/
            ADC1_SSMUX3_R |= ((Copy_ADC->ADC_channel)<<(Copy_ADC->ADC_sample));
            /*Only one read*/
            if(NO == Copy_ADC->ADC_temp_mode)
            {
                ADC1_SSCTL3_R |= (2<<(Copy_ADC->ADC_sample));
            }
            else
            {
                ADC1_SSCTL3_R |= (10<<(Copy_ADC->ADC_sample));
            }
        }

        break;
    }

    /*Enable sequencer*/
    switch (Copy_ADC->ADC_module)
    {
    case ADC_0:
        ADC0_ACTSS_R |= (1<<Copy_ADC->ADC_sequencer);
        break;

    case ADC_1:
        ADC1_ACTSS_R |= (1<<Copy_ADC->ADC_sequencer);
        break;

    default:
        break;
    }

}

/*
 * Description:
 * This function performs an ADC conversion for the specified ADC module, sequencer, and channel.
 * It starts the conversion, waits until it's complete, and returns the converted value.
 * Parameters:
 * - Copy_ADC: A pointer to an ADC_Config structure containing ADC configuration.
 * Returns:
 * - Local_reading: The converted ADC value.
 */
uint32_t ADC_Get_Conversion(ADC_Config * Copy_ADC)
{
    uint32_t Local_reading=0U;

    /*Enable ADC*/
    switch (Copy_ADC->ADC_module)
    {
    case ADC_0:
        /*ADC0 start Conversion */
        ADC0_PSSI_R |= (1<<Copy_ADC->ADC_sequencer);

        /*Pooling till the flag get high*/
        while ( (ADC0_RIS_R>>Copy_ADC->ADC_sequencer)&1);
        /*Clear flag*/

        ADC0_ISC_R &= (~(1<<Copy_ADC->ADC_sequencer));

        switch(Copy_ADC->ADC_sequencer)
        {
        case SS_0:
            /*get the reading*/
            Local_reading = ADC0_SSFIFO0_R;
            break;

        case SS_1:
            /*get the reading*/
            Local_reading = ADC0_SSFIFO1_R;
            break;

        case SS_2:
            /*get the reading*/
            Local_reading = ADC0_SSFIFO2_R;
            break;

        case SS_3:
            /*get the reading*/
            Local_reading = ADC0_SSFIFO3_R;
            break;
        }

        break;

    case ADC_1:
        /*ADC0 start Conversion */
        ADC1_PSSI_R |= (1<<Copy_ADC->ADC_sequencer);

        /*Pooling till the flag get high*/
        while ( (ADC1_RIS_R>>Copy_ADC->ADC_sequencer)&1);

        /*Clear flag*/
        ADC1_ISC_R &= (~(1<<Copy_ADC->ADC_sequencer));

        switch(Copy_ADC->ADC_sequencer)
        {
        case SS_0:
            /*get the reading*/
            Local_reading = ADC1_SSFIFO0_R;
            break;

        case SS_1:
            /*get the reading*/
            Local_reading = ADC1_SSFIFO1_R;
            break;

        case SS_2:
            /*get the reading*/
            Local_reading = ADC1_SSFIFO2_R;
            break;

        case SS_3:
            /*get the reading*/
            Local_reading = ADC1_SSFIFO3_R;
            break;
        }

        break;

    default:
        break;
    }

    return Local_reading;
}
