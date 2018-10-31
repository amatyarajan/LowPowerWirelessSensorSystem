/*
 * adc.c
 *
 *  Created on: Jul 27, 2018
 *      Author: Rajan
 */


#include "clk.h"
#include "driverlib.h"

void adcInit(){

    /* Initializing ADC (MCLK/1/4) */
        MAP_ADC14_enableModule();
        MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK , ADC_PREDIVIDER_4, ADC_DIVIDER_6,
                0);


        /* Configuring ADC Memory in Multisequence mode */
        MAP_ADC14_configureMultiSequenceMode (ADC_MEM0, ADC_MEM8, false);


        /* Setting up GPIO pins as analog inputs */
        /* Pin 5 & Pin 4 for getting current for the Solar, Pin 0 & Pin 1 for measuring current from LTC */
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,
                    GPIO_PIN5 | GPIO_PIN4| GPIO_PIN0| GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);

        /* Setting up GPIO pins as analog inputs */
        /* Port 4, pin 5,6,7 for measuring voltage from solar, ltc and battery */
           MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4,
                       GPIO_PIN6 | GPIO_PIN7| GPIO_PIN5, GPIO_TERTIARY_MODULE_FUNCTION);


    /* Setting A0 & A1 and A4& A5 in differential mode */
    MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS  ,
    ADC_INPUT_A0, true);

    MAP_ADC14_configureConversionMemory(ADC_MEM4, ADC_VREFPOS_AVCC_VREFNEG_VSS  ,
    ADC_INPUT_A4, true);


    /* Setting A6, A7 & A8 in single ended mode */
    MAP_ADC14_configureConversionMemory(ADC_MEM6, ADC_VREFPOS_AVCC_VREFNEG_VSS  ,
    ADC_INPUT_A6, false);

    MAP_ADC14_configureConversionMemory(ADC_MEM7, ADC_VREFPOS_AVCC_VREFNEG_VSS  ,
        ADC_INPUT_A7, false);

    MAP_ADC14_configureConversionMemory(ADC_MEM8, ADC_VREFPOS_AVCC_VREFNEG_VSS  ,
           ADC_INPUT_A8, false);


    /* Configuring Sample Timer */
    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);



    /* Enabling interrupts */
    MAP_ADC14_enableInterrupt(ADC_INT8);

    /* Enabling/Toggling Conversion */
    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();

    MAP_Interrupt_enableInterrupt(INT_ADC14);

}



