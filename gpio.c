/*
 * gpio.c
 *
 *  Created on: Feb 21, 2018
 *      Author: Rajan
 */

#include "driverlib.h"
#include <stdio.h>


void port6_init(void)
{
    /* Selecting P6.4 for Reset*/
     MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P6, GPIO_PIN4);



     MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6,
                      GPIO_PIN4 + GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
}


void port1_init(void)
{
    /* Configuring P1.1 as an input and enabling interrupts */
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1| GPIO_PIN4);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1|GPIO_PIN4);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1|GPIO_PIN4);
    MAP_Interrupt_enableInterrupt(INT_PORT1);
    /* Configuring P3.0 as output and P3.5 (switch) for LCD display intensity control */
       MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN5);

}


void boardInit(void)
{
    // GPIO Port Configuration for lowest power configuration
        P1->OUT = 0x00; P1->DIR = 0xFF;
        P2->OUT = 0x00; P2->DIR = 0xFF;
        P3->OUT = 0x00; P3->DIR = 0xFF;
       // P4->OUT = 0x00; P4->DIR = 0xFF;
       // P5->OUT = 0x00; P5->DIR = 0xFF;
        P6->OUT = 0x00; P6->DIR = 0xFF;
        P7->OUT = 0x00; P7->DIR = 0xFF;
        P8->OUT = 0x00; P8->DIR = 0xFF;
        P9->OUT = 0x00; P9->DIR = 0xFF;
        P10->OUT = 0x00; P10->DIR = 0xFF;
        PJ->OUT = 0x00; PJ->DIR = 0xFF;
}

