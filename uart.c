/*
 * uart.c
 *
 *  Created on: Feb 21, 2018
 *      Author: Rajan
 */

#include "driverlib.h"

const eUSCI_UART_Config uartConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        6,                                      // BRDIV = 26
        8,                                       // UCxBRF = 0
        0x11,                                       // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // MSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Low Frequency Mode
};


void uartA0_init()
{
    /* Selecting P1.2 and P1.3 in UART mode. */
      MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

       /* Configuring UART Module */
       MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);

       /* Enable UART module */
       MAP_UART_enableModule(EUSCI_A0_BASE);

       UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
       Interrupt_enableInterrupt(INT_EUSCIA0);

}


void uartA2_init()
{
          /* Selecting P3.2 and P3.3 in UART mode. */
           MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
                   GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
           /* Configuring UART Module */
           MAP_UART_initModule(EUSCI_A2_BASE, &uartConfig);

           /* Enable UART module */
           MAP_UART_enableModule(EUSCI_A2_BASE);

           UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
           Interrupt_enableInterrupt(INT_EUSCIA2);

           /* Configuring P1.1 as an input and enabling interrupts */
              MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
              MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1);
              MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);
              MAP_Interrupt_enableInterrupt(INT_PORT1);

}
