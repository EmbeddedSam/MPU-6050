/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include <project.h>
#include <interrupts.h>

extern uint8 flag;

CY_ISR(uartInterrupt)
{
    uint32 source = 0;
    /*  Place your Interrupt code here. */
    /* `#START UART_RX_ISR_Interrupt` */
    
    flag = 1; 
    
    source = UART_GetRxInterruptSourceMasked();
    UART_ClearRxInterruptSource(source);

    /* `#END` */
}


/* [] END OF FILE */
