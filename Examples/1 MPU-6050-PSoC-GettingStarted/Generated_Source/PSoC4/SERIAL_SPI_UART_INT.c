/*******************************************************************************
* File Name: SERIAL_SPI_UART_INT.c
* Version 1.20
*
* Description:
*  This file provides the source code to the Interrupt Service Routine for
*  the SCB Component in SPI and UART modes.
*
* Note:
*
********************************************************************************
* Copyright 2013-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "SERIAL_PVT.h"
#include "SERIAL_SPI_UART_PVT.h"


/*******************************************************************************
* Function Name: SERIAL_SPI_UART_ISR
********************************************************************************
*
* Summary:
*  Handles the Interrupt Service Routine for the SCB SPI or UART modes.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
CY_ISR(SERIAL_SPI_UART_ISR)
{
#if(SERIAL_INTERNAL_RX_SW_BUFFER_CONST)
    uint32 locHead;
    uint32 dataRx;
#endif /* (SERIAL_INTERNAL_RX_SW_BUFFER_CONST) */

#if(SERIAL_INTERNAL_TX_SW_BUFFER_CONST)
    uint32 locTail;
#endif /* (SERIAL_INTERNAL_TX_SW_BUFFER_CONST) */

    if(NULL != SERIAL_customIntrHandler)
    {
        SERIAL_customIntrHandler();
    }

    #if(SERIAL_CHECK_SPI_WAKE_ENABLE)
    {
        /* Clear SPI wakeup source */
        SERIAL_ClearSpiExtClkInterruptSource(SERIAL_INTR_SPI_EC_WAKE_UP);
    }
    #endif

    #if(SERIAL_CHECK_RX_SW_BUFFER)
    {
        if(SERIAL_CHECK_INTR_RX_MASKED(SERIAL_INTR_RX_NOT_EMPTY))
        {
            while(0u != SERIAL_GET_RX_FIFO_ENTRIES)
            {
                /* Get data from RX FIFO */
                dataRx = SERIAL_RX_FIFO_RD_REG;

                /* Move local head index */
                locHead = (SERIAL_rxBufferHead + 1u);

                /* Adjust local head index */
                if(SERIAL_RX_BUFFER_SIZE == locHead)
                {
                    locHead = 0u;
                }

                if(locHead == SERIAL_rxBufferTail)
                {
                    /* Overflow: through away new data */
                    SERIAL_rxBufferOverflow = (uint8) SERIAL_INTR_RX_OVERFLOW;
                }
                else
                {
                    /* Store received data */
                    SERIAL_PutWordInRxBuffer(locHead, dataRx);

                    /* Move head index */
                    SERIAL_rxBufferHead = locHead;
                }
            }

            SERIAL_ClearRxInterruptSource(SERIAL_INTR_RX_NOT_EMPTY);
        }
    }
    #endif


    #if(SERIAL_CHECK_TX_SW_BUFFER)
    {
        if(SERIAL_CHECK_INTR_TX_MASKED(SERIAL_INTR_TX_NOT_FULL))
        {
            /* Put data into TX FIFO */
            while(SERIAL_FIFO_SIZE != SERIAL_GET_TX_FIFO_ENTRIES)
            {
                /* Check for a room in TX software buffer */
                if(SERIAL_txBufferHead != SERIAL_txBufferTail)
                {
                    /* Move local tail index */
                    locTail = (SERIAL_txBufferTail + 1u);

                    /* Adjust local tail index */
                    if(SERIAL_TX_BUFFER_SIZE == locTail)
                    {
                        locTail = 0u;
                    }

                    /* Put data into TX FIFO */
                    SERIAL_TX_FIFO_WR_REG = SERIAL_GetWordFromTxBuffer(locTail);

                    /* Move tail index */
                    SERIAL_txBufferTail = locTail;
                }
                else
                {
                    /* TX software buffer is empty: complete transmition */
                    SERIAL_DISABLE_INTR_TX(SERIAL_INTR_TX_NOT_FULL);
                    break;
                }
            }

            SERIAL_ClearTxInterruptSource(SERIAL_INTR_TX_NOT_FULL);
        }
    }
    #endif
}


/* [] END OF FILE */
