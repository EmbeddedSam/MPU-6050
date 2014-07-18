/*******************************************************************************
* File Name: SERIAL_SPI_UART.c
* Version 1.20
*
* Description:
*  This file provides the source code to the API for the SCB Component in
*  SPI and UART modes.
*
* Note:
*
*******************************************************************************
* Copyright 2013-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "SERIAL_PVT.h"
#include "SERIAL_SPI_UART_PVT.h"

/***************************************
*        SPI/UART Private Vars
***************************************/

#if(SERIAL_INTERNAL_RX_SW_BUFFER_CONST)
    volatile uint32 SERIAL_rxBufferHead;
    volatile uint32 SERIAL_rxBufferTail;
    volatile uint8  SERIAL_rxBufferOverflow;
#endif /* (SERIAL_INTERNAL_RX_SW_BUFFER_CONST) */

#if(SERIAL_INTERNAL_TX_SW_BUFFER_CONST)
    volatile uint32 SERIAL_txBufferHead;
    volatile uint32 SERIAL_txBufferTail;
#endif /* (SERIAL_INTERNAL_TX_SW_BUFFER_CONST) */

#if(SERIAL_INTERNAL_RX_SW_BUFFER)
    /* Add one element to the buffer to receive full packet. One byte in receive buffer is always empty */
    volatile uint8 SERIAL_rxBufferInternal[SERIAL_RX_BUFFER_SIZE];
#endif /* (SERIAL_INTERNAL_RX_SW_BUFFER) */

#if(SERIAL_INTERNAL_TX_SW_BUFFER)
    volatile uint8 SERIAL_txBufferInternal[SERIAL_TX_BUFFER_SIZE];
#endif /* (SERIAL_INTERNAL_TX_SW_BUFFER) */


#if(SERIAL_RX_DIRECTION)

    /*******************************************************************************
    * Function Name: SERIAL_SpiUartReadRxData
    ********************************************************************************
    *
    * Summary:
    *  Retrieves the next data element from the receive buffer.
    *   - RX software buffer is disabled: Returns data element retrieved from
    *     RX FIFO. Undefined data will be returned if the RX FIFO is empty.
    *   - RX software buffer is enabled: Returns data element from the software
    *     receive buffer. Zero value is returned if the software receive buffer
    *     is empty.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Next data element from the receive buffer.
    *
    * Global Variables:
    *  Look into SERIAL_SpiInit for description.
    *
    *******************************************************************************/
    uint32 SERIAL_SpiUartReadRxData(void)
    {
        uint32 rxData = 0u;

        #if(SERIAL_INTERNAL_RX_SW_BUFFER_CONST)
            uint32 locTail;
        #endif /* (SERIAL_INTERNAL_RX_SW_BUFFER_CONST) */

        #if(SERIAL_CHECK_RX_SW_BUFFER)
        {
            if(SERIAL_rxBufferHead != SERIAL_rxBufferTail)
            {
                /* There is data in RX software buffer */

                /* Calculate index to read from */
                locTail = (SERIAL_rxBufferTail + 1u);

                if(SERIAL_RX_BUFFER_SIZE == locTail)
                {
                    locTail = 0u;
                }

                /* Get data from RX software buffer */
                rxData = SERIAL_GetWordFromRxBuffer(locTail);

                /* Change index in the buffer */
                SERIAL_rxBufferTail = locTail;
            }
        }
        #else
        {
            rxData = SERIAL_RX_FIFO_RD_REG; /* Read data from RX FIFO */
        }
        #endif

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: SERIAL_SpiUartGetRxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Returns the number of received data elements in the receive buffer.
    *   - RX software buffer disabled: returns the number of used entries in
    *     RX FIFO.
    *   - RX software buffer enabled: returns the number of elements which were
    *     placed in the receive buffer.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Number of received data elements
    *
    *******************************************************************************/
    uint32 SERIAL_SpiUartGetRxBufferSize(void)
    {
        uint32 size;
        #if(SERIAL_INTERNAL_RX_SW_BUFFER_CONST)
            uint32 locHead;
        #endif /* (SERIAL_INTERNAL_RX_SW_BUFFER_CONST) */

        #if(SERIAL_CHECK_RX_SW_BUFFER)
        {
            locHead = SERIAL_rxBufferHead;

            if(locHead >= SERIAL_rxBufferTail)
            {
                size = (locHead - SERIAL_rxBufferTail);
            }
            else
            {
                size = (locHead + (SERIAL_RX_BUFFER_SIZE - SERIAL_rxBufferTail));
            }
        }
        #else
        {
            size = SERIAL_GET_RX_FIFO_ENTRIES;
        }
        #endif

        return(size);
    }


    /*******************************************************************************
    * Function Name: SERIAL_SpiUartClearRxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears the receive buffer and RX FIFO.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void SERIAL_SpiUartClearRxBuffer(void)
    {
        #if(SERIAL_INTERNAL_RX_SW_BUFFER_CONST)
            uint32 intSourceMask;
        #endif /* (SERIAL_INTERNAL_RX_SW_BUFFER_CONST) */

        #if(SERIAL_CHECK_RX_SW_BUFFER)
        {
            intSourceMask = SERIAL_SpiUartDisableIntRx();

            SERIAL_CLEAR_RX_FIFO;

            /* Flush RX software buffer */
            SERIAL_rxBufferHead     = SERIAL_rxBufferTail;
            SERIAL_rxBufferOverflow = 0u;

            /* End RX transfer */
            SERIAL_ClearRxInterruptSource(SERIAL_INTR_RX_ALL);

            SERIAL_SpiUartEnableIntRx(intSourceMask);
        }
        #else
        {
            SERIAL_CLEAR_RX_FIFO;
        }
        #endif
    }

#endif /* (SERIAL_RX_DIRECTION) */


#if(SERIAL_TX_DIRECTION)

    /*******************************************************************************
    * Function Name: SERIAL_SpiUartWriteTxData
    ********************************************************************************
    *
    * Summary:
    *  Places a data entry into the transmit buffer to be sent at the next available
    *  bus time.
    *  This function is blocking and waits until there is space available to put the
    *  requested data in the transmit buffer.
    *
    * Parameters:
    *  txDataByte: the data to be transmitted.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void SERIAL_SpiUartWriteTxData(uint32 txData)
    {
        #if(SERIAL_INTERNAL_TX_SW_BUFFER_CONST)
            uint32 locHead;
            uint32 intSourceMask;
        #endif /* (SERIAL_INTERNAL_TX_SW_BUFFER_CONST) */

        #if(SERIAL_CHECK_TX_SW_BUFFER)
        {
            /* Head index to put data */
            locHead = (SERIAL_txBufferHead + 1u);

            /* Adjust TX software buffer index */
            if(SERIAL_TX_BUFFER_SIZE == locHead)
            {
                locHead = 0u;
            }

            while(locHead == SERIAL_txBufferTail)
            {
                /* Wait for space in TX software buffer */
            }

            /* TX software buffer has at least one room */

            if((SERIAL_txBufferHead == SERIAL_txBufferTail) &&
               (SERIAL_FIFO_SIZE != SERIAL_GET_TX_FIFO_ENTRIES))
            {
                /* TX software buffer is empty: put data directly in TX FIFO */
                SERIAL_TX_FIFO_WR_REG = txData;
            }
            /* Put data in TX software buffer */
            else
            {
                /* Clear old status of INTR_TX_NOT_FULL. It sets at the end of transfer when TX FIFO is empty. */
                SERIAL_ClearTxInterruptSource(SERIAL_INTR_TX_NOT_FULL);

                SERIAL_PutWordInTxBuffer(locHead, txData);

                SERIAL_txBufferHead = locHead;

                /* Enable interrupt to transmit */
                intSourceMask  = SERIAL_INTR_TX_NOT_FULL;
                intSourceMask |= SERIAL_GetTxInterruptMode();
                SERIAL_SpiUartEnableIntTx(intSourceMask);
            }
        }
        #else
        {
            while(SERIAL_FIFO_SIZE == SERIAL_GET_TX_FIFO_ENTRIES)
            {
                /* Block while TX FIFO is FULL */
            }

            SERIAL_TX_FIFO_WR_REG = txData;
        }
        #endif
    }


    /*******************************************************************************
    * Function Name: SERIAL_SpiUartPutArray
    ********************************************************************************
    *
    * Summary:
    *  Places an array of data into the transmit buffer to be sent.
    *  This function is blocking and waits until there is a space available to put
    *  all the requested data in the transmit buffer. The array size can be greater
    *  than transmit buffer size.
    *
    * Parameters:
    *  wrBuf:  pointer to an array with data to be placed in transmit buffer.
    *  count:  number of data elements to be placed in the transmit buffer.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void SERIAL_SpiUartPutArray(const uint8 wrBuf[], uint32 count)
    {
        uint32 i;

        for(i=0u; i < count; i++)
        {
            SERIAL_SpiUartWriteTxData((uint32) wrBuf[i]);
        }
    }


    /*******************************************************************************
    * Function Name: SERIAL_SpiUartGetTxBufferSize
    ********************************************************************************
    *
    * Summary:
    * Returns the number of elements currently in the transmit buffer.
    *  - TX software buffer is disabled: returns the number of used entries in
    *    TX FIFO.
    *  - TX software buffer is enabled: returns the number of elements currently
    *    used in the transmit buffer. This number does not include used entries in
    *    the TX FIFO. The transmit buffer size is zero until the TX FIFO is
    *    not full.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Number of data elements ready to transmit.
    *
    *******************************************************************************/
    uint32 SERIAL_SpiUartGetTxBufferSize(void)
    {
        uint32 size;
        #if(SERIAL_INTERNAL_TX_SW_BUFFER_CONST)
            uint32 locTail;
        #endif /* (SERIAL_INTERNAL_TX_SW_BUFFER_CONST) */

        #if(SERIAL_CHECK_TX_SW_BUFFER)
        {
            /* Get current Tail index */
            locTail = SERIAL_txBufferTail;

            if(SERIAL_txBufferHead >= locTail)
            {
                size = (SERIAL_txBufferHead - locTail);
            }
            else
            {
                size = (SERIAL_txBufferHead + (SERIAL_TX_BUFFER_SIZE - locTail));
            }
        }
        #else
        {
            size = SERIAL_GET_TX_FIFO_ENTRIES;
        }
        #endif

        return(size);
    }


    /*******************************************************************************
    * Function Name: SERIAL_SpiUartClearTxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears the transmit buffer and TX FIFO.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void SERIAL_SpiUartClearTxBuffer(void)
    {
        #if(SERIAL_INTERNAL_TX_SW_BUFFER_CONST)
            uint32 intSourceMask;
        #endif /* (SERIAL_INTERNAL_TX_SW_BUFFER_CONST) */

        #if(SERIAL_CHECK_TX_SW_BUFFER)
        {
            intSourceMask = SERIAL_SpiUartDisableIntTx();

            SERIAL_CLEAR_TX_FIFO;

            /* Flush TX software buffer */
            SERIAL_txBufferHead = SERIAL_txBufferTail;

            /* End TX transfer if it is in progress */
            intSourceMask &= (uint32) ~SERIAL_INTR_TX_NOT_FULL;

            SERIAL_SpiUartEnableIntTx(intSourceMask);
        }
        #else
        {
            SERIAL_CLEAR_TX_FIFO;
        }
        #endif
    }

#endif /* (SERIAL_TX_DIRECTION) */


/*******************************************************************************
* Function Name: SERIAL_SpiUartDisableIntRx
********************************************************************************
*
* Summary:
*  Disables the RX interrupt sources.
*
* Parameters:
*  None
*
* Return:
*  Returns the RX interrupt sources enabled before the function call.
*
*******************************************************************************/
uint32 SERIAL_SpiUartDisableIntRx(void)
{
    uint32 intSource;

    intSource = SERIAL_GetRxInterruptMode();

    SERIAL_SetRxInterruptMode(SERIAL_NO_INTR_SOURCES);

    return(intSource);
}


/*******************************************************************************
* Function Name: SERIAL_SpiUartDisableIntTx
********************************************************************************
*
* Summary:
*  Disables TX interrupt sources.
*
* Parameters:
*  None
*
* Return:
*  Returns TX interrupt sources enabled before function call.
*
*******************************************************************************/
uint32 SERIAL_SpiUartDisableIntTx(void)
{
    uint32 intSourceMask;

    intSourceMask = SERIAL_GetTxInterruptMode();

    SERIAL_SetTxInterruptMode(SERIAL_NO_INTR_SOURCES);

    return(intSourceMask);
}


#if(SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
    /*******************************************************************************
    * Function Name: SERIAL_PutWordInRxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Stores a byte/word into the RX buffer.
    *  Only available in the Unconfigured operation mode.
    *
    * Parameters:
    *  index:      index to store data byte/word in the RX buffer.
    *  rxDataByte: byte/word to store.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void SERIAL_PutWordInRxBuffer(uint32 idx, uint32 rxDataByte)
    {
        /* Put data in buffer */
        if(SERIAL_ONE_BYTE_WIDTH == SERIAL_rxDataBits)
        {
            SERIAL_rxBuffer[idx] = ((uint8) rxDataByte);
        }
        else
        {
            SERIAL_rxBuffer[(uint32)(idx << 1u)]      = LO8(LO16(rxDataByte));
            SERIAL_rxBuffer[(uint32)(idx << 1u) + 1u] = HI8(LO16(rxDataByte));
        }
    }


    /*******************************************************************************
    * Function Name: SERIAL_GetWordFromRxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Reads byte/word from RX buffer.
    *  Only available in the Unconfigured operation mode.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Returns byte/word read from RX buffer.
    *
    *******************************************************************************/
    uint32 SERIAL_GetWordFromRxBuffer(uint32 idx)
    {
        uint32 value;

        if(SERIAL_ONE_BYTE_WIDTH == SERIAL_rxDataBits)
        {
            value = SERIAL_rxBuffer[idx];
        }
        else
        {
            value  = (uint32) SERIAL_rxBuffer[(uint32)(idx << 1u)];
            value |= (uint32) ((uint32)SERIAL_rxBuffer[(uint32)(idx << 1u) + 1u] << 8u);
        }

        return(value);
    }


    /*******************************************************************************
    * Function Name: SERIAL_PutWordInTxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Stores byte/word into the TX buffer.
    *  Only available in the Unconfigured operation mode.
    *
    * Parameters:
    *  idx:        index to store data byte/word in the TX buffer.
    *  txDataByte: byte/word to store.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void SERIAL_PutWordInTxBuffer(uint32 idx, uint32 txDataByte)
    {
        /* Put data in buffer */
        if(SERIAL_ONE_BYTE_WIDTH == SERIAL_txDataBits)
        {
            SERIAL_txBuffer[idx] = ((uint8) txDataByte);
        }
        else
        {
            SERIAL_txBuffer[(uint32)(idx << 1u)]      = LO8(LO16(txDataByte));
            SERIAL_txBuffer[(uint32)(idx << 1u) + 1u] = HI8(LO16(txDataByte));
        }
    }


    /*******************************************************************************
    * Function Name: SERIAL_GetWordFromTxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Reads byte/word from the TX buffer.
    *  Only available in the Unconfigured operation mode.
    *
    * Parameters:
    *  idx: index to get data byte/word from the TX buffer.
    *
    * Return:
    *  Returns byte/word read from the TX buffer.
    *
    *******************************************************************************/
    uint32 SERIAL_GetWordFromTxBuffer(uint32 idx)
    {
        uint32 value;

        if(SERIAL_ONE_BYTE_WIDTH == SERIAL_txDataBits)
        {
            value = (uint32) SERIAL_txBuffer[idx];
        }
        else
        {
            value  = (uint32) SERIAL_txBuffer[(uint32)(idx << 1u)];
            value |= (uint32) ((uint32) SERIAL_txBuffer[(uint32)(idx << 1u) + 1u] << 8u);
        }

        return(value);
    }

#endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */


/* [] END OF FILE */
