/*******************************************************************************
* File Name: SERIAL_UART.c
* Version 1.20
*
* Description:
*  This file provides the source code to the API for the SCB Component in
*  UART mode.
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


#if(SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)

    /***************************************
    *  Config Structure Initialization
    ***************************************/

    const SERIAL_UART_INIT_STRUCT SERIAL_configUart =
    {
        SERIAL_UART_SUB_MODE,
        SERIAL_UART_DIRECTION,
        SERIAL_UART_DATA_BITS_NUM,
        SERIAL_UART_PARITY_TYPE,
        SERIAL_UART_STOP_BITS_NUM,
        SERIAL_UART_OVS_FACTOR,
        SERIAL_UART_IRDA_LOW_POWER,
        SERIAL_UART_MEDIAN_FILTER_ENABLE,
        SERIAL_UART_RETRY_ON_NACK,
        SERIAL_UART_IRDA_POLARITY,
        SERIAL_UART_DROP_ON_PARITY_ERR,
        SERIAL_UART_DROP_ON_FRAME_ERR,
        SERIAL_UART_WAKE_ENABLE,
        0u,
        NULL,
        0u,
        NULL,
        SERIAL_UART_MP_MODE_ENABLE,
        SERIAL_UART_MP_ACCEPT_ADDRESS,
        SERIAL_UART_MP_RX_ADDRESS,
        SERIAL_UART_MP_RX_ADDRESS_MASK,
        (uint32) SERIAL_SCB_IRQ_INTERNAL,
        SERIAL_UART_INTR_RX_MASK,
        SERIAL_UART_RX_TRIGGER_LEVEL,
        SERIAL_UART_INTR_TX_MASK,
        SERIAL_UART_TX_TRIGGER_LEVEL
    };


    /*******************************************************************************
    * Function Name: SERIAL_UartInit
    ********************************************************************************
    *
    * Summary:
    *  Configures the SCB for the UART operation.
    *
    * Parameters:
    *  config:  Pointer to a structure that contains the following ordered list of
    *           fields. These fields match the selections available in the
    *           customizer.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void SERIAL_UartInit(const SERIAL_UART_INIT_STRUCT *config)
    {
        if(NULL == config)
        {
            CYASSERT(0u != 0u); /* Halt execution due bad function parameter */
        }
        else
        {
            /* Configure pins */
            SERIAL_SetPins(SERIAL_SCB_MODE_UART, config->mode, config->direction);

            /* Store internal configuration */
            SERIAL_scbMode       = (uint8) SERIAL_SCB_MODE_UART;
            SERIAL_scbEnableWake = (uint8) config->enableWake;
            SERIAL_scbEnableIntr = (uint8) config->enableInterrupt;

            /* Set RX direction internal variables */
            SERIAL_rxBuffer      =         config->rxBuffer;
            SERIAL_rxDataBits    = (uint8) config->dataBits;
            SERIAL_rxBufferSize  = (uint8) config->rxBufferSize;

            /* Set TX direction internal variables */
            SERIAL_txBuffer      =         config->txBuffer;
            SERIAL_txDataBits    = (uint8) config->dataBits;
            SERIAL_txBufferSize  = (uint8) config->txBufferSize;

            /* Configure UART interface */
            if(SERIAL_UART_MODE_IRDA == config->mode)
            {
                /* OVS settings: IrDA */
                SERIAL_CTRL_REG  = ((0u != config->enableIrdaLowPower) ?
                                                (SERIAL_UART_GET_CTRL_OVS_IRDA_LP(config->oversample)) :
                                                (SERIAL_CTRL_OVS_IRDA_OVS16));
            }
            else
            {
                /* OVS settings: UART and SmartCard */
                SERIAL_CTRL_REG  = SERIAL_GET_CTRL_OVS(config->oversample);
            }

            SERIAL_CTRL_REG     |= SERIAL_GET_CTRL_ADDR_ACCEPT(config->multiprocAcceptAddr) |
                                             SERIAL_CTRL_UART;

            /* Configure sub-mode: UART, SmartCard or IrDA */
            SERIAL_UART_CTRL_REG = SERIAL_GET_UART_CTRL_MODE(config->mode);

            /* Configure RX direction */
            SERIAL_UART_RX_CTRL_REG = SERIAL_GET_UART_RX_CTRL_MODE(config->stopBits)              |
                                        SERIAL_GET_UART_RX_CTRL_POLARITY(config->enableInvertedRx)          |
                                        SERIAL_GET_UART_RX_CTRL_MP_MODE(config->enableMultiproc)            |
                                        SERIAL_GET_UART_RX_CTRL_DROP_ON_PARITY_ERR(config->dropOnParityErr) |
                                        SERIAL_GET_UART_RX_CTRL_DROP_ON_FRAME_ERR(config->dropOnFrameErr);

            if(SERIAL_UART_PARITY_NONE != config->parity)
            {
               SERIAL_UART_RX_CTRL_REG |= SERIAL_GET_UART_RX_CTRL_PARITY(config->parity) |
                                                    SERIAL_UART_RX_CTRL_PARITY_ENABLED;
            }

            SERIAL_RX_CTRL_REG      = SERIAL_GET_RX_CTRL_DATA_WIDTH(config->dataBits)       |
                                                SERIAL_GET_RX_CTRL_MEDIAN(config->enableMedianFilter) |
                                                SERIAL_GET_UART_RX_CTRL_ENABLED(config->direction);

            SERIAL_RX_FIFO_CTRL_REG = SERIAL_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(config->rxTriggerLevel);

            /* Configure MP address */
            SERIAL_RX_MATCH_REG     = SERIAL_GET_RX_MATCH_ADDR(config->multiprocAddr) |
                                                SERIAL_GET_RX_MATCH_MASK(config->multiprocAddrMask);

            /* Configure RX direction */
            SERIAL_UART_TX_CTRL_REG = SERIAL_GET_UART_TX_CTRL_MODE(config->stopBits) |
                                                SERIAL_GET_UART_TX_CTRL_RETRY_NACK(config->enableRetryNack);

            if(SERIAL_UART_PARITY_NONE != config->parity)
            {
               SERIAL_UART_TX_CTRL_REG |= SERIAL_GET_UART_TX_CTRL_PARITY(config->parity) |
                                                    SERIAL_UART_TX_CTRL_PARITY_ENABLED;
            }

            SERIAL_TX_CTRL_REG      = SERIAL_GET_TX_CTRL_DATA_WIDTH(config->dataBits)    |
                                                SERIAL_GET_UART_TX_CTRL_ENABLED(config->direction);

            SERIAL_TX_FIFO_CTRL_REG = SERIAL_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(config->txTriggerLevel);



            /* Configure interrupt with UART handler but do not enable it */
            CyIntDisable    (SERIAL_ISR_NUMBER);
            CyIntSetPriority(SERIAL_ISR_NUMBER, SERIAL_ISR_PRIORITY);
            (void) CyIntSetVector(SERIAL_ISR_NUMBER, &SERIAL_SPI_UART_ISR);

            /* Configure WAKE interrupt */
        #if(SERIAL_UART_RX_WAKEUP_IRQ)
            CyIntDisable    (SERIAL_RX_WAKE_ISR_NUMBER);
            CyIntSetPriority(SERIAL_RX_WAKE_ISR_NUMBER, SERIAL_RX_WAKE_ISR_PRIORITY);
            (void) CyIntSetVector(SERIAL_RX_WAKE_ISR_NUMBER, &SERIAL_UART_WAKEUP_ISR);
        #endif /* (SERIAL_UART_RX_WAKEUP_IRQ) */

            /* Configure interrupt sources */
            SERIAL_INTR_I2C_EC_MASK_REG = SERIAL_NO_INTR_SOURCES;
            SERIAL_INTR_SPI_EC_MASK_REG = SERIAL_NO_INTR_SOURCES;
            SERIAL_INTR_SLAVE_MASK_REG  = SERIAL_NO_INTR_SOURCES;
            SERIAL_INTR_MASTER_MASK_REG = SERIAL_NO_INTR_SOURCES;
            SERIAL_INTR_RX_MASK_REG     = config->rxInterruptMask;
            SERIAL_INTR_TX_MASK_REG     = config->txInterruptMask;

            /* Clear RX buffer indexes */
            SERIAL_rxBufferHead     = 0u;
            SERIAL_rxBufferTail     = 0u;
            SERIAL_rxBufferOverflow = 0u;

            /* Clrea TX buffer indexes */
            SERIAL_txBufferHead = 0u;
            SERIAL_txBufferTail = 0u;
        }
    }

#else

    /*******************************************************************************
    * Function Name: SERIAL_UartInit
    ********************************************************************************
    *
    * Summary:
    *  Configures the SCB for the UART operation.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void SERIAL_UartInit(void)
    {
        /* Configure UART interface */
        SERIAL_CTRL_REG = SERIAL_UART_DEFAULT_CTRL;

        /* Configure sub-mode: UART, SmartCard or IrDA */
        SERIAL_UART_CTRL_REG = SERIAL_UART_DEFAULT_UART_CTRL;

        /* Configure RX direction */
        SERIAL_UART_RX_CTRL_REG = SERIAL_UART_DEFAULT_UART_RX_CTRL;
        SERIAL_RX_CTRL_REG      = SERIAL_UART_DEFAULT_RX_CTRL;
        SERIAL_RX_FIFO_CTRL_REG = SERIAL_UART_DEFAULT_RX_FIFO_CTRL;
        SERIAL_RX_MATCH_REG     = SERIAL_UART_DEFAULT_RX_MATCH_REG;

        /* Configure TX direction */
        SERIAL_UART_TX_CTRL_REG = SERIAL_UART_DEFAULT_UART_TX_CTRL;
        SERIAL_TX_CTRL_REG      = SERIAL_UART_DEFAULT_TX_CTRL;
        SERIAL_TX_FIFO_CTRL_REG = SERIAL_UART_DEFAULT_TX_FIFO_CTRL;

        /* Configure interrupt with UART handler but do not enable it */
    #if(SERIAL_SCB_IRQ_INTERNAL)
        CyIntDisable    (SERIAL_ISR_NUMBER);
        CyIntSetPriority(SERIAL_ISR_NUMBER, SERIAL_ISR_PRIORITY);
        (void) CyIntSetVector(SERIAL_ISR_NUMBER, &SERIAL_SPI_UART_ISR);
    #endif /* (SERIAL_SCB_IRQ_INTERNAL) */

        /* Configure WAKE interrupt */
    #if(SERIAL_UART_RX_WAKEUP_IRQ)
        CyIntDisable    (SERIAL_RX_WAKE_ISR_NUMBER);
        CyIntSetPriority(SERIAL_RX_WAKE_ISR_NUMBER, SERIAL_RX_WAKE_ISR_PRIORITY);
        (void) CyIntSetVector(SERIAL_RX_WAKE_ISR_NUMBER, &SERIAL_UART_WAKEUP_ISR);
    #endif /* (SERIAL_UART_RX_WAKEUP_IRQ) */

        /* Configure interrupt sources */
        SERIAL_INTR_I2C_EC_MASK_REG = SERIAL_UART_DEFAULT_INTR_I2C_EC_MASK;
        SERIAL_INTR_SPI_EC_MASK_REG = SERIAL_UART_DEFAULT_INTR_SPI_EC_MASK;
        SERIAL_INTR_SLAVE_MASK_REG  = SERIAL_UART_DEFAULT_INTR_SLAVE_MASK;
        SERIAL_INTR_MASTER_MASK_REG = SERIAL_UART_DEFAULT_INTR_MASTER_MASK;
        SERIAL_INTR_RX_MASK_REG     = SERIAL_UART_DEFAULT_INTR_RX_MASK;
        SERIAL_INTR_TX_MASK_REG     = SERIAL_UART_DEFAULT_INTR_TX_MASK;

    #if(SERIAL_INTERNAL_RX_SW_BUFFER_CONST)
        SERIAL_rxBufferHead     = 0u;
        SERIAL_rxBufferTail     = 0u;
        SERIAL_rxBufferOverflow = 0u;
    #endif /* (SERIAL_INTERNAL_RX_SW_BUFFER_CONST) */

    #if(SERIAL_INTERNAL_TX_SW_BUFFER_CONST)
        SERIAL_txBufferHead = 0u;
        SERIAL_txBufferTail = 0u;
    #endif /* (SERIAL_INTERNAL_TX_SW_BUFFER_CONST) */
    }
#endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */


/*******************************************************************************
* Function Name: SERIAL_UartSetRxAddress
********************************************************************************
*
* Summary:
*  Sets the hardware detectable receiver address for the UART in the Multiprocessor
*  mode.
*
* Parameters:
*  address: Address for hardware address detection.
*
* Return:
*  None
*
*******************************************************************************/
void SERIAL_UartSetRxAddress(uint32 address)
{
     uint32 matchReg;

    matchReg = SERIAL_RX_MATCH_REG;

    matchReg &= ((uint32) ~SERIAL_RX_MATCH_ADDR_MASK); /* Clear address bits */
    matchReg |= ((uint32)  (address & SERIAL_RX_MATCH_ADDR_MASK)); /* Set address  */

    SERIAL_RX_MATCH_REG = matchReg;
}


/*******************************************************************************
* Function Name: SERIAL_UartSetRxAddressMask
********************************************************************************
*
* Summary:
*  Sets the hardware address mask for the UART in the Multiprocessor mode.
*
* Parameters:
*  addressMask: Address mask.
*   0 - address bit does not care while comparison.
*   1 - address bit is significant while comparison.
*
* Return:
*  None
*
*******************************************************************************/
void SERIAL_UartSetRxAddressMask(uint32 addressMask)
{
    uint32 matchReg;

    matchReg = SERIAL_RX_MATCH_REG;

    matchReg &= ((uint32) ~SERIAL_RX_MATCH_MASK_MASK); /* Clear address mask bits */
    matchReg |= ((uint32) (addressMask << SERIAL_RX_MATCH_MASK_POS));

    SERIAL_RX_MATCH_REG = matchReg;
}


#if(SERIAL_UART_RX_DIRECTION)
    /*******************************************************************************
    * Function Name: SERIAL_UartGetChar
    ********************************************************************************
    *
    * Summary:
    *  Retrieves the next data element from the receive buffer.
    *  This function is designed for ASCII characters and returns a char
    *  where 1 to 255 are valid characters and 0 indicates an error occurred or
    *  no data present.
    *  - The RX software buffer is disabled: returns the data element retrieved from the RX FIFO.
    *    Undefined data will be returned if the RX FIFO is empty.
    *  - The RX software buffer is enabled: returns the data element from the software receive
    *    buffer.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  The next data element from the receive buffer.
    *  ASCII character values from 1 to 255 are valid.
    *  A returned zero signifies an error condition or no data available.
    *
    *******************************************************************************/
    uint32 SERIAL_UartGetChar(void)
    {
        uint32 rxData = 0u;

        /* Read data only if there is data to read */
        if(0u != SERIAL_SpiUartGetRxBufferSize())
        {
            rxData = SERIAL_SpiUartReadRxData();
        }

        if(SERIAL_CHECK_INTR_RX(SERIAL_INTR_RX_ERR))
        {
            rxData = 0u; /* Error occurred: return zero */
            SERIAL_ClearRxInterruptSource(SERIAL_INTR_RX_ERR);
        }

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: SERIAL_UartGetByte
    ********************************************************************************
    *
    * Summary:
    *  Retrieves the next data element from the receive buffer, returns the received byte
    *  and error condition.
    *   - The RX software buffer is disabled: returns the data element retrieved from the RX FIFO.
    *     Undefined data will be returned if the RX FIFO is empty.
    *   - The RX software buffer is enabled: returns data element from the software receive
    *     buffer.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Bits 15-8 contains status and bits 7-0 contains the next data element from
    *  receive buffer. If the bits 15-8 are non-zero, an error has occurred.
    *
    *******************************************************************************/
    uint32 SERIAL_UartGetByte(void)
    {
        uint32 rxData;
        uint32 tmpStatus;
        uint32 intSourceMask;

        intSourceMask = SERIAL_SpiUartDisableIntRx();

        if(0u != SERIAL_SpiUartGetRxBufferSize())
        {
             /*
             * Enable interrupt to receive more bytes: at least one byte is in
             * buffer.
             */
            SERIAL_SpiUartEnableIntRx(intSourceMask);

            /* Get received byte */
            rxData = SERIAL_SpiUartReadRxData();
        }
        else
        {
            /*
            * Read byte directly from RX FIFO: the underflow is raised in case
            * of empty. In other case the first received byte will be read.
            */
            rxData = SERIAL_RX_FIFO_RD_REG;

            /*
            * Enable interrupt to receive more bytes.
            * The RX_NOT_EMPTY interrupt is cleared by the interrupt routine in case
            * the byte was received and read above.
            */
            SERIAL_SpiUartEnableIntRx(intSourceMask);
        }

        /* Get and clear RX error mask */
        tmpStatus = (SERIAL_GetRxInterruptSource() & SERIAL_INTR_RX_ERR);
        SERIAL_ClearRxInterruptSource(SERIAL_INTR_RX_ERR);

        /*
        * Put together data and error status:
        * MP mode and accept address: the 9th bit is set to notify mark.
        */
        rxData |= ((uint32) (tmpStatus << 8u));

        return(rxData);
    }

#endif /* (SERIAL_UART_RX_DIRECTION) */


#if(SERIAL_UART_TX_DIRECTION)
    /*******************************************************************************
    * Function Name: SERIAL_UartPutString
    ********************************************************************************
    *
    * Summary:
    *  Places a NULL terminated string in the transmit buffer to be sent at the
    *  next available bus time.
    *  This function is blocking and waits until there is space available to put
    *  all the requested data into the  transmit buffer.
    *
    * Parameters:
    *  string: pointer to the null terminated string array to be placed in the
    *          transmit buffer.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void SERIAL_UartPutString(const char8 string[])
    {
        uint32 bufIndex;

        bufIndex = 0u;

        /* Block the control flow until all data has been sent */
        while(string[bufIndex] != ((char8) 0))
        {
            SERIAL_UartPutChar((uint32) string[bufIndex]);
            bufIndex++;
        }
    }


    /*******************************************************************************
    * Function Name: SERIAL_UartPutCRLF
    ********************************************************************************
    *
    * Summary:
    *  Places a byte of data followed by a carriage return (0x0D) and line feed (0x0A)
    *  into the transmit buffer.
    *  This function is blocking and waits until there is space available to put
    *  all the requested data into the  transmit buffer.
    *
    * Parameters:
    *  txDataByte : the data to be transmitted.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void SERIAL_UartPutCRLF(uint32 txDataByte)
    {
        SERIAL_UartPutChar(txDataByte);  /* Blocks control flow until all data has been sent */
        SERIAL_UartPutChar(0x0Du);       /* Blocks control flow until all data has been sent */
        SERIAL_UartPutChar(0x0Au);       /* Blocks control flow until all data has been sent */
    }
#endif /* (SERIAL_UART_TX_DIRECTION) */


#if(SERIAL_UART_WAKE_ENABLE_CONST)
    /*******************************************************************************
    * Function Name: SERIAL_UartSaveConfig
    ********************************************************************************
    *
    * Summary:
    *  Clears and enables interrupt on a falling edge of the Rx input. The GPIO
    *  event wakes up the device and SKIP_START feature allows the UART continue
    *  receiving data bytes properly. The GPIO interrupt does not track in the active
    *  mode therefore requires to be cleared by this API.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void SERIAL_UartSaveConfig(void)
    {
        /* Clear interrupt activity:
        *  - set skip start and disable RX. On GPIO wakeup RX will be enabled.
        *  - clear rx_wake interrupt source as it triggers during normal operation.
        *  - clear wake interrupt pending state as it becomes pending in active mode.
        */

        SERIAL_UART_RX_CTRL_REG |= SERIAL_UART_RX_CTRL_SKIP_START;

    #if(SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
        #if(SERIAL_MOSI_SCL_RX_WAKE_PIN)
            (void) SERIAL_spi_mosi_i2c_scl_uart_rx_wake_ClearInterrupt();
        #endif /* (SERIAL_MOSI_SCL_RX_WAKE_PIN) */
    #else
        #if(SERIAL_UART_RX_WAKE_PIN)
            (void) SERIAL_rx_wake_ClearInterrupt();
        #endif /* (SERIAL_UART_RX_WAKE_PIN) */
    #endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */

    #if(SERIAL_UART_RX_WAKEUP_IRQ)
        SERIAL_RxWakeClearPendingInt();
        SERIAL_RxWakeEnableInt();
    #endif /* (SERIAL_UART_RX_WAKEUP_IRQ) */
    }


    /*******************************************************************************
    * Function Name: SERIAL_UartRestoreConfig
    ********************************************************************************
    *
    * Summary:
    *  Disables the RX GPIO interrupt. Until this function is called the interrupt remains
    *  active and triggers on every falling edge of the UART RX line.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void SERIAL_UartRestoreConfig(void)
    {
    /* Disable RX GPIO interrupt: no more triggers in active mode */
    #if(SERIAL_UART_RX_WAKEUP_IRQ)
        SERIAL_RxWakeDisableInt();
    #endif /* (SERIAL_UART_RX_WAKEUP_IRQ) */
    }
#endif /* (SERIAL_UART_WAKE_ENABLE_CONST) */


#if(SERIAL_UART_RX_WAKEUP_IRQ)
    /*******************************************************************************
    * Function Name: SERIAL_UART_WAKEUP_ISR
    ********************************************************************************
    *
    * Summary:
    *  Handles the Interrupt Service Routine for the SCB UART mode GPIO wakeup event. This
    *  event is configured to trigger on a falling edge of the RX line.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    CY_ISR(SERIAL_UART_WAKEUP_ISR)
    {
        /* Clear interrupt source: the event becomes multi triggered and is only disabled
        * by SERIAL_UartRestoreConfig() call.
        */
    #if(SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
        #if(SERIAL_MOSI_SCL_RX_WAKE_PIN)
            (void) SERIAL_spi_mosi_i2c_scl_uart_rx_wake_ClearInterrupt();
        #endif /* (SERIAL_MOSI_SCL_RX_WAKE_PIN) */
    #else
        #if(SERIAL_UART_RX_WAKE_PIN)
            (void) SERIAL_rx_wake_ClearInterrupt();
        #endif /* (SERIAL_UART_RX_WAKE_PIN) */
    #endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */
    }
#endif /* (SERIAL_UART_RX_WAKEUP_IRQ) */


/* [] END OF FILE */
