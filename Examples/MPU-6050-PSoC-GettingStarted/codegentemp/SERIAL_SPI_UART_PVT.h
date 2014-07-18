/*******************************************************************************
* File Name: SERIAL_SPI_UART_PVT.h
* Version 1.20
*
* Description:
*  This private file provides constants and parameter values for the
*  SCB Component in SPI and UART modes.
*  Please do not use this file or its content in your project.
*
* Note:
*
********************************************************************************
* Copyright 2013-2014, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_SPI_UART_PVT_SERIAL_H)
#define CY_SCB_SPI_UART_PVT_SERIAL_H

#include "SERIAL_SPI_UART.h"


/***************************************
*     Internal Global Vars
***************************************/

#if(SERIAL_INTERNAL_RX_SW_BUFFER_CONST)
    extern volatile uint32  SERIAL_rxBufferHead;
    extern volatile uint32  SERIAL_rxBufferTail;
    extern volatile uint8   SERIAL_rxBufferOverflow;
#endif /* (SERIAL_INTERNAL_RX_SW_BUFFER_CONST) */

#if(SERIAL_INTERNAL_TX_SW_BUFFER_CONST)
    extern volatile uint32  SERIAL_txBufferHead;
    extern volatile uint32  SERIAL_txBufferTail;
#endif /* (SERIAL_INTERNAL_TX_SW_BUFFER_CONST) */

#if(SERIAL_INTERNAL_RX_SW_BUFFER)
    extern volatile uint8 SERIAL_rxBufferInternal[SERIAL_RX_BUFFER_SIZE];
#endif /* (SERIAL_INTERNAL_RX_SW_BUFFER) */

#if(SERIAL_INTERNAL_TX_SW_BUFFER)
    extern volatile uint8 SERIAL_txBufferInternal[SERIAL_TX_BUFFER_SIZE];
#endif /* (SERIAL_INTERNAL_TX_SW_BUFFER) */


/***************************************
*     Private Function Prototypes
***************************************/

#if(SERIAL_SCB_MODE_SPI_CONST_CFG)
    void SERIAL_SpiInit(void);
#endif /* (SERIAL_SCB_MODE_SPI_CONST_CFG) */

#if(SERIAL_SPI_WAKE_ENABLE_CONST)
    void SERIAL_SpiSaveConfig(void);
    void SERIAL_SpiRestoreConfig(void);
#endif /* (SERIAL_SPI_WAKE_ENABLE_CONST) */

#if(SERIAL_SCB_MODE_UART_CONST_CFG)
    void SERIAL_UartInit(void);
#endif /* (SERIAL_SCB_MODE_UART_CONST_CFG) */

#if(SERIAL_UART_WAKE_ENABLE_CONST)
    void SERIAL_UartSaveConfig(void);
    void SERIAL_UartRestoreConfig(void);
#endif /* (SERIAL_UART_WAKE_ENABLE_CONST) */

/* Interrupt processing */
#define SERIAL_SpiUartEnableIntRx(intSourceMask)  SERIAL_SetRxInterruptMode(intSourceMask)
#define SERIAL_SpiUartEnableIntTx(intSourceMask)  SERIAL_SetTxInterruptMode(intSourceMask)
uint32  SERIAL_SpiUartDisableIntRx(void);
uint32  SERIAL_SpiUartDisableIntTx(void);

#endif /* (CY_SCB_SPI_UART_PVT_SERIAL_H) */


/* [] END OF FILE */
