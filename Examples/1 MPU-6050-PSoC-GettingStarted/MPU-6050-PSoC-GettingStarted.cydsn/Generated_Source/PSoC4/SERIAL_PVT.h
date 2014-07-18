/*******************************************************************************
* File Name: .h
* Version 1.20
*
* Description:
*  This private file provides constants and parameter values for the
*  SCB Component.
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

#if !defined(CY_SCB_PVT_SERIAL_H)
#define CY_SCB_PVT_SERIAL_H

#include "SERIAL.h"


/***************************************
*     Private Function Prototypes
***************************************/

/* APIs to service INTR_I2C_EC register */
#define SERIAL_SetI2CExtClkInterruptMode(interruptMask) SERIAL_WRITE_INTR_I2C_EC_MASK(interruptMask)
#define SERIAL_ClearI2CExtClkInterruptSource(interruptMask) SERIAL_CLEAR_INTR_I2C_EC(interruptMask)
#define SERIAL_GetI2CExtClkInterruptSource()                (SERIAL_INTR_I2C_EC_REG)
#define SERIAL_GetI2CExtClkInterruptMode()                  (SERIAL_INTR_I2C_EC_MASK_REG)
#define SERIAL_GetI2CExtClkInterruptSourceMasked()          (SERIAL_INTR_I2C_EC_MASKED_REG)

#if(!SERIAL_CY_SCBIP_V1_I2C_ONLY)
/* APIs to service INTR_SPI_EC register */
#define SERIAL_SetSpiExtClkInterruptMode(interruptMask) SERIAL_WRITE_INTR_SPI_EC_MASK(interruptMask)
#define SERIAL_ClearSpiExtClkInterruptSource(interruptMask) SERIAL_CLEAR_INTR_SPI_EC(interruptMask)
#define SERIAL_GetExtSpiClkInterruptSource()                 (SERIAL_INTR_SPI_EC_REG)
#define SERIAL_GetExtSpiClkInterruptMode()                   (SERIAL_INTR_SPI_EC_MASK_REG)
#define SERIAL_GetExtSpiClkInterruptSourceMasked()           (SERIAL_INTR_SPI_EC_MASKED_REG)
#endif /* (!SERIAL_CY_SCBIP_V1_I2C_ONLY) */

#if(SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
    extern void SERIAL_SetPins(uint32 mode, uint32 subMode, uint32 uartTxRx);
#endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*     Vars with External Linkage
***************************************/

#if !defined (CY_REMOVE_SERIAL_CUSTOM_INTR_HANDLER)
    extern cyisraddress SERIAL_customIntrHandler;
#endif /* !defined (CY_REMOVE_SERIAL_CUSTOM_INTR_HANDLER) */

extern SERIAL_BACKUP_STRUCT SERIAL_backup;

#if(SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common config vars */
    extern uint8 SERIAL_scbMode;
    extern uint8 SERIAL_scbEnableWake;
    extern uint8 SERIAL_scbEnableIntr;

    /* I2C config vars */
    extern uint8 SERIAL_mode;
    extern uint8 SERIAL_acceptAddr;

    /* SPI/UART config vars */
    extern volatile uint8 * SERIAL_rxBuffer;
    extern uint8   SERIAL_rxDataBits;
    extern uint32  SERIAL_rxBufferSize;

    extern volatile uint8 * SERIAL_txBuffer;
    extern uint8   SERIAL_txDataBits;
    extern uint32  SERIAL_txBufferSize;

    /* EZI2C config vars */
    extern uint8 SERIAL_numberOfAddr;
    extern uint8 SERIAL_subAddrSize;
#endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*  Conditional Macro
****************************************/

#if(SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Define run time operation mode */
    #define SERIAL_SCB_MODE_I2C_RUNTM_CFG     (SERIAL_SCB_MODE_I2C      == SERIAL_scbMode)
    #define SERIAL_SCB_MODE_SPI_RUNTM_CFG     (SERIAL_SCB_MODE_SPI      == SERIAL_scbMode)
    #define SERIAL_SCB_MODE_UART_RUNTM_CFG    (SERIAL_SCB_MODE_UART     == SERIAL_scbMode)
    #define SERIAL_SCB_MODE_EZI2C_RUNTM_CFG   (SERIAL_SCB_MODE_EZI2C    == SERIAL_scbMode)
    #define SERIAL_SCB_MODE_UNCONFIG_RUNTM_CFG \
                                                        (SERIAL_SCB_MODE_UNCONFIG == SERIAL_scbMode)

    /* Define wakeup enable */
    #define SERIAL_SCB_WAKE_ENABLE_CHECK        (0u != SERIAL_scbEnableWake)
#endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */

#endif /* (CY_SCB_PVT_SERIAL_H) */


/* [] END OF FILE */
