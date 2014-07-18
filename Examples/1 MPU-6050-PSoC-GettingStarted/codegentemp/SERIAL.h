/*******************************************************************************
* File Name: SERIAL.h
* Version 1.20
*
* Description:
*  This file provides constants and parameter values for the SCB Component.
*
* Note:
*
********************************************************************************
* Copyright 2013-2013-2013-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_SERIAL_H)
#define CY_SCB_SERIAL_H

#include <cydevice_trm.h>
#include <cyfitter.h>
#include <cytypes.h>
#include <CyLib.h>


/***************************************
*  Conditional Compilation Parameters
****************************************/

#define SERIAL_CY_SCBIP_V0 (CYIPBLOCK_m0s8scb_VERSION == 0u)
#define SERIAL_CY_SCBIP_V1 (CYIPBLOCK_m0s8scb_VERSION >= 1u)

#if(SERIAL_CY_SCBIP_V1)
    #define SERIAL_CY_SCBIP_V1_I2C_ONLY   (1u) /* SCB IP V1 supports only I2C */
#else
    #define SERIAL_CY_SCBIP_V1_I2C_ONLY   (0u) /* SCB IP V0 supports I2C, SPI, UART */
#endif /* (SERIAL_CY_SCBIP_V1) */

#define SERIAL_SCB_MODE                     (4u)

/* SCB modes enum */
#define SERIAL_SCB_MODE_I2C                 (0x01u)
#define SERIAL_SCB_MODE_SPI                 (0x02u)
#define SERIAL_SCB_MODE_UART                (0x04u)
#define SERIAL_SCB_MODE_EZI2C               (0x08u)
#define SERIAL_SCB_MODE_UNCONFIG            (0xFFu)

/* Condition compilation depends on operation mode: Unconfigured implies apply to all modes */
#define SERIAL_SCB_MODE_I2C_CONST_CFG       (SERIAL_SCB_MODE_I2C       == SERIAL_SCB_MODE)
#define SERIAL_SCB_MODE_SPI_CONST_CFG       (SERIAL_SCB_MODE_SPI       == SERIAL_SCB_MODE)
#define SERIAL_SCB_MODE_UART_CONST_CFG      (SERIAL_SCB_MODE_UART      == SERIAL_SCB_MODE)
#define SERIAL_SCB_MODE_EZI2C_CONST_CFG     (SERIAL_SCB_MODE_EZI2C     == SERIAL_SCB_MODE)
#define SERIAL_SCB_MODE_UNCONFIG_CONST_CFG  (SERIAL_SCB_MODE_UNCONFIG  == SERIAL_SCB_MODE)

/* Condition compilation for includes */
#define SERIAL_SCB_MODE_I2C_INC       (0u !=(SERIAL_SCB_MODE_I2C       & SERIAL_SCB_MODE))
#define SERIAL_SCB_MODE_EZI2C_INC     (0u !=(SERIAL_SCB_MODE_EZI2C     & SERIAL_SCB_MODE))
#if(!SERIAL_CY_SCBIP_V1_I2C_ONLY)
#define SERIAL_SCB_MODE_SPI_INC       (0u !=(SERIAL_SCB_MODE_SPI       & SERIAL_SCB_MODE))
#define SERIAL_SCB_MODE_UART_INC      (0u !=(SERIAL_SCB_MODE_UART      & SERIAL_SCB_MODE))
#else
#define SERIAL_SCB_MODE_SPI_INC       (0u)
#define SERIAL_SCB_MODE_UART_INC      (0u)
#endif /* ((!SERIAL_CY_SCBIP_V1_I2C_ONLY) */

/* Interrupts remove options */
#define SERIAL_REMOVE_SCB_IRQ             (1u)
#define SERIAL_SCB_IRQ_INTERNAL           (0u == SERIAL_REMOVE_SCB_IRQ)

#define SERIAL_REMOVE_UART_RX_WAKEUP_IRQ  (1u)
#define SERIAL_UART_RX_WAKEUP_IRQ         (0u == SERIAL_REMOVE_UART_RX_WAKEUP_IRQ)

/* SCB interrupt enum */
#define SERIAL_SCB_INTR_MODE_NONE     (0u)
#define SERIAL_SCB_INTR_MODE_INTERNAL (1u)
#define SERIAL_SCB_INTR_MODE_EXTERNAL (2u)

/* Bootloader communication interface enable */
#define SERIAL_BTLDR_COMM_ENABLED ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_SERIAL) || \
                                             (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))


/***************************************
*       Includes
****************************************/

#include "SERIAL_PINS.h"


/***************************************
*       Type Definitions
***************************************/

typedef struct
{
    uint8 enableState;
} SERIAL_BACKUP_STRUCT;


/***************************************
*        Function Prototypes
***************************************/

/* Start and Stop APIs */
void SERIAL_Init(void);
void SERIAL_Enable(void);
void SERIAL_Start(void);
void SERIAL_Stop(void);

/* Sleep and Wakeup APis */
void SERIAL_Sleep(void);
void SERIAL_Wakeup(void);

/* Custom interrupt handler */
void SERIAL_SetCustomInterruptHandler(cyisraddress func);

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (SERIAL_BTLDR_COMM_ENABLED)
    /* Bootloader Physical layer functions */
    void SERIAL_CyBtldrCommStart(void);
    void SERIAL_CyBtldrCommStop (void);
    void SERIAL_CyBtldrCommReset(void);
    cystatus SERIAL_CyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus SERIAL_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    #if(CYDEV_BOOTLOADER_IO_COMP == CyBtldr_SERIAL)
        #define CyBtldrCommStart    SERIAL_CyBtldrCommStart
        #define CyBtldrCommStop     SERIAL_CyBtldrCommStop
        #define CyBtldrCommReset    SERIAL_CyBtldrCommReset
        #define CyBtldrCommWrite    SERIAL_CyBtldrCommWrite
        #define CyBtldrCommRead     SERIAL_CyBtldrCommRead
    #endif /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_SERIAL) */

#endif /*defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_SERIAL) || \
                                                    (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface)) */

/* Interface to internal interrupt component */
#if(SERIAL_SCB_IRQ_INTERNAL)
    #define SERIAL_EnableInt()        CyIntEnable      (SERIAL_ISR_NUMBER)
    #define SERIAL_DisableInt()       CyIntDisable     (SERIAL_ISR_NUMBER)
    #define SERIAL_ClearPendingInt()  CyIntClearPending(SERIAL_ISR_NUMBER)
#endif /* (SERIAL_SCB_IRQ_INTERNAL) */

#if(SERIAL_UART_RX_WAKEUP_IRQ)
    #define SERIAL_RxWakeEnableInt()        CyIntEnable      (SERIAL_RX_WAKE_ISR_NUMBER)
    #define SERIAL_RxWakeDisableInt()       CyIntDisable     (SERIAL_RX_WAKE_ISR_NUMBER)
    #define SERIAL_RxWakeClearPendingInt()  CyIntClearPending(SERIAL_RX_WAKE_ISR_NUMBER)
#endif /* (SERIAL_UART_RX_WAKEUP_IRQ) */

/* Get interrupt cause */
#define SERIAL_GetInterruptCause()    (SERIAL_INTR_CAUSE_REG)

/* APIs to service INTR_RX register */
#define SERIAL_SetRxInterruptMode(interruptMask)     SERIAL_WRITE_INTR_RX_MASK(interruptMask)
#define SERIAL_ClearRxInterruptSource(interruptMask) SERIAL_CLEAR_INTR_RX(interruptMask)
#define SERIAL_SetRxInterrupt(interruptMask)         SERIAL_SET_INTR_RX(interruptMask)
#define SERIAL_GetRxInterruptSource()                (SERIAL_INTR_RX_REG)
#define SERIAL_GetRxInterruptMode()                  (SERIAL_INTR_RX_MASK_REG)
#define SERIAL_GetRxInterruptSourceMasked()          (SERIAL_INTR_RX_MASKED_REG)

/* APIs to service INTR_TX register */
#define SERIAL_SetTxInterruptMode(interruptMask)     SERIAL_WRITE_INTR_TX_MASK(interruptMask)
#define SERIAL_ClearTxInterruptSource(interruptMask) SERIAL_CLEAR_INTR_TX(interruptMask)
#define SERIAL_SetTxInterrupt(interruptMask)         SERIAL_SET_INTR_TX(interruptMask)
#define SERIAL_GetTxInterruptSource()                (SERIAL_INTR_TX_REG)
#define SERIAL_GetTxInterruptMode()                  (SERIAL_INTR_TX_MASK_REG)
#define SERIAL_GetTxInterruptSourceMasked()          (SERIAL_INTR_TX_MASKED_REG)

/* APIs to service INTR_MASTER register */
#define SERIAL_SetMasterInterruptMode(interruptMask)    SERIAL_WRITE_INTR_MASTER_MASK(interruptMask)
#define SERIAL_ClearMasterInterruptSource(interruptMask) SERIAL_CLEAR_INTR_MASTER(interruptMask)
#define SERIAL_SetMasterInterrupt(interruptMask)         SERIAL_SET_INTR_MASTER(interruptMask)
#define SERIAL_GetMasterInterruptSource()                (SERIAL_INTR_MASTER_REG)
#define SERIAL_GetMasterInterruptMode()                  (SERIAL_INTR_MASTER_MASK_REG)
#define SERIAL_GetMasterInterruptSourceMasked()          (SERIAL_INTR_MASTER_MASKED_REG)

/* APIs to service INTR_SLAVE register */
#define SERIAL_SetSlaveInterruptMode(interruptMask)     SERIAL_WRITE_INTR_SLAVE_MASK(interruptMask)
#define SERIAL_ClearSlaveInterruptSource(interruptMask) SERIAL_CLEAR_INTR_SLAVE(interruptMask)
#define SERIAL_SetSlaveInterrupt(interruptMask)         SERIAL_SET_INTR_SLAVE(interruptMask)
#define SERIAL_GetSlaveInterruptSource()                (SERIAL_INTR_SLAVE_REG)
#define SERIAL_GetSlaveInterruptMode()                  (SERIAL_INTR_SLAVE_MASK_REG)
#define SERIAL_GetSlaveInterruptSourceMasked()          (SERIAL_INTR_SLAVE_MASKED_REG)


/**********************************
*     Vars with External Linkage
**********************************/

extern uint8 SERIAL_initVar;


/***************************************
*              Registers
***************************************/

#define SERIAL_CTRL_REG               (*(reg32 *) SERIAL_SCB__CTRL)
#define SERIAL_CTRL_PTR               ( (reg32 *) SERIAL_SCB__CTRL)

#define SERIAL_STATUS_REG             (*(reg32 *) SERIAL_SCB__STATUS)
#define SERIAL_STATUS_PTR             ( (reg32 *) SERIAL_SCB__STATUS)

#if(!SERIAL_CY_SCBIP_V1_I2C_ONLY)
    #define SERIAL_SPI_CTRL_REG           (*(reg32 *) SERIAL_SCB__SPI_CTRL)
    #define SERIAL_SPI_CTRL_PTR           ( (reg32 *) SERIAL_SCB__SPI_CTRL)

    #define SERIAL_SPI_STATUS_REG         (*(reg32 *) SERIAL_SCB__SPI_STATUS)
    #define SERIAL_SPI_STATUS_PTR         ( (reg32 *) SERIAL_SCB__SPI_STATUS)

    #define SERIAL_UART_CTRL_REG          (*(reg32 *) SERIAL_SCB__UART_CTRL)
    #define SERIAL_UART_CTRL_PTR          ( (reg32 *) SERIAL_SCB__UART_CTRL)

    #define SERIAL_UART_TX_CTRL_REG       (*(reg32 *) SERIAL_SCB__UART_TX_CTRL)
    #define SERIAL_UART_TX_CTRL_PTR       ( (reg32 *) SERIAL_SCB__UART_RX_CTRL)

    #define SERIAL_UART_RX_CTRL_REG       (*(reg32 *) SERIAL_SCB__UART_RX_CTRL)
    #define SERIAL_UART_RX_CTRL_PTR       ( (reg32 *) SERIAL_SCB__UART_RX_CTRL)

    #define SERIAL_UART_RX_STATUS_REG     (*(reg32 *) SERIAL_SCB__UART_RX_STATUS)
    #define SERIAL_UART_RX_STATUS_PTR     ( (reg32 *) SERIAL_SCB__UART_RX_STATUS)
#endif /* (!SERIAL_CY_SCBIP_V1_I2C_ONLY) */

#define SERIAL_I2C_CTRL_REG           (*(reg32 *) SERIAL_SCB__I2C_CTRL)
#define SERIAL_I2C_CTRL_PTR           ( (reg32 *) SERIAL_SCB__I2C_CTRL)

#define SERIAL_I2C_STATUS_REG         (*(reg32 *) SERIAL_SCB__I2C_STATUS)
#define SERIAL_I2C_STATUS_PTR         ( (reg32 *) SERIAL_SCB__I2C_STATUS)

#define SERIAL_I2C_MASTER_CMD_REG     (*(reg32 *) SERIAL_SCB__I2C_M_CMD)
#define SERIAL_I2C_MASTER_CMD_PTR     ( (reg32 *) SERIAL_SCB__I2C_M_CMD)

#define SERIAL_I2C_SLAVE_CMD_REG      (*(reg32 *) SERIAL_SCB__I2C_S_CMD)
#define SERIAL_I2C_SLAVE_CMD_PTR      ( (reg32 *) SERIAL_SCB__I2C_S_CMD)

#define SERIAL_I2C_CFG_REG            (*(reg32 *) SERIAL_SCB__I2C_CFG)
#define SERIAL_I2C_CFG_PTR            ( (reg32 *) SERIAL_SCB__I2C_CFG)

#define SERIAL_TX_CTRL_REG            (*(reg32 *) SERIAL_SCB__TX_CTRL)
#define SERIAL_TX_CTRL_PTR            ( (reg32 *) SERIAL_SCB__TX_CTRL)

#define SERIAL_TX_FIFO_CTRL_REG       (*(reg32 *) SERIAL_SCB__TX_FIFO_CTRL)
#define SERIAL_TX_FIFO_CTRL_PTR       ( (reg32 *) SERIAL_SCB__TX_FIFO_CTRL)

#define SERIAL_TX_FIFO_STATUS_REG     (*(reg32 *) SERIAL_SCB__TX_FIFO_STATUS)
#define SERIAL_TX_FIFO_STATUS_PTR     ( (reg32 *) SERIAL_SCB__TX_FIFO_STATUS)

#define SERIAL_TX_FIFO_WR_REG         (*(reg32 *) SERIAL_SCB__TX_FIFO_WR)
#define SERIAL_TX_FIFO_WR_PTR         ( (reg32 *) SERIAL_SCB__TX_FIFO_WR)

#define SERIAL_RX_CTRL_REG            (*(reg32 *) SERIAL_SCB__RX_CTRL)
#define SERIAL_RX_CTRL_PTR            ( (reg32 *) SERIAL_SCB__RX_CTRL)

#define SERIAL_RX_FIFO_CTRL_REG       (*(reg32 *) SERIAL_SCB__RX_FIFO_CTRL)
#define SERIAL_RX_FIFO_CTRL_PTR       ( (reg32 *) SERIAL_SCB__RX_FIFO_CTRL)

#define SERIAL_RX_FIFO_STATUS_REG     (*(reg32 *) SERIAL_SCB__RX_FIFO_STATUS)
#define SERIAL_RX_FIFO_STATUS_PTR     ( (reg32 *) SERIAL_SCB__RX_FIFO_STATUS)

#define SERIAL_RX_MATCH_REG           (*(reg32 *) SERIAL_SCB__RX_MATCH)
#define SERIAL_RX_MATCH_PTR           ( (reg32 *) SERIAL_SCB__RX_MATCH)

#define SERIAL_RX_FIFO_RD_REG         (*(reg32 *) SERIAL_SCB__RX_FIFO_RD)
#define SERIAL_RX_FIFO_RD_PTR         ( (reg32 *) SERIAL_SCB__RX_FIFO_RD)

#define SERIAL_RX_FIFO_RD_SILENT_REG  (*(reg32 *) SERIAL_SCB__RX_FIFO_RD_SILENT)
#define SERIAL_RX_FIFO_RD_SILENT_PTR  ( (reg32 *) SERIAL_SCB__RX_FIFO_RD_SILENT)

#define SERIAL_EZBUF_DATA00_REG       (*(reg32 *) SERIAL_SCB__EZ_DATA00)
#define SERIAL_EZBUF_DATA00_PTR       ( (reg32 *) SERIAL_SCB__EZ_DATA00)

#define SERIAL_INTR_CAUSE_REG         (*(reg32 *) SERIAL_SCB__INTR_CAUSE)
#define SERIAL_INTR_CAUSE_PTR         ( (reg32 *) SERIAL_SCB__INTR_CAUSE)

#define SERIAL_INTR_I2C_EC_REG        (*(reg32 *) SERIAL_SCB__INTR_I2C_EC)
#define SERIAL_INTR_I2C_EC_PTR        ( (reg32 *) SERIAL_SCB__INTR_I2C_EC)

#define SERIAL_INTR_I2C_EC_MASK_REG   (*(reg32 *) SERIAL_SCB__INTR_I2C_EC_MASK)
#define SERIAL_INTR_I2C_EC_MASK_PTR   ( (reg32 *) SERIAL_SCB__INTR_I2C_EC_MASK)

#define SERIAL_INTR_I2C_EC_MASKED_REG (*(reg32 *) SERIAL_SCB__INTR_I2C_EC_MASKED)
#define SERIAL_INTR_I2C_EC_MASKED_PTR ( (reg32 *) SERIAL_SCB__INTR_I2C_EC_MASKED)

#if(!SERIAL_CY_SCBIP_V1_I2C_ONLY)
    #define SERIAL_INTR_SPI_EC_REG        (*(reg32 *) SERIAL_SCB__INTR_SPI_EC)
    #define SERIAL_INTR_SPI_EC_PTR        ( (reg32 *) SERIAL_SCB__INTR_SPI_EC)

    #define SERIAL_INTR_SPI_EC_MASK_REG   (*(reg32 *) SERIAL_SCB__INTR_SPI_EC_MASK)
    #define SERIAL_INTR_SPI_EC_MASK_PTR   ( (reg32 *) SERIAL_SCB__INTR_SPI_EC_MASK)

    #define SERIAL_INTR_SPI_EC_MASKED_REG (*(reg32 *) SERIAL_SCB__INTR_SPI_EC_MASKED)
    #define SERIAL_INTR_SPI_EC_MASKED_PTR ( (reg32 *) SERIAL_SCB__INTR_SPI_EC_MASKED)
#endif /* (!SERIAL_CY_SCBIP_V1_I2C_ONLY) */

#define SERIAL_INTR_MASTER_REG        (*(reg32 *) SERIAL_SCB__INTR_M)
#define SERIAL_INTR_MASTER_PTR        ( (reg32 *) SERIAL_SCB__INTR_M)

#define SERIAL_INTR_MASTER_SET_REG    (*(reg32 *) SERIAL_SCB__INTR_M_SET)
#define SERIAL_INTR_MASTER_SET_PTR    ( (reg32 *) SERIAL_SCB__INTR_M_SET)

#define SERIAL_INTR_MASTER_MASK_REG   (*(reg32 *) SERIAL_SCB__INTR_M_MASK)
#define SERIAL_INTR_MASTER_MASK_PTR   ( (reg32 *) SERIAL_SCB__INTR_M_MASK)

#define SERIAL_INTR_MASTER_MASKED_REG (*(reg32 *) SERIAL_SCB__INTR_M_MASKED)
#define SERIAL_INTR_MASTER_MASKED_PTR ( (reg32 *) SERIAL_SCB__INTR_M_MASKED)

#define SERIAL_INTR_SLAVE_REG         (*(reg32 *) SERIAL_SCB__INTR_S)
#define SERIAL_INTR_SLAVE_PTR         ( (reg32 *) SERIAL_SCB__INTR_S)

#define SERIAL_INTR_SLAVE_SET_REG     (*(reg32 *) SERIAL_SCB__INTR_S_SET)
#define SERIAL_INTR_SLAVE_SET_PTR     ( (reg32 *) SERIAL_SCB__INTR_S_SET)

#define SERIAL_INTR_SLAVE_MASK_REG    (*(reg32 *) SERIAL_SCB__INTR_S_MASK)
#define SERIAL_INTR_SLAVE_MASK_PTR    ( (reg32 *) SERIAL_SCB__INTR_S_MASK)

#define SERIAL_INTR_SLAVE_MASKED_REG  (*(reg32 *) SERIAL_SCB__INTR_S_MASKED)
#define SERIAL_INTR_SLAVE_MASKED_PTR  ( (reg32 *) SERIAL_SCB__INTR_S_MASKED)

#define SERIAL_INTR_TX_REG            (*(reg32 *) SERIAL_SCB__INTR_TX)
#define SERIAL_INTR_TX_PTR            ( (reg32 *) SERIAL_SCB__INTR_TX)

#define SERIAL_INTR_TX_SET_REG        (*(reg32 *) SERIAL_SCB__INTR_TX_SET)
#define SERIAL_INTR_TX_SET_PTR        ( (reg32 *) SERIAL_SCB__INTR_TX_SET)

#define SERIAL_INTR_TX_MASK_REG       (*(reg32 *) SERIAL_SCB__INTR_TX_MASK)
#define SERIAL_INTR_TX_MASK_PTR       ( (reg32 *) SERIAL_SCB__INTR_TX_MASK)

#define SERIAL_INTR_TX_MASKED_REG     (*(reg32 *) SERIAL_SCB__INTR_TX_MASKED)
#define SERIAL_INTR_TX_MASKED_PTR     ( (reg32 *) SERIAL_SCB__INTR_TX_MASKED)

#define SERIAL_INTR_RX_REG            (*(reg32 *) SERIAL_SCB__INTR_RX)
#define SERIAL_INTR_RX_PTR            ( (reg32 *) SERIAL_SCB__INTR_RX)

#define SERIAL_INTR_RX_SET_REG        (*(reg32 *) SERIAL_SCB__INTR_RX_SET)
#define SERIAL_INTR_RX_SET_PTR        ( (reg32 *) SERIAL_SCB__INTR_RX_SET)

#define SERIAL_INTR_RX_MASK_REG       (*(reg32 *) SERIAL_SCB__INTR_RX_MASK)
#define SERIAL_INTR_RX_MASK_PTR       ( (reg32 *) SERIAL_SCB__INTR_RX_MASK)

#define SERIAL_INTR_RX_MASKED_REG     (*(reg32 *) SERIAL_SCB__INTR_RX_MASKED)
#define SERIAL_INTR_RX_MASKED_PTR     ( (reg32 *) SERIAL_SCB__INTR_RX_MASKED)


/***************************************
*        Registers Constants
***************************************/

#if(SERIAL_SCB_IRQ_INTERNAL)
    #define SERIAL_ISR_NUMBER     ((uint8) SERIAL_SCB_IRQ__INTC_NUMBER)
    #define SERIAL_ISR_PRIORITY   ((uint8) SERIAL_SCB_IRQ__INTC_PRIOR_NUM)
#endif /* (SERIAL_SCB_IRQ_INTERNAL) */

#if(SERIAL_UART_RX_WAKEUP_IRQ)
    #define SERIAL_RX_WAKE_ISR_NUMBER     ((uint8) SERIAL_RX_WAKEUP_IRQ__INTC_NUMBER)
    #define SERIAL_RX_WAKE_ISR_PRIORITY   ((uint8) SERIAL_RX_WAKEUP_IRQ__INTC_PRIOR_NUM)
#endif /* (SERIAL_UART_RX_WAKEUP_IRQ) */

/* SERIAL_CTRL_REG */
#define SERIAL_CTRL_OVS_POS           (0u)  /* [3:0]   Oversampling factor                 */
#define SERIAL_CTRL_EC_AM_MODE_POS    (8u)  /* [8]     Externally clocked address match    */
#define SERIAL_CTRL_EC_OP_MODE_POS    (9u)  /* [9]     Externally clocked operation mode   */
#define SERIAL_CTRL_EZBUF_MODE_POS    (10u) /* [10]    EZ buffer is enabled                */
#define SERIAL_CTRL_ADDR_ACCEPT_POS   (16u) /* [16]    Put matched address in RX FIFO      */
#define SERIAL_CTRL_BLOCK_POS         (17u) /* [17]    Ext and Int logic to resolve colide */
#define SERIAL_CTRL_MODE_POS          (24u) /* [25:24] Operation mode                      */
#define SERIAL_CTRL_ENABLED_POS       (31u) /* [31]    Enable SCB block                    */
#define SERIAL_CTRL_OVS_MASK          ((uint32) 0x0Fu)
#define SERIAL_CTRL_EC_AM_MODE        ((uint32) 0x01u << SERIAL_CTRL_EC_AM_MODE_POS)
#define SERIAL_CTRL_EC_OP_MODE        ((uint32) 0x01u << SERIAL_CTRL_EC_OP_MODE_POS)
#define SERIAL_CTRL_EZBUF_MODE        ((uint32) 0x01u << SERIAL_CTRL_EZBUF_MODE_POS)
#define SERIAL_CTRL_ADDR_ACCEPT       ((uint32) 0x01u << SERIAL_CTRL_ADDR_ACCEPT_POS)
#define SERIAL_CTRL_BLOCK             ((uint32) 0x01u << SERIAL_CTRL_BLOCK_POS)
#define SERIAL_CTRL_MODE_MASK         ((uint32) 0x03u << SERIAL_CTRL_MODE_POS)
#define SERIAL_CTRL_MODE_I2C          ((uint32) 0x00u)
#define SERIAL_CTRL_MODE_SPI          ((uint32) 0x01u << SERIAL_CTRL_MODE_POS)
#define SERIAL_CTRL_MODE_UART         ((uint32) 0x02u << SERIAL_CTRL_MODE_POS)
#define SERIAL_CTRL_ENABLED           ((uint32) 0x01u << SERIAL_CTRL_ENABLED_POS)

/* SERIAL_STATUS_REG */
#define SERIAL_STATUS_EC_BUSY_POS     (0u)  /* [0] Bus busy. Externaly clocked loigc access to EZ memory */
#define SERIAL_STATUS_EC_BUSY         ((uint32) 0x0Fu)

/* SERIAL_SPI_CTRL_REG  */
#define SERIAL_SPI_CTRL_CONTINUOUS_POS        (0u)  /* [0]     Continuous or Separated SPI data transfers */
#define SERIAL_SPI_CTRL_SELECT_PRECEDE_POS    (1u)  /* [1]     Precedes or coincides start of data frame  */
#define SERIAL_SPI_CTRL_CPHA_POS              (2u)  /* [2]     SCLK phase                                 */
#define SERIAL_SPI_CTRL_CPOL_POS              (3u)  /* [3]     SCLK polarity                              */
#define SERIAL_SPI_CTRL_LATE_MISO_SAMPLE_POS  (4u)  /* [4]     Late MISO sample enabled                   */
#define SERIAL_SPI_CTRL_LOOPBACK_POS          (16u) /* [16]    Local loopback control enabled             */
#define SERIAL_SPI_CTRL_MODE_POS              (24u) /* [25:24] Submode of SPI operation                   */
#define SERIAL_SPI_CTRL_SLAVE_SELECT_POS      (26u) /* [27:26] Selects SPI SS signal                      */
#define SERIAL_SPI_CTRL_MASTER_MODE_POS       (31u) /* [31]    Master mode enabled                        */
#define SERIAL_SPI_CTRL_CONTINUOUS            ((uint32) 0x01u)
#define SERIAL_SPI_CTRL_SELECT_PRECEDE        ((uint32) 0x01u << SERIAL_SPI_CTRL_SELECT_PRECEDE_POS)
#define SERIAL_SPI_CTRL_SCLK_MODE_MASK        ((uint32) 0x03u << SERIAL_SPI_CTRL_CPHA_POS)
#define SERIAL_SPI_CTRL_CPHA                  ((uint32) 0x01u << SERIAL_SPI_CTRL_CPHA_POS)
#define SERIAL_SPI_CTRL_CPOL                  ((uint32) 0x01u << SERIAL_SPI_CTRL_CPOL_POS)
#define SERIAL_SPI_CTRL_LATE_MISO_SAMPLE      ((uint32) 0x01u << \
                                                                    SERIAL_SPI_CTRL_LATE_MISO_SAMPLE_POS)
#define SERIAL_SPI_CTRL_LOOPBACK              ((uint32) 0x01u << SERIAL_SPI_CTRL_LOOPBACK_POS)
#define SERIAL_SPI_CTRL_MODE_MASK             ((uint32) 0x03u << SERIAL_SPI_CTRL_MODE_POS)
#define SERIAL_SPI_CTRL_MODE_MOTOROLA         ((uint32) 0x00u)
#define SERIAL_SPI_CTRL_MODE_TI               ((uint32) 0x01u << SERIAL_CTRL_MODE_POS)
#define SERIAL_SPI_CTRL_MODE_NS               ((uint32) 0x02u << SERIAL_CTRL_MODE_POS)
#define SERIAL_SPI_CTRL_SLAVE_SELECT_MASK     ((uint32) 0x03u << SERIAL_SPI_CTRL_SLAVE_SELECT_POS)
#define SERIAL_SPI_CTRL_SLAVE_SELECT0         ((uint32) 0x00u)
#define SERIAL_SPI_CTRL_SLAVE_SELECT1         ((uint32) 0x01u << SERIAL_SPI_CTRL_SLAVE_SELECT_POS)
#define SERIAL_SPI_CTRL_SLAVE_SELECT2         ((uint32) 0x02u << SERIAL_SPI_CTRL_SLAVE_SELECT_POS)
#define SERIAL_SPI_CTRL_SLAVE_SELECT3         ((uint32) 0x03u << SERIAL_SPI_CTRL_SLAVE_SELECT_POS)
#define SERIAL_SPI_CTRL_MASTER                ((uint32) 0x01u << SERIAL_SPI_CTRL_MASTER_MODE_POS)
#define SERIAL_SPI_CTRL_SLAVE                 ((uint32) 0x00u)

/* SERIAL_SPI_STATUS_REG  */
#define SERIAL_SPI_STATUS_BUS_BUSY_POS    (0u)  /* [0]    Bus busy - slave selected */
#define SERIAL_SPI_STATUS_EZBUF_ADDR_POS  (8u)  /* [15:8] EzAddress                 */
#define SERIAL_SPI_STATUS_BUS_BUSY        ((uint32) 0x01u)
#define SERIAL_SPI_STATUS_EZBUF_ADDR_MASK ((uint32) 0xFFu << SERIAL_I2C_STATUS_EZBUF_ADDR_POS)

/* SERIAL_UART_CTRL */
#define SERIAL_UART_CTRL_LOOPBACK_POS         (16u) /* [16] Loopback     */
#define SERIAL_UART_CTRL_MODE_POS             (24u) /* [24] UART subMode */
#define SERIAL_UART_CTRL_LOOPBACK             ((uint32) 0x01u << SERIAL_UART_CTRL_LOOPBACK_POS)
#define SERIAL_UART_CTRL_MODE_UART_STD        ((uint32) 0x00u)
#define SERIAL_UART_CTRL_MODE_UART_SMARTCARD  ((uint32) 0x01u << SERIAL_UART_CTRL_MODE_POS)
#define SERIAL_UART_CTRL_MODE_UART_IRDA       ((uint32) 0x02u << SERIAL_UART_CTRL_MODE_POS)
#define SERIAL_UART_CTRL_MODE_MASK            ((uint32) 0x03u << SERIAL_UART_CTRL_MODE_POS)

/* SERIAL_UART_TX_CTRL */
#define SERIAL_UART_TX_CTRL_STOP_BITS_POS         (0u)  /* [2:0] Stop bits: (Stop bits + 1) * 0.5 period */
#define SERIAL_UART_TX_CTRL_PARITY_POS            (4u)  /* [4]   Parity bit                              */
#define SERIAL_UART_TX_CTRL_PARITY_ENABLED_POS    (5u)  /* [5]   Parity enable                           */
#define SERIAL_UART_TX_CTRL_RETRY_ON_NACK_POS     (8u)  /* [8]   Smart Card: re-send frame on NACK       */
#define SERIAL_UART_TX_CTRL_ONE_STOP_BIT          ((uint32) 0x01u)
#define SERIAL_UART_TX_CTRL_ONE_HALF_STOP_BITS    ((uint32) 0x02u)
#define SERIAL_UART_TX_CTRL_TWO_STOP_BITS         ((uint32) 0x03u)
#define SERIAL_UART_TX_CTRL_STOP_BITS_MASK        ((uint32) 0x07u)
#define SERIAL_UART_TX_CTRL_PARITY                ((uint32) 0x01u << \
                                                                    SERIAL_UART_TX_CTRL_PARITY_POS)
#define SERIAL_UART_TX_CTRL_PARITY_ENABLED        ((uint32) 0x01u << \
                                                                    SERIAL_UART_TX_CTRL_PARITY_ENABLED_POS)
#define SERIAL_UART_TX_CTRL_RETRY_ON_NACK         ((uint32) 0x01u << \
                                                                    SERIAL_UART_TX_CTRL_RETRY_ON_NACK_POS)

/* SERIAL_UART_RX_CTRL */
#define SERIAL_UART_RX_CTRL_STOP_BITS_POS             (0u)  /* [2:0] Stop bits: (Stop bits + 1) * 0.5 prd   */
#define SERIAL_UART_RX_CTRL_PARITY_POS                (4u)  /* [4]   Parity bit                             */
#define SERIAL_UART_RX_CTRL_PARITY_ENABLED_POS        (5u)  /* [5]   Parity enable                          */
#define SERIAL_UART_RX_CTRL_POLARITY_POS              (6u)  /* [6]   IrDA: inverts polarity of RX signal    */
#define SERIAL_UART_RX_CTRL_DROP_ON_PARITY_ERR_POS    (8u)  /* [8]   Drop and lost RX FIFO on parity error  */
#define SERIAL_UART_RX_CTRL_DROP_ON_FRAME_ERR_POS     (9u)  /* [9]   Drop and lost RX FIFO on frame error   */
#define SERIAL_UART_RX_CTRL_MP_MODE_POS               (10u) /* [10]  Multi-processor mode                   */
#define SERIAL_UART_RX_CTRL_LIN_MODE_POS              (12u) /* [12]  Lin mode: applicable for UART Standart */
#define SERIAL_UART_RX_CTRL_SKIP_START_POS            (13u) /* [13]  Skip start not: only for UART Standart */
#define SERIAL_UART_RX_CTRL_BREAK_WIDTH_POS           (16u) /* [19:16]  Break width: (Break width + 1)      */
#define SERIAL_UART_TX_CTRL_ONE_STOP_BIT              ((uint32) 0x01u)
#define SERIAL_UART_TX_CTRL_ONE_HALF_STOP_BITS        ((uint32) 0x02u)
#define SERIAL_UART_TX_CTRL_TWO_STOP_BITS             ((uint32) 0x03u)
#define SERIAL_UART_RX_CTRL_STOP_BITS_MASK            ((uint32) 0x07u)
#define SERIAL_UART_RX_CTRL_PARITY                    ((uint32) 0x01u << \
                                                                    SERIAL_UART_RX_CTRL_PARITY_POS)
#define SERIAL_UART_RX_CTRL_PARITY_ENABLED            ((uint32) 0x01u << \
                                                                    SERIAL_UART_RX_CTRL_PARITY_ENABLED_POS)
#define SERIAL_UART_RX_CTRL_POLARITY                  ((uint32) 0x01u << \
                                                                    SERIAL_UART_RX_CTRL_POLARITY_POS)
#define SERIAL_UART_RX_CTRL_DROP_ON_PARITY_ERR        ((uint32) 0x01u << \
                                                                   SERIAL_UART_RX_CTRL_DROP_ON_PARITY_ERR_POS)
#define SERIAL_UART_RX_CTRL_DROP_ON_FRAME_ERR         ((uint32) 0x01u << \
                                                                    SERIAL_UART_RX_CTRL_DROP_ON_FRAME_ERR_POS)
#define SERIAL_UART_RX_CTRL_MP_MODE                   ((uint32) 0x01u << \
                                                                    SERIAL_UART_RX_CTRL_MP_MODE_POS)
#define SERIAL_UART_RX_CTRL_LIN_MODE                  ((uint32) 0x01u << \
                                                                    SERIAL_UART_RX_CTRL_LIN_MODE_POS)
#define SERIAL_UART_RX_CTRL_SKIP_START                ((uint32) 0x01u << \
                                                                    SERIAL_UART_RX_CTRL_SKIP_START_POS)
#define SERIAL_UART_RX_CTRL_BREAK_WIDTH_MASK          ((uint32) 0x0Fu << \
                                                                    SERIAL_UART_RX_CTRL_BREAK_WIDTH_POS)
/* SERIAL_UART_RX_STATUS_REG */
#define SERIAL_UART_RX_STATUS_BR_COUNTER_POS     (0u)  /* [11:0] Baute Rate counter */
#define SERIAL_UART_RX_STATUS_BR_COUNTER_MASK    ((uint32) 0xFFFu)

/* SERIAL_I2C_CTRL */
#define SERIAL_I2C_CTRL_HIGH_PHASE_OVS_POS           (0u)   /* [3:0] Oversampling factor high: masrer only */
#define SERIAL_I2C_CTRL_LOW_PHASE_OVS_POS            (4u)   /* [7:4] Oversampling factor low:  masrer only */
#define SERIAL_I2C_CTRL_M_READY_DATA_ACK_POS         (8u)   /* [8]   Master ACKs data wgile RX FIFO != FULL*/
#define SERIAL_I2C_CTRL_M_NOT_READY_DATA_NACK_POS    (9u)   /* [9]   Master NACKs data if RX FIFO ==  FULL */
#define SERIAL_I2C_CTRL_S_GENERAL_IGNORE_POS         (11u)  /* [11]  Slave ignores General call            */
#define SERIAL_I2C_CTRL_S_READY_ADDR_ACK_POS         (12u)  /* [12]  Slave ACKs Address if RX FIFO != FULL */
#define SERIAL_I2C_CTRL_S_READY_DATA_ACK_POS         (13u)  /* [13]  Slave ACKs data while RX FIFO == FULL */
#define SERIAL_I2C_CTRL_S_NOT_READY_ADDR_NACK_POS    (14u)  /* [14]  Slave NACKs address if RX FIFO == FULL*/
#define SERIAL_I2C_CTRL_S_NOT_READY_DATA_NACK_POS    (15u)  /* [15]  Slave NACKs data if RX FIFO is  FULL  */
#define SERIAL_I2C_CTRL_LOOPBACK_POS                 (16u)  /* [16]  Loopback                              */
#define SERIAL_I2C_CTRL_SLAVE_MODE_POS               (30u)  /* [30]  Slave mode enabled                    */
#define SERIAL_I2C_CTRL_MASTER_MODE_POS              (31u)  /* [31]  Master mode enabled                   */
#define SERIAL_I2C_CTRL_HIGH_PHASE_OVS_MASK  ((uint32) 0x0Fu)
#define SERIAL_I2C_CTRL_LOW_PHASE_OVS_MASK   ((uint32) 0x0Fu << \
                                                                SERIAL_I2C_CTRL_LOW_PHASE_OVS_POS)
#define SERIAL_I2C_CTRL_M_READY_DATA_ACK      ((uint32) 0x01u << \
                                                                SERIAL_I2C_CTRL_M_READY_DATA_ACK_POS)
#define SERIAL_I2C_CTRL_M_NOT_READY_DATA_NACK ((uint32) 0x01u << \
                                                                SERIAL_I2C_CTRL_M_NOT_READY_DATA_NACK_POS)
#define SERIAL_I2C_CTRL_S_GENERAL_IGNORE      ((uint32) 0x01u << \
                                                                SERIAL_I2C_CTRL_S_GENERAL_IGNORE_POS)
#define SERIAL_I2C_CTRL_S_READY_ADDR_ACK      ((uint32) 0x01u << \
                                                                SERIAL_I2C_CTRL_S_READY_ADDR_ACK_POS)
#define SERIAL_I2C_CTRL_S_READY_DATA_ACK      ((uint32) 0x01u << \
                                                                SERIAL_I2C_CTRL_S_READY_DATA_ACK_POS)
#define SERIAL_I2C_CTRL_S_NOT_READY_ADDR_NACK ((uint32) 0x01u << \
                                                                SERIAL_I2C_CTRL_S_NOT_READY_ADDR_NACK_POS)
#define SERIAL_I2C_CTRL_S_NOT_READY_DATA_NACK ((uint32) 0x01u << \
                                                                SERIAL_I2C_CTRL_S_NOT_READY_DATA_NACK_POS)
#define SERIAL_I2C_CTRL_LOOPBACK              ((uint32) 0x01u << \
                                                                SERIAL_I2C_CTRL_LOOPBACK_POS)
#define SERIAL_I2C_CTRL_SLAVE_MODE            ((uint32) 0x01u << \
                                                                SERIAL_I2C_CTRL_SLAVE_MODE_POS)
#define SERIAL_I2C_CTRL_MASTER_MODE           ((uint32) 0x01u << \
                                                                SERIAL_I2C_CTRL_MASTER_MODE_POS)
#define SERIAL_I2C_CTRL_SLAVE_MASTER_MODE_MASK    ((uint32) 0x03u << \
                                                                SERIAL_I2C_CTRL_SLAVE_MODE_POS)

/* SERIAL_I2C_STATUS_REG  */
#define SERIAL_I2C_STATUS_BUS_BUSY_POS    (0u)  /* [0]    Bus busy: internally clocked */
#define SERIAL_I2C_STATUS_S_READ_POS      (4u)  /* [4]    Slave is read by master      */
#define SERIAL_I2C_STATUS_M_READ_POS      (5u)  /* [5]    Master reads Slave           */
#define SERIAL_I2C_STATUS_EZBUF_ADDR_POS  (8u)  /* [15:8] EZAddress                    */
#define SERIAL_I2C_STATUS_BUS_BUSY        ((uint32) 0x01u)
#define SERIAL_I2C_STATUS_S_READ          ((uint32) 0x01u << SERIAL_I2C_STATUS_S_READ_POS)
#define SERIAL_I2C_STATUS_M_READ          ((uint32) 0x01u << SERIAL_I2C_STATUS_M_READ_POS)
#define SERIAL_I2C_STATUS_EZBUF_ADDR_MASK ((uint32) 0xFFu << SERIAL_I2C_STATUS_EZBUF_ADDR_POS)

/* SERIAL_I2C_MASTER_CMD_REG */
#define SERIAL_I2C_MASTER_CMD_M_START_POS             (0u)  /* [0] Master generate Start                */
#define SERIAL_I2C_MASTER_CMD_M_START_ON_IDLE_POS     (1u)  /* [1] Master generate Start if bus is free */
#define SERIAL_I2C_MASTER_CMD_M_ACK_POS               (2u)  /* [2] Master generate ACK                  */
#define SERIAL_I2C_MASTER_CMD_M_NACK_POS              (3u)  /* [3] Master generate NACK                 */
#define SERIAL_I2C_MASTER_CMD_M_STOP_POS              (4u)  /* [4] Master generate Stop                 */
#define SERIAL_I2C_MASTER_CMD_M_START         ((uint32) 0x01u)
#define SERIAL_I2C_MASTER_CMD_M_START_ON_IDLE ((uint32) 0x01u << \
                                                                   SERIAL_I2C_MASTER_CMD_M_START_ON_IDLE_POS)
#define SERIAL_I2C_MASTER_CMD_M_ACK           ((uint32) 0x01u << \
                                                                   SERIAL_I2C_MASTER_CMD_M_ACK_POS)
#define SERIAL_I2C_MASTER_CMD_M_NACK          ((uint32) 0x01u << \
                                                                    SERIAL_I2C_MASTER_CMD_M_NACK_POS)
#define SERIAL_I2C_MASTER_CMD_M_STOP          ((uint32) 0x01u << \
                                                                    SERIAL_I2C_MASTER_CMD_M_STOP_POS)

/* SERIAL_I2C_SLAVE_CMD_REG  */
#define SERIAL_I2C_SLAVE_CMD_S_ACK_POS    (0u)  /* [0] Slave generate ACK  */
#define SERIAL_I2C_SLAVE_CMD_S_NACK_POS   (1u)  /* [1] Slave generate NACK */
#define SERIAL_I2C_SLAVE_CMD_S_ACK        ((uint32) 0x01u)
#define SERIAL_I2C_SLAVE_CMD_S_NACK       ((uint32) 0x01u << SERIAL_I2C_SLAVE_CMD_S_NACK_POS)

#define SERIAL_I2C_SLAVE_CMD_S_ACK_POS    (0u)  /* [0] Slave generate ACK  */
#define SERIAL_I2C_SLAVE_CMD_S_NACK_POS   (1u)  /* [1] Slave generate NACK */
#define SERIAL_I2C_SLAVE_CMD_S_ACK        ((uint32) 0x01u)
#define SERIAL_I2C_SLAVE_CMD_S_NACK       ((uint32) 0x01u << SERIAL_I2C_SLAVE_CMD_S_NACK_POS)
/* SERIAL_I2C_CFG  */
#define SERIAL_I2C_CFG_SDA_FILT_HYS_POS           (0u)  /* [1:0]   Trim bits for the I2C SDA filter         */
#define SERIAL_I2C_CFG_SDA_FILT_TRIM_POS          (2u)  /* [3:2]   Trim bits for the I2C SDA filter         */
#define SERIAL_I2C_CFG_SCL_FILT_HYS_POS           (4u)  /* [5:4]   Trim bits for the I2C SCL filter         */
#define SERIAL_I2C_CFG_SCL_FILT_TRIM_POS          (6u)  /* [7:6]   Trim bits for the I2C SCL filter         */
#define SERIAL_I2C_CFG_SDA_FILT_OUT_HYS_POS       (8u)  /* [9:8]   Trim bits for I2C SDA filter output path */
#define SERIAL_I2C_CFG_SDA_FILT_OUT_TRIM_POS      (10u) /* [11:10] Trim bits for I2C SDA filter output path */
#define SERIAL_I2C_CFG_SDA_FILT_HS_POS            (16u) /* [16]    '0': 50 ns filter, '1': 10 ns filter     */
#define SERIAL_I2C_CFG_SDA_FILT_ENABLED_POS       (17u) /* [17]    I2C SDA filter enabled                   */
#define SERIAL_I2C_CFG_SCL_FILT_HS_POS            (24u) /* [24]    '0': 50 ns filter, '1': 10 ns filter     */
#define SERIAL_I2C_CFG_SCL_FILT_ENABLED_POS       (25u) /* [25]    I2C SCL filter enabled                   */
#define SERIAL_I2C_CFG_SDA_FILT_OUT_HS_POS        (26u) /* [26]    '0': 50ns filter, '1': 10 ns filter      */
#define SERIAL_I2C_CFG_SDA_FILT_OUT_ENABLED_POS   (27u) /* [27]    I2C SDA output delay filter enabled      */
#define SERIAL_I2C_CFG_SDA_FILT_HYS_MASK          ((uint32) 0x00u)
#define SERIAL_I2C_CFG_SDA_FILT_TRIM_MASK         ((uint32) 0x03u << \
                                                                SERIAL_I2C_CFG_SDA_FILT_TRIM_POS)
#define SERIAL_I2C_CFG_SCL_FILT_HYS_MASK          ((uint32) 0x03u << \
                                                                SERIAL_I2C_CFG_SCL_FILT_HYS_POS)
#define SERIAL_I2C_CFG_SCL_FILT_TRIM_MASK         ((uint32) 0x03u << \
                                                                SERIAL_I2C_CFG_SCL_FILT_TRIM_POS)
#define SERIAL_I2C_CFG_SDA_FILT_OUT_HYS_MASK      ((uint32) 0x03u << \
                                                                SERIAL_I2C_CFG_SDA_FILT_OUT_HYS_POS)
#define SERIAL_I2C_CFG_SDA_FILT_OUT_TRIM_MASK     ((uint32) 0x03u << \
                                                                SERIAL_I2C_CFG_SDA_FILT_OUT_TRIM_POS)
#define SERIAL_I2C_CFG_SDA_FILT_HS                ((uint32) 0x01u << \
                                                                SERIAL_I2C_CFG_SDA_FILT_HS_POS)
#define SERIAL_I2C_CFG_SDA_FILT_ENABLED           ((uint32) 0x01u << \
                                                                SERIAL_I2C_CFG_SDA_FILT_ENABLED_POS)
#define SERIAL_I2C_CFG_SCL_FILT_HS                ((uint32) 0x01u << \
                                                                SERIAL_I2C_CFG_SCL_FILT_HS_POS)
#define SERIAL_I2C_CFG_SCL_FILT_ENABLED           ((uint32) 0x01u << \
                                                                SERIAL_I2C_CFG_SCL_FILT_ENABLED_POS)
#define SERIAL_I2C_CFG_SDA_FILT_OUT_HS            ((uint32) 0x01u << \
                                                                SERIAL_I2C_CFG_SDA_FILT_OUT_HS_POS)
#define SERIAL_I2C_CFG_SDA_FILT_OUT_ENABLED       ((uint32) 0x01u << \
                                                                SERIAL_I2C_CFG_SDA_FILT_OUT_ENABLED_POS)

/* SERIAL_TX_CTRL_REG */
#define SERIAL_TX_CTRL_DATA_WIDTH_POS     (0u)  /* [3:0] Dataframe width: (Data width - 1) */
#define SERIAL_TX_CTRL_MSB_FIRST_POS      (8u)  /* [8]   MSB first shifter-out             */
#define SERIAL_TX_CTRL_ENABLED_POS        (31u) /* [31]  Transmitter enabled               */
#define SERIAL_TX_CTRL_DATA_WIDTH_MASK    ((uint32) 0x0Fu)
#define SERIAL_TX_CTRL_MSB_FIRST          ((uint32) 0x01u << SERIAL_TX_CTRL_MSB_FIRST_POS)
#define SERIAL_TX_CTRL_LSB_FIRST          ((uint32) 0x00u)
#define SERIAL_TX_CTRL_ENABLED            ((uint32) 0x01u << SERIAL_TX_CTRL_ENABLED_POS)

/* SERIAL_TX_CTRL_FIFO_REG */
#define SERIAL_TX_FIFO_CTRL_TRIGGER_LEVEL_POS     (0u)  /* [2:0] Trigger level                              */
#define SERIAL_TX_FIFO_CTRL_CLEAR_POS             (16u) /* [16]  Clear TX FIFO: claared after set           */
#define SERIAL_TX_FIFO_CTRL_FREEZE_POS            (17u) /* [17]  Freeze TX FIFO: HW do not inc read pointer */
#define SERIAL_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK    ((uint32) 0x07u)
#define SERIAL_TX_FIFO_CTRL_CLEAR                 ((uint32) 0x01u << SERIAL_TX_FIFO_CTRL_CLEAR_POS)
#define SERIAL_TX_FIFO_CTRL_FREEZE                ((uint32) 0x01u << SERIAL_TX_FIFO_CTRL_FREEZE_POS)

/* SERIAL_TX_FIFO_STATUS_REG */
#define SERIAL_TX_FIFO_STATUS_USED_POS    (0u)  /* [3:0]   Amount of entries in TX FIFO */
#define SERIAL_TX_FIFO_SR_VALID_POS       (15u) /* [15]    Shifter status of TX FIFO    */
#define SERIAL_TX_FIFO_STATUS_RD_PTR_POS  (16u) /* [18:16] TX FIFO read pointer         */
#define SERIAL_TX_FIFO_STATUS_WR_PTR_POS  (24u) /* [26:24] TX FIFO write pointer        */
#define SERIAL_TX_FIFO_STATUS_USED_MASK   ((uint32) 0x0Fu)
#define SERIAL_TX_FIFO_SR_VALID           ((uint32) 0x01u << SERIAL_TX_FIFO_SR_VALID_POS)
#define SERIAL_TX_FIFO_STATUS_RD_PTR_MASK ((uint32) 0x07u << SERIAL_TX_FIFO_STATUS_RD_PTR_POS)
#define SERIAL_TX_FIFO_STATUS_WR_PTR_MASK ((uint32) 0x07u << SERIAL_TX_FIFO_STATUS_WR_PTR_POS)

/* SERIAL_TX_FIFO_WR_REG */
#define SERIAL_TX_FIFO_WR_POS    (0u)  /* [15:0] Data written into TX FIFO */
#define SERIAL_TX_FIFO_WR_MASK   ((uint32) 0xFFu)

/* SERIAL_RX_CTRL_REG */
#define SERIAL_RX_CTRL_DATA_WIDTH_POS     (0u)  /* [3:0] Dataframe width: (Data width - 1) */
#define SERIAL_RX_CTRL_MSB_FIRST_POS      (8u)  /* [8]   MSB first shifter-out             */
#define SERIAL_RX_CTRL_MEDIAN_POS         (9u)  /* [9]   Median filter                     */
#define SERIAL_RX_CTRL_ENABLED_POS        (31u) /* [31]  Receiver enabled                  */
#define SERIAL_RX_CTRL_DATA_WIDTH_MASK    ((uint32) 0x0Fu)
#define SERIAL_RX_CTRL_MSB_FIRST          ((uint32) 0x01u << SERIAL_RX_CTRL_MSB_FIRST_POS)
#define SERIAL_RX_CTRL_LSB_FIRST          ((uint32) 0x00u)
#define SERIAL_RX_CTRL_MEDIAN             ((uint32) 0x01u << SERIAL_RX_CTRL_MEDIAN_POS)
#define SERIAL_RX_CTRL_ENABLED            ((uint32) 0x01u << SERIAL_RX_CTRL_ENABLED_POS)


/* SERIAL_RX_FIFO_CTRL_REG */
#define SERIAL_RX_FIFO_CTRL_TRIGGER_LEVEL_POS     (0u)   /* [2:0] Trigger level                            */
#define SERIAL_RX_FIFO_CTRL_CLEAR_POS             (16u)  /* [16]  Clear RX FIFO: claar after set           */
#define SERIAL_RX_FIFO_CTRL_FREEZE_POS            (17u)  /* [17]  Freeze RX FIFO: HW writes has not effect */
#define SERIAL_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK    ((uint32) 0x07u)
#define SERIAL_RX_FIFO_CTRL_CLEAR                 ((uint32) 0x01u << SERIAL_RX_FIFO_CTRL_CLEAR_POS)
#define SERIAL_RX_FIFO_CTRL_FREEZE                ((uint32) 0x01u << SERIAL_RX_FIFO_CTRL_FREEZE_POS)

/* SERIAL_RX_FIFO_STATUS_REG */
#define SERIAL_RX_FIFO_STATUS_USED_POS    (0u)   /* [3:0]   Amount of entries in RX FIFO */
#define SERIAL_RX_FIFO_SR_VALID_POS       (15u)  /* [15]    Shifter status of RX FIFO    */
#define SERIAL_RX_FIFO_STATUS_RD_PTR_POS  (16u)  /* [18:16] RX FIFO read pointer         */
#define SERIAL_RX_FIFO_STATUS_WR_PTR_POS  (24u)  /* [26:24] RX FIFO write pointer        */
#define SERIAL_RX_FIFO_STATUS_USED_MASK   ((uint32) 0x0Fu)
#define SERIAL_RX_FIFO_SR_VALID           ((uint32) 0x01u << SERIAL_RX_FIFO_SR_VALID_POS)
#define SERIAL_RX_FIFO_STATUS_RD_PTR_MASK ((uint32) 0x07u << SERIAL_RX_FIFO_STATUS_RD_PTR_POS)
#define SERIAL_RX_FIFO_STATUS_WR_PTR_MASK ((uint32) 0x07u << SERIAL_RX_FIFO_STATUS_WR_PTR_POS)

/* SERIAL_RX_MATCH_REG */
#define SERIAL_RX_MATCH_ADDR_POS     (0u)  /* [7:0]   Slave address                        */
#define SERIAL_RX_MATCH_MASK_POS     (16u) /* [23:16] Slave address mask: 0 - doesn't care */
#define SERIAL_RX_MATCH_ADDR_MASK    ((uint32) 0xFFu)
#define SERIAL_RX_MATCH_MASK_MASK    ((uint32) 0xFFu << SERIAL_RX_MATCH_MASK_POS)

/* SERIAL_RX_FIFO_WR_REG */
#define SERIAL_RX_FIFO_RD_POS    (0u)  /* [15:0] Data read from RX FIFO */
#define SERIAL_RX_FIFO_RD_MASK   ((uint32) 0xFFu)

/* SERIAL_RX_FIFO_RD_SILENT_REG */
#define SERIAL_RX_FIFO_RD_SILENT_POS     (0u)  /* [15:0] Data read from RX FIFO: not remove data from FIFO */
#define SERIAL_RX_FIFO_RD_SILENT_MASK    ((uint32) 0xFFu)

/* SERIAL_RX_FIFO_RD_SILENT_REG */
#define SERIAL_RX_FIFO_RD_SILENT_POS     (0u)  /* [15:0] Data read from RX FIFO: not remove data from FIFO */
#define SERIAL_RX_FIFO_RD_SILENT_MASK    ((uint32) 0xFFu)

/* SERIAL_EZBUF_DATA_REG */
#define SERIAL_EZBUF_DATA_POS   (0u)  /* [7:0] Data from Ez Memory */
#define SERIAL_EZBUF_DATA_MASK  ((uint32) 0xFFu)

/*  SERIAL_INTR_CAUSE_REG */
#define SERIAL_INTR_CAUSE_MASTER_POS  (0u)  /* [0] Master interrupt active                 */
#define SERIAL_INTR_CAUSE_SLAVE_POS   (1u)  /* [1] Slave interrupt active                  */
#define SERIAL_INTR_CAUSE_TX_POS      (2u)  /* [2] Transmitter interrupt active            */
#define SERIAL_INTR_CAUSE_RX_POS      (3u)  /* [3] Receiver interrupt active               */
#define SERIAL_INTR_CAUSE_I2C_EC_POS  (4u)  /* [4] Externally clock I2C interrupt active   */
#define SERIAL_INTR_CAUSE_SPI_EC_POS  (5u)  /* [5] Externally clocked SPI interrupt active */
#define SERIAL_INTR_CAUSE_MASTER      ((uint32) 0x01u)
#define SERIAL_INTR_CAUSE_SLAVE       ((uint32) 0x01u << SERIAL_INTR_CAUSE_SLAVE_POS)
#define SERIAL_INTR_CAUSE_TX          ((uint32) 0x01u << SERIAL_INTR_CAUSE_TX_POS)
#define SERIAL_INTR_CAUSE_RX          ((uint32) 0x01u << SERIAL_INTR_CAUSE_RX_POS)
#define SERIAL_INTR_CAUSE_I2C_EC      ((uint32) 0x01u << SERIAL_INTR_CAUSE_I2C_EC_POS)
#define SERIAL_INTR_CAUSE_SPI_EC      ((uint32) 0x01u << SERIAL_INTR_CAUSE_SPI_EC_POS)

/* SERIAL_INTR_SPI_EC_REG, SERIAL_INTR_SPI_EC_MASK_REG, SERIAL_INTR_SPI_EC_MASKED_REG */
#define SERIAL_INTR_SPI_EC_WAKE_UP_POS          (0u)  /* [0] Address match: triggers wakeup of chip */
#define SERIAL_INTR_SPI_EC_EZBUF_STOP_POS       (1u)  /* [1] Externally clocked Stop detected       */
#define SERIAL_INTR_SPI_EC_EZBUF_WRITE_STOP_POS (2u)  /* [2] Externally clocked Write Stop detected */
#define SERIAL_INTR_SPI_EC_WAKE_UP              ((uint32) 0x01u)
#define SERIAL_INTR_SPI_EC_EZBUF_STOP           ((uint32) 0x01u << \
                                                                    SERIAL_INTR_SPI_EC_EZBUF_STOP_POS)
#define SERIAL_INTR_SPI_EC_EZBUF_WRITE_STOP     ((uint32) 0x01u << \
                                                                    SERIAL_INTR_SPI_EC_EZBUF_WRITE_STOP_POS)

/* SERIAL_INTR_I2C_EC, SERIAL_INTR_I2C_EC_MASK, SERIAL_INTR_I2C_EC_MASKED */
#define SERIAL_INTR_I2C_EC_WAKE_UP_POS          (0u)  /* [0] Address match: triggers wakeup of chip */
#define SERIAL_INTR_I2C_EC_EZBUF_STOP_POS       (1u)  /* [1] Externally clocked Stop detected       */
#define SERIAL_INTR_I2C_EC_EZBUF_WRITE_STOP_POS (2u)  /* [2] Externally clocked Write Stop detected */
#define SERIAL_INTR_I2C_EC_WAKE_UP              ((uint32) 0x01u)
#define SERIAL_INTR_I2C_EC_EZBUF_STOP           ((uint32) 0x01u << \
                                                                    SERIAL_INTR_I2C_EC_EZBUF_STOP_POS)
#define SERIAL_INTR_I2C_EC_EZBUF_WRITE_STOP     ((uint32) 0x01u << \
                                                                    SERIAL_INTR_I2C_EC_EZBUF_WRITE_STOP_POS)

/* SERIAL_INTR_MASTER, SERIAL_INTR_MASTER_SET,
   SERIAL_INTR_MASTER_MASK, SERIAL_INTR_MASTER_MASKED */
#define SERIAL_INTR_MASTER_I2C_ARB_LOST_POS   (0u)  /* [0] Master lost arbitration                          */
#define SERIAL_INTR_MASTER_I2C_NACK_POS       (1u)  /* [1] Master receives NACK: address or write to slave  */
#define SERIAL_INTR_MASTER_I2C_ACK_POS        (2u)  /* [2] Master receives NACK: address or write to slave  */
#define SERIAL_INTR_MASTER_I2C_STOP_POS       (4u)  /* [4] Master detects the Stop: only self generated Stop*/
#define SERIAL_INTR_MASTER_I2C_BUS_ERROR_POS  (8u)  /* [8] Master detects bus error: misplaced Start or Stop*/
#define SERIAL_INTR_MASTER_SPI_DONE_POS       (9u)  /* [9] Master complete trasfer: Only for SPI            */
#define SERIAL_INTR_MASTER_I2C_ARB_LOST       ((uint32) 0x01u)
#define SERIAL_INTR_MASTER_I2C_NACK           ((uint32) 0x01u << SERIAL_INTR_MASTER_I2C_NACK_POS)
#define SERIAL_INTR_MASTER_I2C_ACK            ((uint32) 0x01u << SERIAL_INTR_MASTER_I2C_ACK_POS)
#define SERIAL_INTR_MASTER_I2C_STOP           ((uint32) 0x01u << SERIAL_INTR_MASTER_I2C_STOP_POS)
#define SERIAL_INTR_MASTER_I2C_BUS_ERROR      ((uint32) 0x01u << \
                                                                    SERIAL_INTR_MASTER_I2C_BUS_ERROR_POS)
#define SERIAL_INTR_MASTER_SPI_DONE           ((uint32) 0x01u << SERIAL_INTR_MASTER_SPI_DONE_POS)

/*
* SERIAL_INTR_SLAVE, SERIAL_INTR_SLAVE_SET,
* SERIAL_INTR_SLAVE_MASK, SERIAL_INTR_SLAVE_MASKED
*/
#define SERIAL_INTR_SLAVE_I2C_ARB_LOST_POS         (0u)  /* [0]  Slave lost arbitration                   */
#define SERIAL_INTR_SLAVE_I2C_NACK_POS             (1u)  /* [1]  Slave receives NACK: master reads data   */
#define SERIAL_INTR_SLAVE_I2C_ACK_POS              (2u)  /* [2]  Slave receives ACK: master reads data    */
#define SERIAL_INTR_SLAVE_I2C_WRITE_STOP_POS       (3u)  /* [3]  Slave detects end of write transaction   */
#define SERIAL_INTR_SLAVE_I2C_STOP_POS             (4u)  /* [4]  Slave detects end of transaction intened */
#define SERIAL_INTR_SLAVE_I2C_START_POS            (5u)  /* [5]  Slave detects Start                      */
#define SERIAL_INTR_SLAVE_I2C_ADDR_MATCH_POS       (6u)  /* [6]  Slave address matches                    */
#define SERIAL_INTR_SLAVE_I2C_GENERAL_POS          (7u)  /* [7]  General call received                    */
#define SERIAL_INTR_SLAVE_I2C_BUS_ERROR_POS        (8u)  /* [8]  Slave detects bus error                  */
#define SERIAL_INTR_SLAVE_SPI_EZBUF_WRITE_STOP_POS (9u)  /* [9]  Slave write complete: Only for SPI       */
#define SERIAL_INTR_SLAVE_SPI_EZBUF_STOP_POS       (10u) /* [10] Slave end of transaciton: Only for SPI   */
#define SERIAL_INTR_SLAVE_SPI_BUS_ERROR_POS        (11u) /* [11] Slave detects bus error: Only for SPI    */
#define SERIAL_INTR_SLAVE_I2C_ARB_LOST             ((uint32) 0x01u)
#define SERIAL_INTR_SLAVE_I2C_NACK                 ((uint32) 0x01u << \
                                                                    SERIAL_INTR_SLAVE_I2C_NACK_POS)
#define SERIAL_INTR_SLAVE_I2C_ACK                  ((uint32) 0x01u << \
                                                                    SERIAL_INTR_SLAVE_I2C_ACK_POS)
#define SERIAL_INTR_SLAVE_I2C_WRITE_STOP           ((uint32) 0x01u << \
                                                                    SERIAL_INTR_SLAVE_I2C_WRITE_STOP_POS)
#define SERIAL_INTR_SLAVE_I2C_STOP                 ((uint32) 0x01u << \
                                                                    SERIAL_INTR_SLAVE_I2C_STOP_POS)
#define SERIAL_INTR_SLAVE_I2C_START                ((uint32) 0x01u << \
                                                                    SERIAL_INTR_SLAVE_I2C_START_POS)
#define SERIAL_INTR_SLAVE_I2C_ADDR_MATCH           ((uint32) 0x01u << \
                                                                    SERIAL_INTR_SLAVE_I2C_ADDR_MATCH_POS)
#define SERIAL_INTR_SLAVE_I2C_GENERAL              ((uint32) 0x01u << \
                                                                    SERIAL_INTR_SLAVE_I2C_GENERAL_POS)
#define SERIAL_INTR_SLAVE_I2C_BUS_ERROR            ((uint32) 0x01u << \
                                                                    SERIAL_INTR_SLAVE_I2C_BUS_ERROR_POS)
#define SERIAL_INTR_SLAVE_SPI_EZBUF_WRITE_STOP     ((uint32) 0x01u << \
                                                                   SERIAL_INTR_SLAVE_SPI_EZBUF_WRITE_STOP_POS)
#define SERIAL_INTR_SLAVE_SPI_EZBUF_STOP           ((uint32) 0x01u << \
                                                                    SERIAL_INTR_SLAVE_SPI_EZBUF_STOP_POS)
#define SERIAL_INTR_SLAVE_SPI_BUS_ERROR           ((uint32) 0x01u << \
                                                                    SERIAL_INTR_SLAVE_SPI_BUS_ERROR_POS)

/*
* SERIAL_INTR_TX, SERIAL_INTR_TX_SET,
* SERIAL_INTR_TX_MASK, SERIAL_INTR_TX_MASKED
*/
#define SERIAL_INTR_TX_TRIGGER_POS        (0u)  /* [0]  Trigger on TX FIFO entires                       */
#define SERIAL_INTR_TX_NOT_FULL_POS       (1u)  /* [1]  TX FIFO is not full                              */
#define SERIAL_INTR_TX_EMPTY_POS          (4u)  /* [4]  TX FIFO is empty                                 */
#define SERIAL_INTR_TX_OVERFLOW_POS       (5u)  /* [5]  Attempt to write to a full TX FIFO               */
#define SERIAL_INTR_TX_UNDERFLOW_POS      (6u)  /* [6]  Attempt to read from an empty TX FIFO            */
#define SERIAL_INTR_TX_BLOCKED_POS        (7u)  /* [7]  No access to the EZ memory                       */
#define SERIAL_INTR_TX_UART_NACK_POS      (8u)  /* [8]  UART transmitter received a NACK: SmartCard mode */
#define SERIAL_INTR_TX_UART_DONE_POS      (9u)  /* [9]  UART transmitter done even                       */
#define SERIAL_INTR_TX_UART_ARB_LOST_POS  (10u) /* [10] UART lost arbitration: LIN or SmartCard          */
#define SERIAL_INTR_TX_TRIGGER            ((uint32) 0x01u)
#define SERIAL_INTR_TX_NOT_FULL           ((uint32) 0x01u << SERIAL_INTR_TX_NOT_FULL_POS)
#define SERIAL_INTR_TX_EMPTY              ((uint32) 0x01u << SERIAL_INTR_TX_EMPTY_POS)
#define SERIAL_INTR_TX_OVERFLOW           ((uint32) 0x01u << SERIAL_INTR_TX_OVERFLOW_POS)
#define SERIAL_INTR_TX_UNDERFLOW          ((uint32) 0x01u << SERIAL_INTR_TX_UNDERFLOW_POS)
#define SERIAL_INTR_TX_BLOCKED            ((uint32) 0x01u << SERIAL_INTR_TX_BLOCKED_POS)
#define SERIAL_INTR_TX_UART_NACK          ((uint32) 0x01u << SERIAL_INTR_TX_UART_NACK_POS)
#define SERIAL_INTR_TX_UART_DONE          ((uint32) 0x01u << SERIAL_INTR_TX_UART_DONE_POS)
#define SERIAL_INTR_TX_UART_ARB_LOST      ((uint32) 0x01u << SERIAL_INTR_TX_UART_ARB_LOST_POS)

/*
* SERIAL_INTR_RX, SERIAL_INTR_RX_SET,
* SERIAL_INTR_RX_MASK, SERIAL_INTR_RX_MASKED
*/
#define SERIAL_INTR_RX_TRIGGER_POS        (0u)   /* [0]  Trigger on RX FIFO entires            */
#define SERIAL_INTR_RX_NOT_EMPTY_POS      (2u)   /* [2]  RX FIFO is not empty                  */
#define SERIAL_INTR_RX_FULL_POS           (3u)   /* [3]  RX FIFO is full                       */
#define SERIAL_INTR_RX_OVERFLOW_POS       (5u)   /* [5]  Attempt to write to a full RX FIFO    */
#define SERIAL_INTR_RX_UNDERFLOW_POS      (6u)   /* [6]  Attempt to read from an empty RX FIFO */
#define SERIAL_INTR_RX_BLOCKED_POS        (7u)   /* [7]  No access to the EZ memory            */
#define SERIAL_INTR_RX_FRAME_ERROR_POS    (8u)   /* [8]  Frame error in received data frame    */
#define SERIAL_INTR_RX_PARITY_ERROR_POS   (9u)   /* [9]  Parity error in received data frame   */
#define SERIAL_INTR_RX_BAUD_DETECT_POS    (10u)  /* [10] LIN baudrate detection is completed   */
#define SERIAL_INTR_RX_BREAK_DETECT_POS   (11u)  /* [11] Break detection is successful         */
#define SERIAL_INTR_RX_TRIGGER            ((uint32) 0x01u)
#define SERIAL_INTR_RX_NOT_EMPTY          ((uint32) 0x01u << SERIAL_INTR_RX_NOT_EMPTY_POS)
#define SERIAL_INTR_RX_FULL               ((uint32) 0x01u << SERIAL_INTR_RX_FULL_POS)
#define SERIAL_INTR_RX_OVERFLOW           ((uint32) 0x01u << SERIAL_INTR_RX_OVERFLOW_POS)
#define SERIAL_INTR_RX_UNDERFLOW          ((uint32) 0x01u << SERIAL_INTR_RX_UNDERFLOW_POS)
#define SERIAL_INTR_RX_BLOCKED            ((uint32) 0x01u << SERIAL_INTR_RX_BLOCKED_POS)
#define SERIAL_INTR_RX_FRAME_ERROR        ((uint32) 0x01u << SERIAL_INTR_RX_FRAME_ERROR_POS)
#define SERIAL_INTR_RX_PARITY_ERROR       ((uint32) 0x01u << SERIAL_INTR_RX_PARITY_ERROR_POS)
#define SERIAL_INTR_RX_BAUD_DETECT        ((uint32) 0x01u << SERIAL_INTR_RX_BAUD_DETECT_POS)
#define SERIAL_INTR_RX_BREAK_DETECT       ((uint32) 0x01u << SERIAL_INTR_RX_BREAK_DETECT_POS)

/* Define all interupt soureces */
#define SERIAL_INTR_I2C_EC_ALL    (SERIAL_INTR_I2C_EC_WAKE_UP    | \
                                             SERIAL_INTR_I2C_EC_EZBUF_STOP | \
                                             SERIAL_INTR_I2C_EC_EZBUF_WRITE_STOP)

#define SERIAL_INTR_SPI_EC_ALL    (SERIAL_INTR_SPI_EC_WAKE_UP    | \
                                             SERIAL_INTR_SPI_EC_EZBUF_STOP | \
                                             SERIAL_INTR_SPI_EC_EZBUF_WRITE_STOP)

#define SERIAL_INTR_MASTER_ALL    (SERIAL_INTR_MASTER_I2C_ARB_LOST  | \
                                             SERIAL_INTR_MASTER_I2C_NACK      | \
                                             SERIAL_INTR_MASTER_I2C_ACK       | \
                                             SERIAL_INTR_MASTER_I2C_STOP      | \
                                             SERIAL_INTR_MASTER_I2C_BUS_ERROR | \
                                             SERIAL_INTR_MASTER_SPI_DONE )

#define SERIAL_INTR_SLAVE_ALL     (SERIAL_INTR_SLAVE_I2C_ARB_LOST      | \
                                             SERIAL_INTR_SLAVE_I2C_NACK          | \
                                             SERIAL_INTR_SLAVE_I2C_ACK           | \
                                             SERIAL_INTR_SLAVE_I2C_WRITE_STOP    | \
                                             SERIAL_INTR_SLAVE_I2C_STOP          | \
                                             SERIAL_INTR_SLAVE_I2C_START         | \
                                             SERIAL_INTR_SLAVE_I2C_ADDR_MATCH    | \
                                             SERIAL_INTR_SLAVE_I2C_GENERAL       | \
                                             SERIAL_INTR_SLAVE_I2C_BUS_ERROR     | \
                                             SERIAL_INTR_SLAVE_SPI_EZBUF_WRITE_STOP | \
                                             SERIAL_INTR_SLAVE_SPI_EZBUF_STOP       | \
                                             SERIAL_INTR_SLAVE_SPI_BUS_ERROR)

#define SERIAL_INTR_TX_ALL        (SERIAL_INTR_TX_TRIGGER   | \
                                             SERIAL_INTR_TX_NOT_FULL  | \
                                             SERIAL_INTR_TX_EMPTY     | \
                                             SERIAL_INTR_TX_OVERFLOW  | \
                                             SERIAL_INTR_TX_UNDERFLOW | \
                                             SERIAL_INTR_TX_BLOCKED   | \
                                             SERIAL_INTR_TX_UART_NACK | \
                                             SERIAL_INTR_TX_UART_DONE | \
                                             SERIAL_INTR_TX_UART_ARB_LOST)

#define SERIAL_INTR_RX_ALL        (SERIAL_INTR_RX_TRIGGER      | \
                                             SERIAL_INTR_RX_NOT_EMPTY    | \
                                             SERIAL_INTR_RX_FULL         | \
                                             SERIAL_INTR_RX_OVERFLOW     | \
                                             SERIAL_INTR_RX_UNDERFLOW    | \
                                             SERIAL_INTR_RX_BLOCKED      | \
                                             SERIAL_INTR_RX_FRAME_ERROR  | \
                                             SERIAL_INTR_RX_PARITY_ERROR | \
                                             SERIAL_INTR_RX_BAUD_DETECT  | \
                                             SERIAL_INTR_RX_BREAK_DETECT)

/* General usage HW definitions */
#define SERIAL_ONE_BYTE_WIDTH (8u)   /* Number of bits in one byte           */
#define SERIAL_FIFO_SIZE      (8u)   /* Size of TX or RX FIFO: defined by HW */
#define SERIAL_EZBUFFER_SIZE  (32u)  /* EZ Buffer size: defined by HW        */

/* I2C and EZI2C slave address defines */
#define SERIAL_I2C_SLAVE_ADDR_POS    (0x01u)    /* 7-bit address shift */
#define SERIAL_I2C_SLAVE_ADDR_MASK   (0xFEu)    /* 8-bit address mask */

/* OVS constants for IrDA Low Power operation */
#define SERIAL_CTRL_OVS_IRDA_LP_OVS16     (0x00u)
#define SERIAL_CTRL_OVS_IRDA_LP_OVS32     (0x01u)
#define SERIAL_CTRL_OVS_IRDA_LP_OVS48     (0x02u)
#define SERIAL_CTRL_OVS_IRDA_LP_OVS96     (0x03u)
#define SERIAL_CTRL_OVS_IRDA_LP_OVS192    (0x04u)
#define SERIAL_CTRL_OVS_IRDA_LP_OVS768    (0x05u)
#define SERIAL_CTRL_OVS_IRDA_LP_OVS1536   (0x06u)

/* OVS constant for IrDA */
#define SERIAL_CTRL_OVS_IRDA_OVS16        (SERIAL_UART_IRDA_LP_OVS16)


/***************************************
*    SCB Common Macro Definitions
***************************************/

/* Re-enables the SCB IP. The clear enable bit has a different effect on the scb IP depending on the version.* CY_SCBIP_V0: resets state, status, TX and RX FIFOs.
* CY_SCBIP_V1 or later: resets state, status, TX and RX FIFOs and interrupt sources.
*/
#define SERIAL_SCB_SW_RESET \
                        do{           \
                            SERIAL_CTRL_REG &= ((uint32) ~SERIAL_CTRL_ENABLED); \
                            SERIAL_CTRL_REG |= ((uint32)  SERIAL_CTRL_ENABLED); \
                        }while(0)

/* TX FIFO macro */
#define SERIAL_CLEAR_TX_FIFO \
                            do{        \
                                SERIAL_TX_FIFO_CTRL_REG |= ((uint32)  SERIAL_TX_FIFO_CTRL_CLEAR); \
                                SERIAL_TX_FIFO_CTRL_REG &= ((uint32) ~SERIAL_TX_FIFO_CTRL_CLEAR); \
                            }while(0)

#define SERIAL_GET_TX_FIFO_ENTRIES    (SERIAL_TX_FIFO_STATUS_REG & \
                                                 SERIAL_TX_FIFO_STATUS_USED_MASK)

#define SERIAL_GET_TX_FIFO_SR_VALID   ((0u != (SERIAL_TX_FIFO_STATUS_REG & \
                                                         SERIAL_TX_FIFO_SR_VALID)) ? (1u) : (0u))

/* RX FIFO macro */
#define SERIAL_CLEAR_RX_FIFO \
                            do{        \
                                SERIAL_RX_FIFO_CTRL_REG |= ((uint32)  SERIAL_RX_FIFO_CTRL_CLEAR); \
                                SERIAL_RX_FIFO_CTRL_REG &= ((uint32) ~SERIAL_RX_FIFO_CTRL_CLEAR); \
                            }while(0)

#define SERIAL_GET_RX_FIFO_ENTRIES    (SERIAL_RX_FIFO_STATUS_REG & \
                                                    SERIAL_RX_FIFO_STATUS_USED_MASK)

#define SERIAL_GET_RX_FIFO_SR_VALID   ((0u != (SERIAL_RX_FIFO_STATUS_REG & \
                                                         SERIAL_RX_FIFO_SR_VALID)) ? (1u) : (0u))

/* Write interrupt source: set sourceMask bits in SERIAL_INTR_X_MASK_REG */
#define SERIAL_WRITE_INTR_I2C_EC_MASK(sourceMask) \
                                                do{         \
                                                    SERIAL_INTR_I2C_EC_MASK_REG = (uint32) (sourceMask); \
                                                }while(0)

#define SERIAL_WRITE_INTR_SPI_EC_MASK(sourceMask) \
                                                do{         \
                                                    SERIAL_INTR_SPI_EC_MASK_REG = (uint32) (sourceMask); \
                                                }while(0)

#define SERIAL_WRITE_INTR_MASTER_MASK(sourceMask) \
                                                do{         \
                                                    SERIAL_INTR_MASTER_MASK_REG = (uint32) (sourceMask); \
                                                }while(0)

#define SERIAL_WRITE_INTR_SLAVE_MASK(sourceMask)  \
                                                do{         \
                                                    SERIAL_INTR_SLAVE_MASK_REG = (uint32) (sourceMask); \
                                                }while(0)

#define SERIAL_WRITE_INTR_TX_MASK(sourceMask)     \
                                                do{         \
                                                    SERIAL_INTR_TX_MASK_REG = (uint32) (sourceMask); \
                                                }while(0)

#define SERIAL_WRITE_INTR_RX_MASK(sourceMask)     \
                                                do{         \
                                                    SERIAL_INTR_RX_MASK_REG = (uint32) (sourceMask); \
                                                }while(0)

/* Enable interrupt source: set sourceMask bits in SERIAL_INTR_X_MASK_REG */
#define SERIAL_ENABLE_INTR_I2C_EC(sourceMask) \
                                                do{     \
                                                    SERIAL_INTR_I2C_EC_MASK_REG |= (uint32) (sourceMask); \
                                                }while(0)

#define SERIAL_ENABLE_INTR_SPI_EC(sourceMask) \
                                                do{     \
                                                    SERIAL_INTR_SPI_EC_MASK_REG |= (uint32) (sourceMask); \
                                                }while(0)

#define SERIAL_ENABLE_INTR_MASTER(sourceMask) \
                                                do{     \
                                                    SERIAL_INTR_MASTER_MASK_REG |= (uint32) (sourceMask); \
                                                }while(0)

#define SERIAL_ENABLE_INTR_SLAVE(sourceMask)  \
                                                do{     \
                                                    SERIAL_INTR_SLAVE_MASK_REG |= (uint32) (sourceMask); \
                                                }while(0)

#define SERIAL_ENABLE_INTR_TX(sourceMask)     \
                                                do{     \
                                                    SERIAL_INTR_TX_MASK_REG |= (uint32) (sourceMask); \
                                                }while(0)

#define SERIAL_ENABLE_INTR_RX(sourceMask)     \
                                                do{     \
                                                    SERIAL_INTR_RX_MASK_REG |= (uint32) (sourceMask); \
                                                }while(0)

/* Disable interrupt source: clear sourceMask bits in SERIAL_INTR_X_MASK_REG */
#define SERIAL_DISABLE_INTR_I2C_EC(sourceMask) \
                                do{                      \
                                    SERIAL_INTR_I2C_EC_MASK_REG &= ((uint32) ~((uint32) (sourceMask))); \
                                }while(0)

#define SERIAL_DISABLE_INTR_SPI_EC(sourceMask) \
                                do{                      \
                                    SERIAL_INTR_SPI_EC_MASK_REG &= ((uint32) ~((uint32) (sourceMask))); \
                                 }while(0)

#define SERIAL_DISABLE_INTR_MASTER(sourceMask) \
                                do{                      \
                                SERIAL_INTR_MASTER_MASK_REG &= ((uint32) ~((uint32) (sourceMask))); \
                                }while(0)

#define SERIAL_DISABLE_INTR_SLAVE(sourceMask) \
                                do{                     \
                                    SERIAL_INTR_SLAVE_MASK_REG &= ((uint32) ~((uint32) (sourceMask))); \
                                }while(0)

#define SERIAL_DISABLE_INTR_TX(sourceMask)    \
                                do{                     \
                                    SERIAL_INTR_TX_MASK_REG &= ((uint32) ~((uint32) (sourceMask))); \
                                 }while(0)

#define SERIAL_DISABLE_INTR_RX(sourceMask)    \
                                do{                     \
                                    SERIAL_INTR_RX_MASK_REG &= ((uint32) ~((uint32) (sourceMask))); \
                                }while(0)

/* Set interrupt sources: write sourceMask bits in SERIAL_INTR_X_SET_REG */
#define SERIAL_SET_INTR_MASTER(sourceMask)    \
                                                do{     \
                                                    SERIAL_INTR_MASTER_SET_REG = (uint32) (sourceMask); \
                                                }while(0)

#define SERIAL_SET_INTR_SLAVE(sourceMask) \
                                                do{ \
                                                    SERIAL_INTR_SLAVE_SET_REG = (uint32) (sourceMask); \
                                                }while(0)

#define SERIAL_SET_INTR_TX(sourceMask)    \
                                                do{ \
                                                    SERIAL_INTR_TX_SET_REG = (uint32) (sourceMask); \
                                                }while(0)

#define SERIAL_SET_INTR_RX(sourceMask)    \
                                                do{ \
                                                    SERIAL_INTR_RX_SET_REG = (uint32) (sourceMask); \
                                                }while(0)

/* Clear interrupt sources: write sourceMask bits in SERIAL_INTR_X_REG */
#define SERIAL_CLEAR_INTR_I2C_EC(sourceMask)  \
                                                do{     \
                                                    SERIAL_INTR_I2C_EC_REG = (uint32) (sourceMask); \
                                                }while(0)

#define SERIAL_CLEAR_INTR_SPI_EC(sourceMask)  \
                                                do{     \
                                                    SERIAL_INTR_SPI_EC_REG = (uint32) (sourceMask); \
                                                }while(0)

#define SERIAL_CLEAR_INTR_MASTER(sourceMask)  \
                                                do{     \
                                                    SERIAL_INTR_MASTER_REG = (uint32) (sourceMask); \
                                                }while(0)

#define SERIAL_CLEAR_INTR_SLAVE(sourceMask)   \
                                                do{     \
                                                    SERIAL_INTR_SLAVE_REG  = (uint32) (sourceMask); \
                                                }while(0)

#define SERIAL_CLEAR_INTR_TX(sourceMask)      \
                                                do{     \
                                                    SERIAL_INTR_TX_REG     = (uint32) (sourceMask); \
                                                }while(0)

#define SERIAL_CLEAR_INTR_RX(sourceMask)      \
                                                do{     \
                                                    SERIAL_INTR_RX_REG     = (uint32) (sourceMask); \
                                                }while(0)

/* Return true if sourceMask is set in SERIAL_INTR_CAUSE_REG */
#define SERIAL_CHECK_CAUSE_INTR(sourceMask)    (0u != (SERIAL_INTR_CAUSE_REG & (sourceMask)))

/* Return true if sourceMask is set in INTR_X_MASKED_REG */
#define SERIAL_CHECK_INTR_I2C_EC(sourceMask)  (0u != (SERIAL_INTR_I2C_EC_REG & (sourceMask)))
#define SERIAL_CHECK_INTR_SPI_EC(sourceMask)  (0u != (SERIAL_INTR_SPI_EC_REG & (sourceMask)))
#define SERIAL_CHECK_INTR_MASTER(sourceMask)  (0u != (SERIAL_INTR_MASTER_REG & (sourceMask)))
#define SERIAL_CHECK_INTR_SLAVE(sourceMask)   (0u != (SERIAL_INTR_SLAVE_REG  & (sourceMask)))
#define SERIAL_CHECK_INTR_TX(sourceMask)      (0u != (SERIAL_INTR_TX_REG     & (sourceMask)))
#define SERIAL_CHECK_INTR_RX(sourceMask)      (0u != (SERIAL_INTR_RX_REG     & (sourceMask)))

/* Return true if sourceMask is set in SERIAL_INTR_X_MASKED_REG */
#define SERIAL_CHECK_INTR_I2C_EC_MASKED(sourceMask)   (0u != (SERIAL_INTR_I2C_EC_MASKED_REG & \
                                                                       (sourceMask)))
#define SERIAL_CHECK_INTR_SPI_EC_MASKED(sourceMask)   (0u != (SERIAL_INTR_SPI_EC_MASKED_REG & \
                                                                       (sourceMask)))
#define SERIAL_CHECK_INTR_MASTER_MASKED(sourceMask)   (0u != (SERIAL_INTR_MASTER_MASKED_REG & \
                                                                       (sourceMask)))
#define SERIAL_CHECK_INTR_SLAVE_MASKED(sourceMask)    (0u != (SERIAL_INTR_SLAVE_MASKED_REG  & \
                                                                       (sourceMask)))
#define SERIAL_CHECK_INTR_TX_MASKED(sourceMask)       (0u != (SERIAL_INTR_TX_MASKED_REG     & \
                                                                       (sourceMask)))
#define SERIAL_CHECK_INTR_RX_MASKED(sourceMask)       (0u != (SERIAL_INTR_RX_MASKED_REG     & \
                                                                       (sourceMask)))

/* Return true if sourceMask is set in SERIAL_CTRL_REG: generaly is used to check enable bit */
#define SERIAL_GET_CTRL_ENABLED    (0u != (SERIAL_CTRL_REG & SERIAL_CTRL_ENABLED))

#define SERIAL_CHECK_SLAVE_AUTO_ADDR_NACK     (0u != (SERIAL_I2C_CTRL_REG & \
                                                                SERIAL_I2C_CTRL_S_NOT_READY_DATA_NACK))


/***************************************
*      I2C Macro Definitions
***************************************/

/* Enable auto ACK/NACK */
#define SERIAL_ENABLE_SLAVE_AUTO_ADDR_NACK \
                            do{                      \
                                SERIAL_I2C_CTRL_REG |= SERIAL_I2C_CTRL_S_NOT_READY_DATA_NACK; \
                            }while(0)

#define SERIAL_ENABLE_SLAVE_AUTO_DATA_ACK \
                            do{                     \
                                SERIAL_I2C_CTRL_REG |= SERIAL_I2C_CTRL_S_READY_DATA_ACK; \
                            }while(0)

#define SERIAL_ENABLE_SLAVE_AUTO_DATA_NACK \
                            do{                      \
                                SERIAL_I2C_CTRL_REG |= SERIAL_I2C_CTRL_S_NOT_READY_DATA_NACK; \
                            }while(0)

#define SERIAL_ENABLE_MASTER_AUTO_DATA_ACK \
                            do{                      \
                                SERIAL_I2C_CTRL_REG |= SERIAL_I2C_CTRL_M_READY_DATA_ACK; \
                            }while(0)

#define SERIAL_ENABLE_MASTER_AUTO_DATA_NACK \
                            do{                       \
                                SERIAL_I2C_CTRL_REG |= SERIAL_I2C_CTRL_M_NOT_READY_DATA_NACK; \
                            }while(0)

/* Disable auto ACK/NACK */
#define SERIAL_DISABLE_SLAVE_AUTO_ADDR_NACK \
                            do{                       \
                                SERIAL_I2C_CTRL_REG &= ~SERIAL_I2C_CTRL_S_NOT_READY_DATA_NACK; \
                            }while(0)

#define SERIAL_DISABLE_SLAVE_AUTO_DATA_ACK \
                            do{                      \
                                SERIAL_I2C_CTRL_REG &= ~SERIAL_I2C_CTRL_S_READY_DATA_ACK; \
                            }while(0)

#define SERIAL_DISABLE_SLAVE_AUTO_DATA_NACK \
                            do{                       \
                                SERIAL_I2C_CTRL_REG &= ~SERIAL_I2C_CTRL_S_NOT_READY_DATA_NACK; \
                            }while(0)

#define SERIAL_DISABLE_MASTER_AUTO_DATA_ACK \
                            do{                       \
                                SERIAL_I2C_CTRL_REG &= ~SERIAL_I2C_CTRL_M_READY_DATA_ACK; \
                            }while(0)

#define SERIAL_DISABLE_MASTER_AUTO_DATA_NACK \
                            do{                        \
                                SERIAL_I2C_CTRL_REG &= ~SERIAL_I2C_CTRL_M_NOT_READY_DATA_NACK; \
                            }while(0)

/* Enable Slave autoACK/NACK Data */
#define SERIAL_ENABLE_SLAVE_AUTO_DATA \
                            do{                 \
                                SERIAL_I2C_CTRL_REG |= (SERIAL_I2C_CTRL_S_READY_DATA_ACK |      \
                                                                  SERIAL_I2C_CTRL_S_NOT_READY_DATA_NACK); \
                            }while(0)

/* Disable Slave autoACK/NACK Data */
#define SERIAL_DISABLE_SLAVE_AUTO_DATA \
                            do{                  \
                                SERIAL_I2C_CTRL_REG &= ((uint32) \
                                                                  ~(SERIAL_I2C_CTRL_S_READY_DATA_ACK |       \
                                                                    SERIAL_I2C_CTRL_S_NOT_READY_DATA_NACK)); \
                            }while(0)

/* Disable Master autoACK/NACK Data */
#define SERIAL_DISABLE_MASTER_AUTO_DATA \
                            do{                   \
                                SERIAL_I2C_CTRL_REG &= ((uint32) \
                                                                  ~(SERIAL_I2C_CTRL_M_READY_DATA_ACK |       \
                                                                    SERIAL_I2C_CTRL_M_NOT_READY_DATA_NACK)); \
                            }while(0)

/* Master commands */
#define SERIAL_I2C_MASTER_GENERATE_START \
                            do{                    \
                                SERIAL_I2C_MASTER_CMD_REG = SERIAL_I2C_MASTER_CMD_M_START_ON_IDLE; \
                            }while(0)

#define SERIAL_I2C_MASTER_CLEAR_START \
                            do{                 \
                                SERIAL_I2C_MASTER_CMD_REG =  ((uint32) 0u); \
                            }while(0)

#define SERIAL_I2C_MASTER_GENERATE_RESTART SERIAL_I2CReStartGeneration()

#define SERIAL_I2C_MASTER_GENERATE_STOP \
                            do{                   \
                                SERIAL_I2C_MASTER_CMD_REG =                                            \
                                    (SERIAL_I2C_MASTER_CMD_M_STOP |                                    \
                                        (SERIAL_CHECK_I2C_STATUS(SERIAL_I2C_STATUS_M_READ) ? \
                                            (SERIAL_I2C_MASTER_CMD_M_NACK) : (0u)));                   \
                            }while(0)

#define SERIAL_I2C_MASTER_GENERATE_ACK \
                            do{                  \
                                SERIAL_I2C_MASTER_CMD_REG = SERIAL_I2C_MASTER_CMD_M_ACK; \
                            }while(0)

#define SERIAL_I2C_MASTER_GENERATE_NACK \
                            do{                   \
                                SERIAL_I2C_MASTER_CMD_REG = SERIAL_I2C_MASTER_CMD_M_NACK; \
                            }while(0)

/* Slave commands */
#define SERIAL_I2C_SLAVE_GENERATE_ACK \
                            do{                 \
                                SERIAL_I2C_SLAVE_CMD_REG = SERIAL_I2C_SLAVE_CMD_S_ACK; \
                            }while(0)

#define SERIAL_I2C_SLAVE_GENERATE_NACK \
                            do{                  \
                                SERIAL_I2C_SLAVE_CMD_REG = SERIAL_I2C_SLAVE_CMD_S_NACK; \
                            }while(0)

#define SERIAL_I2C_SLAVE_CLEAR_NACK \
                            do{               \
                                SERIAL_I2C_SLAVE_CMD_REG = 0u; \
                            }while(0)

/* Return 8-bit address. The input address should be 7-bits */
#define SERIAL_GET_I2C_8BIT_ADDRESS(addr) (((uint32) ((uint32) (addr) << \
                                                                    SERIAL_I2C_SLAVE_ADDR_POS)) & \
                                                                        SERIAL_I2C_SLAVE_ADDR_MASK)

#define SERIAL_GET_I2C_7BIT_ADDRESS(addr) ((uint32) (addr) >> SERIAL_I2C_SLAVE_ADDR_POS)

/* Adjust SDA filter Trim settings */
#define SERIAL_DEFAULT_I2C_CFG_SDA_FILT_TRIM  (0x02u)
#define SERIAL_EC_AM_I2C_CFG_SDA_FILT_TRIM    (0x03u)

#define SERIAL_SET_I2C_CFG_SDA_FILT_TRIM(sdaTrim) \
        do{                                                 \
            SERIAL_I2C_CFG_REG =                  \
                            ((SERIAL_I2C_CFG_REG & (uint32) ~SERIAL_I2C_CFG_SDA_FILT_TRIM_MASK) | \
                             ((uint32) ((uint32) (sdaTrim) <<SERIAL_I2C_CFG_SDA_FILT_TRIM_POS)));           \
        }while(0)

/* Return slave select number from SPI_CTRL register */
#define SERIAL_GET_SPI_CTRL_SS(activeSelect) (((uint32) ((uint32) (activeSelect) << \
                                                                    SERIAL_SPI_CTRL_SLAVE_SELECT_POS)) & \
                                                                        SERIAL_SPI_CTRL_SLAVE_SELECT_MASK)

/* Return true if bit is set in SERIAL_I2C_STATUS_REG */
#define SERIAL_CHECK_I2C_STATUS(sourceMask)   (0u != (SERIAL_I2C_STATUS_REG & (sourceMask)))

/* Return true if bit is set in SERIAL_SPI_STATUS_REG */
#define SERIAL_CHECK_SPI_STATUS(sourceMask)   (0u != (SERIAL_SPI_STATUS_REG & (sourceMask)))


/***************************************
*       SCB Init Macros Definitions
***************************************/

/* SERIAL_CTRL */
#define SERIAL_GET_CTRL_OVS(oversample)   ((((uint32) (oversample)) - 1u) & SERIAL_CTRL_OVS_MASK)

#define SERIAL_GET_CTRL_EC_OP_MODE(opMode)        ((0u != (opMode)) ? \
                                                                (SERIAL_CTRL_EC_OP_MODE)  : (0u))

#define SERIAL_GET_CTRL_EC_AM_MODE(amMode)        ((0u != (amMode)) ? \
                                                                (SERIAL_CTRL_EC_AM_MODE)  : (0u))

#define SERIAL_GET_CTRL_BLOCK(block)              ((0u != (block))  ? \
                                                                (SERIAL_CTRL_BLOCK)       : (0u))

#define SERIAL_GET_CTRL_ADDR_ACCEPT(acceptAddr)   ((0u != (acceptAddr)) ? \
                                                                (SERIAL_CTRL_ADDR_ACCEPT) : (0u))

/* SERIAL_I2C_CTRL */
#define SERIAL_GET_I2C_CTRL_HIGH_PHASE_OVS(oversampleHigh) (((uint32) (oversampleHigh) - 1u) & \
                                                                        SERIAL_I2C_CTRL_HIGH_PHASE_OVS_MASK)

#define SERIAL_GET_I2C_CTRL_LOW_PHASE_OVS(oversampleLow)   (((uint32) (((uint32) (oversampleLow) - 1u) << \
                                                                    SERIAL_I2C_CTRL_LOW_PHASE_OVS_POS)) &  \
                                                                    SERIAL_I2C_CTRL_LOW_PHASE_OVS_MASK)

#define SERIAL_GET_I2C_CTRL_S_NOT_READY_ADDR_NACK(wakeNack) ((0u != (wakeNack)) ? \
                                                            (SERIAL_I2C_CTRL_S_NOT_READY_ADDR_NACK) : (0u))

#define SERIAL_GET_I2C_CTRL_SL_MSTR_MODE(mode)    ((uint32) ((uint32)(mode) << \
                                                                    SERIAL_I2C_CTRL_SLAVE_MODE_POS))

/* SERIAL_SPI_CTRL */
#define SERIAL_GET_SPI_CTRL_CONTINUOUS(separate)  ((0u != (separate)) ? \
                                                                (SERIAL_SPI_CTRL_CONTINUOUS) : (0u))

#define SERIAL_GET_SPI_CTRL_SELECT_PRECEDE(mode)  ((0u != (mode)) ? \
                                                                      (SERIAL_SPI_CTRL_SELECT_PRECEDE) : (0u))

#define SERIAL_GET_SPI_CTRL_SCLK_MODE(mode)       (((uint32) ((uint32) (mode) << \
                                                                        SERIAL_SPI_CTRL_CPHA_POS)) & \
                                                                        SERIAL_SPI_CTRL_SCLK_MODE_MASK)

#define SERIAL_GET_SPI_CTRL_LATE_MISO_SAMPLE(lateMiso) ((0u != (lateMiso)) ? \
                                                                    (SERIAL_SPI_CTRL_LATE_MISO_SAMPLE) : (0u))

#define SERIAL_GET_SPI_CTRL_SUB_MODE(mode)        (((uint32) (((uint32) (mode)) << \
                                                                        SERIAL_SPI_CTRL_MODE_POS)) & \
                                                                        SERIAL_SPI_CTRL_MODE_MASK)

#define SERIAL_GET_SPI_CTRL_SLAVE_SELECT(select)  (((uint32) ((uint32) (select) << \
                                                                      SERIAL_SPI_CTRL_SLAVE_SELECT_POS)) & \
                                                                      SERIAL_SPI_CTRL_SLAVE_SELECT_MASK)

#define SERIAL_GET_SPI_CTRL_MASTER_MODE(mode)     ((0u != (mode)) ? \
                                                                (SERIAL_SPI_CTRL_MASTER) : (0u))

/* SERIAL_UART_CTRL */
#define SERIAL_GET_UART_CTRL_MODE(mode)           (((uint32) ((uint32) (mode) << \
                                                                            SERIAL_UART_CTRL_MODE_POS)) & \
                                                                                SERIAL_UART_CTRL_MODE_MASK)

/* SERIAL_UART_RX_CTRL */
#define SERIAL_GET_UART_RX_CTRL_MODE(stopBits)    (((uint32) (stopBits) - 1u) & \
                                                                        SERIAL_UART_RX_CTRL_STOP_BITS_MASK)

#define SERIAL_GET_UART_RX_CTRL_PARITY(parity)    ((0u != (parity)) ? \
                                                                    (SERIAL_UART_RX_CTRL_PARITY) : (0u))

#define SERIAL_GET_UART_RX_CTRL_POLARITY(polarity)    ((0u != (polarity)) ? \
                                                                    (SERIAL_UART_RX_CTRL_POLARITY) : (0u))

#define SERIAL_GET_UART_RX_CTRL_DROP_ON_PARITY_ERR(dropErr) ((0u != (dropErr)) ? \
                                                        (SERIAL_UART_RX_CTRL_DROP_ON_PARITY_ERR) : (0u))

#define SERIAL_GET_UART_RX_CTRL_DROP_ON_FRAME_ERR(dropErr) ((0u != (dropErr)) ? \
                                                        (SERIAL_UART_RX_CTRL_DROP_ON_FRAME_ERR) : (0u))

#define SERIAL_GET_UART_RX_CTRL_MP_MODE(mpMode)   ((0u != (mpMode)) ? \
                                                        (SERIAL_UART_RX_CTRL_MP_MODE) : (0u))

/* SERIAL_UART_TX_CTRL */
#define SERIAL_GET_UART_TX_CTRL_MODE(stopBits)    (((uint32) (stopBits) - 1u) & \
                                                                SERIAL_UART_RX_CTRL_STOP_BITS_MASK)

#define SERIAL_GET_UART_TX_CTRL_PARITY(parity)    ((0u != (parity)) ? \
                                                               (SERIAL_UART_TX_CTRL_PARITY) : (0u))

#define SERIAL_GET_UART_TX_CTRL_RETRY_NACK(nack)  ((0u != (nack)) ? \
                                                               (SERIAL_UART_TX_CTRL_RETRY_ON_NACK) : (0u))

/* SERIAL_RX_CTRL */
#define SERIAL_GET_RX_CTRL_DATA_WIDTH(dataWidth)  (((uint32) (dataWidth) - 1u) & \
                                                                SERIAL_RX_CTRL_DATA_WIDTH_MASK)

#define SERIAL_GET_RX_CTRL_BIT_ORDER(bitOrder)    ((0u != (bitOrder)) ? \
                                                                (SERIAL_RX_CTRL_MSB_FIRST) : (0u))

#define SERIAL_GET_RX_CTRL_MEDIAN(filterEn)       ((0u != (filterEn)) ? \
                                                                (SERIAL_RX_CTRL_MEDIAN) : (0u))

/* SERIAL_RX_MATCH */
#define SERIAL_GET_RX_MATCH_ADDR(addr)    ((uint32) (addr) & SERIAL_RX_MATCH_ADDR_MASK)
#define SERIAL_GET_RX_MATCH_MASK(mask)    (((uint32) ((uint32) (mask) << \
                                                            SERIAL_RX_MATCH_MASK_POS)) & \
                                                            SERIAL_RX_MATCH_MASK_MASK)

/* SERIAL_RX_FIFO_CTRL */
#define SERIAL_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(level)  ((uint32) (level) & \
                                                                    SERIAL_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK)

/* SERIAL_TX_CTRL */
#define SERIAL_GET_TX_CTRL_DATA_WIDTH(dataWidth)  (((uint32) (dataWidth) - 1u) & \
                                                                SERIAL_RX_CTRL_DATA_WIDTH_MASK)

#define SERIAL_GET_TX_CTRL_BIT_ORDER(bitOrder)    ((0u != (bitOrder)) ? \
                                                                (SERIAL_TX_CTRL_MSB_FIRST) : (0u))

/* SERIAL_TX_FIFO_CTRL */
#define SERIAL_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(level)  ((uint32) (level) & \
                                                                    SERIAL_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK)

/* Clears register: configuration and interrupt mask */
#define SERIAL_CLEAR_REG          ((uint32) (0u))
#define SERIAL_NO_INTR_SOURCES    ((uint32) (0u))
#define SERIAL_DUMMY_PARAM        ((uint32) (0u))
#define SERIAL_SUBMODE_SPI_SLAVE  ((uint32) (0u))

/* Return in case I2C read error */
#define SERIAL_I2C_INVALID_BYTE   ((uint32) 0xFFFFFFFFu)
#define SERIAL_CHECK_VALID_BYTE   ((uint32) 0x80000000u)


/***************************************
*       Obsolete definitions
***************************************/

/* The following definitions are for version compatibility.
* They are obsolete in SCB v1_20. Please do not use them in new projects
*/
#define SERIAL_CHECK_INTR_EC_I2C(sourceMask)  SERIAL_CHECK_INTR_I2C_EC(sourceMask)
#define SERIAL_CHECK_INTR_EC_SPI(sourceMask)  SERIAL_CHECK_INTR_SPI_EC(sourceMask)

#endif /* (CY_SCB_SERIAL_H) */


/* [] END OF FILE */
