/*******************************************************************************
* File Name: .h
* Version 1.20
*
* Description:
*  This private file provides constants and parameter values for the
*  SCB Component in I2C mode.
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

#if !defined(CY_SCB_I2C_PVT_I2C_MPU6050_H)
#define CY_SCB_I2C_PVT_I2C_MPU6050_H

#include "I2C_MPU6050_I2C.h"


/***************************************
*     Private Global Vars
***************************************/

extern volatile uint8 I2C_MPU6050_state; /* Current state of I2C FSM */

#if(I2C_MPU6050_I2C_SLAVE_CONST)
    extern volatile uint8 I2C_MPU6050_slStatus;          /* Slave Status */

    /* Receive buffer variables */
    extern volatile uint8 * I2C_MPU6050_slWrBufPtr;      /* Pointer to Receive buffer  */
    extern volatile uint32  I2C_MPU6050_slWrBufSize;     /* Slave Receive buffer size  */
    extern volatile uint32  I2C_MPU6050_slWrBufIndex;    /* Slave Receive buffer Index */

    /* Transmit buffer variables */
    extern volatile uint8 * I2C_MPU6050_slRdBufPtr;      /* Pointer to Transmit buffer  */
    extern volatile uint32  I2C_MPU6050_slRdBufSize;     /* Slave Transmit buffer size  */
    extern volatile uint32  I2C_MPU6050_slRdBufIndex;    /* Slave Transmit buffer Index */
    extern volatile uint32  I2C_MPU6050_slRdBufIndexTmp; /* Slave Transmit buffer Index Tmp */
    extern volatile uint8   I2C_MPU6050_slOverFlowCount; /* Slave Transmit Overflow counter */
#endif /* (I2C_MPU6050_I2C_SLAVE_CONST) */

#if(I2C_MPU6050_I2C_MASTER_CONST)
    extern volatile uint16 I2C_MPU6050_mstrStatus;      /* Master Status byte  */
    extern volatile uint8  I2C_MPU6050_mstrControl;     /* Master Control byte */

    /* Receive buffer variables */
    extern volatile uint8 * I2C_MPU6050_mstrRdBufPtr;   /* Pointer to Master Read buffer */
    extern volatile uint32  I2C_MPU6050_mstrRdBufSize;  /* Master Read buffer size       */
    extern volatile uint32  I2C_MPU6050_mstrRdBufIndex; /* Master Read buffer Index      */

    /* Transmit buffer variables */
    extern volatile uint8 * I2C_MPU6050_mstrWrBufPtr;   /* Pointer to Master Write buffer */
    extern volatile uint32  I2C_MPU6050_mstrWrBufSize;  /* Master Write buffer size       */
    extern volatile uint32  I2C_MPU6050_mstrWrBufIndex; /* Master Write buffer Index      */
    extern volatile uint32  I2C_MPU6050_mstrWrBufIndexTmp; /* Master Write buffer Index Tmp */
#endif /* (I2C_MPU6050_I2C_MASTER_CONST) */


/***************************************
*     Private Function Prototypes
***************************************/

#if(I2C_MPU6050_SCB_MODE_I2C_CONST_CFG)
    void I2C_MPU6050_I2CInit(void);
#endif /* (I2C_MPU6050_SCB_MODE_I2C_CONST_CFG) */

void I2C_MPU6050_I2CStop(void);
void I2C_MPU6050_I2CSaveConfig(void);
void I2C_MPU6050_I2CRestoreConfig(void);

#if(I2C_MPU6050_I2C_MASTER_CONST)
    void I2C_MPU6050_I2CReStartGeneration(void);
#endif /* (I2C_MPU6050_I2C_MASTER_CONST) */

#endif /* (CY_SCB_I2C_PVT_I2C_MPU6050_H) */


/* [] END OF FILE */
