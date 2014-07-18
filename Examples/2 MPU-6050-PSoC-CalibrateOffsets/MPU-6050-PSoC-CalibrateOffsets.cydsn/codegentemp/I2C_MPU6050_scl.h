/*******************************************************************************
* File Name: I2C_MPU6050_scl.h  
* Version 2.0
*
* Description:
*  This file containts Control Register function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_I2C_MPU6050_scl_H) /* Pins I2C_MPU6050_scl_H */
#define CY_PINS_I2C_MPU6050_scl_H

#include "cytypes.h"
#include "cyfitter.h"
#include "I2C_MPU6050_scl_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    I2C_MPU6050_scl_Write(uint8 value) ;
void    I2C_MPU6050_scl_SetDriveMode(uint8 mode) ;
uint8   I2C_MPU6050_scl_ReadDataReg(void) ;
uint8   I2C_MPU6050_scl_Read(void) ;
uint8   I2C_MPU6050_scl_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define I2C_MPU6050_scl_DRIVE_MODE_BITS        (3)
#define I2C_MPU6050_scl_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - I2C_MPU6050_scl_DRIVE_MODE_BITS))
#define I2C_MPU6050_scl_DRIVE_MODE_SHIFT       (0x00u)
#define I2C_MPU6050_scl_DRIVE_MODE_MASK        (0x07u << I2C_MPU6050_scl_DRIVE_MODE_SHIFT)

#define I2C_MPU6050_scl_DM_ALG_HIZ         (0x00u << I2C_MPU6050_scl_DRIVE_MODE_SHIFT)
#define I2C_MPU6050_scl_DM_DIG_HIZ         (0x01u << I2C_MPU6050_scl_DRIVE_MODE_SHIFT)
#define I2C_MPU6050_scl_DM_RES_UP          (0x02u << I2C_MPU6050_scl_DRIVE_MODE_SHIFT)
#define I2C_MPU6050_scl_DM_RES_DWN         (0x03u << I2C_MPU6050_scl_DRIVE_MODE_SHIFT)
#define I2C_MPU6050_scl_DM_OD_LO           (0x04u << I2C_MPU6050_scl_DRIVE_MODE_SHIFT)
#define I2C_MPU6050_scl_DM_OD_HI           (0x05u << I2C_MPU6050_scl_DRIVE_MODE_SHIFT)
#define I2C_MPU6050_scl_DM_STRONG          (0x06u << I2C_MPU6050_scl_DRIVE_MODE_SHIFT)
#define I2C_MPU6050_scl_DM_RES_UPDWN       (0x07u << I2C_MPU6050_scl_DRIVE_MODE_SHIFT)

/* Digital Port Constants */
#define I2C_MPU6050_scl_MASK               I2C_MPU6050_scl__MASK
#define I2C_MPU6050_scl_SHIFT              I2C_MPU6050_scl__SHIFT
#define I2C_MPU6050_scl_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define I2C_MPU6050_scl_PS                     (* (reg32 *) I2C_MPU6050_scl__PS)
/* Port Configuration */
#define I2C_MPU6050_scl_PC                     (* (reg32 *) I2C_MPU6050_scl__PC)
/* Data Register */
#define I2C_MPU6050_scl_DR                     (* (reg32 *) I2C_MPU6050_scl__DR)
/* Input Buffer Disable Override */
#define I2C_MPU6050_scl_INP_DIS                (* (reg32 *) I2C_MPU6050_scl__PC2)


#if defined(I2C_MPU6050_scl__INTSTAT)  /* Interrupt Registers */

    #define I2C_MPU6050_scl_INTSTAT                (* (reg32 *) I2C_MPU6050_scl__INTSTAT)

#endif /* Interrupt Registers */

#endif /* End Pins I2C_MPU6050_scl_H */


/* [] END OF FILE */
