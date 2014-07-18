/*******************************************************************************
* File Name: SERIAL_tx.h  
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

#if !defined(CY_PINS_SERIAL_tx_H) /* Pins SERIAL_tx_H */
#define CY_PINS_SERIAL_tx_H

#include "cytypes.h"
#include "cyfitter.h"
#include "SERIAL_tx_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    SERIAL_tx_Write(uint8 value) ;
void    SERIAL_tx_SetDriveMode(uint8 mode) ;
uint8   SERIAL_tx_ReadDataReg(void) ;
uint8   SERIAL_tx_Read(void) ;
uint8   SERIAL_tx_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define SERIAL_tx_DRIVE_MODE_BITS        (3)
#define SERIAL_tx_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - SERIAL_tx_DRIVE_MODE_BITS))
#define SERIAL_tx_DRIVE_MODE_SHIFT       (0x00u)
#define SERIAL_tx_DRIVE_MODE_MASK        (0x07u << SERIAL_tx_DRIVE_MODE_SHIFT)

#define SERIAL_tx_DM_ALG_HIZ         (0x00u << SERIAL_tx_DRIVE_MODE_SHIFT)
#define SERIAL_tx_DM_DIG_HIZ         (0x01u << SERIAL_tx_DRIVE_MODE_SHIFT)
#define SERIAL_tx_DM_RES_UP          (0x02u << SERIAL_tx_DRIVE_MODE_SHIFT)
#define SERIAL_tx_DM_RES_DWN         (0x03u << SERIAL_tx_DRIVE_MODE_SHIFT)
#define SERIAL_tx_DM_OD_LO           (0x04u << SERIAL_tx_DRIVE_MODE_SHIFT)
#define SERIAL_tx_DM_OD_HI           (0x05u << SERIAL_tx_DRIVE_MODE_SHIFT)
#define SERIAL_tx_DM_STRONG          (0x06u << SERIAL_tx_DRIVE_MODE_SHIFT)
#define SERIAL_tx_DM_RES_UPDWN       (0x07u << SERIAL_tx_DRIVE_MODE_SHIFT)

/* Digital Port Constants */
#define SERIAL_tx_MASK               SERIAL_tx__MASK
#define SERIAL_tx_SHIFT              SERIAL_tx__SHIFT
#define SERIAL_tx_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define SERIAL_tx_PS                     (* (reg32 *) SERIAL_tx__PS)
/* Port Configuration */
#define SERIAL_tx_PC                     (* (reg32 *) SERIAL_tx__PC)
/* Data Register */
#define SERIAL_tx_DR                     (* (reg32 *) SERIAL_tx__DR)
/* Input Buffer Disable Override */
#define SERIAL_tx_INP_DIS                (* (reg32 *) SERIAL_tx__PC2)


#if defined(SERIAL_tx__INTSTAT)  /* Interrupt Registers */

    #define SERIAL_tx_INTSTAT                (* (reg32 *) SERIAL_tx__INTSTAT)

#endif /* Interrupt Registers */

#endif /* End Pins SERIAL_tx_H */


/* [] END OF FILE */
