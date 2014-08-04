/*******************************************************************************
* File Name: TX_LED.h  
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

#if !defined(CY_PINS_TX_LED_H) /* Pins TX_LED_H */
#define CY_PINS_TX_LED_H

#include "cytypes.h"
#include "cyfitter.h"
#include "TX_LED_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    TX_LED_Write(uint8 value) ;
void    TX_LED_SetDriveMode(uint8 mode) ;
uint8   TX_LED_ReadDataReg(void) ;
uint8   TX_LED_Read(void) ;
uint8   TX_LED_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define TX_LED_DRIVE_MODE_BITS        (3)
#define TX_LED_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - TX_LED_DRIVE_MODE_BITS))
#define TX_LED_DRIVE_MODE_SHIFT       (0x00u)
#define TX_LED_DRIVE_MODE_MASK        (0x07u << TX_LED_DRIVE_MODE_SHIFT)

#define TX_LED_DM_ALG_HIZ         (0x00u << TX_LED_DRIVE_MODE_SHIFT)
#define TX_LED_DM_DIG_HIZ         (0x01u << TX_LED_DRIVE_MODE_SHIFT)
#define TX_LED_DM_RES_UP          (0x02u << TX_LED_DRIVE_MODE_SHIFT)
#define TX_LED_DM_RES_DWN         (0x03u << TX_LED_DRIVE_MODE_SHIFT)
#define TX_LED_DM_OD_LO           (0x04u << TX_LED_DRIVE_MODE_SHIFT)
#define TX_LED_DM_OD_HI           (0x05u << TX_LED_DRIVE_MODE_SHIFT)
#define TX_LED_DM_STRONG          (0x06u << TX_LED_DRIVE_MODE_SHIFT)
#define TX_LED_DM_RES_UPDWN       (0x07u << TX_LED_DRIVE_MODE_SHIFT)

/* Digital Port Constants */
#define TX_LED_MASK               TX_LED__MASK
#define TX_LED_SHIFT              TX_LED__SHIFT
#define TX_LED_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define TX_LED_PS                     (* (reg32 *) TX_LED__PS)
/* Port Configuration */
#define TX_LED_PC                     (* (reg32 *) TX_LED__PC)
/* Data Register */
#define TX_LED_DR                     (* (reg32 *) TX_LED__DR)
/* Input Buffer Disable Override */
#define TX_LED_INP_DIS                (* (reg32 *) TX_LED__PC2)


#if defined(TX_LED__INTSTAT)  /* Interrupt Registers */

    #define TX_LED_INTSTAT                (* (reg32 *) TX_LED__INTSTAT)

#endif /* Interrupt Registers */

#endif /* End Pins TX_LED_H */


/* [] END OF FILE */
