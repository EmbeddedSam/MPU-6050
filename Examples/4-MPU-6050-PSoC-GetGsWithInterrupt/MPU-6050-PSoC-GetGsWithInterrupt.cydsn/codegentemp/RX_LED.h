/*******************************************************************************
* File Name: RX_LED.h  
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

#if !defined(CY_PINS_RX_LED_H) /* Pins RX_LED_H */
#define CY_PINS_RX_LED_H

#include "cytypes.h"
#include "cyfitter.h"
#include "RX_LED_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    RX_LED_Write(uint8 value) ;
void    RX_LED_SetDriveMode(uint8 mode) ;
uint8   RX_LED_ReadDataReg(void) ;
uint8   RX_LED_Read(void) ;
uint8   RX_LED_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define RX_LED_DRIVE_MODE_BITS        (3)
#define RX_LED_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - RX_LED_DRIVE_MODE_BITS))
#define RX_LED_DRIVE_MODE_SHIFT       (0x00u)
#define RX_LED_DRIVE_MODE_MASK        (0x07u << RX_LED_DRIVE_MODE_SHIFT)

#define RX_LED_DM_ALG_HIZ         (0x00u << RX_LED_DRIVE_MODE_SHIFT)
#define RX_LED_DM_DIG_HIZ         (0x01u << RX_LED_DRIVE_MODE_SHIFT)
#define RX_LED_DM_RES_UP          (0x02u << RX_LED_DRIVE_MODE_SHIFT)
#define RX_LED_DM_RES_DWN         (0x03u << RX_LED_DRIVE_MODE_SHIFT)
#define RX_LED_DM_OD_LO           (0x04u << RX_LED_DRIVE_MODE_SHIFT)
#define RX_LED_DM_OD_HI           (0x05u << RX_LED_DRIVE_MODE_SHIFT)
#define RX_LED_DM_STRONG          (0x06u << RX_LED_DRIVE_MODE_SHIFT)
#define RX_LED_DM_RES_UPDWN       (0x07u << RX_LED_DRIVE_MODE_SHIFT)

/* Digital Port Constants */
#define RX_LED_MASK               RX_LED__MASK
#define RX_LED_SHIFT              RX_LED__SHIFT
#define RX_LED_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define RX_LED_PS                     (* (reg32 *) RX_LED__PS)
/* Port Configuration */
#define RX_LED_PC                     (* (reg32 *) RX_LED__PC)
/* Data Register */
#define RX_LED_DR                     (* (reg32 *) RX_LED__DR)
/* Input Buffer Disable Override */
#define RX_LED_INP_DIS                (* (reg32 *) RX_LED__PC2)


#if defined(RX_LED__INTSTAT)  /* Interrupt Registers */

    #define RX_LED_INTSTAT                (* (reg32 *) RX_LED__INTSTAT)

#endif /* Interrupt Registers */

#endif /* End Pins RX_LED_H */


/* [] END OF FILE */
