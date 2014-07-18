/*******************************************************************************
* File Name: SERIAL_rx.h  
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

#if !defined(CY_PINS_SERIAL_rx_H) /* Pins SERIAL_rx_H */
#define CY_PINS_SERIAL_rx_H

#include "cytypes.h"
#include "cyfitter.h"
#include "SERIAL_rx_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    SERIAL_rx_Write(uint8 value) ;
void    SERIAL_rx_SetDriveMode(uint8 mode) ;
uint8   SERIAL_rx_ReadDataReg(void) ;
uint8   SERIAL_rx_Read(void) ;
uint8   SERIAL_rx_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define SERIAL_rx_DRIVE_MODE_BITS        (3)
#define SERIAL_rx_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - SERIAL_rx_DRIVE_MODE_BITS))
#define SERIAL_rx_DRIVE_MODE_SHIFT       (0x00u)
#define SERIAL_rx_DRIVE_MODE_MASK        (0x07u << SERIAL_rx_DRIVE_MODE_SHIFT)

#define SERIAL_rx_DM_ALG_HIZ         (0x00u << SERIAL_rx_DRIVE_MODE_SHIFT)
#define SERIAL_rx_DM_DIG_HIZ         (0x01u << SERIAL_rx_DRIVE_MODE_SHIFT)
#define SERIAL_rx_DM_RES_UP          (0x02u << SERIAL_rx_DRIVE_MODE_SHIFT)
#define SERIAL_rx_DM_RES_DWN         (0x03u << SERIAL_rx_DRIVE_MODE_SHIFT)
#define SERIAL_rx_DM_OD_LO           (0x04u << SERIAL_rx_DRIVE_MODE_SHIFT)
#define SERIAL_rx_DM_OD_HI           (0x05u << SERIAL_rx_DRIVE_MODE_SHIFT)
#define SERIAL_rx_DM_STRONG          (0x06u << SERIAL_rx_DRIVE_MODE_SHIFT)
#define SERIAL_rx_DM_RES_UPDWN       (0x07u << SERIAL_rx_DRIVE_MODE_SHIFT)

/* Digital Port Constants */
#define SERIAL_rx_MASK               SERIAL_rx__MASK
#define SERIAL_rx_SHIFT              SERIAL_rx__SHIFT
#define SERIAL_rx_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define SERIAL_rx_PS                     (* (reg32 *) SERIAL_rx__PS)
/* Port Configuration */
#define SERIAL_rx_PC                     (* (reg32 *) SERIAL_rx__PC)
/* Data Register */
#define SERIAL_rx_DR                     (* (reg32 *) SERIAL_rx__DR)
/* Input Buffer Disable Override */
#define SERIAL_rx_INP_DIS                (* (reg32 *) SERIAL_rx__PC2)


#if defined(SERIAL_rx__INTSTAT)  /* Interrupt Registers */

    #define SERIAL_rx_INTSTAT                (* (reg32 *) SERIAL_rx__INTSTAT)

#endif /* Interrupt Registers */

#endif /* End Pins SERIAL_rx_H */


/* [] END OF FILE */
