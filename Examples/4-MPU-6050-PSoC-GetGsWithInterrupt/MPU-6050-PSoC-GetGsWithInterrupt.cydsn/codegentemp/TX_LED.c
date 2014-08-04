/*******************************************************************************
* File Name: TX_LED.c  
* Version 2.0
*
* Description:
*  This file contains API to enable firmware control of a Pins component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "TX_LED.h"

#define SetP4PinDriveMode(shift, mode)  \
    do { \
        TX_LED_PC =   (TX_LED_PC & \
                                (uint32)(~(uint32)(TX_LED_DRIVE_MODE_IND_MASK << (TX_LED_DRIVE_MODE_BITS * (shift))))) | \
                                (uint32)((uint32)(mode) << (TX_LED_DRIVE_MODE_BITS * (shift))); \
    } while (0)


/*******************************************************************************
* Function Name: TX_LED_Write
********************************************************************************
*
* Summary:
*  Assign a new value to the digital port's data output register.  
*
* Parameters:  
*  prtValue:  The value to be assigned to the Digital Port. 
*
* Return: 
*  None 
*  
*******************************************************************************/
void TX_LED_Write(uint8 value) 
{
    uint8 drVal = (uint8)(TX_LED_DR & (uint8)(~TX_LED_MASK));
    drVal = (drVal | ((uint8)(value << TX_LED_SHIFT) & TX_LED_MASK));
    TX_LED_DR = (uint32)drVal;
}


/*******************************************************************************
* Function Name: TX_LED_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  TX_LED_DM_STRONG     Strong Drive 
*  TX_LED_DM_OD_HI      Open Drain, Drives High 
*  TX_LED_DM_OD_LO      Open Drain, Drives Low 
*  TX_LED_DM_RES_UP     Resistive Pull Up 
*  TX_LED_DM_RES_DWN    Resistive Pull Down 
*  TX_LED_DM_RES_UPDWN  Resistive Pull Up/Down 
*  TX_LED_DM_DIG_HIZ    High Impedance Digital 
*  TX_LED_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void TX_LED_SetDriveMode(uint8 mode) 
{
	SetP4PinDriveMode(TX_LED__0__SHIFT, mode);
}


/*******************************************************************************
* Function Name: TX_LED_Read
********************************************************************************
*
* Summary:
*  Read the current value on the pins of the Digital Port in right justified 
*  form.
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value of the Digital Port as a right justified number
*  
* Note:
*  Macro TX_LED_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 TX_LED_Read(void) 
{
    return (uint8)((TX_LED_PS & TX_LED_MASK) >> TX_LED_SHIFT);
}


/*******************************************************************************
* Function Name: TX_LED_ReadDataReg
********************************************************************************
*
* Summary:
*  Read the current value assigned to a Digital Port's data output register
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value assigned to the Digital Port's data output register
*  
*******************************************************************************/
uint8 TX_LED_ReadDataReg(void) 
{
    return (uint8)((TX_LED_DR & TX_LED_MASK) >> TX_LED_SHIFT);
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(TX_LED_INTSTAT) 

    /*******************************************************************************
    * Function Name: TX_LED_ClearInterrupt
    ********************************************************************************
    *
    * Summary:
    *  Clears any active interrupts attached to port and returns the value of the 
    *  interrupt status register.
    *
    * Parameters:  
    *  None 
    *
    * Return: 
    *  Returns the value of the interrupt status register
    *  
    *******************************************************************************/
    uint8 TX_LED_ClearInterrupt(void) 
    {
		uint8 maskedStatus = (uint8)(TX_LED_INTSTAT & TX_LED_MASK);
		TX_LED_INTSTAT = maskedStatus;
        return maskedStatus >> TX_LED_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
