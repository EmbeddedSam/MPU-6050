/*******************************************************************************
* File Name: UART_RX_FULL.h
* Version 1.70
*
*  Description:
*   Provides the function definitions for the Interrupt Controller.
*
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/
#if !defined(CY_ISR_UART_RX_FULL_H)
#define CY_ISR_UART_RX_FULL_H


#include <cytypes.h>
#include <cyfitter.h>

/* Interrupt Controller API. */
void UART_RX_FULL_Start(void);
void UART_RX_FULL_StartEx(cyisraddress address);
void UART_RX_FULL_Stop(void);

CY_ISR_PROTO(UART_RX_FULL_Interrupt);

void UART_RX_FULL_SetVector(cyisraddress address);
cyisraddress UART_RX_FULL_GetVector(void);

void UART_RX_FULL_SetPriority(uint8 priority);
uint8 UART_RX_FULL_GetPriority(void);

void UART_RX_FULL_Enable(void);
uint8 UART_RX_FULL_GetState(void);
void UART_RX_FULL_Disable(void);

void UART_RX_FULL_SetPending(void);
void UART_RX_FULL_ClearPending(void);


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the UART_RX_FULL ISR. */
#define UART_RX_FULL_INTC_VECTOR            ((reg32 *) UART_RX_FULL__INTC_VECT)

/* Address of the UART_RX_FULL ISR priority. */
#define UART_RX_FULL_INTC_PRIOR             ((reg32 *) UART_RX_FULL__INTC_PRIOR_REG)

/* Priority of the UART_RX_FULL interrupt. */
#define UART_RX_FULL_INTC_PRIOR_NUMBER      UART_RX_FULL__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable UART_RX_FULL interrupt. */
#define UART_RX_FULL_INTC_SET_EN            ((reg32 *) UART_RX_FULL__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the UART_RX_FULL interrupt. */
#define UART_RX_FULL_INTC_CLR_EN            ((reg32 *) UART_RX_FULL__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the UART_RX_FULL interrupt state to pending. */
#define UART_RX_FULL_INTC_SET_PD            ((reg32 *) UART_RX_FULL__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the UART_RX_FULL interrupt. */
#define UART_RX_FULL_INTC_CLR_PD            ((reg32 *) UART_RX_FULL__INTC_CLR_PD_REG)



#endif /* CY_ISR_UART_RX_FULL_H */


/* [] END OF FILE */
