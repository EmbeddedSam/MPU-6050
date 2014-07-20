/*******************************************************************************
* File Name: SERIAL_PM.c
* Version 1.20
*
* Description:
*  This file provides the source code to the Power Management support for
*  the SCB Component.
*
* Note:
*
********************************************************************************
* Copyright 2013-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "SERIAL.h"
#include "SERIAL_PVT.h"

#if(SERIAL_SCB_MODE_I2C_INC)
    #include "SERIAL_I2C_PVT.h"
#endif /* (SERIAL_SCB_MODE_I2C_INC) */

#if(SERIAL_SCB_MODE_EZI2C_INC)
    #include "SERIAL_EZI2C_PVT.h"
#endif /* (SERIAL_SCB_MODE_EZI2C_INC) */

#if(SERIAL_SCB_MODE_SPI_INC || SERIAL_SCB_MODE_UART_INC)
    #include "SERIAL_SPI_UART_PVT.h"
#endif /* (SERIAL_SCB_MODE_SPI_INC || SERIAL_SCB_MODE_UART_INC) */


/***************************************
*   Backup Structure declaration
***************************************/

#if(SERIAL_SCB_MODE_UNCONFIG_CONST_CFG || \
   (SERIAL_SCB_MODE_I2C_CONST_CFG   && (!SERIAL_I2C_WAKE_ENABLE_CONST))   || \
   (SERIAL_SCB_MODE_EZI2C_CONST_CFG && (!SERIAL_EZI2C_WAKE_ENABLE_CONST)) || \
   (SERIAL_SCB_MODE_SPI_CONST_CFG   && (!SERIAL_SPI_WAKE_ENABLE_CONST))   || \
   (SERIAL_SCB_MODE_UART_CONST_CFG  && (!SERIAL_UART_WAKE_ENABLE_CONST)))

    SERIAL_BACKUP_STRUCT SERIAL_backup =
    {
        0u, /* enableState */
    };
#endif


/*******************************************************************************
* Function Name: SERIAL_Sleep
********************************************************************************
*
* Summary:
*  Prepares the component to enter Deep Sleep.
*  The “Enable wakeup from Sleep Mode” selection has an influence on
*  this function implementation.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void SERIAL_Sleep(void)
{
#if(SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)

    if(SERIAL_SCB_WAKE_ENABLE_CHECK)
    {
        if(SERIAL_SCB_MODE_I2C_RUNTM_CFG)
        {
            SERIAL_I2CSaveConfig();
        }
        else if(SERIAL_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            SERIAL_EzI2CSaveConfig();
        }
    #if(!SERIAL_CY_SCBIP_V1_I2C_ONLY)
        else if(SERIAL_SCB_MODE_SPI_RUNTM_CFG)
        {
            SERIAL_SpiSaveConfig();
        }
        else if(SERIAL_SCB_MODE_UART_RUNTM_CFG)
        {
            SERIAL_UartSaveConfig();
        }
    #endif /* (!SERIAL_CY_SCBIP_V1_I2C_ONLY) */
        else
        {
            /* Unknown mode: do nothing */
        }
    }
    else
    {
        SERIAL_backup.enableState = (uint8) SERIAL_GET_CTRL_ENABLED;

        if(0u != SERIAL_backup.enableState)
        {
            SERIAL_Stop();
        }
    }

#else

    #if (SERIAL_SCB_MODE_I2C_CONST_CFG && SERIAL_I2C_WAKE_ENABLE_CONST)
        SERIAL_I2CSaveConfig();

    #elif (SERIAL_SCB_MODE_EZI2C_CONST_CFG && SERIAL_EZI2C_WAKE_ENABLE_CONST)
        SERIAL_EzI2CSaveConfig();

    #elif (SERIAL_SCB_MODE_SPI_CONST_CFG && SERIAL_SPI_WAKE_ENABLE_CONST)
        SERIAL_SpiSaveConfig();

    #elif (SERIAL_SCB_MODE_UART_CONST_CFG && SERIAL_UART_WAKE_ENABLE_CONST)
        SERIAL_UartSaveConfig();

    #else

        SERIAL_backup.enableState = (uint8) SERIAL_GET_CTRL_ENABLED;

        if(0u != SERIAL_backup.enableState)
        {
            SERIAL_Stop();
        }

    #endif /* defined (SERIAL_SCB_MODE_I2C_CONST_CFG) && (SERIAL_I2C_WAKE_ENABLE_CONST) */

#endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: SERIAL_Wakeup
********************************************************************************
*
* Summary:
*  Prepares the component for the Active mode operation after exiting Deep Sleep.
*  The “Enable wakeup from Sleep Mode” option has an influence on this function
*  implementation.
*  This function should not be called after exiting Sleep.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void SERIAL_Wakeup(void)
{
#if(SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)

    if(SERIAL_SCB_WAKE_ENABLE_CHECK)
    {
        if(SERIAL_SCB_MODE_I2C_RUNTM_CFG)
        {
            SERIAL_I2CRestoreConfig();
        }
        else if(SERIAL_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            SERIAL_EzI2CRestoreConfig();
        }
    #if(!SERIAL_CY_SCBIP_V1_I2C_ONLY)
        else if(SERIAL_SCB_MODE_SPI_RUNTM_CFG)
        {
            SERIAL_SpiRestoreConfig();
        }
        else if(SERIAL_SCB_MODE_UART_RUNTM_CFG)
        {
            SERIAL_UartRestoreConfig();
        }
    #endif /* (!SERIAL_CY_SCBIP_V1_I2C_ONLY) */
        else
        {
            /* Unknown mode: do nothing */
        }
    }
    else
    {
        if(0u != SERIAL_backup.enableState)
        {
            SERIAL_Enable();
        }
    }

#else

    #if (SERIAL_SCB_MODE_I2C_CONST_CFG  && SERIAL_I2C_WAKE_ENABLE_CONST)
        SERIAL_I2CRestoreConfig();

    #elif (SERIAL_SCB_MODE_EZI2C_CONST_CFG && SERIAL_EZI2C_WAKE_ENABLE_CONST)
        SERIAL_EzI2CRestoreConfig();

    #elif (SERIAL_SCB_MODE_SPI_CONST_CFG && SERIAL_SPI_WAKE_ENABLE_CONST)
        SERIAL_SpiRestoreConfig();

    #elif (SERIAL_SCB_MODE_UART_CONST_CFG && SERIAL_UART_WAKE_ENABLE_CONST)
        SERIAL_UartRestoreConfig();

    #else

        if(0u != SERIAL_backup.enableState)
        {
            SERIAL_Enable();
        }

    #endif /* (SERIAL_I2C_WAKE_ENABLE_CONST) */

#endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/* [] END OF FILE */
