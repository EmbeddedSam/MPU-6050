/*******************************************************************************
* File Name: SERIAL_BOOT.c
* Version 1.20
*
* Description:
*  This file provides the source code to the API for the bootloader
*  communication support in SCB Component.
*
* Note:
*
********************************************************************************
* Copyright 2013-2013-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "SERIAL.h"

#if(SERIAL_SCB_MODE_I2C_INC)
    #include "SERIAL_I2C.h"
#endif /* (SERIAL_SCB_MODE_I2C_INC) */

#if(SERIAL_SCB_MODE_EZI2C_INC)
    #include "SERIAL_EZI2C.h"
#endif /* (SERIAL_SCB_MODE_EZI2C_INC) */

#if(SERIAL_SCB_MODE_SPI_INC || SERIAL_SCB_MODE_UART_INC)
    #include "SERIAL_SPI_UART.h"
#endif /* (SERIAL_SCB_MODE_SPI_INC || SERIAL_SCB_MODE_UART_INC) */


#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_SERIAL) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))

/*******************************************************************************
* Function Name: SERIAL_CyBtldrCommStart
********************************************************************************
*
* Summary:
*  Calls the CyBtldrCommStart function of the bootloader communication
*  component for the selected mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void SERIAL_CyBtldrCommStart(void)
{
    #if(SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
        if(SERIAL_SCB_MODE_I2C_RUNTM_CFG)
        {
            SERIAL_I2CCyBtldrCommStart();
        }
        else if(SERIAL_SCB_MODE_SPI_RUNTM_CFG)
        {
            SERIAL_SpiCyBtldrCommStart();
        }
        else if(SERIAL_SCB_MODE_UART_RUNTM_CFG)
        {
            SERIAL_UartCyBtldrCommStart();
        }
        else if(SERIAL_SCB_MODE_EZI2C_RUNTM_CFG)
        {
             SERIAL_EzI2CCyBtldrCommStart();
        }
        else
        {
            /* Unknown mode: do nothing */
        }
    #elif(SERIAL_SCB_MODE_I2C_CONST_CFG)
        SERIAL_I2CCyBtldrCommStart();

    #elif(SERIAL_SCB_MODE_SPI_CONST_CFG)
        SERIAL_SpiCyBtldrCommStart();

    #elif(SERIAL_SCB_MODE_UART_CONST_CFG)
        SERIAL_UartCyBtldrCommStart();

    #elif(SERIAL_SCB_MODE_EZI2C_CONST_CFG)
        SERIAL_EzI2CCyBtldrCommStart();

    #else
        /* Do nothing */

    #endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: SERIAL_CyBtldrCommStop
********************************************************************************
*
* Summary:
*  Calls CyBtldrCommStop function function of the bootloader communication
*  component for selected mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void SERIAL_CyBtldrCommStop(void)
{
    #if(SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
        if(SERIAL_SCB_MODE_I2C_RUNTM_CFG)
        {
            SERIAL_I2CCyBtldrCommStop();
        }
        else if(SERIAL_SCB_MODE_SPI_RUNTM_CFG)
        {
            SERIAL_SpiCyBtldrCommStop();
        }
        else if(SERIAL_SCB_MODE_UART_RUNTM_CFG)
        {
            SERIAL_UartCyBtldrCommStop();
        }
        else if(SERIAL_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            SERIAL_EzI2CCyBtldrCommStop();
        }
        else
        {
            /* Unknown mode: do nothing */
        }
    #elif(SERIAL_SCB_MODE_I2C_CONST_CFG)
        SERIAL_I2CCyBtldrCommStop();

    #elif(SERIAL_SCB_MODE_SPI_CONST_CFG)
        SERIAL_SpiCyBtldrCommStop();

    #elif(SERIAL_SCB_MODE_UART_CONST_CFG)
        SERIAL_UartCyBtldrCommStop();

    #elif(SERIAL_SCB_MODE_EZI2C_CONST_CFG)
        SERIAL_EzI2CCyBtldrCommStop();

    #else
        /* Do nothing */

    #endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: SERIAL_CyBtldrCommReset
********************************************************************************
*
* Summary:
*  Calls the CyBtldrCommReset function of the bootloader communication
*  component for the selected mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void SERIAL_CyBtldrCommReset(void)
{
    #if(SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
        if(SERIAL_SCB_MODE_I2C_RUNTM_CFG)
        {
            SERIAL_I2CCyBtldrCommReset();
        }
        else if(SERIAL_SCB_MODE_SPI_RUNTM_CFG)
        {
            SERIAL_SpiCyBtldrCommReset();
        }
        else if(SERIAL_SCB_MODE_UART_RUNTM_CFG)
        {
            SERIAL_UartCyBtldrCommReset();
        }
        else if(SERIAL_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            SERIAL_EzI2CCyBtldrCommReset();
        }
        else
        {
            /* Unknown mode: do nothing */
        }
    #elif(SERIAL_SCB_MODE_I2C_CONST_CFG)
        SERIAL_I2CCyBtldrCommReset();

    #elif(SERIAL_SCB_MODE_SPI_CONST_CFG)
        SERIAL_SpiCyBtldrCommReset();

    #elif(SERIAL_SCB_MODE_UART_CONST_CFG)
        SERIAL_UartCyBtldrCommReset();

    #elif(SERIAL_SCB_MODE_EZI2C_CONST_CFG)
        SERIAL_EzI2CCyBtldrCommReset();

    #else
        /* Do nothing */

    #endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: SERIAL_CyBtldrCommRead
********************************************************************************
*
* Summary:
*  Calls the CyBtldrCommRead function of the bootloader communication
*  component for the selected mode.
*
* Parameters:
*  pData:    Pointer to storage for the block of data to be read from the
*            bootloader host
*  size:     Number of bytes to be read.
*  count:    Pointer to the variable to write the number of bytes actually
*            read.
*  timeOut:  Number of units in 10 ms to wait before returning because of a
*            timeout.
*
* Return:
*  Returns CYRET_SUCCESS if no problem was encountered or returns the value
*  that best describes the problem.
*
*******************************************************************************/
cystatus SERIAL_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut)
{
    cystatus status;

    #if(SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
        if(SERIAL_SCB_MODE_I2C_RUNTM_CFG)
        {
            status = SERIAL_I2CCyBtldrCommRead(pData, size, count, timeOut);
        }
        else if(SERIAL_SCB_MODE_SPI_RUNTM_CFG)
        {
            status = SERIAL_SpiCyBtldrCommRead(pData, size, count, timeOut);
        }
        else if(SERIAL_SCB_MODE_UART_RUNTM_CFG)
        {
            status = SERIAL_UartCyBtldrCommRead(pData, size, count, timeOut);
        }
        else if(SERIAL_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            status = SERIAL_EzI2CCyBtldrCommRead(pData, size, count, timeOut);
        }
        else
        {
            status = CYRET_INVALID_STATE; /* Unknown mode: return status */
        }

    #elif(SERIAL_SCB_MODE_I2C_CONST_CFG)
        status = SERIAL_I2CCyBtldrCommRead(pData, size, count, timeOut);

    #elif(SERIAL_SCB_MODE_SPI_CONST_CFG)
        status = SERIAL_SpiCyBtldrCommRead(pData, size, count, timeOut);

    #elif(SERIAL_SCB_MODE_UART_CONST_CFG)
        status = SERIAL_UartCyBtldrCommRead(pData, size, count, timeOut);

    #elif(SERIAL_SCB_MODE_EZI2C_CONST_CFG)
        status = SERIAL_EzI2CCyBtldrCommRead(pData, size, count, timeOut);

    #else
        status = CYRET_INVALID_STATE; /* Unknown mode: return status */

    #endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */

    return(status);
}


/*******************************************************************************
* Function Name: SERIAL_CyBtldrCommWrite
********************************************************************************
*
* Summary:
*  Calls the CyBtldrCommWrite  function of the bootloader communication
*  component for the selected mode.
*
* Parameters:
*  pData:    Pointer to the block of data to be written to the bootloader host.
*  size:     Number of bytes to be written.
*  count:    Pointer to the variable to write the number of bytes actually
*            written.
*  timeOut:  Number of units in 10 ms to wait before returning because of a
*            timeout.
*
* Return:
*  Returns CYRET_SUCCESS if no problem was encountered or returns the value
*  that best describes the problem.
*
*******************************************************************************/
cystatus SERIAL_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut)
{
    cystatus status;

    #if(SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
        if(SERIAL_SCB_MODE_I2C_RUNTM_CFG)
        {
            status = SERIAL_I2CCyBtldrCommWrite(pData, size, count, timeOut);
        }
        else if(SERIAL_SCB_MODE_SPI_RUNTM_CFG)
        {
            status = SERIAL_SpiCyBtldrCommWrite(pData, size, count, timeOut);
        }
        else if(SERIAL_SCB_MODE_UART_RUNTM_CFG)
        {
            status = SERIAL_UartCyBtldrCommWrite(pData, size, count, timeOut);
        }
        else if(SERIAL_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            status = SERIAL_EzI2CCyBtldrCommWrite(pData, size, count, timeOut);
        }
        else
        {
            status = CYRET_INVALID_STATE; /* Unknown mode */
        }
    #elif(SERIAL_SCB_MODE_I2C_CONST_CFG)
        status = SERIAL_I2CCyBtldrCommWrite(pData, size, count, timeOut);

    #elif(SERIAL_SCB_MODE_SPI_CONST_CFG)
        status = SERIAL_SpiCyBtldrCommWrite(pData, size, count, timeOut);

    #elif(SERIAL_SCB_MODE_UART_CONST_CFG)
        status = SERIAL_UartCyBtldrCommWrite(pData, size, count, timeOut);

    #elif(SERIAL_SCB_MODE_EZI2C_CONST_CFG)
        status = SERIAL_EzI2CCyBtldrCommWrite(pData, size, count, timeOut);

    #else
        status = CYRET_INVALID_STATE; /* Unknown mode */

    #endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */

    return(status);
}

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_SERIAL) || \
                                                    (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface)) */


/* [] END OF FILE */
