/*******************************************************************************
* File Name: I2C_MPU6050.c
* Version 1.20
*
* Description:
*  This file provides the source code to the API for the SCB Component.
*
* Note:
*
*******************************************************************************
* Copyright 2013-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "I2C_MPU6050_PVT.h"

#if(I2C_MPU6050_SCB_MODE_I2C_INC)
    #include "I2C_MPU6050_I2C_PVT.h"
#endif /* (I2C_MPU6050_SCB_MODE_I2C_INC) */

#if(I2C_MPU6050_SCB_MODE_EZI2C_INC)
    #include "I2C_MPU6050_EZI2C_PVT.h"
#endif /* (I2C_MPU6050_SCB_MODE_EZI2C_INC) */

#if(I2C_MPU6050_SCB_MODE_SPI_INC || I2C_MPU6050_SCB_MODE_UART_INC)
    #include "I2C_MPU6050_SPI_UART_PVT.h"
#endif /* (I2C_MPU6050_SCB_MODE_SPI_INC || I2C_MPU6050_SCB_MODE_UART_INC) */


/***************************************
*    Run Time Configuration Vars
***************************************/

/* Stores internal component configuration for unconfigured mode */
#if(I2C_MPU6050_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common config vars */
    uint8 I2C_MPU6050_scbMode = I2C_MPU6050_SCB_MODE_UNCONFIG;
    uint8 I2C_MPU6050_scbEnableWake;
    uint8 I2C_MPU6050_scbEnableIntr;

    /* I2C config vars */
    uint8 I2C_MPU6050_mode;
    uint8 I2C_MPU6050_acceptAddr;

    /* SPI/UART config vars */
    volatile uint8 * I2C_MPU6050_rxBuffer;
    uint8  I2C_MPU6050_rxDataBits;
    uint32 I2C_MPU6050_rxBufferSize;

    volatile uint8 * I2C_MPU6050_txBuffer;
    uint8  I2C_MPU6050_txDataBits;
    uint32 I2C_MPU6050_txBufferSize;

    /* EZI2C config vars */
    uint8 I2C_MPU6050_numberOfAddr;
    uint8 I2C_MPU6050_subAddrSize;
#endif /* (I2C_MPU6050_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*     Common SCB Vars
***************************************/

uint8 I2C_MPU6050_initVar = 0u;

#if !defined (CY_REMOVE_I2C_MPU6050_CUSTOM_INTR_HANDLER)
    cyisraddress I2C_MPU6050_customIntrHandler = NULL;
#endif /* !defined (CY_REMOVE_I2C_MPU6050_CUSTOM_INTR_HANDLER) */


/***************************************
*    Private Function Prototypes
***************************************/

static void I2C_MPU6050_ScbEnableIntr(void);
static void I2C_MPU6050_ScbModeStop(void);


/*******************************************************************************
* Function Name: I2C_MPU6050_Init
********************************************************************************
*
* Summary:
*  Initializes the SCB component to operate in one of the selected configurations:
*  I2C, SPI, UART or EZ I2C.
*  When the configuration is set to “Unconfigured SCB”, this function does
*  not do any initialization. Use mode-specific initialization APIs instead:
*  SCB_I2CInit, SCB_SpiInit, SCB_UartInit or SCB_EzI2CInit.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void I2C_MPU6050_Init(void)
{
#if(I2C_MPU6050_SCB_MODE_UNCONFIG_CONST_CFG)
    if(I2C_MPU6050_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        I2C_MPU6050_initVar = 0u; /* Clear init var */
    }
    else
    {
        /* Initialization was done before call */
    }

#elif(I2C_MPU6050_SCB_MODE_I2C_CONST_CFG)
    I2C_MPU6050_I2CInit();

#elif(I2C_MPU6050_SCB_MODE_SPI_CONST_CFG)
    I2C_MPU6050_SpiInit();

#elif(I2C_MPU6050_SCB_MODE_UART_CONST_CFG)
    I2C_MPU6050_UartInit();

#elif(I2C_MPU6050_SCB_MODE_EZI2C_CONST_CFG)
    I2C_MPU6050_EzI2CInit();

#endif /* (I2C_MPU6050_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: I2C_MPU6050_Enable
********************************************************************************
*
* Summary:
*  Enables the SCB component operation.
*  The SCB configuration should be not changed when the component is enabled.
*  Any configuration changes should be made after disabling the component.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void I2C_MPU6050_Enable(void)
{
#if(I2C_MPU6050_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Enable SCB block, only if it is already configured */
    if(!I2C_MPU6050_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        I2C_MPU6050_CTRL_REG |= I2C_MPU6050_CTRL_ENABLED;

        I2C_MPU6050_ScbEnableIntr();
    }
#else
    I2C_MPU6050_CTRL_REG |= I2C_MPU6050_CTRL_ENABLED;

    I2C_MPU6050_ScbEnableIntr();
#endif /* (I2C_MPU6050_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: I2C_MPU6050_Start
********************************************************************************
*
* Summary:
*  Invokes SCB_Init() and SCB_Enable().
*  After this function call, the component is enabled and ready for operation.
*  When configuration is set to “Unconfigured SCB”, the component must first be
*  initialized to operate in one of the following configurations: I2C, SPI, UART
*  or EZ I2C. Otherwise this function does not enable the component.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  I2C_MPU6050_initVar - used to check initial configuration, modified
*  on first function call.
*
*******************************************************************************/
void I2C_MPU6050_Start(void)
{
    if(0u == I2C_MPU6050_initVar)
    {
        I2C_MPU6050_Init();
        I2C_MPU6050_initVar = 1u; /* Component was initialized */
    }

    I2C_MPU6050_Enable();
}


/*******************************************************************************
* Function Name: I2C_MPU6050_Stop
********************************************************************************
*
* Summary:
*  Disables the SCB component and its interrupt.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void I2C_MPU6050_Stop(void)
{
#if(I2C_MPU6050_SCB_IRQ_INTERNAL)
    I2C_MPU6050_DisableInt();
#endif /* (I2C_MPU6050_SCB_IRQ_INTERNAL) */

    I2C_MPU6050_CTRL_REG &= (uint32) ~I2C_MPU6050_CTRL_ENABLED;  /* Disable SCB block */

#if(I2C_MPU6050_SCB_IRQ_INTERNAL)
    I2C_MPU6050_ClearPendingInt();
#endif /* (I2C_MPU6050_SCB_IRQ_INTERNAL) */

    I2C_MPU6050_ScbModeStop(); /* Calls scbMode specific Stop function */
}


/*******************************************************************************
* Function Name: I2C_MPU6050_SetCustomInterruptHandler
********************************************************************************
*
* Summary:
*  Registers a function to be called by the internal interrupt handler.
*  First the function that is registered is called, then the internal interrupt
*  handler performs any operations such as software buffer management functions
*  before the interrupt returns.  It is the user's responsibility not to break the
*  software buffer operations. Only one custom handler is supported, which is
*  the function provided by the most recent call.
*  At initialization time no custom handler is registered.
*
* Parameters:
*  func: Pointer to the function to register.
*        The value NULL indicates to remove the current custom interrupt
*        handler.
*
* Return:
*  None
*
*******************************************************************************/
void I2C_MPU6050_SetCustomInterruptHandler(cyisraddress func)
{
#if !defined (CY_REMOVE_I2C_MPU6050_CUSTOM_INTR_HANDLER)
    I2C_MPU6050_customIntrHandler = func; /* Register interrupt handler */
#else
    if(NULL != func)
    {
        /* Suppress compiler warning */
    }
#endif /* !defined (CY_REMOVE_I2C_MPU6050_CUSTOM_INTR_HANDLER) */
}


/*******************************************************************************
* Function Name: I2C_MPU6050_ScbModeEnableIntr
********************************************************************************
*
* Summary:
*  Enables an interrupt for a specific mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void I2C_MPU6050_ScbEnableIntr(void)
{
#if(I2C_MPU6050_SCB_IRQ_INTERNAL)
    #if(I2C_MPU6050_SCB_MODE_UNCONFIG_CONST_CFG)
        /* Enable interrupt in the NVIC */
        if(0u != I2C_MPU6050_scbEnableIntr)
        {
            I2C_MPU6050_EnableInt();
        }
    #else
        I2C_MPU6050_EnableInt();

    #endif /* (I2C_MPU6050_SCB_MODE_UNCONFIG_CONST_CFG) */
#endif /* (I2C_MPU6050_SCB_IRQ_INTERNAL) */
}


/*******************************************************************************
* Function Name: I2C_MPU6050_ScbModeStop
********************************************************************************
*
* Summary:
*  Calls the Stop function for a specific operation mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void I2C_MPU6050_ScbModeStop(void)
{
#if(I2C_MPU6050_SCB_MODE_UNCONFIG_CONST_CFG)
    if(I2C_MPU6050_SCB_MODE_I2C_RUNTM_CFG)
    {
        I2C_MPU6050_I2CStop();
    }
    else if(I2C_MPU6050_SCB_MODE_EZI2C_RUNTM_CFG)
    {
        I2C_MPU6050_EzI2CStop();
    }
    else
    {
        /* Do nohing for other modes */
    }
#elif(I2C_MPU6050_SCB_MODE_I2C_CONST_CFG)
    I2C_MPU6050_I2CStop();

#elif(I2C_MPU6050_SCB_MODE_EZI2C_CONST_CFG)
    I2C_MPU6050_EzI2CStop();

#endif /* (I2C_MPU6050_SCB_MODE_UNCONFIG_CONST_CFG) */
}


#if(I2C_MPU6050_SCB_MODE_UNCONFIG_CONST_CFG)
    /*******************************************************************************
    * Function Name: I2C_MPU6050_SetPins
    ********************************************************************************
    *
    * Summary:
    *  Sets the pins settings accordingly to the selected operation mode.
    *  Only available in the Unconfigured operation mode. The mode specific
    *  initialization function calls it.
    *  Pins configuration is set by PSoC Creator when a specific mode of operation
    *  is selected in design time.
    *
    * Parameters:
    *  mode:      Mode of SCB operation.
    *  subMode:   Sub-mode of SCB operation. It is only required for SPI and UART
    *             modes.
    *  uartTxRx:  Direction for UART.
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void I2C_MPU6050_SetPins(uint32 mode, uint32 subMode, uint32 uartTxRx)
    {
        uint32 hsiomSel [I2C_MPU6050_SCB_PINS_NUMBER];
        uint32 pinsDm   [I2C_MPU6050_SCB_PINS_NUMBER];
        uint32 pinsInBuf = 0u;

        uint32 i;

        /* Set default HSIOM to GPIO and Drive Mode to Analog Hi-Z */
        for(i = 0u; i < I2C_MPU6050_SCB_PINS_NUMBER; i++)
        {
            hsiomSel[i]  = I2C_MPU6050_HSIOM_DEF_SEL;
            pinsDm[i]    = I2C_MPU6050_PIN_DM_ALG_HIZ;
        }

        if((I2C_MPU6050_SCB_MODE_I2C   == mode) ||
           (I2C_MPU6050_SCB_MODE_EZI2C == mode))
        {
            hsiomSel[I2C_MPU6050_MOSI_SCL_RX_PIN_INDEX] = I2C_MPU6050_HSIOM_I2C_SEL;
            hsiomSel[I2C_MPU6050_MISO_SDA_TX_PIN_INDEX] = I2C_MPU6050_HSIOM_I2C_SEL;

            pinsDm[I2C_MPU6050_MOSI_SCL_RX_PIN_INDEX] = I2C_MPU6050_PIN_DM_OD_LO;
            pinsDm[I2C_MPU6050_MISO_SDA_TX_PIN_INDEX] = I2C_MPU6050_PIN_DM_OD_LO;
        }
    #if(!I2C_MPU6050_CY_SCBIP_V1_I2C_ONLY)
        else if(I2C_MPU6050_SCB_MODE_SPI == mode)
        {
            hsiomSel[I2C_MPU6050_MOSI_SCL_RX_PIN_INDEX] = I2C_MPU6050_HSIOM_SPI_SEL;
            hsiomSel[I2C_MPU6050_MISO_SDA_TX_PIN_INDEX] = I2C_MPU6050_HSIOM_SPI_SEL;
            hsiomSel[I2C_MPU6050_SCLK_PIN_INDEX]        = I2C_MPU6050_HSIOM_SPI_SEL;

            if(I2C_MPU6050_SPI_SLAVE == subMode)
            {
                /* Slave */
                pinsDm[I2C_MPU6050_MOSI_SCL_RX_PIN_INDEX] = I2C_MPU6050_PIN_DM_DIG_HIZ;
                pinsDm[I2C_MPU6050_MISO_SDA_TX_PIN_INDEX] = I2C_MPU6050_PIN_DM_STRONG;
                pinsDm[I2C_MPU6050_SCLK_PIN_INDEX]        = I2C_MPU6050_PIN_DM_DIG_HIZ;

            #if(I2C_MPU6050_SS0_PIN)
                /* Only SS0 is valid choice for Slave */
                hsiomSel[I2C_MPU6050_SS0_PIN_INDEX] = I2C_MPU6050_HSIOM_SPI_SEL;
                pinsDm  [I2C_MPU6050_SS0_PIN_INDEX] = I2C_MPU6050_PIN_DM_DIG_HIZ;
            #endif /* (I2C_MPU6050_SS1_PIN) */

            #if(I2C_MPU6050_MISO_SDA_TX_PIN)
                /* Disable input buffer */
                 pinsInBuf |= I2C_MPU6050_MISO_SDA_TX_PIN_MASK;
            #endif /* (I2C_MPU6050_MISO_SDA_TX_PIN_PIN) */
            }
            else /* (Master) */
            {
                pinsDm[I2C_MPU6050_MOSI_SCL_RX_PIN_INDEX] = I2C_MPU6050_PIN_DM_STRONG;
                pinsDm[I2C_MPU6050_MISO_SDA_TX_PIN_INDEX] = I2C_MPU6050_PIN_DM_DIG_HIZ;
                pinsDm[I2C_MPU6050_SCLK_PIN_INDEX]        = I2C_MPU6050_PIN_DM_STRONG;

            #if(I2C_MPU6050_SS0_PIN)
                hsiomSel [I2C_MPU6050_SS0_PIN_INDEX] = I2C_MPU6050_HSIOM_SPI_SEL;
                pinsDm   [I2C_MPU6050_SS0_PIN_INDEX] = I2C_MPU6050_PIN_DM_STRONG;
                pinsInBuf                                |= I2C_MPU6050_SS0_PIN_MASK;
            #endif /* (I2C_MPU6050_SS0_PIN) */

            #if(I2C_MPU6050_SS1_PIN)
                hsiomSel [I2C_MPU6050_SS1_PIN_INDEX] = I2C_MPU6050_HSIOM_SPI_SEL;
                pinsDm   [I2C_MPU6050_SS1_PIN_INDEX] = I2C_MPU6050_PIN_DM_STRONG;
                pinsInBuf                                |= I2C_MPU6050_SS1_PIN_MASK;
            #endif /* (I2C_MPU6050_SS1_PIN) */

            #if(I2C_MPU6050_SS2_PIN)
                hsiomSel [I2C_MPU6050_SS2_PIN_INDEX] = I2C_MPU6050_HSIOM_SPI_SEL;
                pinsDm   [I2C_MPU6050_SS2_PIN_INDEX] = I2C_MPU6050_PIN_DM_STRONG;
                pinsInBuf                                |= I2C_MPU6050_SS2_PIN_MASK;
            #endif /* (I2C_MPU6050_SS2_PIN) */

            #if(I2C_MPU6050_SS3_PIN)
                hsiomSel [I2C_MPU6050_SS3_PIN_INDEX] = I2C_MPU6050_HSIOM_SPI_SEL;
                pinsDm   [I2C_MPU6050_SS3_PIN_INDEX] = I2C_MPU6050_PIN_DM_STRONG;
                pinsInBuf                                |= I2C_MPU6050_SS3_PIN_MASK;
            #endif /* (I2C_MPU6050_SS2_PIN) */

                /* Disable input buffers */
            #if(I2C_MPU6050_MOSI_SCL_RX_PIN)
                pinsInBuf |= I2C_MPU6050_MOSI_SCL_RX_PIN_MASK;
            #endif /* (I2C_MPU6050_MOSI_SCL_RX_PIN) */

             #if(I2C_MPU6050_MOSI_SCL_RX_WAKE_PIN)
                pinsInBuf |= I2C_MPU6050_MOSI_SCL_RX_WAKE_PIN_MASK;
            #endif /* (I2C_MPU6050_MOSI_SCL_RX_WAKE_PIN) */

            #if(I2C_MPU6050_SCLK_PIN)
                pinsInBuf |= I2C_MPU6050_SCLK_PIN_MASK;
            #endif /* (I2C_MPU6050_SCLK_PIN) */
            }
        }
        else /* UART */
        {
            if(I2C_MPU6050_UART_MODE_SMARTCARD == subMode)
            {
                /* SmartCard */
                hsiomSel[I2C_MPU6050_MISO_SDA_TX_PIN_INDEX] = I2C_MPU6050_HSIOM_UART_SEL;
                pinsDm  [I2C_MPU6050_MISO_SDA_TX_PIN_INDEX] = I2C_MPU6050_PIN_DM_OD_LO;
            }
            else /* Standard or IrDA */
            {
                if(0u != (I2C_MPU6050_UART_RX & uartTxRx))
                {
                    hsiomSel[I2C_MPU6050_MOSI_SCL_RX_PIN_INDEX] = I2C_MPU6050_HSIOM_UART_SEL;
                    pinsDm  [I2C_MPU6050_MOSI_SCL_RX_PIN_INDEX] = I2C_MPU6050_PIN_DM_DIG_HIZ;
                }

                if(0u != (I2C_MPU6050_UART_TX & uartTxRx))
                {
                    hsiomSel[I2C_MPU6050_MISO_SDA_TX_PIN_INDEX] = I2C_MPU6050_HSIOM_UART_SEL;
                    pinsDm  [I2C_MPU6050_MISO_SDA_TX_PIN_INDEX] = I2C_MPU6050_PIN_DM_STRONG;

                #if(I2C_MPU6050_MISO_SDA_TX_PIN)
                     pinsInBuf |= I2C_MPU6050_MISO_SDA_TX_PIN_MASK;
                #endif /* (I2C_MPU6050_MISO_SDA_TX_PIN_PIN) */
                }
            }
        }
    #endif /* (!I2C_MPU6050_CY_SCBIP_V1_I2C_ONLY) */

    /* Configure pins: set HSIOM, DM and InputBufEnable */
    /* Note: the DR register settigns do not effect the pin output if HSIOM is other than GPIO */

    #if(I2C_MPU6050_MOSI_SCL_RX_PIN)
        I2C_MPU6050_SET_HSIOM_SEL(I2C_MPU6050_MOSI_SCL_RX_HSIOM_REG,
                                       I2C_MPU6050_MOSI_SCL_RX_HSIOM_MASK,
                                       I2C_MPU6050_MOSI_SCL_RX_HSIOM_POS,
                                       hsiomSel[I2C_MPU6050_MOSI_SCL_RX_PIN_INDEX]);

        I2C_MPU6050_spi_mosi_i2c_scl_uart_rx_SetDriveMode((uint8) pinsDm[I2C_MPU6050_MOSI_SCL_RX_PIN_INDEX]);

        I2C_MPU6050_SET_INP_DIS(I2C_MPU6050_spi_mosi_i2c_scl_uart_rx_INP_DIS,
                                     I2C_MPU6050_spi_mosi_i2c_scl_uart_rx_MASK,
                                     (0u != (pinsInBuf & I2C_MPU6050_MOSI_SCL_RX_PIN_MASK)));
    #endif /* (I2C_MPU6050_MOSI_SCL_RX_PIN) */

    #if(I2C_MPU6050_MOSI_SCL_RX_WAKE_PIN)
        I2C_MPU6050_SET_HSIOM_SEL(I2C_MPU6050_MOSI_SCL_RX_WAKE_HSIOM_REG,
                                       I2C_MPU6050_MOSI_SCL_RX_WAKE_HSIOM_MASK,
                                       I2C_MPU6050_MOSI_SCL_RX_WAKE_HSIOM_POS,
                                       hsiomSel[I2C_MPU6050_MOSI_SCL_RX_WAKE_PIN_INDEX]);

        I2C_MPU6050_spi_mosi_i2c_scl_uart_rx_wake_SetDriveMode((uint8)
                                                               pinsDm[I2C_MPU6050_MOSI_SCL_RX_WAKE_PIN_INDEX]);

        I2C_MPU6050_SET_INP_DIS(I2C_MPU6050_spi_mosi_i2c_scl_uart_rx_wake_INP_DIS,
                                     I2C_MPU6050_spi_mosi_i2c_scl_uart_rx_wake_MASK,
                                     (0u != (pinsInBuf & I2C_MPU6050_MOSI_SCL_RX_WAKE_PIN_MASK)));

         /* Set interrupt on falling edge */
        I2C_MPU6050_SET_INCFG_TYPE(I2C_MPU6050_MOSI_SCL_RX_WAKE_INTCFG_REG,
                                        I2C_MPU6050_MOSI_SCL_RX_WAKE_INTCFG_TYPE_MASK,
                                        I2C_MPU6050_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS,
                                        I2C_MPU6050_INTCFG_TYPE_FALLING_EDGE);
    #endif /* (I2C_MPU6050_MOSI_SCL_RX_WAKE_PIN) */

    #if(I2C_MPU6050_MISO_SDA_TX_PIN)
        I2C_MPU6050_SET_HSIOM_SEL(I2C_MPU6050_MISO_SDA_TX_HSIOM_REG,
                                       I2C_MPU6050_MISO_SDA_TX_HSIOM_MASK,
                                       I2C_MPU6050_MISO_SDA_TX_HSIOM_POS,
                                       hsiomSel[I2C_MPU6050_MISO_SDA_TX_PIN_INDEX]);

        I2C_MPU6050_spi_miso_i2c_sda_uart_tx_SetDriveMode((uint8) pinsDm[I2C_MPU6050_MISO_SDA_TX_PIN_INDEX]);

        I2C_MPU6050_SET_INP_DIS(I2C_MPU6050_spi_miso_i2c_sda_uart_tx_INP_DIS,
                                     I2C_MPU6050_spi_miso_i2c_sda_uart_tx_MASK,
                                    (0u != (pinsInBuf & I2C_MPU6050_MISO_SDA_TX_PIN_MASK)));
    #endif /* (I2C_MPU6050_MOSI_SCL_RX_PIN) */

    #if(I2C_MPU6050_SCLK_PIN)
        I2C_MPU6050_SET_HSIOM_SEL(I2C_MPU6050_SCLK_HSIOM_REG, I2C_MPU6050_SCLK_HSIOM_MASK,
                                       I2C_MPU6050_SCLK_HSIOM_POS, hsiomSel[I2C_MPU6050_SCLK_PIN_INDEX]);

        I2C_MPU6050_spi_sclk_SetDriveMode((uint8) pinsDm[I2C_MPU6050_SCLK_PIN_INDEX]);

        I2C_MPU6050_SET_INP_DIS(I2C_MPU6050_spi_sclk_INP_DIS,
                             I2C_MPU6050_spi_sclk_MASK,
                            (0u != (pinsInBuf & I2C_MPU6050_SCLK_PIN_MASK)));
    #endif /* (I2C_MPU6050_SCLK_PIN) */

    #if(I2C_MPU6050_SS0_PIN)
        I2C_MPU6050_SET_HSIOM_SEL(I2C_MPU6050_SS0_HSIOM_REG, I2C_MPU6050_SS0_HSIOM_MASK,
                                       I2C_MPU6050_SS0_HSIOM_POS, hsiomSel[I2C_MPU6050_SS0_PIN_INDEX]);

        I2C_MPU6050_spi_ss0_SetDriveMode((uint8) pinsDm[I2C_MPU6050_SS0_PIN_INDEX]);

        I2C_MPU6050_SET_INP_DIS(I2C_MPU6050_spi_ss0_INP_DIS,
                                     I2C_MPU6050_spi_ss0_MASK,
                                     (0u != (pinsInBuf & I2C_MPU6050_SS0_PIN_MASK)));
    #endif /* (I2C_MPU6050_SS1_PIN) */

    #if(I2C_MPU6050_SS1_PIN)
        I2C_MPU6050_SET_HSIOM_SEL(I2C_MPU6050_SS1_HSIOM_REG, I2C_MPU6050_SS1_HSIOM_MASK,
                                       I2C_MPU6050_SS1_HSIOM_POS, hsiomSel[I2C_MPU6050_SS1_PIN_INDEX]);

        I2C_MPU6050_spi_ss1_SetDriveMode((uint8) pinsDm[I2C_MPU6050_SS1_PIN_INDEX]);

        I2C_MPU6050_SET_INP_DIS(I2C_MPU6050_spi_ss1_INP_DIS,
                                     I2C_MPU6050_spi_ss1_MASK,
                                     (0u != (pinsInBuf & I2C_MPU6050_SS1_PIN_MASK)));
    #endif /* (I2C_MPU6050_SS1_PIN) */

    #if(I2C_MPU6050_SS2_PIN)
        I2C_MPU6050_SET_HSIOM_SEL(I2C_MPU6050_SS2_HSIOM_REG, I2C_MPU6050_SS2_HSIOM_MASK,
                                       I2C_MPU6050_SS2_HSIOM_POS, hsiomSel[I2C_MPU6050_SS2_PIN_INDEX]);

        I2C_MPU6050_spi_ss2_SetDriveMode((uint8) pinsDm[I2C_MPU6050_SS2_PIN_INDEX]);

        I2C_MPU6050_SET_INP_DIS(I2C_MPU6050_spi_ss2_INP_DIS,
                                     I2C_MPU6050_spi_ss2_MASK,
                                     (0u != (pinsInBuf & I2C_MPU6050_SS2_PIN_MASK)));
    #endif /* (I2C_MPU6050_SS2_PIN) */

    #if(I2C_MPU6050_SS3_PIN)
        I2C_MPU6050_SET_HSIOM_SEL(I2C_MPU6050_SS3_HSIOM_REG,  I2C_MPU6050_SS3_HSIOM_MASK,
                                       I2C_MPU6050_SS3_HSIOM_POS, hsiomSel[I2C_MPU6050_SS3_PIN_INDEX]);

        I2C_MPU6050_spi_ss3_SetDriveMode((uint8) pinsDm[I2C_MPU6050_SS3_PIN_INDEX]);

        I2C_MPU6050_SET_INP_DIS(I2C_MPU6050_spi_ss3_INP_DIS,
                                     I2C_MPU6050_spi_ss3_MASK,
                                     (0u != (pinsInBuf & I2C_MPU6050_SS3_PIN_MASK)));
    #endif /* (I2C_MPU6050_SS3_PIN) */
    }

#endif /* (I2C_MPU6050_SCB_MODE_UNCONFIG_CONST_CFG) */


/* [] END OF FILE */
