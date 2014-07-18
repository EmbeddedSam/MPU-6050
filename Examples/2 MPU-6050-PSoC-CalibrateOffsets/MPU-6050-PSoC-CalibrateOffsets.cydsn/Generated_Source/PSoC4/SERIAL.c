/*******************************************************************************
* File Name: SERIAL.c
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
*    Run Time Configuration Vars
***************************************/

/* Stores internal component configuration for unconfigured mode */
#if(SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common config vars */
    uint8 SERIAL_scbMode = SERIAL_SCB_MODE_UNCONFIG;
    uint8 SERIAL_scbEnableWake;
    uint8 SERIAL_scbEnableIntr;

    /* I2C config vars */
    uint8 SERIAL_mode;
    uint8 SERIAL_acceptAddr;

    /* SPI/UART config vars */
    volatile uint8 * SERIAL_rxBuffer;
    uint8  SERIAL_rxDataBits;
    uint32 SERIAL_rxBufferSize;

    volatile uint8 * SERIAL_txBuffer;
    uint8  SERIAL_txDataBits;
    uint32 SERIAL_txBufferSize;

    /* EZI2C config vars */
    uint8 SERIAL_numberOfAddr;
    uint8 SERIAL_subAddrSize;
#endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*     Common SCB Vars
***************************************/

uint8 SERIAL_initVar = 0u;

#if !defined (CY_REMOVE_SERIAL_CUSTOM_INTR_HANDLER)
    cyisraddress SERIAL_customIntrHandler = NULL;
#endif /* !defined (CY_REMOVE_SERIAL_CUSTOM_INTR_HANDLER) */


/***************************************
*    Private Function Prototypes
***************************************/

static void SERIAL_ScbEnableIntr(void);
static void SERIAL_ScbModeStop(void);


/*******************************************************************************
* Function Name: SERIAL_Init
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
void SERIAL_Init(void)
{
#if(SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
    if(SERIAL_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        SERIAL_initVar = 0u; /* Clear init var */
    }
    else
    {
        /* Initialization was done before call */
    }

#elif(SERIAL_SCB_MODE_I2C_CONST_CFG)
    SERIAL_I2CInit();

#elif(SERIAL_SCB_MODE_SPI_CONST_CFG)
    SERIAL_SpiInit();

#elif(SERIAL_SCB_MODE_UART_CONST_CFG)
    SERIAL_UartInit();

#elif(SERIAL_SCB_MODE_EZI2C_CONST_CFG)
    SERIAL_EzI2CInit();

#endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: SERIAL_Enable
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
void SERIAL_Enable(void)
{
#if(SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Enable SCB block, only if it is already configured */
    if(!SERIAL_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        SERIAL_CTRL_REG |= SERIAL_CTRL_ENABLED;

        SERIAL_ScbEnableIntr();
    }
#else
    SERIAL_CTRL_REG |= SERIAL_CTRL_ENABLED;

    SERIAL_ScbEnableIntr();
#endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: SERIAL_Start
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
*  SERIAL_initVar - used to check initial configuration, modified
*  on first function call.
*
*******************************************************************************/
void SERIAL_Start(void)
{
    if(0u == SERIAL_initVar)
    {
        SERIAL_Init();
        SERIAL_initVar = 1u; /* Component was initialized */
    }

    SERIAL_Enable();
}


/*******************************************************************************
* Function Name: SERIAL_Stop
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
void SERIAL_Stop(void)
{
#if(SERIAL_SCB_IRQ_INTERNAL)
    SERIAL_DisableInt();
#endif /* (SERIAL_SCB_IRQ_INTERNAL) */

    SERIAL_CTRL_REG &= (uint32) ~SERIAL_CTRL_ENABLED;  /* Disable SCB block */

#if(SERIAL_SCB_IRQ_INTERNAL)
    SERIAL_ClearPendingInt();
#endif /* (SERIAL_SCB_IRQ_INTERNAL) */

    SERIAL_ScbModeStop(); /* Calls scbMode specific Stop function */
}


/*******************************************************************************
* Function Name: SERIAL_SetCustomInterruptHandler
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
void SERIAL_SetCustomInterruptHandler(cyisraddress func)
{
#if !defined (CY_REMOVE_SERIAL_CUSTOM_INTR_HANDLER)
    SERIAL_customIntrHandler = func; /* Register interrupt handler */
#else
    if(NULL != func)
    {
        /* Suppress compiler warning */
    }
#endif /* !defined (CY_REMOVE_SERIAL_CUSTOM_INTR_HANDLER) */
}


/*******************************************************************************
* Function Name: SERIAL_ScbModeEnableIntr
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
static void SERIAL_ScbEnableIntr(void)
{
#if(SERIAL_SCB_IRQ_INTERNAL)
    #if(SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
        /* Enable interrupt in the NVIC */
        if(0u != SERIAL_scbEnableIntr)
        {
            SERIAL_EnableInt();
        }
    #else
        SERIAL_EnableInt();

    #endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */
#endif /* (SERIAL_SCB_IRQ_INTERNAL) */
}


/*******************************************************************************
* Function Name: SERIAL_ScbModeStop
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
static void SERIAL_ScbModeStop(void)
{
#if(SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
    if(SERIAL_SCB_MODE_I2C_RUNTM_CFG)
    {
        SERIAL_I2CStop();
    }
    else if(SERIAL_SCB_MODE_EZI2C_RUNTM_CFG)
    {
        SERIAL_EzI2CStop();
    }
    else
    {
        /* Do nohing for other modes */
    }
#elif(SERIAL_SCB_MODE_I2C_CONST_CFG)
    SERIAL_I2CStop();

#elif(SERIAL_SCB_MODE_EZI2C_CONST_CFG)
    SERIAL_EzI2CStop();

#endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */
}


#if(SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
    /*******************************************************************************
    * Function Name: SERIAL_SetPins
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
    void SERIAL_SetPins(uint32 mode, uint32 subMode, uint32 uartTxRx)
    {
        uint32 hsiomSel [SERIAL_SCB_PINS_NUMBER];
        uint32 pinsDm   [SERIAL_SCB_PINS_NUMBER];
        uint32 pinsInBuf = 0u;

        uint32 i;

        /* Set default HSIOM to GPIO and Drive Mode to Analog Hi-Z */
        for(i = 0u; i < SERIAL_SCB_PINS_NUMBER; i++)
        {
            hsiomSel[i]  = SERIAL_HSIOM_DEF_SEL;
            pinsDm[i]    = SERIAL_PIN_DM_ALG_HIZ;
        }

        if((SERIAL_SCB_MODE_I2C   == mode) ||
           (SERIAL_SCB_MODE_EZI2C == mode))
        {
            hsiomSel[SERIAL_MOSI_SCL_RX_PIN_INDEX] = SERIAL_HSIOM_I2C_SEL;
            hsiomSel[SERIAL_MISO_SDA_TX_PIN_INDEX] = SERIAL_HSIOM_I2C_SEL;

            pinsDm[SERIAL_MOSI_SCL_RX_PIN_INDEX] = SERIAL_PIN_DM_OD_LO;
            pinsDm[SERIAL_MISO_SDA_TX_PIN_INDEX] = SERIAL_PIN_DM_OD_LO;
        }
    #if(!SERIAL_CY_SCBIP_V1_I2C_ONLY)
        else if(SERIAL_SCB_MODE_SPI == mode)
        {
            hsiomSel[SERIAL_MOSI_SCL_RX_PIN_INDEX] = SERIAL_HSIOM_SPI_SEL;
            hsiomSel[SERIAL_MISO_SDA_TX_PIN_INDEX] = SERIAL_HSIOM_SPI_SEL;
            hsiomSel[SERIAL_SCLK_PIN_INDEX]        = SERIAL_HSIOM_SPI_SEL;

            if(SERIAL_SPI_SLAVE == subMode)
            {
                /* Slave */
                pinsDm[SERIAL_MOSI_SCL_RX_PIN_INDEX] = SERIAL_PIN_DM_DIG_HIZ;
                pinsDm[SERIAL_MISO_SDA_TX_PIN_INDEX] = SERIAL_PIN_DM_STRONG;
                pinsDm[SERIAL_SCLK_PIN_INDEX]        = SERIAL_PIN_DM_DIG_HIZ;

            #if(SERIAL_SS0_PIN)
                /* Only SS0 is valid choice for Slave */
                hsiomSel[SERIAL_SS0_PIN_INDEX] = SERIAL_HSIOM_SPI_SEL;
                pinsDm  [SERIAL_SS0_PIN_INDEX] = SERIAL_PIN_DM_DIG_HIZ;
            #endif /* (SERIAL_SS1_PIN) */

            #if(SERIAL_MISO_SDA_TX_PIN)
                /* Disable input buffer */
                 pinsInBuf |= SERIAL_MISO_SDA_TX_PIN_MASK;
            #endif /* (SERIAL_MISO_SDA_TX_PIN_PIN) */
            }
            else /* (Master) */
            {
                pinsDm[SERIAL_MOSI_SCL_RX_PIN_INDEX] = SERIAL_PIN_DM_STRONG;
                pinsDm[SERIAL_MISO_SDA_TX_PIN_INDEX] = SERIAL_PIN_DM_DIG_HIZ;
                pinsDm[SERIAL_SCLK_PIN_INDEX]        = SERIAL_PIN_DM_STRONG;

            #if(SERIAL_SS0_PIN)
                hsiomSel [SERIAL_SS0_PIN_INDEX] = SERIAL_HSIOM_SPI_SEL;
                pinsDm   [SERIAL_SS0_PIN_INDEX] = SERIAL_PIN_DM_STRONG;
                pinsInBuf                                |= SERIAL_SS0_PIN_MASK;
            #endif /* (SERIAL_SS0_PIN) */

            #if(SERIAL_SS1_PIN)
                hsiomSel [SERIAL_SS1_PIN_INDEX] = SERIAL_HSIOM_SPI_SEL;
                pinsDm   [SERIAL_SS1_PIN_INDEX] = SERIAL_PIN_DM_STRONG;
                pinsInBuf                                |= SERIAL_SS1_PIN_MASK;
            #endif /* (SERIAL_SS1_PIN) */

            #if(SERIAL_SS2_PIN)
                hsiomSel [SERIAL_SS2_PIN_INDEX] = SERIAL_HSIOM_SPI_SEL;
                pinsDm   [SERIAL_SS2_PIN_INDEX] = SERIAL_PIN_DM_STRONG;
                pinsInBuf                                |= SERIAL_SS2_PIN_MASK;
            #endif /* (SERIAL_SS2_PIN) */

            #if(SERIAL_SS3_PIN)
                hsiomSel [SERIAL_SS3_PIN_INDEX] = SERIAL_HSIOM_SPI_SEL;
                pinsDm   [SERIAL_SS3_PIN_INDEX] = SERIAL_PIN_DM_STRONG;
                pinsInBuf                                |= SERIAL_SS3_PIN_MASK;
            #endif /* (SERIAL_SS2_PIN) */

                /* Disable input buffers */
            #if(SERIAL_MOSI_SCL_RX_PIN)
                pinsInBuf |= SERIAL_MOSI_SCL_RX_PIN_MASK;
            #endif /* (SERIAL_MOSI_SCL_RX_PIN) */

             #if(SERIAL_MOSI_SCL_RX_WAKE_PIN)
                pinsInBuf |= SERIAL_MOSI_SCL_RX_WAKE_PIN_MASK;
            #endif /* (SERIAL_MOSI_SCL_RX_WAKE_PIN) */

            #if(SERIAL_SCLK_PIN)
                pinsInBuf |= SERIAL_SCLK_PIN_MASK;
            #endif /* (SERIAL_SCLK_PIN) */
            }
        }
        else /* UART */
        {
            if(SERIAL_UART_MODE_SMARTCARD == subMode)
            {
                /* SmartCard */
                hsiomSel[SERIAL_MISO_SDA_TX_PIN_INDEX] = SERIAL_HSIOM_UART_SEL;
                pinsDm  [SERIAL_MISO_SDA_TX_PIN_INDEX] = SERIAL_PIN_DM_OD_LO;
            }
            else /* Standard or IrDA */
            {
                if(0u != (SERIAL_UART_RX & uartTxRx))
                {
                    hsiomSel[SERIAL_MOSI_SCL_RX_PIN_INDEX] = SERIAL_HSIOM_UART_SEL;
                    pinsDm  [SERIAL_MOSI_SCL_RX_PIN_INDEX] = SERIAL_PIN_DM_DIG_HIZ;
                }

                if(0u != (SERIAL_UART_TX & uartTxRx))
                {
                    hsiomSel[SERIAL_MISO_SDA_TX_PIN_INDEX] = SERIAL_HSIOM_UART_SEL;
                    pinsDm  [SERIAL_MISO_SDA_TX_PIN_INDEX] = SERIAL_PIN_DM_STRONG;

                #if(SERIAL_MISO_SDA_TX_PIN)
                     pinsInBuf |= SERIAL_MISO_SDA_TX_PIN_MASK;
                #endif /* (SERIAL_MISO_SDA_TX_PIN_PIN) */
                }
            }
        }
    #endif /* (!SERIAL_CY_SCBIP_V1_I2C_ONLY) */

    /* Configure pins: set HSIOM, DM and InputBufEnable */
    /* Note: the DR register settigns do not effect the pin output if HSIOM is other than GPIO */

    #if(SERIAL_MOSI_SCL_RX_PIN)
        SERIAL_SET_HSIOM_SEL(SERIAL_MOSI_SCL_RX_HSIOM_REG,
                                       SERIAL_MOSI_SCL_RX_HSIOM_MASK,
                                       SERIAL_MOSI_SCL_RX_HSIOM_POS,
                                       hsiomSel[SERIAL_MOSI_SCL_RX_PIN_INDEX]);

        SERIAL_spi_mosi_i2c_scl_uart_rx_SetDriveMode((uint8) pinsDm[SERIAL_MOSI_SCL_RX_PIN_INDEX]);

        SERIAL_SET_INP_DIS(SERIAL_spi_mosi_i2c_scl_uart_rx_INP_DIS,
                                     SERIAL_spi_mosi_i2c_scl_uart_rx_MASK,
                                     (0u != (pinsInBuf & SERIAL_MOSI_SCL_RX_PIN_MASK)));
    #endif /* (SERIAL_MOSI_SCL_RX_PIN) */

    #if(SERIAL_MOSI_SCL_RX_WAKE_PIN)
        SERIAL_SET_HSIOM_SEL(SERIAL_MOSI_SCL_RX_WAKE_HSIOM_REG,
                                       SERIAL_MOSI_SCL_RX_WAKE_HSIOM_MASK,
                                       SERIAL_MOSI_SCL_RX_WAKE_HSIOM_POS,
                                       hsiomSel[SERIAL_MOSI_SCL_RX_WAKE_PIN_INDEX]);

        SERIAL_spi_mosi_i2c_scl_uart_rx_wake_SetDriveMode((uint8)
                                                               pinsDm[SERIAL_MOSI_SCL_RX_WAKE_PIN_INDEX]);

        SERIAL_SET_INP_DIS(SERIAL_spi_mosi_i2c_scl_uart_rx_wake_INP_DIS,
                                     SERIAL_spi_mosi_i2c_scl_uart_rx_wake_MASK,
                                     (0u != (pinsInBuf & SERIAL_MOSI_SCL_RX_WAKE_PIN_MASK)));

         /* Set interrupt on falling edge */
        SERIAL_SET_INCFG_TYPE(SERIAL_MOSI_SCL_RX_WAKE_INTCFG_REG,
                                        SERIAL_MOSI_SCL_RX_WAKE_INTCFG_TYPE_MASK,
                                        SERIAL_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS,
                                        SERIAL_INTCFG_TYPE_FALLING_EDGE);
    #endif /* (SERIAL_MOSI_SCL_RX_WAKE_PIN) */

    #if(SERIAL_MISO_SDA_TX_PIN)
        SERIAL_SET_HSIOM_SEL(SERIAL_MISO_SDA_TX_HSIOM_REG,
                                       SERIAL_MISO_SDA_TX_HSIOM_MASK,
                                       SERIAL_MISO_SDA_TX_HSIOM_POS,
                                       hsiomSel[SERIAL_MISO_SDA_TX_PIN_INDEX]);

        SERIAL_spi_miso_i2c_sda_uart_tx_SetDriveMode((uint8) pinsDm[SERIAL_MISO_SDA_TX_PIN_INDEX]);

        SERIAL_SET_INP_DIS(SERIAL_spi_miso_i2c_sda_uart_tx_INP_DIS,
                                     SERIAL_spi_miso_i2c_sda_uart_tx_MASK,
                                    (0u != (pinsInBuf & SERIAL_MISO_SDA_TX_PIN_MASK)));
    #endif /* (SERIAL_MOSI_SCL_RX_PIN) */

    #if(SERIAL_SCLK_PIN)
        SERIAL_SET_HSIOM_SEL(SERIAL_SCLK_HSIOM_REG, SERIAL_SCLK_HSIOM_MASK,
                                       SERIAL_SCLK_HSIOM_POS, hsiomSel[SERIAL_SCLK_PIN_INDEX]);

        SERIAL_spi_sclk_SetDriveMode((uint8) pinsDm[SERIAL_SCLK_PIN_INDEX]);

        SERIAL_SET_INP_DIS(SERIAL_spi_sclk_INP_DIS,
                             SERIAL_spi_sclk_MASK,
                            (0u != (pinsInBuf & SERIAL_SCLK_PIN_MASK)));
    #endif /* (SERIAL_SCLK_PIN) */

    #if(SERIAL_SS0_PIN)
        SERIAL_SET_HSIOM_SEL(SERIAL_SS0_HSIOM_REG, SERIAL_SS0_HSIOM_MASK,
                                       SERIAL_SS0_HSIOM_POS, hsiomSel[SERIAL_SS0_PIN_INDEX]);

        SERIAL_spi_ss0_SetDriveMode((uint8) pinsDm[SERIAL_SS0_PIN_INDEX]);

        SERIAL_SET_INP_DIS(SERIAL_spi_ss0_INP_DIS,
                                     SERIAL_spi_ss0_MASK,
                                     (0u != (pinsInBuf & SERIAL_SS0_PIN_MASK)));
    #endif /* (SERIAL_SS1_PIN) */

    #if(SERIAL_SS1_PIN)
        SERIAL_SET_HSIOM_SEL(SERIAL_SS1_HSIOM_REG, SERIAL_SS1_HSIOM_MASK,
                                       SERIAL_SS1_HSIOM_POS, hsiomSel[SERIAL_SS1_PIN_INDEX]);

        SERIAL_spi_ss1_SetDriveMode((uint8) pinsDm[SERIAL_SS1_PIN_INDEX]);

        SERIAL_SET_INP_DIS(SERIAL_spi_ss1_INP_DIS,
                                     SERIAL_spi_ss1_MASK,
                                     (0u != (pinsInBuf & SERIAL_SS1_PIN_MASK)));
    #endif /* (SERIAL_SS1_PIN) */

    #if(SERIAL_SS2_PIN)
        SERIAL_SET_HSIOM_SEL(SERIAL_SS2_HSIOM_REG, SERIAL_SS2_HSIOM_MASK,
                                       SERIAL_SS2_HSIOM_POS, hsiomSel[SERIAL_SS2_PIN_INDEX]);

        SERIAL_spi_ss2_SetDriveMode((uint8) pinsDm[SERIAL_SS2_PIN_INDEX]);

        SERIAL_SET_INP_DIS(SERIAL_spi_ss2_INP_DIS,
                                     SERIAL_spi_ss2_MASK,
                                     (0u != (pinsInBuf & SERIAL_SS2_PIN_MASK)));
    #endif /* (SERIAL_SS2_PIN) */

    #if(SERIAL_SS3_PIN)
        SERIAL_SET_HSIOM_SEL(SERIAL_SS3_HSIOM_REG,  SERIAL_SS3_HSIOM_MASK,
                                       SERIAL_SS3_HSIOM_POS, hsiomSel[SERIAL_SS3_PIN_INDEX]);

        SERIAL_spi_ss3_SetDriveMode((uint8) pinsDm[SERIAL_SS3_PIN_INDEX]);

        SERIAL_SET_INP_DIS(SERIAL_spi_ss3_INP_DIS,
                                     SERIAL_spi_ss3_MASK,
                                     (0u != (pinsInBuf & SERIAL_SS3_PIN_MASK)));
    #endif /* (SERIAL_SS3_PIN) */
    }

#endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */


/* [] END OF FILE */
