/*******************************************************************************
* File Name: SERIAL_PINS.h
* Version 1.20
*
* Description:
*  This file provides constants and parameter values for the pin components
*  buried into SCB Component.
*
* Note:
*
********************************************************************************
* Copyright 2013-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_PINS_SERIAL_H)
#define CY_SCB_PINS_SERIAL_H

#include "cydevice_trm.h"
#include "cyfitter.h"
#include "cytypes.h"


/***************************************
*   Conditional Compilation Parameters
****************************************/

/* Unconfigured pins */
#define SERIAL_REMOVE_MOSI_SCL_RX_WAKE_PIN    (1u)
#define SERIAL_REMOVE_MOSI_SCL_RX_PIN         (1u)
#define SERIAL_REMOVE_MISO_SDA_TX_PIN         (1u)
#define SERIAL_REMOVE_SCLK_PIN                (1u)
#define SERIAL_REMOVE_SS0_PIN                 (1u)
#define SERIAL_REMOVE_SS1_PIN                 (1u)
#define SERIAL_REMOVE_SS2_PIN                 (1u)
#define SERIAL_REMOVE_SS3_PIN                 (1u)

/* Mode defined pins */
#define SERIAL_REMOVE_I2C_PINS                (1u)
#define SERIAL_REMOVE_SPI_MASTER_PINS         (1u)
#define SERIAL_REMOVE_SPI_SLAVE_PINS          (1u)
#define SERIAL_REMOVE_SPI_MASTER_SS0_PIN      (1u)
#define SERIAL_REMOVE_SPI_MASTER_SS1_PIN      (1u)
#define SERIAL_REMOVE_SPI_MASTER_SS2_PIN      (1u)
#define SERIAL_REMOVE_SPI_MASTER_SS3_PIN      (1u)
#define SERIAL_REMOVE_UART_TX_PIN             (0u)
#define SERIAL_REMOVE_UART_RX_TX_PIN          (1u)
#define SERIAL_REMOVE_UART_RX_PIN             (0u)
#define SERIAL_REMOVE_UART_RX_WAKE_PIN        (1u)

/* Unconfigured pins */
#define SERIAL_MOSI_SCL_RX_WAKE_PIN   (0u == SERIAL_REMOVE_MOSI_SCL_RX_WAKE_PIN)
#define SERIAL_MOSI_SCL_RX_PIN        (0u == SERIAL_REMOVE_MOSI_SCL_RX_PIN)
#define SERIAL_MISO_SDA_TX_PIN        (0u == SERIAL_REMOVE_MISO_SDA_TX_PIN)
#define SERIAL_SCLK_PIN               (0u == SERIAL_REMOVE_SCLK_PIN)
#define SERIAL_SS0_PIN                (0u == SERIAL_REMOVE_SS0_PIN)
#define SERIAL_SS1_PIN                (0u == SERIAL_REMOVE_SS1_PIN)
#define SERIAL_SS2_PIN                (0u == SERIAL_REMOVE_SS2_PIN)
#define SERIAL_SS3_PIN                (0u == SERIAL_REMOVE_SS3_PIN)

/* Mode defined pins */
#define SERIAL_I2C_PINS               (0u == SERIAL_REMOVE_I2C_PINS)
#define SERIAL_SPI_MASTER_PINS        (0u == SERIAL_REMOVE_SPI_MASTER_PINS)
#define SERIAL_SPI_SLAVE_PINS         (0u == SERIAL_REMOVE_SPI_SLAVE_PINS)
#define SERIAL_SPI_MASTER_SS0_PIN     (0u == SERIAL_REMOVE_SPI_MASTER_SS0_PIN)
#define SERIAL_SPI_MASTER_SS1_PIN     (0u == SERIAL_REMOVE_SPI_MASTER_SS1_PIN)
#define SERIAL_SPI_MASTER_SS2_PIN     (0u == SERIAL_REMOVE_SPI_MASTER_SS2_PIN)
#define SERIAL_SPI_MASTER_SS3_PIN     (0u == SERIAL_REMOVE_SPI_MASTER_SS3_PIN)
#define SERIAL_UART_TX_PIN            (0u == SERIAL_REMOVE_UART_TX_PIN)
#define SERIAL_UART_RX_TX_PIN         (0u == SERIAL_REMOVE_UART_RX_TX_PIN)
#define SERIAL_UART_RX_PIN            (0u == SERIAL_REMOVE_UART_RX_PIN)
#define SERIAL_UART_RX_WAKE_PIN       (0u == SERIAL_REMOVE_UART_RX_WAKE_PIN)


/***************************************
*             Includes
****************************************/

#if(SERIAL_MOSI_SCL_RX_WAKE_PIN)
    #include "SERIAL_spi_mosi_i2c_scl_uart_rx_wake.h"
#endif /* (SERIAL_MOSI_SCL_RX_WAKE_PIN) */

#if(SERIAL_MOSI_SCL_RX_PIN)
    #include "SERIAL_spi_mosi_i2c_scl_uart_rx.h"
#endif /* (SERIAL_MOSI_SCL_RX_PIN) */

#if(SERIAL_MISO_SDA_TX_PIN)
    #include "SERIAL_spi_miso_i2c_sda_uart_tx.h"
#endif /* (SERIAL_MISO_SDA_TX_PIN_PIN) */

#if(SERIAL_SCLK_PIN)
    #include "SERIAL_spi_sclk.h"
#endif /* (SERIAL_SCLK_PIN) */

#if(SERIAL_SS0_PIN)
    #include "SERIAL_spi_ss0.h"
#endif /* (SERIAL_SS1_PIN) */

#if(SERIAL_SS1_PIN)
    #include "SERIAL_spi_ss1.h"
#endif /* (SERIAL_SS1_PIN) */

#if(SERIAL_SS2_PIN)
    #include "SERIAL_spi_ss2.h"
#endif /* (SERIAL_SS2_PIN) */

#if(SERIAL_SS3_PIN)
    #include "SERIAL_spi_ss3.h"
#endif /* (SERIAL_SS3_PIN) */

#if(SERIAL_I2C_PINS)
    #include "SERIAL_scl.h"
    #include "SERIAL_sda.h"
#endif /* (SERIAL_I2C_PINS) */

#if(SERIAL_SPI_MASTER_PINS)
    #include "SERIAL_sclk_m.h"
    #include "SERIAL_mosi_m.h"
    #include "SERIAL_miso_m.h"
#endif /* (SERIAL_SPI_MASTER_PINS) */

#if(SERIAL_SPI_SLAVE_PINS)
    #include "SERIAL_sclk_s.h"
    #include "SERIAL_mosi_s.h"
    #include "SERIAL_miso_s.h"
    #include "SERIAL_ss_s.h"
#endif /* (SERIAL_SPI_SLAVE_PINS) */

#if(SERIAL_SPI_MASTER_SS0_PIN)
    #include "SERIAL_ss0_m.h"
#endif /* (SERIAL_SPI_MASTER_SS0_PIN) */

#if(SERIAL_SPI_MASTER_SS1_PIN)
    #include "SERIAL_ss1_m.h"
#endif /* (SERIAL_SPI_MASTER_SS1_PIN) */

#if(SERIAL_SPI_MASTER_SS2_PIN)
    #include "SERIAL_ss2_m.h"
#endif /* (SERIAL_SPI_MASTER_SS2_PIN) */

#if(SERIAL_SPI_MASTER_SS3_PIN)
    #include "SERIAL_ss3_m.h"
#endif /* (SERIAL_SPI_MASTER_SS3_PIN) */

#if(SERIAL_UART_TX_PIN)
    #include "SERIAL_tx.h"
#endif /* (SERIAL_UART_TX_PIN) */

#if(SERIAL_UART_RX_TX_PIN)
    #include "SERIAL_rx_tx.h"
#endif /* (SERIAL_UART_RX_TX_PIN) */

#if(SERIAL_UART_RX_PIN)
    #include "SERIAL_rx.h"
#endif /* (SERIAL_UART_RX_PIN) */

#if(SERIAL_UART_RX_WAKE_PIN)
    #include "SERIAL_rx_wake.h"
#endif /* (SERIAL_UART_RX_WAKE_PIN) */


/***************************************
*              Registers
***************************************/

#if(SERIAL_MOSI_SCL_RX_WAKE_PIN)
    #define SERIAL_MOSI_SCL_RX_WAKE_HSIOM_REG  \
                                                (*(reg32 *) SERIAL_spi_mosi_i2c_scl_uart_rx_wake__0__HSIOM)
    #define SERIAL_MOSI_SCL_RX_WAKE_HSIOM_PTR  \
                                                ( (reg32 *) SERIAL_spi_mosi_i2c_scl_uart_rx_wake__0__HSIOM)
    #define SERIAL_MOSI_SCL_RX_WAKE_HSIOM_MASK \
                                                (SERIAL_spi_mosi_i2c_scl_uart_rx_wake__0__HSIOM_MASK)
    #define SERIAL_MOSI_SCL_RX_WAKE_HSIOM_POS  \
                                                (SERIAL_spi_mosi_i2c_scl_uart_rx_wake__0__HSIOM_SHIFT)

    #define SERIAL_MOSI_SCL_RX_WAKE_INTCFG_REG    (*(reg32 *) \
                                                              SERIAL_spi_mosi_i2c_scl_uart_rx_wake__0__INTCFG)
    #define SERIAL_MOSI_SCL_RX_WAKE_INTCFG_PTR    ( (reg32 *) \
                                                              SERIAL_spi_mosi_i2c_scl_uart_rx_wake__0__INTCFG)

    #define SERIAL_INTCFG_TYPE_MASK                  (0x03u)
    #define SERIAL_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS  (SERIAL_spi_mosi_i2c_scl_uart_rx_wake__SHIFT)
    #define SERIAL_MOSI_SCL_RX_WAKE_INTCFG_TYPE_MASK ((uint32)                                           \
                                                                    ((uint32) SERIAL_INTCFG_TYPE_MASK << \
                                                                    SERIAL_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS))
#endif /* (SERIAL_MOSI_SCL_RX_WAKE_PIN) */

#if(SERIAL_MOSI_SCL_RX_PIN)
    #define SERIAL_MOSI_SCL_RX_HSIOM_REG      (*(reg32 *) SERIAL_spi_mosi_i2c_scl_uart_rx__0__HSIOM)
    #define SERIAL_MOSI_SCL_RX_HSIOM_PTR      ( (reg32 *) SERIAL_spi_mosi_i2c_scl_uart_rx__0__HSIOM)
    #define SERIAL_MOSI_SCL_RX_HSIOM_MASK     (SERIAL_spi_mosi_i2c_scl_uart_rx__0__HSIOM_MASK)
    #define SERIAL_MOSI_SCL_RX_HSIOM_POS      (SERIAL_spi_mosi_i2c_scl_uart_rx__0__HSIOM_SHIFT)
#endif /* (SERIAL_MOSI_SCL_RX_PIN) */

#if(SERIAL_MISO_SDA_TX_PIN)
    #define SERIAL_MISO_SDA_TX_HSIOM_REG      (*(reg32 *) SERIAL_spi_miso_i2c_sda_uart_tx__0__HSIOM)
    #define SERIAL_MISO_SDA_TX_HSIOM_PTR      ( (reg32 *) SERIAL_spi_miso_i2c_sda_uart_tx__0__HSIOM)
    #define SERIAL_MISO_SDA_TX_HSIOM_MASK     (SERIAL_spi_miso_i2c_sda_uart_tx__0__HSIOM_MASK)
    #define SERIAL_MISO_SDA_TX_HSIOM_POS      (SERIAL_spi_miso_i2c_sda_uart_tx__0__HSIOM_SHIFT)
#endif /* (SERIAL_MISO_SDA_TX_PIN_PIN) */

#if(SERIAL_SCLK_PIN)
    #define SERIAL_SCLK_HSIOM_REG     (*(reg32 *) SERIAL_spi_sclk__0__HSIOM)
    #define SERIAL_SCLK_HSIOM_PTR     ( (reg32 *) SERIAL_spi_sclk__0__HSIOM)
    #define SERIAL_SCLK_HSIOM_MASK    (SERIAL_spi_sclk__0__HSIOM_MASK)
    #define SERIAL_SCLK_HSIOM_POS     (SERIAL_spi_sclk__0__HSIOM_SHIFT)
#endif /* (SERIAL_SCLK_PIN) */

#if(SERIAL_SS0_PIN)
    #define SERIAL_SS0_HSIOM_REG      (*(reg32 *) SERIAL_spi_ss0__0__HSIOM)
    #define SERIAL_SS0_HSIOM_PTR      ( (reg32 *) SERIAL_spi_ss0__0__HSIOM)
    #define SERIAL_SS0_HSIOM_MASK     (SERIAL_spi_ss0__0__HSIOM_MASK)
    #define SERIAL_SS0_HSIOM_POS      (SERIAL_spi_ss0__0__HSIOM_SHIFT)
#endif /* (SERIAL_SS1_PIN) */

#if(SERIAL_SS1_PIN)
    #define SERIAL_SS1_HSIOM_REG      (*(reg32 *) SERIAL_spi_ss1__0__HSIOM)
    #define SERIAL_SS1_HSIOM_PTR      ( (reg32 *) SERIAL_spi_ss1__0__HSIOM)
    #define SERIAL_SS1_HSIOM_MASK     (SERIAL_spi_ss1__0__HSIOM_MASK)
    #define SERIAL_SS1_HSIOM_POS      (SERIAL_spi_ss1__0__HSIOM_SHIFT)
#endif /* (SERIAL_SS1_PIN) */

#if(SERIAL_SS2_PIN)
    #define SERIAL_SS2_HSIOM_REG     (*(reg32 *) SERIAL_spi_ss2__0__HSIOM)
    #define SERIAL_SS2_HSIOM_PTR     ( (reg32 *) SERIAL_spi_ss2__0__HSIOM)
    #define SERIAL_SS2_HSIOM_MASK    (SERIAL_spi_ss2__0__HSIOM_MASK)
    #define SERIAL_SS2_HSIOM_POS     (SERIAL_spi_ss2__0__HSIOM_SHIFT)
#endif /* (SERIAL_SS2_PIN) */

#if(SERIAL_SS3_PIN)
    #define SERIAL_SS3_HSIOM_REG     (*(reg32 *) SERIAL_spi_ss3__0__HSIOM)
    #define SERIAL_SS3_HSIOM_PTR     ( (reg32 *) SERIAL_spi_ss3__0__HSIOM)
    #define SERIAL_SS3_HSIOM_MASK    (SERIAL_spi_ss3__0__HSIOM_MASK)
    #define SERIAL_SS3_HSIOM_POS     (SERIAL_spi_ss3__0__HSIOM_SHIFT)
#endif /* (SERIAL_SS3_PIN) */

#if(SERIAL_I2C_PINS)
    #define SERIAL_SCL_HSIOM_REG     (*(reg32 *) SERIAL_scl__0__HSIOM)
    #define SERIAL_SCL_HSIOM_PTR     ( (reg32 *) SERIAL_scl__0__HSIOM)
    #define SERIAL_SCL_HSIOM_MASK    (SERIAL_scl__0__HSIOM_MASK)
    #define SERIAL_SCL_HSIOM_POS     (SERIAL_scl__0__HSIOM_SHIFT)

    #define SERIAL_SDA_HSIOM_REG     (*(reg32 *) SERIAL_sda__0__HSIOM)
    #define SERIAL_SDA_HSIOM_PTR     ( (reg32 *) SERIAL_sda__0__HSIOM)
    #define SERIAL_SDA_HSIOM_MASK    (SERIAL_sda__0__HSIOM_MASK)
    #define SERIAL_SDA_HSIOM_POS     (SERIAL_sda__0__HSIOM_SHIFT)
#endif /* (SERIAL_I2C_PINS) */


/***************************************
*        Registers Constants
***************************************/

/* Pins constants */
#define SERIAL_HSIOM_DEF_SEL      (0x00u)
#define SERIAL_HSIOM_GPIO_SEL     (0x00u)
#define SERIAL_HSIOM_UART_SEL     (0x09u)
#define SERIAL_HSIOM_I2C_SEL      (0x0Eu)
#define SERIAL_HSIOM_SPI_SEL      (0x0Fu)

#if(!SERIAL_CY_SCBIP_V1_I2C_ONLY)
    #define SERIAL_SCB_PINS_NUMBER    (7u)
#else
    #define SERIAL_SCB_PINS_NUMBER    (2u)
#endif /* (!SERIAL_CY_SCBIP_V1_I2C_ONLY) */

#define SERIAL_MOSI_SCL_RX_PIN_INDEX      (0u) /* RX pins without interrupt */
#define SERIAL_MOSI_SCL_RX_WAKE_PIN_INDEX (0u) /* RX pin with interrupt     */
#define SERIAL_MISO_SDA_TX_PIN_INDEX      (1u)
#define SERIAL_SCLK_PIN_INDEX             (2u)
#define SERIAL_SS0_PIN_INDEX              (3u)
#define SERIAL_SS1_PIN_INDEX              (4u)
#define SERIAL_SS2_PIN_INDEX              (5u)
#define SERIAL_SS3_PIN_INDEX              (6u)

#define SERIAL_MOSI_SCL_RX_PIN_MASK      ((uint32) 0x01u << SERIAL_MOSI_SCL_RX_PIN_INDEX)
#define SERIAL_MOSI_SCL_RX_WAKE_PIN_MASK ((uint32) 0x01u << SERIAL_MOSI_SCL_RX_WAKE_PIN_INDEX)
#define SERIAL_MISO_SDA_TX_PIN_MASK      ((uint32) 0x01u << SERIAL_MISO_SDA_TX_PIN_INDEX)
#define SERIAL_SCLK_PIN_MASK             ((uint32) 0x01u << SERIAL_SCLK_PIN_INDEX)
#define SERIAL_SS0_PIN_MASK              ((uint32) 0x01u << SERIAL_SS0_PIN_INDEX)
#define SERIAL_SS1_PIN_MASK              ((uint32) 0x01u << SERIAL_SS1_PIN_INDEX)
#define SERIAL_SS2_PIN_MASK              ((uint32) 0x01u << SERIAL_SS2_PIN_INDEX)
#define SERIAL_SS3_PIN_MASK              ((uint32) 0x01u << SERIAL_SS3_PIN_INDEX)

#define SERIAL_INTCFG_TYPE_FALLING_EDGE   (0x02u)

/* Pin DM defines */
#define SERIAL_PIN_DM_ALG_HIZ  (0u)
#define SERIAL_PIN_DM_DIG_HIZ  (1u)
#define SERIAL_PIN_DM_OD_LO    (4u)
#define SERIAL_PIN_DM_STRONG   (6u)


/***************************************
*          Macro Definitions
***************************************/

/* Set bits-mask in register */
#define SERIAL_SET_REGISTER_BITS(reg, mask, pos, mode) \
                    do                                           \
                    {                                            \
                        (reg) = (((reg) & ((uint32) ~(uint32) (mask))) | ((uint32) ((uint32) (mode) << (pos)))); \
                    }while(0)

/* Set bit the in register */
#define SERIAL_SET_REGISTER_BIT(reg, mask, val) \
                    ((val) ? ((reg) |= (mask)) : ((reg) &= ((uint32) ~((uint32) (mask)))))

#define SERIAL_SET_HSIOM_SEL(reg, mask, pos, sel) SERIAL_SET_REGISTER_BITS(reg, mask, pos, sel)
#define SERIAL_SET_INCFG_TYPE(reg, mask, pos, intType) \
                                                        SERIAL_SET_REGISTER_BITS(reg, mask, pos, intType)
#define SERIAL_SET_INP_DIS(reg, mask, val) SERIAL_SET_REGISTER_BIT(reg, mask, val)

/* SERIAL_SET_I2C_SCL_DR(val) - Sets I2C SCL DR register.
*  SERIAL_SET_I2C_SCL_HSIOM_SEL(sel) - Sets I2C SCL HSIOM settings.
*/
/* SCB I2C: scl signal */
#if(SERIAL_I2C_PINS)
    #define SERIAL_SET_I2C_SCL_DR(val) SERIAL_scl_Write(val)

    #define SERIAL_SET_I2C_SCL_HSIOM_SEL(sel) \
                          SERIAL_SET_HSIOM_SEL(SERIAL_SCL_HSIOM_REG,  \
                                                         SERIAL_SCL_HSIOM_MASK, \
                                                         SERIAL_SCL_HSIOM_POS,  \
                                                         (sel))
    #define SERIAL_WAIT_SCL_SET_HIGH  (0u == SERIAL_scl_Read())

/* Unconfigured SCB: scl signal */
#elif(SERIAL_MOSI_SCL_RX_WAKE_PIN)
    #define SERIAL_SET_I2C_SCL_DR(val) \
                            SERIAL_spi_mosi_i2c_scl_uart_rx_wake_Write(val)

    #define SERIAL_SET_I2C_SCL_HSIOM_SEL(sel) \
                    SERIAL_SET_HSIOM_SEL(SERIAL_MOSI_SCL_RX_WAKE_HSIOM_REG,  \
                                                   SERIAL_MOSI_SCL_RX_WAKE_HSIOM_MASK, \
                                                   SERIAL_MOSI_SCL_RX_WAKE_HSIOM_POS,  \
                                                   (sel))

    #define SERIAL_WAIT_SCL_SET_HIGH  (0u == SERIAL_spi_mosi_i2c_scl_uart_rx_wake_Read())

#elif(SERIAL_MOSI_SCL_RX_PIN)
    #define SERIAL_SET_I2C_SCL_DR(val) \
                            SERIAL_spi_mosi_i2c_scl_uart_rx_Write(val)


    #define SERIAL_SET_I2C_SCL_HSIOM_SEL(sel) \
                            SERIAL_SET_HSIOM_SEL(SERIAL_MOSI_SCL_RX_HSIOM_REG,  \
                                                           SERIAL_MOSI_SCL_RX_HSIOM_MASK, \
                                                           SERIAL_MOSI_SCL_RX_HSIOM_POS,  \
                                                           (sel))

    #define SERIAL_WAIT_SCL_SET_HIGH  (0u == SERIAL_spi_mosi_i2c_scl_uart_rx_Read())

#else
    #define SERIAL_SET_I2C_SCL_DR(val) \
                                                    do{ ; }while(0)
    #define SERIAL_SET_I2C_SCL_HSIOM_SEL(sel) \
                                                    do{ ; }while(0)

    #define SERIAL_WAIT_SCL_SET_HIGH  (0u)
#endif /* (SERIAL_MOSI_SCL_RX_PIN) */

/* SCB I2C: sda signal */
#if(SERIAL_I2C_PINS)
    #define SERIAL_WAIT_SDA_SET_HIGH  (0u == SERIAL_sda_Read())

/* Unconfigured SCB: sda signal */
#elif(SERIAL_MISO_SDA_TX_PIN)
    #define SERIAL_WAIT_SDA_SET_HIGH  (0u == SERIAL_spi_miso_i2c_sda_uart_tx_Read())

#else
    #define SERIAL_WAIT_SDA_SET_HIGH  (0u)
#endif /* (SERIAL_MOSI_SCL_RX_PIN) */

#endif /* (CY_SCB_PINS_SERIAL_H) */


/* [] END OF FILE */
