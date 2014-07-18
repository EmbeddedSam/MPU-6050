/*******************************************************************************
* File Name: I2C_MPU6050_PINS.h
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

#if !defined(CY_SCB_PINS_I2C_MPU6050_H)
#define CY_SCB_PINS_I2C_MPU6050_H

#include "cydevice_trm.h"
#include "cyfitter.h"
#include "cytypes.h"


/***************************************
*   Conditional Compilation Parameters
****************************************/

/* Unconfigured pins */
#define I2C_MPU6050_REMOVE_MOSI_SCL_RX_WAKE_PIN    (1u)
#define I2C_MPU6050_REMOVE_MOSI_SCL_RX_PIN         (1u)
#define I2C_MPU6050_REMOVE_MISO_SDA_TX_PIN         (1u)
#define I2C_MPU6050_REMOVE_SCLK_PIN                (1u)
#define I2C_MPU6050_REMOVE_SS0_PIN                 (1u)
#define I2C_MPU6050_REMOVE_SS1_PIN                 (1u)
#define I2C_MPU6050_REMOVE_SS2_PIN                 (1u)
#define I2C_MPU6050_REMOVE_SS3_PIN                 (1u)

/* Mode defined pins */
#define I2C_MPU6050_REMOVE_I2C_PINS                (0u)
#define I2C_MPU6050_REMOVE_SPI_MASTER_PINS         (1u)
#define I2C_MPU6050_REMOVE_SPI_SLAVE_PINS          (1u)
#define I2C_MPU6050_REMOVE_SPI_MASTER_SS0_PIN      (1u)
#define I2C_MPU6050_REMOVE_SPI_MASTER_SS1_PIN      (1u)
#define I2C_MPU6050_REMOVE_SPI_MASTER_SS2_PIN      (1u)
#define I2C_MPU6050_REMOVE_SPI_MASTER_SS3_PIN      (1u)
#define I2C_MPU6050_REMOVE_UART_TX_PIN             (1u)
#define I2C_MPU6050_REMOVE_UART_RX_TX_PIN          (1u)
#define I2C_MPU6050_REMOVE_UART_RX_PIN             (1u)
#define I2C_MPU6050_REMOVE_UART_RX_WAKE_PIN        (1u)

/* Unconfigured pins */
#define I2C_MPU6050_MOSI_SCL_RX_WAKE_PIN   (0u == I2C_MPU6050_REMOVE_MOSI_SCL_RX_WAKE_PIN)
#define I2C_MPU6050_MOSI_SCL_RX_PIN        (0u == I2C_MPU6050_REMOVE_MOSI_SCL_RX_PIN)
#define I2C_MPU6050_MISO_SDA_TX_PIN        (0u == I2C_MPU6050_REMOVE_MISO_SDA_TX_PIN)
#define I2C_MPU6050_SCLK_PIN               (0u == I2C_MPU6050_REMOVE_SCLK_PIN)
#define I2C_MPU6050_SS0_PIN                (0u == I2C_MPU6050_REMOVE_SS0_PIN)
#define I2C_MPU6050_SS1_PIN                (0u == I2C_MPU6050_REMOVE_SS1_PIN)
#define I2C_MPU6050_SS2_PIN                (0u == I2C_MPU6050_REMOVE_SS2_PIN)
#define I2C_MPU6050_SS3_PIN                (0u == I2C_MPU6050_REMOVE_SS3_PIN)

/* Mode defined pins */
#define I2C_MPU6050_I2C_PINS               (0u == I2C_MPU6050_REMOVE_I2C_PINS)
#define I2C_MPU6050_SPI_MASTER_PINS        (0u == I2C_MPU6050_REMOVE_SPI_MASTER_PINS)
#define I2C_MPU6050_SPI_SLAVE_PINS         (0u == I2C_MPU6050_REMOVE_SPI_SLAVE_PINS)
#define I2C_MPU6050_SPI_MASTER_SS0_PIN     (0u == I2C_MPU6050_REMOVE_SPI_MASTER_SS0_PIN)
#define I2C_MPU6050_SPI_MASTER_SS1_PIN     (0u == I2C_MPU6050_REMOVE_SPI_MASTER_SS1_PIN)
#define I2C_MPU6050_SPI_MASTER_SS2_PIN     (0u == I2C_MPU6050_REMOVE_SPI_MASTER_SS2_PIN)
#define I2C_MPU6050_SPI_MASTER_SS3_PIN     (0u == I2C_MPU6050_REMOVE_SPI_MASTER_SS3_PIN)
#define I2C_MPU6050_UART_TX_PIN            (0u == I2C_MPU6050_REMOVE_UART_TX_PIN)
#define I2C_MPU6050_UART_RX_TX_PIN         (0u == I2C_MPU6050_REMOVE_UART_RX_TX_PIN)
#define I2C_MPU6050_UART_RX_PIN            (0u == I2C_MPU6050_REMOVE_UART_RX_PIN)
#define I2C_MPU6050_UART_RX_WAKE_PIN       (0u == I2C_MPU6050_REMOVE_UART_RX_WAKE_PIN)


/***************************************
*             Includes
****************************************/

#if(I2C_MPU6050_MOSI_SCL_RX_WAKE_PIN)
    #include "I2C_MPU6050_spi_mosi_i2c_scl_uart_rx_wake.h"
#endif /* (I2C_MPU6050_MOSI_SCL_RX_WAKE_PIN) */

#if(I2C_MPU6050_MOSI_SCL_RX_PIN)
    #include "I2C_MPU6050_spi_mosi_i2c_scl_uart_rx.h"
#endif /* (I2C_MPU6050_MOSI_SCL_RX_PIN) */

#if(I2C_MPU6050_MISO_SDA_TX_PIN)
    #include "I2C_MPU6050_spi_miso_i2c_sda_uart_tx.h"
#endif /* (I2C_MPU6050_MISO_SDA_TX_PIN_PIN) */

#if(I2C_MPU6050_SCLK_PIN)
    #include "I2C_MPU6050_spi_sclk.h"
#endif /* (I2C_MPU6050_SCLK_PIN) */

#if(I2C_MPU6050_SS0_PIN)
    #include "I2C_MPU6050_spi_ss0.h"
#endif /* (I2C_MPU6050_SS1_PIN) */

#if(I2C_MPU6050_SS1_PIN)
    #include "I2C_MPU6050_spi_ss1.h"
#endif /* (I2C_MPU6050_SS1_PIN) */

#if(I2C_MPU6050_SS2_PIN)
    #include "I2C_MPU6050_spi_ss2.h"
#endif /* (I2C_MPU6050_SS2_PIN) */

#if(I2C_MPU6050_SS3_PIN)
    #include "I2C_MPU6050_spi_ss3.h"
#endif /* (I2C_MPU6050_SS3_PIN) */

#if(I2C_MPU6050_I2C_PINS)
    #include "I2C_MPU6050_scl.h"
    #include "I2C_MPU6050_sda.h"
#endif /* (I2C_MPU6050_I2C_PINS) */

#if(I2C_MPU6050_SPI_MASTER_PINS)
    #include "I2C_MPU6050_sclk_m.h"
    #include "I2C_MPU6050_mosi_m.h"
    #include "I2C_MPU6050_miso_m.h"
#endif /* (I2C_MPU6050_SPI_MASTER_PINS) */

#if(I2C_MPU6050_SPI_SLAVE_PINS)
    #include "I2C_MPU6050_sclk_s.h"
    #include "I2C_MPU6050_mosi_s.h"
    #include "I2C_MPU6050_miso_s.h"
    #include "I2C_MPU6050_ss_s.h"
#endif /* (I2C_MPU6050_SPI_SLAVE_PINS) */

#if(I2C_MPU6050_SPI_MASTER_SS0_PIN)
    #include "I2C_MPU6050_ss0_m.h"
#endif /* (I2C_MPU6050_SPI_MASTER_SS0_PIN) */

#if(I2C_MPU6050_SPI_MASTER_SS1_PIN)
    #include "I2C_MPU6050_ss1_m.h"
#endif /* (I2C_MPU6050_SPI_MASTER_SS1_PIN) */

#if(I2C_MPU6050_SPI_MASTER_SS2_PIN)
    #include "I2C_MPU6050_ss2_m.h"
#endif /* (I2C_MPU6050_SPI_MASTER_SS2_PIN) */

#if(I2C_MPU6050_SPI_MASTER_SS3_PIN)
    #include "I2C_MPU6050_ss3_m.h"
#endif /* (I2C_MPU6050_SPI_MASTER_SS3_PIN) */

#if(I2C_MPU6050_UART_TX_PIN)
    #include "I2C_MPU6050_tx.h"
#endif /* (I2C_MPU6050_UART_TX_PIN) */

#if(I2C_MPU6050_UART_RX_TX_PIN)
    #include "I2C_MPU6050_rx_tx.h"
#endif /* (I2C_MPU6050_UART_RX_TX_PIN) */

#if(I2C_MPU6050_UART_RX_PIN)
    #include "I2C_MPU6050_rx.h"
#endif /* (I2C_MPU6050_UART_RX_PIN) */

#if(I2C_MPU6050_UART_RX_WAKE_PIN)
    #include "I2C_MPU6050_rx_wake.h"
#endif /* (I2C_MPU6050_UART_RX_WAKE_PIN) */


/***************************************
*              Registers
***************************************/

#if(I2C_MPU6050_MOSI_SCL_RX_WAKE_PIN)
    #define I2C_MPU6050_MOSI_SCL_RX_WAKE_HSIOM_REG  \
                                                (*(reg32 *) I2C_MPU6050_spi_mosi_i2c_scl_uart_rx_wake__0__HSIOM)
    #define I2C_MPU6050_MOSI_SCL_RX_WAKE_HSIOM_PTR  \
                                                ( (reg32 *) I2C_MPU6050_spi_mosi_i2c_scl_uart_rx_wake__0__HSIOM)
    #define I2C_MPU6050_MOSI_SCL_RX_WAKE_HSIOM_MASK \
                                                (I2C_MPU6050_spi_mosi_i2c_scl_uart_rx_wake__0__HSIOM_MASK)
    #define I2C_MPU6050_MOSI_SCL_RX_WAKE_HSIOM_POS  \
                                                (I2C_MPU6050_spi_mosi_i2c_scl_uart_rx_wake__0__HSIOM_SHIFT)

    #define I2C_MPU6050_MOSI_SCL_RX_WAKE_INTCFG_REG    (*(reg32 *) \
                                                              I2C_MPU6050_spi_mosi_i2c_scl_uart_rx_wake__0__INTCFG)
    #define I2C_MPU6050_MOSI_SCL_RX_WAKE_INTCFG_PTR    ( (reg32 *) \
                                                              I2C_MPU6050_spi_mosi_i2c_scl_uart_rx_wake__0__INTCFG)

    #define I2C_MPU6050_INTCFG_TYPE_MASK                  (0x03u)
    #define I2C_MPU6050_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS  (I2C_MPU6050_spi_mosi_i2c_scl_uart_rx_wake__SHIFT)
    #define I2C_MPU6050_MOSI_SCL_RX_WAKE_INTCFG_TYPE_MASK ((uint32)                                           \
                                                                    ((uint32) I2C_MPU6050_INTCFG_TYPE_MASK << \
                                                                    I2C_MPU6050_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS))
#endif /* (I2C_MPU6050_MOSI_SCL_RX_WAKE_PIN) */

#if(I2C_MPU6050_MOSI_SCL_RX_PIN)
    #define I2C_MPU6050_MOSI_SCL_RX_HSIOM_REG      (*(reg32 *) I2C_MPU6050_spi_mosi_i2c_scl_uart_rx__0__HSIOM)
    #define I2C_MPU6050_MOSI_SCL_RX_HSIOM_PTR      ( (reg32 *) I2C_MPU6050_spi_mosi_i2c_scl_uart_rx__0__HSIOM)
    #define I2C_MPU6050_MOSI_SCL_RX_HSIOM_MASK     (I2C_MPU6050_spi_mosi_i2c_scl_uart_rx__0__HSIOM_MASK)
    #define I2C_MPU6050_MOSI_SCL_RX_HSIOM_POS      (I2C_MPU6050_spi_mosi_i2c_scl_uart_rx__0__HSIOM_SHIFT)
#endif /* (I2C_MPU6050_MOSI_SCL_RX_PIN) */

#if(I2C_MPU6050_MISO_SDA_TX_PIN)
    #define I2C_MPU6050_MISO_SDA_TX_HSIOM_REG      (*(reg32 *) I2C_MPU6050_spi_miso_i2c_sda_uart_tx__0__HSIOM)
    #define I2C_MPU6050_MISO_SDA_TX_HSIOM_PTR      ( (reg32 *) I2C_MPU6050_spi_miso_i2c_sda_uart_tx__0__HSIOM)
    #define I2C_MPU6050_MISO_SDA_TX_HSIOM_MASK     (I2C_MPU6050_spi_miso_i2c_sda_uart_tx__0__HSIOM_MASK)
    #define I2C_MPU6050_MISO_SDA_TX_HSIOM_POS      (I2C_MPU6050_spi_miso_i2c_sda_uart_tx__0__HSIOM_SHIFT)
#endif /* (I2C_MPU6050_MISO_SDA_TX_PIN_PIN) */

#if(I2C_MPU6050_SCLK_PIN)
    #define I2C_MPU6050_SCLK_HSIOM_REG     (*(reg32 *) I2C_MPU6050_spi_sclk__0__HSIOM)
    #define I2C_MPU6050_SCLK_HSIOM_PTR     ( (reg32 *) I2C_MPU6050_spi_sclk__0__HSIOM)
    #define I2C_MPU6050_SCLK_HSIOM_MASK    (I2C_MPU6050_spi_sclk__0__HSIOM_MASK)
    #define I2C_MPU6050_SCLK_HSIOM_POS     (I2C_MPU6050_spi_sclk__0__HSIOM_SHIFT)
#endif /* (I2C_MPU6050_SCLK_PIN) */

#if(I2C_MPU6050_SS0_PIN)
    #define I2C_MPU6050_SS0_HSIOM_REG      (*(reg32 *) I2C_MPU6050_spi_ss0__0__HSIOM)
    #define I2C_MPU6050_SS0_HSIOM_PTR      ( (reg32 *) I2C_MPU6050_spi_ss0__0__HSIOM)
    #define I2C_MPU6050_SS0_HSIOM_MASK     (I2C_MPU6050_spi_ss0__0__HSIOM_MASK)
    #define I2C_MPU6050_SS0_HSIOM_POS      (I2C_MPU6050_spi_ss0__0__HSIOM_SHIFT)
#endif /* (I2C_MPU6050_SS1_PIN) */

#if(I2C_MPU6050_SS1_PIN)
    #define I2C_MPU6050_SS1_HSIOM_REG      (*(reg32 *) I2C_MPU6050_spi_ss1__0__HSIOM)
    #define I2C_MPU6050_SS1_HSIOM_PTR      ( (reg32 *) I2C_MPU6050_spi_ss1__0__HSIOM)
    #define I2C_MPU6050_SS1_HSIOM_MASK     (I2C_MPU6050_spi_ss1__0__HSIOM_MASK)
    #define I2C_MPU6050_SS1_HSIOM_POS      (I2C_MPU6050_spi_ss1__0__HSIOM_SHIFT)
#endif /* (I2C_MPU6050_SS1_PIN) */

#if(I2C_MPU6050_SS2_PIN)
    #define I2C_MPU6050_SS2_HSIOM_REG     (*(reg32 *) I2C_MPU6050_spi_ss2__0__HSIOM)
    #define I2C_MPU6050_SS2_HSIOM_PTR     ( (reg32 *) I2C_MPU6050_spi_ss2__0__HSIOM)
    #define I2C_MPU6050_SS2_HSIOM_MASK    (I2C_MPU6050_spi_ss2__0__HSIOM_MASK)
    #define I2C_MPU6050_SS2_HSIOM_POS     (I2C_MPU6050_spi_ss2__0__HSIOM_SHIFT)
#endif /* (I2C_MPU6050_SS2_PIN) */

#if(I2C_MPU6050_SS3_PIN)
    #define I2C_MPU6050_SS3_HSIOM_REG     (*(reg32 *) I2C_MPU6050_spi_ss3__0__HSIOM)
    #define I2C_MPU6050_SS3_HSIOM_PTR     ( (reg32 *) I2C_MPU6050_spi_ss3__0__HSIOM)
    #define I2C_MPU6050_SS3_HSIOM_MASK    (I2C_MPU6050_spi_ss3__0__HSIOM_MASK)
    #define I2C_MPU6050_SS3_HSIOM_POS     (I2C_MPU6050_spi_ss3__0__HSIOM_SHIFT)
#endif /* (I2C_MPU6050_SS3_PIN) */

#if(I2C_MPU6050_I2C_PINS)
    #define I2C_MPU6050_SCL_HSIOM_REG     (*(reg32 *) I2C_MPU6050_scl__0__HSIOM)
    #define I2C_MPU6050_SCL_HSIOM_PTR     ( (reg32 *) I2C_MPU6050_scl__0__HSIOM)
    #define I2C_MPU6050_SCL_HSIOM_MASK    (I2C_MPU6050_scl__0__HSIOM_MASK)
    #define I2C_MPU6050_SCL_HSIOM_POS     (I2C_MPU6050_scl__0__HSIOM_SHIFT)

    #define I2C_MPU6050_SDA_HSIOM_REG     (*(reg32 *) I2C_MPU6050_sda__0__HSIOM)
    #define I2C_MPU6050_SDA_HSIOM_PTR     ( (reg32 *) I2C_MPU6050_sda__0__HSIOM)
    #define I2C_MPU6050_SDA_HSIOM_MASK    (I2C_MPU6050_sda__0__HSIOM_MASK)
    #define I2C_MPU6050_SDA_HSIOM_POS     (I2C_MPU6050_sda__0__HSIOM_SHIFT)
#endif /* (I2C_MPU6050_I2C_PINS) */


/***************************************
*        Registers Constants
***************************************/

/* Pins constants */
#define I2C_MPU6050_HSIOM_DEF_SEL      (0x00u)
#define I2C_MPU6050_HSIOM_GPIO_SEL     (0x00u)
#define I2C_MPU6050_HSIOM_UART_SEL     (0x09u)
#define I2C_MPU6050_HSIOM_I2C_SEL      (0x0Eu)
#define I2C_MPU6050_HSIOM_SPI_SEL      (0x0Fu)

#if(!I2C_MPU6050_CY_SCBIP_V1_I2C_ONLY)
    #define I2C_MPU6050_SCB_PINS_NUMBER    (7u)
#else
    #define I2C_MPU6050_SCB_PINS_NUMBER    (2u)
#endif /* (!I2C_MPU6050_CY_SCBIP_V1_I2C_ONLY) */

#define I2C_MPU6050_MOSI_SCL_RX_PIN_INDEX      (0u) /* RX pins without interrupt */
#define I2C_MPU6050_MOSI_SCL_RX_WAKE_PIN_INDEX (0u) /* RX pin with interrupt     */
#define I2C_MPU6050_MISO_SDA_TX_PIN_INDEX      (1u)
#define I2C_MPU6050_SCLK_PIN_INDEX             (2u)
#define I2C_MPU6050_SS0_PIN_INDEX              (3u)
#define I2C_MPU6050_SS1_PIN_INDEX              (4u)
#define I2C_MPU6050_SS2_PIN_INDEX              (5u)
#define I2C_MPU6050_SS3_PIN_INDEX              (6u)

#define I2C_MPU6050_MOSI_SCL_RX_PIN_MASK      ((uint32) 0x01u << I2C_MPU6050_MOSI_SCL_RX_PIN_INDEX)
#define I2C_MPU6050_MOSI_SCL_RX_WAKE_PIN_MASK ((uint32) 0x01u << I2C_MPU6050_MOSI_SCL_RX_WAKE_PIN_INDEX)
#define I2C_MPU6050_MISO_SDA_TX_PIN_MASK      ((uint32) 0x01u << I2C_MPU6050_MISO_SDA_TX_PIN_INDEX)
#define I2C_MPU6050_SCLK_PIN_MASK             ((uint32) 0x01u << I2C_MPU6050_SCLK_PIN_INDEX)
#define I2C_MPU6050_SS0_PIN_MASK              ((uint32) 0x01u << I2C_MPU6050_SS0_PIN_INDEX)
#define I2C_MPU6050_SS1_PIN_MASK              ((uint32) 0x01u << I2C_MPU6050_SS1_PIN_INDEX)
#define I2C_MPU6050_SS2_PIN_MASK              ((uint32) 0x01u << I2C_MPU6050_SS2_PIN_INDEX)
#define I2C_MPU6050_SS3_PIN_MASK              ((uint32) 0x01u << I2C_MPU6050_SS3_PIN_INDEX)

#define I2C_MPU6050_INTCFG_TYPE_FALLING_EDGE   (0x02u)

/* Pin DM defines */
#define I2C_MPU6050_PIN_DM_ALG_HIZ  (0u)
#define I2C_MPU6050_PIN_DM_DIG_HIZ  (1u)
#define I2C_MPU6050_PIN_DM_OD_LO    (4u)
#define I2C_MPU6050_PIN_DM_STRONG   (6u)


/***************************************
*          Macro Definitions
***************************************/

/* Set bits-mask in register */
#define I2C_MPU6050_SET_REGISTER_BITS(reg, mask, pos, mode) \
                    do                                           \
                    {                                            \
                        (reg) = (((reg) & ((uint32) ~(uint32) (mask))) | ((uint32) ((uint32) (mode) << (pos)))); \
                    }while(0)

/* Set bit the in register */
#define I2C_MPU6050_SET_REGISTER_BIT(reg, mask, val) \
                    ((val) ? ((reg) |= (mask)) : ((reg) &= ((uint32) ~((uint32) (mask)))))

#define I2C_MPU6050_SET_HSIOM_SEL(reg, mask, pos, sel) I2C_MPU6050_SET_REGISTER_BITS(reg, mask, pos, sel)
#define I2C_MPU6050_SET_INCFG_TYPE(reg, mask, pos, intType) \
                                                        I2C_MPU6050_SET_REGISTER_BITS(reg, mask, pos, intType)
#define I2C_MPU6050_SET_INP_DIS(reg, mask, val) I2C_MPU6050_SET_REGISTER_BIT(reg, mask, val)

/* I2C_MPU6050_SET_I2C_SCL_DR(val) - Sets I2C SCL DR register.
*  I2C_MPU6050_SET_I2C_SCL_HSIOM_SEL(sel) - Sets I2C SCL HSIOM settings.
*/
/* SCB I2C: scl signal */
#if(I2C_MPU6050_I2C_PINS)
    #define I2C_MPU6050_SET_I2C_SCL_DR(val) I2C_MPU6050_scl_Write(val)

    #define I2C_MPU6050_SET_I2C_SCL_HSIOM_SEL(sel) \
                          I2C_MPU6050_SET_HSIOM_SEL(I2C_MPU6050_SCL_HSIOM_REG,  \
                                                         I2C_MPU6050_SCL_HSIOM_MASK, \
                                                         I2C_MPU6050_SCL_HSIOM_POS,  \
                                                         (sel))
    #define I2C_MPU6050_WAIT_SCL_SET_HIGH  (0u == I2C_MPU6050_scl_Read())

/* Unconfigured SCB: scl signal */
#elif(I2C_MPU6050_MOSI_SCL_RX_WAKE_PIN)
    #define I2C_MPU6050_SET_I2C_SCL_DR(val) \
                            I2C_MPU6050_spi_mosi_i2c_scl_uart_rx_wake_Write(val)

    #define I2C_MPU6050_SET_I2C_SCL_HSIOM_SEL(sel) \
                    I2C_MPU6050_SET_HSIOM_SEL(I2C_MPU6050_MOSI_SCL_RX_WAKE_HSIOM_REG,  \
                                                   I2C_MPU6050_MOSI_SCL_RX_WAKE_HSIOM_MASK, \
                                                   I2C_MPU6050_MOSI_SCL_RX_WAKE_HSIOM_POS,  \
                                                   (sel))

    #define I2C_MPU6050_WAIT_SCL_SET_HIGH  (0u == I2C_MPU6050_spi_mosi_i2c_scl_uart_rx_wake_Read())

#elif(I2C_MPU6050_MOSI_SCL_RX_PIN)
    #define I2C_MPU6050_SET_I2C_SCL_DR(val) \
                            I2C_MPU6050_spi_mosi_i2c_scl_uart_rx_Write(val)


    #define I2C_MPU6050_SET_I2C_SCL_HSIOM_SEL(sel) \
                            I2C_MPU6050_SET_HSIOM_SEL(I2C_MPU6050_MOSI_SCL_RX_HSIOM_REG,  \
                                                           I2C_MPU6050_MOSI_SCL_RX_HSIOM_MASK, \
                                                           I2C_MPU6050_MOSI_SCL_RX_HSIOM_POS,  \
                                                           (sel))

    #define I2C_MPU6050_WAIT_SCL_SET_HIGH  (0u == I2C_MPU6050_spi_mosi_i2c_scl_uart_rx_Read())

#else
    #define I2C_MPU6050_SET_I2C_SCL_DR(val) \
                                                    do{ ; }while(0)
    #define I2C_MPU6050_SET_I2C_SCL_HSIOM_SEL(sel) \
                                                    do{ ; }while(0)

    #define I2C_MPU6050_WAIT_SCL_SET_HIGH  (0u)
#endif /* (I2C_MPU6050_MOSI_SCL_RX_PIN) */

/* SCB I2C: sda signal */
#if(I2C_MPU6050_I2C_PINS)
    #define I2C_MPU6050_WAIT_SDA_SET_HIGH  (0u == I2C_MPU6050_sda_Read())

/* Unconfigured SCB: sda signal */
#elif(I2C_MPU6050_MISO_SDA_TX_PIN)
    #define I2C_MPU6050_WAIT_SDA_SET_HIGH  (0u == I2C_MPU6050_spi_miso_i2c_sda_uart_tx_Read())

#else
    #define I2C_MPU6050_WAIT_SDA_SET_HIGH  (0u)
#endif /* (I2C_MPU6050_MOSI_SCL_RX_PIN) */

#endif /* (CY_SCB_PINS_I2C_MPU6050_H) */


/* [] END OF FILE */
