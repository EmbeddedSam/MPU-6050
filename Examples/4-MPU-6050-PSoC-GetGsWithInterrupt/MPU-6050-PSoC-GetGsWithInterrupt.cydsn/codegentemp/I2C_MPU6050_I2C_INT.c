/*******************************************************************************
* File Name: I2C_MPU6050_I2C_INT.c
* Version 1.20
*
* Description:
*  This file provides the source code to the Interrupt Service Routine for
*  the SCB Component in I2C mode.
*
* Note:
*
********************************************************************************
* Copyright 2013-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "I2C_MPU6050_PVT.h"
#include "I2C_MPU6050_I2C_PVT.h"


/*******************************************************************************
* Function Name: I2C_MPU6050_I2C_ISR
********************************************************************************
*
* Summary:
*  Handles the Interrupt Service Routine for the SCB I2C mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
CY_ISR(I2C_MPU6050_I2C_ISR)
{
    uint32 diffCount;
    uint32 endTransfer;

    #if(I2C_MPU6050_CHECK_I2C_ACCEPT_ADDRESS_CONST)
        uint32 address;
    #endif /* (I2C_MPU6050_CHECK_I2C_ACCEPT_ADDRESS_CONST) */

    endTransfer = 0u; /* Continue active transfer */

    /* Call customer routine if registered */
    if(NULL != I2C_MPU6050_customIntrHandler)
    {
        I2C_MPU6050_customIntrHandler();
    }

    if(I2C_MPU6050_CHECK_INTR_I2C_EC_MASKED(I2C_MPU6050_INTR_I2C_EC_WAKE_UP))
    {
        /* Mask-off after wakeup */
        I2C_MPU6050_SetI2CExtClkInterruptMode(I2C_MPU6050_NO_INTR_SOURCES);
    }

    /* Master and Slave error tracking:
    * Add the master state check to track only the master errors when the master is active or
    * track slave errors when the slave is active or idle.
    * A special MMS case: on the address phase with misplaced Start: the master sets the LOST_ARB and
    * slave BUS_ERR. The valid event is LOST_ARB comes from the master.
    */
    if(I2C_MPU6050_CHECK_I2C_FSM_MASTER)
    {
        #if(I2C_MPU6050_I2C_MASTER)
        {
            /* INTR_MASTER_I2C_BUS_ERROR:
            * A misplaced Start or Stop condition occurred on the bus: complete the transaction.
            * The interrupt is cleared in the I2C_FSM_EXIT_IDLE.
            */
            if(I2C_MPU6050_CHECK_INTR_MASTER_MASKED(I2C_MPU6050_INTR_MASTER_I2C_BUS_ERROR))
            {
                I2C_MPU6050_mstrStatus |= (uint16) (I2C_MPU6050_I2C_MSTAT_ERR_XFER |
                                                         I2C_MPU6050_I2C_MSTAT_ERR_BUS_ERROR);

                endTransfer = I2C_MPU6050_I2C_CMPLT_ANY_TRANSFER;
            }

            /* INTR_MASTER_I2C_ARB_LOST:
            * The MultiMaster lost arbitrage during the transaction.
            * A Misplaced Start or Stop condition is treated as lost arbitration when the master drives the SDA.
            * The interrupt source is cleared in the I2C_FSM_EXIT_IDLE.
            */
            if(I2C_MPU6050_CHECK_INTR_MASTER_MASKED(I2C_MPU6050_INTR_MASTER_I2C_ARB_LOST))
            {
                I2C_MPU6050_mstrStatus |= (uint16) (I2C_MPU6050_I2C_MSTAT_ERR_XFER |
                                                         I2C_MPU6050_I2C_MSTAT_ERR_ARB_LOST);

                endTransfer = I2C_MPU6050_I2C_CMPLT_ANY_TRANSFER;
            }

            #if(I2C_MPU6050_I2C_MULTI_MASTER_SLAVE)
            {
                /* I2C_MASTER_CMD_M_START_ON_IDLE:
                * The MultiMaster-Slave does not generate a start, because the Slave was addressed.
                * Pass control to the slave.
                */
                if(I2C_MPU6050_CHECK_I2C_MASTER_CMD(I2C_MPU6050_I2C_MASTER_CMD_M_START_ON_IDLE))
                {
                    I2C_MPU6050_mstrStatus |= (uint16) (I2C_MPU6050_I2C_MSTAT_ERR_XFER |
                                                             I2C_MPU6050_I2C_MSTAT_ERR_ABORT_XFER);

                    endTransfer = I2C_MPU6050_I2C_CMPLT_ANY_TRANSFER;
                }
            }
            #endif

            /* The error handling common part:
            * Set a completion flag of the master transaction and pass control to:
            *  - I2C_FSM_EXIT_IDLE - to complete a transaction in case of: ARB_LOST or BUS_ERR.
            *  - I2C_FSM_IDLE      - to take a chance for the slave to process the incoming transaction.
            */
            if(0u != endTransfer)
            {
                /* Set completion flags for master */
                I2C_MPU6050_mstrStatus |= (uint16) I2C_MPU6050_GET_I2C_MSTAT_CMPLT;

                #if(I2C_MPU6050_I2C_MULTI_MASTER_SLAVE)
                {
                    if(I2C_MPU6050_CHECK_I2C_FSM_ADDR)
                    {
                        /* The Start generation was set after another master start accessing the Slave.
                        * Clean-up the master and turn to the slave. Set the state to IDLE.
                        */
                        if(I2C_MPU6050_CHECK_I2C_MASTER_CMD(I2C_MPU6050_I2C_MASTER_CMD_M_START_ON_IDLE))
                        {
                            I2C_MPU6050_I2C_MASTER_CLEAR_START;

                            endTransfer = I2C_MPU6050_I2C_CMPLT_ANY_TRANSFER; /* Pass control to Slave */
                        }
                        /* The valid arbitration lost on the address phase happens only when: master LOST_ARB is set and
                        * slave BUS_ERR is cleared. Only in that case set the state to IDLE without the SCB IP re-enable.
                        */
                        else if((!I2C_MPU6050_CHECK_INTR_SLAVE_MASKED(I2C_MPU6050_INTR_SLAVE_I2C_BUS_ERROR))
                               && I2C_MPU6050_CHECK_INTR_MASTER_MASKED(I2C_MPU6050_INTR_MASTER_I2C_ARB_LOST))
                        {
                            endTransfer = I2C_MPU6050_I2C_CMPLT_ANY_TRANSFER; /* Pass control to Slave */
                        }
                        else
                        {
                            endTransfer = 0u; /* Causes I2C_FSM_EXIT_IDLE to be set below */
                        }

                        if(0u != endTransfer) /* Clean-up master to proceed with slave */
                        {
                            I2C_MPU6050_CLEAR_TX_FIFO; /* Shifter keeps address, clear it */

                            I2C_MPU6050_DISABLE_MASTER_AUTO_DATA_ACK; /* In case of reading disable autoACK */

                            /* Clean-up master interrupt sources */
                            I2C_MPU6050_ClearMasterInterruptSource(I2C_MPU6050_INTR_MASTER_ALL);

                            /* Disable data processing interrupts: they should be cleared before */
                            I2C_MPU6050_SetRxInterruptMode(I2C_MPU6050_NO_INTR_SOURCES);
                            I2C_MPU6050_SetTxInterruptMode(I2C_MPU6050_NO_INTR_SOURCES);

                            I2C_MPU6050_state = I2C_MPU6050_I2C_FSM_IDLE;
                        }
                        else
                        {
                            /* Set I2C_FSM_EXIT_IDLE for BUS_ERR and ARB_LOST (that is really bus error) */
                            I2C_MPU6050_state = I2C_MPU6050_I2C_FSM_EXIT_IDLE;
                        }
                    }
                    else
                    {
                        /* Set I2C_FSM_EXIT_IDLE if any other state than address */
                        I2C_MPU6050_state = I2C_MPU6050_I2C_FSM_EXIT_IDLE;
                    }
                }
                #else
                {
                    /* In case of LOST*/
                    I2C_MPU6050_state = I2C_MPU6050_I2C_FSM_EXIT_IDLE;
                }
                #endif
            }
        }
        #endif
    }
    else /* (I2C_MPU6050_CHECK_I2C_FSM_SLAVE) */
    {
        #if(I2C_MPU6050_I2C_SLAVE)
        {
            /* INTR_SLAVE_I2C_BUS_ERROR or I2C_MPU6050_INTR_SLAVE_I2C_ARB_LOST:
            * A Misplaced Start or Stop condition occurred on the bus: set a flag
            * to notify an error condition.
            */
            if(I2C_MPU6050_CHECK_INTR_SLAVE_MASKED(I2C_MPU6050_INTR_SLAVE_I2C_BUS_ERROR |
                                                        I2C_MPU6050_INTR_SLAVE_I2C_ARB_LOST))
            {
                if(I2C_MPU6050_CHECK_I2C_FSM_RD)
                {
                    /* TX direction: master reads from slave */
                    I2C_MPU6050_slStatus &= (uint8) ~I2C_MPU6050_I2C_SSTAT_RD_BUSY;
                    I2C_MPU6050_slStatus |= (uint8) (I2C_MPU6050_I2C_SSTAT_RD_ERR |
                                                          I2C_MPU6050_I2C_SSTAT_RD_CMPLT);
                }
                else
                {
                    /* RX direction: master writes into slave */
                    I2C_MPU6050_slStatus &= (uint8) ~I2C_MPU6050_I2C_SSTAT_WR_BUSY;
                    I2C_MPU6050_slStatus |= (uint8) (I2C_MPU6050_I2C_SSTAT_WR_ERR |
                                                          I2C_MPU6050_I2C_SSTAT_WR_CMPLT);
                }

                I2C_MPU6050_state = I2C_MPU6050_I2C_FSM_EXIT_IDLE;
            }
        }
        #endif
    }

    /* States description:
    * Any Master operation starts from: the ADDR_RD/WR state as the master generates traffic on the bus.
    * Any Slave operation starts from: the IDLE state as the slave always waits for actions from the master.
    */

    /* FSM Master */
    if(I2C_MPU6050_CHECK_I2C_FSM_MASTER)
    {
        #if(I2C_MPU6050_I2C_MASTER)
        {
            /* INTR_MASTER_I2C_STOP:
            * A Stop condition was generated by the master: the end of the transaction.
            * Set completion flags to notify the API.
            */
            if(I2C_MPU6050_CHECK_INTR_MASTER_MASKED(I2C_MPU6050_INTR_MASTER_I2C_STOP))
            {
                I2C_MPU6050_ClearMasterInterruptSource(I2C_MPU6050_INTR_MASTER_I2C_STOP);

                I2C_MPU6050_mstrStatus |= (uint16) I2C_MPU6050_GET_I2C_MSTAT_CMPLT;
                I2C_MPU6050_state       = I2C_MPU6050_I2C_FSM_IDLE;
            }
            else
            {
                if(I2C_MPU6050_CHECK_I2C_FSM_ADDR) /* Address stage */
                {
                    /* INTR_MASTER_I2C_NACK:
                    * The master has sent an address but it was NACKed by the slave. Complete transaction.
                    */
                    if(I2C_MPU6050_CHECK_INTR_MASTER_MASKED(I2C_MPU6050_INTR_MASTER_I2C_NACK))
                    {
                        I2C_MPU6050_ClearMasterInterruptSource(I2C_MPU6050_INTR_MASTER_I2C_NACK);

                        I2C_MPU6050_mstrStatus |= (uint16) (I2C_MPU6050_I2C_MSTAT_ERR_XFER |
                                                                 I2C_MPU6050_I2C_MSTAT_ERR_ADDR_NAK);

                        endTransfer = I2C_MPU6050_I2C_CMPLT_ANY_TRANSFER;
                    }
                    /* INTR_TX_UNDERFLOW. The master has sent an address:
                    *  - TX direction: the clock is stretched after the ACK phase, because the TX FIFO is
                    *    EMPTY. The TX EMPTY cleans all the TX interrupt sources.
                    *  - RX direction: the 1st byte is received, but there is no ACK permission,
                    *    the clock is stretched after 1 byte is received.
                    */
                    else
                    {
                        if(I2C_MPU6050_CHECK_I2C_FSM_RD) /* Reading */
                        {
                            I2C_MPU6050_state = I2C_MPU6050_I2C_FSM_MSTR_RD_DATA;
                        }
                        else /* Writing */
                        {
                            I2C_MPU6050_state = I2C_MPU6050_I2C_FSM_MSTR_WR_DATA;
                            if(0u != I2C_MPU6050_mstrWrBufSize)
                            {
                                /* Enable INTR.TX_EMPTY if there is data to transmit */
                                I2C_MPU6050_SetTxInterruptMode(I2C_MPU6050_INTR_TX_EMPTY);
                            }
                        }
                    }
                }

                if(I2C_MPU6050_CHECK_I2C_FSM_DATA) /* Data phase */
                {
                    if(I2C_MPU6050_CHECK_I2C_FSM_RD) /* Reading */
                    {
                        /* INTR_RX_FULL:
                        * RX direction: the master has received 8 bytes.
                        * Get data from the RX FIFO and decide whether to ACK or  NACK the following bytes.
                        */
                        if(I2C_MPU6050_CHECK_INTR_RX_MASKED(I2C_MPU6050_INTR_RX_FULL))
                        {
                            /* Calculate difference */
                            diffCount =  I2C_MPU6050_mstrRdBufSize -
                                        (I2C_MPU6050_mstrRdBufIndex + I2C_MPU6050_GET_RX_FIFO_ENTRIES);

                            /* Proceed transaction or end it when RX FIFO becomes FULL again */
                            if(diffCount > I2C_MPU6050_FIFO_SIZE)
                            {
                                diffCount = I2C_MPU6050_FIFO_SIZE;
                            }
                            else
                            {
                                if(0u == diffCount)
                                {
                                    I2C_MPU6050_DISABLE_MASTER_AUTO_DATA_ACK;

                                    diffCount   = I2C_MPU6050_FIFO_SIZE;
                                    endTransfer = I2C_MPU6050_I2C_CMPLT_ANY_TRANSFER;
                                }
                            }

                            for(; (0u != diffCount); diffCount--)
                            {
                                I2C_MPU6050_mstrRdBufPtr[I2C_MPU6050_mstrRdBufIndex] = (uint8)
                                                                                        I2C_MPU6050_RX_FIFO_RD_REG;
                                I2C_MPU6050_mstrRdBufIndex++;
                            }
                        }
                        /* INTR_RX_NOT_EMPTY:
                        * RX direction: the master has received one data byte, ACK or NACK it.
                        * The last byte is stored and NACKed by the master. The NACK and Stop is
                        * generated by one command generate Stop.
                        */
                        else if(I2C_MPU6050_CHECK_INTR_RX_MASKED(I2C_MPU6050_INTR_RX_NOT_EMPTY))
                        {
                            /* Put data in component buffer */
                            I2C_MPU6050_mstrRdBufPtr[I2C_MPU6050_mstrRdBufIndex] = (uint8) I2C_MPU6050_RX_FIFO_RD_REG;
                            I2C_MPU6050_mstrRdBufIndex++;

                            if(I2C_MPU6050_mstrRdBufIndex < I2C_MPU6050_mstrRdBufSize)
                            {
                                I2C_MPU6050_I2C_MASTER_GENERATE_ACK;
                            }
                            else
                            {
                               endTransfer = I2C_MPU6050_I2C_CMPLT_ANY_TRANSFER;
                            }
                        }
                        else
                        {
                            /* Do nothing */
                        }

                        I2C_MPU6050_ClearRxInterruptSource(I2C_MPU6050_INTR_RX_ALL);
                    }
                    else /* Writing */
                    {
                        /* INTR_MASTER_I2C_NACK :
                        * The master writes data to the slave and NACK was received: not all the bytes were
                        * written to the slave from the TX FIFO. Revert the index if there is data in
                        * the TX FIFO and pass control to a complete transfer.
                        */
                        if(I2C_MPU6050_CHECK_INTR_MASTER_MASKED(I2C_MPU6050_INTR_MASTER_I2C_NACK))
                        {
                            I2C_MPU6050_ClearMasterInterruptSource(I2C_MPU6050_INTR_MASTER_I2C_NACK);

                            /* Rollback write buffer index: NACKed byte remains in shifter */
                            I2C_MPU6050_mstrWrBufIndexTmp -= (I2C_MPU6050_GET_TX_FIFO_ENTRIES +
                                                                   I2C_MPU6050_GET_TX_FIFO_SR_VALID);

                            /* Update number of transferred bytes */
                            I2C_MPU6050_mstrWrBufIndex = I2C_MPU6050_mstrWrBufIndexTmp;

                            I2C_MPU6050_mstrStatus |= (uint16) (I2C_MPU6050_I2C_MSTAT_ERR_XFER |
                                                                     I2C_MPU6050_I2C_MSTAT_ERR_SHORT_XFER);

                            I2C_MPU6050_CLEAR_TX_FIFO;

                            endTransfer = I2C_MPU6050_I2C_CMPLT_ANY_TRANSFER;
                        }
                        /* INTR_TX_EMPTY :
                        * TX direction: the TX FIFO is EMPTY, the data from the buffer needs to be put there.
                        * When there is no data in the component buffer, the underflow interrupt is
                        * enabled to catch when all the data has been transferred.
                        */
                        else if(I2C_MPU6050_CHECK_INTR_TX_MASKED(I2C_MPU6050_INTR_TX_EMPTY))
                        {
                            while(I2C_MPU6050_FIFO_SIZE != I2C_MPU6050_GET_TX_FIFO_ENTRIES)
                            {
                                /* The temporary mstrWrBufIndexTmp is used because slave could NACK the byte and index
                                * roll-back required in this case. The mstrWrBufIndex is updated at the end of transfer
                                */
                                if(I2C_MPU6050_mstrWrBufIndexTmp < I2C_MPU6050_mstrWrBufSize)
                                {
                                #if(!I2C_MPU6050_CY_SCBIP_V0)
                                   /* Clear INTR_TX.UNDERFLOW before put last byte into the TX FIFO. This ensures
                                    * proper trigger at the end of transaction when INTR_TX.UNDERFLOW single trigger
                                    * event. Ticket ID# 156735.
                                    */
                                    if(I2C_MPU6050_mstrWrBufIndexTmp == (I2C_MPU6050_mstrWrBufSize - 1u))
                                    {
                                        I2C_MPU6050_ClearTxInterruptSource(I2C_MPU6050_INTR_TX_UNDERFLOW);
                                        I2C_MPU6050_SetTxInterruptMode(I2C_MPU6050_INTR_TX_UNDERFLOW);
                                    }
                                 #endif /* (!I2C_MPU6050_CY_SCBIP_V0) */

                                    /* Put data into TX FIFO */
                                    I2C_MPU6050_TX_FIFO_WR_REG = (uint32) I2C_MPU6050_mstrWrBufPtr[I2C_MPU6050_mstrWrBufIndexTmp];
                                    I2C_MPU6050_mstrWrBufIndexTmp++;
                                }
                                else
                                {
                                    break; /* No more data to put */
                                }
                            }

                        #if(I2C_MPU6050_CY_SCBIP_V0)
                            if(I2C_MPU6050_mstrWrBufIndexTmp == I2C_MPU6050_mstrWrBufSize)
                            {
                                I2C_MPU6050_SetTxInterruptMode(I2C_MPU6050_INTR_TX_UNDERFLOW);
                            }

                            I2C_MPU6050_ClearTxInterruptSource(I2C_MPU6050_INTR_TX_ALL);
                        #else
                            I2C_MPU6050_ClearTxInterruptSource(I2C_MPU6050_INTR_TX_EMPTY);
                        #endif /* (I2C_MPU6050_CY_SCBIP_V0) */
                        }
                        /* INTR_TX_UNDERFLOW:
                        * TX direction: all data from the TX FIFO was transferred to the slave.
                        * The transaction needs to be completed.
                        */
                        else if(I2C_MPU6050_CHECK_INTR_TX_MASKED(I2C_MPU6050_INTR_TX_UNDERFLOW))
                        {
                            /* Update number of transferred bytes */
                            I2C_MPU6050_mstrWrBufIndex = I2C_MPU6050_mstrWrBufIndexTmp;

                            endTransfer = I2C_MPU6050_I2C_CMPLT_ANY_TRANSFER;
                        }
                        else
                        {
                            /* Do nothing */
                        }
                    }
                }

                if(0u != endTransfer) /* Complete transfer */
                {
                    /* Clean-up master after reading: only in case of NACK */
                    I2C_MPU6050_DISABLE_MASTER_AUTO_DATA_ACK;

                    /* Disable data processing interrupts: they should be cleared before */
                    I2C_MPU6050_SetRxInterruptMode(I2C_MPU6050_NO_INTR_SOURCES);
                    I2C_MPU6050_SetTxInterruptMode(I2C_MPU6050_NO_INTR_SOURCES);

                    if(I2C_MPU6050_CHECK_I2C_MODE_NO_STOP(I2C_MPU6050_mstrControl))
                    {
                        /* On-going transaction is suspended: the ReStart is generated by the API request */
                        I2C_MPU6050_mstrStatus |= (uint16) (I2C_MPU6050_I2C_MSTAT_XFER_HALT |
                                                                 I2C_MPU6050_GET_I2C_MSTAT_CMPLT);

                        I2C_MPU6050_state = I2C_MPU6050_I2C_FSM_MSTR_HALT;
                    }
                    else
                    {
                        /* Complete transaction: exclude the data processing state and generate Stop.
                        * The completion status will be set after Stop generation.
                        * A special case is read: because NACK and Stop are generated by command below.
                        * The lost arbitration could occur during NACK generation in case when
                        * other master is still reading from the slave.
                        */
                        I2C_MPU6050_I2C_MASTER_GENERATE_STOP;
                    }
                }
            }

        } /* (I2C_MPU6050_I2C_MASTER) */
        #endif

    } /* (I2C_MPU6050_CHECK_I2C_FSM_MASTER) */


    /* FSM Slave */
    else if(I2C_MPU6050_CHECK_I2C_FSM_SLAVE)
    {
        #if(I2C_MPU6050_I2C_SLAVE)
        {
            /* INTR_SLAVE_NACK:
            * The master completes reading the slave: the appropriate flags have to be set.
            * The TX FIFO is cleared after an overflow condition is set.
            */
            if(I2C_MPU6050_CHECK_INTR_SLAVE_MASKED(I2C_MPU6050_INTR_SLAVE_I2C_NACK))
            {
                I2C_MPU6050_ClearSlaveInterruptSource(I2C_MPU6050_INTR_SLAVE_I2C_NACK);

                /* All entries that remain in TX FIFO max value is 9: 8 (FIFO) + 1 (SHIFTER) */
                diffCount = (I2C_MPU6050_GET_TX_FIFO_ENTRIES + I2C_MPU6050_GET_TX_FIFO_SR_VALID);

                if(I2C_MPU6050_slOverFlowCount > diffCount) /* Overflow */
                {
                    I2C_MPU6050_slStatus |= (uint8) I2C_MPU6050_I2C_SSTAT_RD_OVFL;
                }
                else /* No Overflow */
                {
                    /* Roll-back temporary index */
                    I2C_MPU6050_slRdBufIndexTmp -= (diffCount - I2C_MPU6050_slOverFlowCount);
                }

                /* Update slave of transferred bytes */
                I2C_MPU6050_slRdBufIndex = I2C_MPU6050_slRdBufIndexTmp;

                /* Clean-up TX FIFO */
                I2C_MPU6050_SetTxInterruptMode(I2C_MPU6050_NO_INTR_SOURCES);
                I2C_MPU6050_slOverFlowCount = 0u;
                I2C_MPU6050_CLEAR_TX_FIFO;

                /* Complete master reading */
                I2C_MPU6050_slStatus &= (uint8) ~I2C_MPU6050_I2C_SSTAT_RD_BUSY;
                I2C_MPU6050_slStatus |= (uint8)  I2C_MPU6050_I2C_SSTAT_RD_CMPLT;
                I2C_MPU6050_state     =  I2C_MPU6050_I2C_FSM_IDLE;
            }


            /* INTR_SLAVE_I2C_WRITE_STOP:
            * The master completes writing to the slave: the appropriate flags have to be set.
            * The RX FIFO contains 1-8 bytes from the previous transaction which needs to be read.
            * There is a possibility that the RX FIFO contains an address, it needs to leave it there.
            */
            if(I2C_MPU6050_CHECK_INTR_SLAVE_MASKED(I2C_MPU6050_INTR_SLAVE_I2C_WRITE_STOP))
            {
                I2C_MPU6050_ClearSlaveInterruptSource(I2C_MPU6050_INTR_SLAVE_I2C_WRITE_STOP);

                /* Read bytes from the RX FIFO when auto data ACK receive logic is enabled. Otherwise all data bytes
                * were already read from the RX FIFO accept address byte which has to stay here to be handled by 
                * I2C_ADDR_MATCH.
                */
                if (0u != (I2C_MPU6050_I2C_CTRL_REG & I2C_MPU6050_I2C_CTRL_S_READY_DATA_ACK))
                {
                    while(0u != I2C_MPU6050_GET_RX_FIFO_ENTRIES)
                    {
                        #if(I2C_MPU6050_CHECK_I2C_ACCEPT_ADDRESS)
                        {
                            if((1u == I2C_MPU6050_GET_RX_FIFO_ENTRIES) &&
                               (I2C_MPU6050_CHECK_INTR_SLAVE_MASKED(I2C_MPU6050_INTR_SLAVE_I2C_ADDR_MATCH)))
                            {
                                break; /* Leave address in RX FIFO */
                            }
                        }
                        #endif

                        /* Put data in component buffer */
                        I2C_MPU6050_slWrBufPtr[I2C_MPU6050_slWrBufIndex] = (uint8) I2C_MPU6050_RX_FIFO_RD_REG;
                        I2C_MPU6050_slWrBufIndex++;
                    }
                    
                    I2C_MPU6050_DISABLE_SLAVE_AUTO_DATA;
                }

                if(I2C_MPU6050_CHECK_INTR_RX(I2C_MPU6050_INTR_RX_OVERFLOW))
                {
                    I2C_MPU6050_slStatus |= (uint8) I2C_MPU6050_I2C_SSTAT_WR_OVFL;
                }

                /* Clears RX interrupt sources triggered on data receiving */
                I2C_MPU6050_SetRxInterruptMode(I2C_MPU6050_NO_INTR_SOURCES);
                I2C_MPU6050_ClearRxInterruptSource(I2C_MPU6050_INTR_RX_ALL);

                /* Complete master writing */
                I2C_MPU6050_slStatus &= (uint8) ~I2C_MPU6050_I2C_SSTAT_WR_BUSY;
                I2C_MPU6050_slStatus |= (uint8)  I2C_MPU6050_I2C_SSTAT_WR_CMPLT;
                I2C_MPU6050_state     =  I2C_MPU6050_I2C_FSM_IDLE;
            }


            /* INTR_SLAVE_I2C_ADDR_MATCH:
            * The address match event starts the slave operation: after leaving the TX or RX
            * direction has to be chosen.
            * The wakeup interrupt must be cleared only after an address match is set.
            */
            if(I2C_MPU6050_CHECK_INTR_SLAVE_MASKED(I2C_MPU6050_INTR_SLAVE_I2C_ADDR_MATCH))
            {
                #if(I2C_MPU6050_CHECK_I2C_ACCEPT_ADDRESS)
                {
                    address = I2C_MPU6050_RX_FIFO_RD_REG; /* Address in the RX FIFO */

                    /* Clears RX sources if address was received in RX FIFO */
                    I2C_MPU6050_ClearRxInterruptSource(I2C_MPU6050_INTR_RX_ALL);

                    if(0u != address)
                    {
                        /* Suppress compiler warning */
                    }
                }
                #endif

                if(I2C_MPU6050_CHECK_I2C_STATUS(I2C_MPU6050_I2C_STATUS_S_READ))
                /* TX direction: master reads from slave */
                {
                    I2C_MPU6050_SetTxInterruptMode(I2C_MPU6050_INTR_TX_EMPTY);

                    /* Set temporary index to address buffer clear from API */
                    I2C_MPU6050_slRdBufIndexTmp = I2C_MPU6050_slRdBufIndex;

                    /* Start master reading */
                    I2C_MPU6050_slStatus |= (uint8) I2C_MPU6050_I2C_SSTAT_RD_BUSY;
                    I2C_MPU6050_state     = I2C_MPU6050_I2C_FSM_SL_RD;
                }
                else
                /* RX direction: master writes into slave */
                {
                    /* Calculate available buffer size */
                    diffCount = (I2C_MPU6050_slWrBufSize - I2C_MPU6050_slWrBufIndex);

                #if (I2C_MPU6050_CY_SCBIP_V0)
                    
                    if(diffCount < I2C_MPU6050_FIFO_SIZE)
                    /* Receive data: byte-by-byte */
                    {
                        I2C_MPU6050_SetRxInterruptMode(I2C_MPU6050_INTR_RX_NOT_EMPTY);
                    }
                    else
                    /* Receive data: into RX FIFO */
                    {
                        if(diffCount == I2C_MPU6050_FIFO_SIZE)
                        {
                            /* NACK when RX FIFO become FULL */
                            I2C_MPU6050_ENABLE_SLAVE_AUTO_DATA;
                        }
                        else
                        {
                            /* Stretch clock when RX FIFO becomes FULL */
                            I2C_MPU6050_ENABLE_SLAVE_AUTO_DATA_ACK;
                            I2C_MPU6050_SetRxInterruptMode(I2C_MPU6050_INTR_RX_FULL);
                        }
                    }
                    
                #else
                    
                    #if(I2C_MPU6050_CHECK_I2C_ACCEPT_ADDRESS)
                    {
                        /* Enable RX.NOT_EMPTY interrupt source to receive byte by byte.
                        * The byte by byte receive is always chosen for when address is accpected in the RX FIFO. 
                        * Ticket ID#175559.
                        */
                        I2C_MPU6050_SetRxInterruptMode(I2C_MPU6050_INTR_RX_NOT_EMPTY);
                    }
                    #else
                    {
                        if(diffCount < I2C_MPU6050_FIFO_SIZE)
                        /* Receive data: byte-by-byte */
                        {
                            I2C_MPU6050_SetRxInterruptMode(I2C_MPU6050_INTR_RX_NOT_EMPTY);
                        }
                        else
                        /* Receive data: into RX FIFO */
                        {
                            if(diffCount == I2C_MPU6050_FIFO_SIZE)
                            {
                                /* NACK when RX FIFO become FULL */
                                I2C_MPU6050_ENABLE_SLAVE_AUTO_DATA;
                            }
                            else
                            {
                                /* Stretch clock when RX FIFO becomes FULL */
                                I2C_MPU6050_ENABLE_SLAVE_AUTO_DATA_ACK;
                                I2C_MPU6050_SetRxInterruptMode(I2C_MPU6050_INTR_RX_FULL);
                            }
                        }
                    }
                    #endif
                    
                #endif /* (I2C_MPU6050_CY_SCBIP_V0) */

                    /* Start master reading */
                    I2C_MPU6050_slStatus |= (uint8) I2C_MPU6050_I2C_SSTAT_WR_BUSY;
                    I2C_MPU6050_state     = I2C_MPU6050_I2C_FSM_SL_WR;
                }

                /* Clear interrupts before ACK address */
                I2C_MPU6050_ClearI2CExtClkInterruptSource(I2C_MPU6050_INTR_I2C_EC_WAKE_UP);
                I2C_MPU6050_ClearSlaveInterruptSource(I2C_MPU6050_INTR_SLAVE_ALL);

                /* The preparation complete: ACK the address */
                I2C_MPU6050_I2C_SLAVE_GENERATE_ACK;
            }

            /* I2C_MPU6050_INTR_RX_FULL":
            * Get data from the RX FIFO and decide whether to ACK or NACK the following bytes
            */
            if(I2C_MPU6050_CHECK_INTR_RX_MASKED(I2C_MPU6050_INTR_RX_FULL))
            {
                /* Calculate available buffer size to take into account that RX FIFO is FULL */
                diffCount =  I2C_MPU6050_slWrBufSize -
                            (I2C_MPU6050_slWrBufIndex + I2C_MPU6050_FIFO_SIZE);

                if(diffCount > I2C_MPU6050_FIFO_SIZE) /* Proceed transaction */
                {
                    diffCount   = I2C_MPU6050_FIFO_SIZE;
                    endTransfer = 0u;  /* Continue active transfer */
                }
                else /* End when FIFO becomes FULL again */
                {
                    endTransfer = I2C_MPU6050_I2C_CMPLT_ANY_TRANSFER;
                }

                for(; (0u != diffCount); diffCount--)
                {
                    /* Put data in component buffer */
                    I2C_MPU6050_slWrBufPtr[I2C_MPU6050_slWrBufIndex] = (uint8) I2C_MPU6050_RX_FIFO_RD_REG;
                    I2C_MPU6050_slWrBufIndex++;
                }

                if(0u != endTransfer) /* End transfer sending NACK */
                {
                    I2C_MPU6050_ENABLE_SLAVE_AUTO_DATA_NACK;

                    /* The INTR_RX_FULL triggers earlier than INTR_SLAVE_I2C_STOP:
                    * disable all the RX interrupt sources.
                    */
                    I2C_MPU6050_SetRxInterruptMode(I2C_MPU6050_NO_INTR_SOURCES);
                }

                I2C_MPU6050_ClearRxInterruptSource(I2C_MPU6050_INTR_RX_FULL);
            }
            /* I2C_MPU6050_INTR_RX_NOT_EMPTY:
            * The buffer size is less than 8: it requires processing in byte-by-byte mode.
            */
            else if(I2C_MPU6050_CHECK_INTR_RX_MASKED(I2C_MPU6050_INTR_RX_NOT_EMPTY))
            {
                diffCount = I2C_MPU6050_RX_FIFO_RD_REG;

                if(I2C_MPU6050_slWrBufIndex < I2C_MPU6050_slWrBufSize)
                {
                    I2C_MPU6050_I2C_SLAVE_GENERATE_ACK;

                    /* Put data into component buffer */
                    I2C_MPU6050_slWrBufPtr[I2C_MPU6050_slWrBufIndex] = (uint8) diffCount;
                    I2C_MPU6050_slWrBufIndex++;
                }
                else /* Overflow: there is no space in write buffer */
                {
                    I2C_MPU6050_I2C_SLAVE_GENERATE_NACK;

                    I2C_MPU6050_slStatus |= (uint8) I2C_MPU6050_I2C_SSTAT_WR_OVFL;
                }

                I2C_MPU6050_ClearRxInterruptSource(I2C_MPU6050_INTR_RX_NOT_EMPTY);
            }
            else
            {
                /* Does nothing */
            }


            /* I2C_MPU6050_INTR_TX_EMPTY:
            * The master reads the slave: provide data to read or 0xFF in case of the end of the buffer
            * The overflow condition must be captured, but not set until the end of a transaction.
            * There is a possibility of a false overflow due of the TX FIFO utilization.
            */
            if(I2C_MPU6050_CHECK_INTR_TX_MASKED(I2C_MPU6050_INTR_TX_EMPTY))
            {
                while(I2C_MPU6050_FIFO_SIZE != I2C_MPU6050_GET_TX_FIFO_ENTRIES)
                {
                    /* The temporary slRdBufIndexTmp is used because the master could NACK the byte and
                    * index roll-back is required in this case. The slRdBufIndex is updated at the end
                    * of the read transfer.
                    */
                    if(I2C_MPU6050_slRdBufIndexTmp < I2C_MPU6050_slRdBufSize)
                    /* Data from buffer */
                    {
                        I2C_MPU6050_TX_FIFO_WR_REG = (uint32) I2C_MPU6050_slRdBufPtr[I2C_MPU6050_slRdBufIndexTmp];
                        I2C_MPU6050_slRdBufIndexTmp++;
                    }
                    else
                    /* Probably Overflow */
                    {
                        I2C_MPU6050_TX_FIFO_WR_REG = I2C_MPU6050_I2C_SLAVE_OVFL_RETURN;

                        if(0u == (I2C_MPU6050_INTR_TX_OVERFLOW & I2C_MPU6050_slOverFlowCount))
                        {
                            /* Get counter in range of the byte: value 10 is overflow */
                            I2C_MPU6050_slOverFlowCount++;
                        }
                    }
                }

                I2C_MPU6050_ClearTxInterruptSource(I2C_MPU6050_INTR_TX_EMPTY);
            }

        }  /* (I2C_MPU6050_I2C_SLAVE) */
        #endif
    }


    /* FSM EXIT:
    * Slave:  INTR_SLAVE_I2C_BUS_ERROR, INTR_SLAVE_I2C_ARB_LOST
    * Master: INTR_MASTER_I2C_BUS_ERROR, INTR_MASTER_I2C_ARB_LOST.
    */
    else
    {
        I2C_MPU6050_CTRL_REG &= (uint32) ~I2C_MPU6050_CTRL_ENABLED; /* Disable SCB block */

        I2C_MPU6050_state = I2C_MPU6050_I2C_FSM_IDLE;

        I2C_MPU6050_DISABLE_SLAVE_AUTO_DATA;
        I2C_MPU6050_DISABLE_MASTER_AUTO_DATA;

    #if(I2C_MPU6050_CY_SCBIP_V0)
        I2C_MPU6050_SetRxInterruptMode(I2C_MPU6050_NO_INTR_SOURCES);
        I2C_MPU6050_SetTxInterruptMode(I2C_MPU6050_NO_INTR_SOURCES);

        /* Clear interrupt sources as they are not automatically cleared after SCB is disabled */
        I2C_MPU6050_ClearTxInterruptSource(I2C_MPU6050_INTR_RX_ALL);
        I2C_MPU6050_ClearRxInterruptSource(I2C_MPU6050_INTR_TX_ALL);
        I2C_MPU6050_ClearSlaveInterruptSource(I2C_MPU6050_INTR_SLAVE_ALL);
        I2C_MPU6050_ClearMasterInterruptSource(I2C_MPU6050_INTR_MASTER_ALL);
    #endif /* (I2C_MPU6050_CY_SCBIP_V0) */

        I2C_MPU6050_CTRL_REG |= (uint32) I2C_MPU6050_CTRL_ENABLED;  /* Enable SCB block */
    }
}


/* [] END OF FILE */
