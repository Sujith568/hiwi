//*****************************************************************************
//
//! @file am_hal_mspi.c
//!
//! @brief Functions for interfacing with the MSPI.
//!
//! @addtogroup mspi3p MSPI - Multi-bit SPI
//! @ingroup apollo3p_hal
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2023, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision release_sdk_3_1_1-10cda4b5e0 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"

//*****************************************************************************
//
// Private Types.
//
//*****************************************************************************

#define AM_HAL_MAGIC_MSPI               0xBEBEBE
#define AM_HAL_MSPI_CHK_HANDLE(h)       ((h) && ((am_hal_handle_prefix_t *)(h))->s.bInit && (((am_hal_handle_prefix_t *)(h))->s.magic == AM_HAL_MAGIC_MSPI))
#define AM_HAL_MSPI_HW_IDX_MAX          (AM_REG_MSPI_CQCURIDX_CQCURIDX_M >> AM_REG_MSPI_CQCURIDX_CQCURIDX_S)    // 8 bit value
#define AM_HAL_MSPI_MAX_CQ_ENTRIES      (256)
#define AM_HAL_MSPI_OCTAL_MODULE        (1)



// For MSPI - Need to Set the flag for unpausing
#define AM_HAL_MSPI_SC_PAUSE_CQ         AM_HAL_MSPI_SC_PAUSE(AM_HAL_MSPI_PAUSE_FLAG_CQ)
#define AM_HAL_MSPI_SC_PAUSE_SEQLOOP    AM_HAL_MSPI_SC_PAUSE(AM_HAL_MSPI_PAUSE_FLAG_SEQLOOP)
#define AM_HAL_MSPI_SC_UNPAUSE_CQ       AM_HAL_MSPI_SC_UNPAUSE(AM_HAL_MSPI_PAUSE_FLAG_CQ)
#define AM_HAL_MSPI_SC_UNPAUSE_SEQLOOP  AM_HAL_MSPI_SC_UNPAUSE(AM_HAL_MSPI_PAUSE_FLAG_SEQLOOP)
#define AM_HAL_MSPI_SC_PAUSE_BLOCK      AM_HAL_MSPI_SC_PAUSE(AM_HAL_MSPI_PAUSE_FLAG_BLOCK)
#define AM_HAL_MSPI_SC_UNPAUSE_BLOCK    AM_HAL_MSPI_SC_UNPAUSE(AM_HAL_MSPI_PAUSE_FLAG_BLOCK)


// Max time to wait when attempting to pause the command queue
#define AM_HAL_MSPI_MAX_PAUSE_DELAY     (100*1000) // 100ms

//
// Command Queue entry structure for DMA transfer.
//
typedef struct
{
    uint32_t                    ui32PAUSENAddr;
    uint32_t                    ui32PAUSEENVal;
    uint32_t                    ui32PAUSEN2Addr;
    uint32_t                    ui32PAUSEEN2Val;
    uint32_t                    ui32DMATARGADDRAddr;
    uint32_t                    ui32DMATARGADDRVal;
    uint32_t                    ui32DMADEVADDRAddr;
    uint32_t                    ui32DMADEVADDRVal;
    uint32_t                    ui32DMATOTCOUNTAddr;
    uint32_t                    ui32DMATOTCOUNTVal;
    uint32_t                    ui32DMACFG1Addr;
    uint32_t                    ui32DMACFG1Val;
    // Need to insert couple of Dummy's
    uint32_t                    ui32DummyAddr1;
    uint32_t                    ui32DummyVal1;
    uint32_t                    ui32DummyAddr2;
    uint32_t                    ui32DummyVal2;
    // Need to disable the DMA to prepare for next reconfig
    // Need to have this following the DMAEN for CMDQ
    uint32_t                    ui32DMACFG2Addr;
    uint32_t                    ui32DMACFG2Val;
    uint32_t                    ui32SETCLRAddr;
    uint32_t                    ui32SETCLRVal;
} am_hal_mspi_cq_dma_entry_t;

//
// structure for Hi Prio DMA transfer.
//
typedef struct
{
    uint32_t                    ui32DMATARGADDRVal;
    uint32_t                    ui32DMADEVADDRVal;
    uint32_t                    ui32DMATOTCOUNTVal;
    uint32_t                    ui32DMACFG1Val;
    am_hal_mspi_callback_t      pfnCallback;
    void                        *pCallbackCtxt;
} am_hal_mspi_dma_entry_t;

//
// Command Queue entry structure for Sequence Repeat
//
typedef struct
{
    uint32_t                    ui32PAUSENAddr;
    uint32_t                    ui32PAUSEENVal;
    uint32_t                    ui32PAUSEN2Addr;
    uint32_t                    ui32PAUSEEN2Val;
    uint32_t                    ui32SETCLRAddr;
    uint32_t                    ui32SETCLRVal;
} am_hal_mspi_cq_loop_entry_t;

//
// Command Queue entry structure for PIO transfer.
//
typedef struct
{
    uint32_t                    ui32ADDRAddr;
    uint32_t                    ui32ADDRVal;
    uint32_t                    ui32INSTRAddr;
    uint32_t                    ui32INSTRVal;
    uint32_t                    ui32CTRLAddr;
    uint32_t                    ui32CTRLVal;
} am_hal_mspi_cq_pio_entry_t;

typedef struct
{
    bool        bValid;
    uint32_t    regCFG;
    uint32_t    regMSPICFG;
    uint32_t    regPADCFG;
    uint32_t    regPADOUTEN;
    uint32_t    regFLASH;
    uint32_t    regSCRAMBLING;
    uint32_t    regCQCFG;
    uint32_t    regCQADDR;
    uint32_t    regCQPAUSE;
    uint32_t    regCQCURIDX;
    uint32_t    regCQENDIDX;
    uint32_t    regINTEN;
    uint32_t    regCQFLAGS;
    uint32_t    regXIPINSTR;
    uint32_t    regDMABCOUNT;
    uint32_t    regDMATHRESH;
    uint32_t    regTHRESHOLD;
} am_hal_mspi_register_state_t;

//
// Command Queue control structure.
//
typedef struct
{
    void     *pCmdQHdl;
} am_hal_mspi_CQ_t;

typedef enum
{
    AM_HAL_MSPI_SEQ_NONE,
    AM_HAL_MSPI_SEQ_UNDER_CONSTRUCTION,
    AM_HAL_MSPI_SEQ_RUNNING,
} am_hal_mspi_seq_e;

//
// MSPI State structure.
//
typedef struct
{
    //
    // Handle validation prefix.
    //
    am_hal_handle_prefix_t  prefix;

    //
    // Physical module number.
    //
    uint32_t                      ui32Module;

    //
    // Selected flash device configuration.
    //
    am_hal_mspi_device_e          eDeviceConfig;

    //
    // Clock frequency
    //
    am_hal_mspi_clock_e           eClockFreq;

    //
    // Endianess of the FIFO interface.
    //
    bool                          bBigEndian;

    //
    // Delay timeout value.
    //
    uint32_t                      waitTimeout;
    // DMA Transfer Control Buffer size in words.
    uint32_t                      ui32TCBSize;

    // DMA Transfer Control Buffer
    uint32_t                      *pTCB;

    uint32_t ui32LastIdxProcessed;
    uint32_t ui32NumCQEntries;
    uint32_t ui32TxnInt;

    //
    // Stores the CQ callbacks.
    //
    am_hal_mspi_callback_t pfnCallback[AM_HAL_MSPI_MAX_CQ_ENTRIES];

    void                   *pCallbackCtxt[AM_HAL_MSPI_MAX_CQ_ENTRIES];
    //
    // Command Queue.
    //
    am_hal_mspi_CQ_t              CQ;
    // To support sequence
    am_hal_mspi_seq_e             eSeq;
    bool                          bAutonomous;
    uint32_t                      ui32NumTransactions;
    volatile bool                 bRestart;
    uint32_t                      block;
    // To support high priority transactions - out of band
    // High Priority DMA transactions
    volatile bool                 bHP;
    uint32_t                      ui32NumHPEntries;
    uint32_t                      ui32NumHPPendingEntries;
    uint32_t                      ui32MaxHPTransactions;
    uint32_t                      ui32NextHPIdx;
    uint32_t                      ui32LastHPIdxProcessed;
    am_hal_mspi_dma_entry_t       *pHPTransactions;
    // Max pending transactions based on NB Buffer size
    uint32_t                      ui32MaxPending;
    // Number of back to back transactions with no callbacks
    uint32_t                      ui32NumUnSolicited;

    //
    // MSPI Capabilities.
    //
    am_hal_mspi_capabilities_t    capabilities;

    // Power Save-Restore register state
    am_hal_mspi_register_state_t  registerState;
} am_hal_mspi_state_t;

//*****************************************************************************
//
// Global Variables.
//
//*****************************************************************************
static am_hal_mspi_state_t  g_MSPIState[AM_REG_MSPI_NUM_MODULES];


//*****************************************************************************
//
// Internal Functions.
//
//*****************************************************************************
static uint32_t
get_pause_val(am_hal_mspi_state_t *pMSPIState, uint32_t pause)
{
    uint32_t retval;
    switch (pMSPIState->block)
    {
        case 1:
            // Pause the CQ till the whole block is built
            retval = pause | AM_HAL_MSPI_CQP_PAUSE_DEFAULT | AM_HAL_MSPI_PAUSE_FLAG_BLOCK;
            pMSPIState->block = 2;
            break;
        case 2:
            // No pausing allowed
            retval = AM_HAL_MSPI_PAUSE_DEFAULT;
            break;
        default: // case 0
            retval = pause | AM_HAL_MSPI_CQP_PAUSE_DEFAULT;
    }
    return retval;
}

uint32_t
build_dma_cmdlist(am_hal_mspi_state_t *pMSPIState,
                  am_hal_mspi_trans_e eMode,
                  void  *pCQEntry,
                  void  *pTransaction)
{
    uint32_t ui32Module = pMSPIState->ui32Module;

    switch(eMode)
    {
        case AM_HAL_MSPI_TRANS_PIO:
        {
            am_hal_mspi_cq_pio_entry_t    *pPIOEntry = (am_hal_mspi_cq_pio_entry_t*)pCQEntry;
            am_hal_mspi_pio_transfer_t    *pPIOTrans = (am_hal_mspi_pio_transfer_t*)pTransaction;

            //
            // Perform some sanity checks on the transaction.
            //
            if (pPIOTrans->ui32NumBytes > 65535)
            {
              return AM_HAL_STATUS_OUT_OF_RANGE;
            }

            //
            // Command to set the CTRL register.
            //
            pPIOEntry->ui32ADDRAddr            = (uint32_t)&MSPIn(ui32Module)->ADDR;
            pPIOEntry->ui32ADDRVal             = _VAL2FLD(MSPI0_ADDR_ADDR, pPIOTrans->ui32DeviceAddr);
            pPIOEntry->ui32INSTRAddr           = (uint32_t)&MSPIn(ui32Module)->INSTR;
            pPIOEntry->ui32INSTRVal            = _VAL2FLD(MSPI0_INSTR_INSTR, pPIOTrans->ui16DeviceInstr);
            pPIOEntry->ui32CTRLAddr            = (uint32_t)&MSPIn(ui32Module)->CTRL;
            pPIOEntry->ui32CTRLVal             =
                _VAL2FLD(MSPI0_CTRL_XFERBYTES, pPIOTrans->ui32NumBytes)  |   // Set the number of bytes to transfer.
                _VAL2FLD(MSPI0_CTRL_ENDCX, pPIOTrans->bDCX)              |   // Enable DCX if selected.
                _VAL2FLD(MSPI0_CTRL_PIOSCRAMBLE, pPIOTrans->bScrambling) |   // Set the scrambling if selected.
                _VAL2FLD(MSPI0_CTRL_TXRX, pPIOTrans->eDirection)         |   // Set transmit or receive operation.
                _VAL2FLD(MSPI0_CTRL_SENDI, pPIOTrans->bSendInstr)        |   // Enable sending the instruction.
                _VAL2FLD(MSPI0_CTRL_SENDA, pPIOTrans->bSendAddr)         |   // Enable sending the address.
                _VAL2FLD(MSPI0_CTRL_ENTURN, pPIOTrans->bTurnaround)      |   // Set the turn-around if needed.
                _VAL2FLD(MSPI0_CTRL_BIGENDIAN, pMSPIState->bBigEndian)   |   // Set the FIFO endian format.
                _VAL2FLD(MSPI0_CTRL_CONT, pPIOTrans->bContinue)          |   // Continuation transfer.
                _VAL2FLD(MSPI0_CTRL_ENWLAT, pPIOTrans->bEnWRLatency)     |   // Enable write latency counter.
                _VAL2FLD(MSPI0_CTRL_QUADCMD, pPIOTrans->bQuadCmd)        |   // Set the Quad Command if indicated.
                _VAL2FLD(MSPI0_CTRL_START, 1);                               // Start the transfer.

        }
        break;
        case AM_HAL_MSPI_TRANS_DMA:
        {
            am_hal_mspi_cq_dma_entry_t    *pDMAEntry = (am_hal_mspi_cq_dma_entry_t *)pCQEntry;
            am_hal_mspi_dma_transfer_t    *pDMATrans = (am_hal_mspi_dma_transfer_t *)pTransaction;

            //
            // Perform some sanity checks on the transaction.
            //
            if (pDMATrans->ui32TransferCount > AM_HAL_MSPI_MAX_TRANS_SIZE)
            {
              return AM_HAL_STATUS_OUT_OF_RANGE;
            }
            if (pMSPIState->block && (pDMATrans->ui32PauseCondition != 0))
            {
                // Paused operations not allowed in block mode
                return AM_HAL_STATUS_INVALID_OPERATION;
            }

            //
            // Command to set the DMACFG to disable DMA.
            // Need to make sure we disable DMA before we can reprogram
            //
            pDMAEntry->ui32DMACFG2Addr     = (uint32_t)&MSPIn(ui32Module)->DMACFG;
            pDMAEntry->ui32DMACFG2Val      = _VAL2FLD(MSPI0_DMACFG_DMAEN, 0);

            //
            // Command to set the DMATARGADDR
            //
            pDMAEntry->ui32DMATARGADDRAddr = (uint32_t)&MSPIn(ui32Module)->DMATARGADDR;
            pDMAEntry->ui32DMATARGADDRVal  = pDMATrans->ui32SRAMAddress;

            //
            // Command to set the DMADEVADDR
            //
            pDMAEntry->ui32DMADEVADDRAddr  = (uint32_t)&MSPIn(ui32Module)->DMADEVADDR;
            pDMAEntry->ui32DMADEVADDRVal   = pDMATrans->ui32DeviceAddress;

            //
            // Command to set the DMATOTALCOUNT
            //
            pDMAEntry->ui32DMATOTCOUNTAddr = (uint32_t)&MSPIn(ui32Module)->DMATOTCOUNT;
            pDMAEntry->ui32DMATOTCOUNTVal  = pDMATrans->ui32TransferCount;

            //
            // Command to set the DMACFG to start DMA.
            //
            pDMAEntry->ui32DMACFG1Addr     = (uint32_t)&MSPIn(ui32Module)->DMACFG;
            pDMAEntry->ui32DMACFG1Val      =
                _VAL2FLD(MSPI0_DMACFG_DMAPWROFF, 0)   |  // DMA Auto Power-off not supported!
                _VAL2FLD(MSPI0_DMACFG_DMAPRI, pDMATrans->ui8Priority)    |
                _VAL2FLD(MSPI0_DMACFG_DMADIR, pDMATrans->eDirection)     |
                _VAL2FLD(MSPI0_DMACFG_DMAEN, 3);
            pDMAEntry->ui32DummyAddr1 = (uint32_t)&MSPIn(ui32Module)->CQSETCLEAR;
            pDMAEntry->ui32DummyVal1 = 0;
            pDMAEntry->ui32DummyAddr2 = (uint32_t)&MSPIn(ui32Module)->CQSETCLEAR;
            pDMAEntry->ui32DummyVal2 = 0;
            pDMAEntry->ui32PAUSENAddr = pDMAEntry->ui32PAUSEN2Addr = (uint32_t)&MSPIn(ui32Module)->CQPAUSE;
            pDMAEntry->ui32PAUSEENVal = get_pause_val(pMSPIState, pDMATrans->ui32PauseCondition);
            pDMAEntry->ui32PAUSEEN2Val = AM_HAL_MSPI_PAUSE_DEFAULT;
            pDMAEntry->ui32SETCLRVal = pDMATrans->ui32StatusSetClr;
            pDMAEntry->ui32SETCLRAddr = (uint32_t)&MSPIn(ui32Module)->CQSETCLEAR;
        }
        break;
    }

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Writes data to the MSPI FIFO.
//!
//! @param ui32Module   - Selects the MSPI module to use (zero or one).
//! @param pui32Data    - Pointer to an array of the data to be written.
//! @param ui32NumBytes - Number of BYTES to copy into the FIFO.
//! @param ui32Timeout  - timeout microsecond delay loops
//!
//! This function copies data from the array \e pui32Data into the MSPI FIFO.
//!
//! @return HAL status of the operation.
//
//*****************************************************************************
static uint32_t
mspi_fifo_write(uint32_t ui32Module, uint32_t *pui32Data,
                uint32_t ui32NumBytes, uint32_t ui32Timeout)
{
    uint32_t ui32Index;
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

    //
    // Validate parameters
    //
    if ( ui32Module >= AM_REG_MSPI_NUM_MODULES )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    //
    // Loop over the words in the array until we have the correct number of
    // bytes.
    //
    for ( ui32Index = 0; (4 * ui32Index) < ui32NumBytes; ui32Index++ )
    {
        //
        // Write the word to the FIFO.
        //
        MSPIn(ui32Module)->TXFIFO = pui32Data[ui32Index];

        //
        // Wait for the word to go out if there is no room in the FIFO.
        //
        ui32Status = am_hal_flash_delay_status_check(ui32Timeout,
                                                     (uint32_t)&MSPIn(ui32Module)->TXENTRIES,
                                                     MSPI0_TXENTRIES_TXENTRIES_Msk,
                                                     _VAL2FLD(MSPI0_TXENTRIES_TXENTRIES, AM_HAL_MSPI_MAX_FIFO_SIZE),
                                                     false);
    }

    //
    // Return the status.
    //
    return ui32Status;
}

//*****************************************************************************
//
//! @brief Reads data from the MSPI FIFO.
//!
//! @param ui32Module - Selects the IOM module to use (zero or one).
//! @param pui32Data - Pointer to an array where the FIFO data will be copied.
//! @param ui32NumBytes - Number of bytes to copy into array.
//! @param ui32Timeout  - Timeout in microseconds
//!
//! This function copies data from the MSPI FIFO into the array \e pui32Data.
//!
//! @return HAL status of the operation.
//
//*****************************************************************************
static uint32_t
mspi_fifo_read(uint32_t ui32Module, uint32_t *pui32Data,
               uint32_t ui32NumBytes, uint32_t ui32Timeout)
{
    am_hal_mspi_buffer(4) sTempBuffer;
    uint32_t i, ui32NumWords, ui32Leftovers;
    uint32_t ui32Status;

    //
    // Validate parameters
    //
    if ( ui32Module >= AM_REG_MSPI_NUM_MODULES )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    //
    // Figure out how many whole words we're reading from the fifo, and how
    // many bytes will be left over when we're done.
    //
    ui32NumWords = ui32NumBytes / 4;
    ui32Leftovers = ui32NumBytes - (ui32NumWords * 4);

    //
    // Copy out as many full words as we can.
    //
    for ( i = 0; i < ui32NumWords; i++ )
    {
        //
        // Wait for additinal entries in the MSPI RX FIFO.
        //
        ui32Status = am_hal_flash_delay_status_check(ui32Timeout,
                                                     (uint32_t)&MSPIn(ui32Module)->RXENTRIES,
                                                     MSPI0_RXENTRIES_RXENTRIES_Msk,
                                                     _VAL2FLD(MSPI0_RXENTRIES_RXENTRIES, 0),
                                                     false);

        //
        // Check for timeout
        //
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return ui32Status;
        }

        //
        // Copy data out of the FIFO, one word at a time.
        //
        pui32Data[i] = MSPIn(ui32Module)->RXFIFO;
    }

    //
    // If there were leftovers, we'll copy them carefully. Pull the last word
    // from the fifo (there should only be one) into a temporary buffer. Also,
    // create an 8-bit pointer to help us copy the remaining bytes one at a
    // time.
    //
    // Note: If the data buffer we were given was truly a word pointer like the
    // definition requests, we wouldn't need to do this. It's possible to call
    // this function with a re-cast or packed pointer instead though. If that
    // happens, we want to be careful not to overwrite any data that might be
    // sitting just past the end of the destination array.
    //
    if ( ui32Leftovers )
    {
        //
        // Wait for additinal entries in the MSPI RX FIFO.
        //
        ui32Status = am_hal_flash_delay_status_check(ui32Timeout,
                                                     (uint32_t)&MSPIn(ui32Module)->RXENTRIES,
                                                     MSPI0_RXENTRIES_RXENTRIES_Msk,
                                                     _VAL2FLD(MSPI0_RXENTRIES_RXENTRIES, 0),
                                                     false);

        //
        // Check for timeout
        //
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return ui32Status;
        }

        //
        // Read the next word from the RX FIFO.
        //
        sTempBuffer.words[0] = MSPIn(ui32Module)->RXFIFO;
        uint8_t *pui8Data;
        pui8Data = (uint8_t *) (&pui32Data[i]);

        //
        // If we had leftover bytes, copy them out one byte at a time.
        //
        for ( int j = 0; j < ui32Leftovers; j++ )
        {
            pui8Data[j] = sTempBuffer.bytes[j];
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Initializes the MSPI Command Queue.
//!
//! @param ui32Module   - module number for the interface.
//! @param ui32Length   - length of the SRAM Command Queue buffer in words.
//! @param pTCB         - pointer to the SRAM to use for the Command Queue.
//!
//! This function initializes the global command queue structure.
//!
//! @return HAL status of the operation.
//
//*****************************************************************************
static uint32_t
mspi_cq_init(uint32_t ui32Module, uint32_t ui32Length,
             uint32_t *pTCB)
{
    am_hal_cmdq_cfg_t cqCfg;

    cqCfg.pCmdQBuf = pTCB;
    cqCfg.cmdQSize = ui32Length / 2;
    cqCfg.priority = AM_HAL_CMDQ_PRIO_HI;
    return am_hal_cmdq_init((am_hal_cmdq_if_e)(AM_HAL_CMDQ_IF_MSPI0 + ui32Module),
                            &cqCfg, &g_MSPIState[ui32Module].CQ.pCmdQHdl);
}

//*****************************************************************************
//
//! @brief Terminates the MSPI Command Queue.
//!
//! @param pHandle   - MSPI handlke.
//!
//! This function resets the global command queue structure.
//!
//! @return HAL status of the operation.
//
//*****************************************************************************
static uint32_t
mspi_cq_term(void *pHandle)
{
    am_hal_mspi_state_t           *pMSPIState = (am_hal_mspi_state_t *)pHandle;
    uint32_t                      ui32Module = pMSPIState->ui32Module;

    if (g_MSPIState[ui32Module].CQ.pCmdQHdl)
    {
        am_hal_cmdq_term(g_MSPIState[ui32Module].CQ.pCmdQHdl, true);
        g_MSPIState[ui32Module].CQ.pCmdQHdl = NULL;
    }

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Adds a transaction the MSPI Command Queue.
//!
//! @param pHandle       - handle for the interface.
//! @param pTransaction  - transaction to add to the CQ
//! @param eMode         - mode
//! @param pfnCallback   - pointer the callback function to be executed when
//!                       transaction is complete.
//! @param pCallbackCtxt - pointer to the state/context to pass to callback
//!                       function.
//! @param pCallbackCtxt - value passed to callback
//!
//! This function copies data from the IOM FIFO into the array \e pui32Data.
//! This is how input data from SPI or I2C transactions may be retrieved.
//!
//!
//! @return HAL status of the operation.
//
//*****************************************************************************
static uint32_t
mspi_cq_add_transaction(void *pHandle,
                        void *pTransaction,
                        am_hal_mspi_trans_e eMode,
                        am_hal_mspi_callback_t pfnCallback,
                        void *pCallbackCtxt)
{
    am_hal_mspi_state_t     *pMSPIState = (am_hal_mspi_state_t *)pHandle;
    am_hal_cmdq_entry_t     *pCQBlock;
    uint32_t                index;
    uint32_t                size = 1;
    am_hal_mspi_CQ_t        *pCQ = &pMSPIState->CQ;
    uint32_t                ui32Status = AM_HAL_STATUS_SUCCESS;

    //
    // Determine the transfer mode and set up accordingly.
    //
    switch(eMode)
    {
        case AM_HAL_MSPI_TRANS_PIO:
            size = sizeof(am_hal_mspi_cq_pio_entry_t);
            break;
        case AM_HAL_MSPI_TRANS_DMA:
            size = sizeof(am_hal_mspi_cq_dma_entry_t);
            break;
    }

    //
    // Check to see if there is enough room in the CQ
    //
    if (pMSPIState->ui32NumCQEntries == AM_HAL_MSPI_MAX_CQ_ENTRIES)
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }
    ui32Status = am_hal_cmdq_alloc_block(pCQ->pCmdQHdl, size / 8, &pCQBlock, &index);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    ui32Status = build_dma_cmdlist(pMSPIState, eMode, pCQBlock, pTransaction);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        am_hal_cmdq_release_block(pCQ->pCmdQHdl);
        return ui32Status;
    }

    //
    // Because we set AM_HAL_IOM_CQUPD_INT_FLAG, an interrupt will occur once
    // we reach this point in the Command Queue.  In the service routine, we'll
    // look for the appropriate callback.
    //
    // If ENDIDX has been reached, the CQ will pause here. Otherwise will
    // continue with the next CQ entry.
    //

    //
    // Store the callback function pointer.
    //
    pMSPIState->pfnCallback[index & (AM_HAL_MSPI_MAX_CQ_ENTRIES - 1)] = pfnCallback;
    pMSPIState->pCallbackCtxt[index & (AM_HAL_MSPI_MAX_CQ_ENTRIES - 1)] = pCallbackCtxt;

    //
    // Return the status.
    //
    return ui32Status;
}

//*****************************************************************************
//
//! @brief Enable the Command Queue operation.
//!
//! @param pHandle       - handle for the interface.
//!
//! @return HAL status of the operation.
//
//*****************************************************************************
static uint32_t
mspi_cq_enable(void *pHandle)
{
    am_hal_mspi_state_t *pMSPIState = (am_hal_mspi_state_t *)pHandle;

    //
    // Enable the Command Queue operation
    //
    return am_hal_cmdq_enable(pMSPIState->CQ.pCmdQHdl);
}

//*****************************************************************************
//
//! @brief Disable the Command Queue operation.
//!
//! @param pHandle       - handle for the interface.
//!
//! @return HAL status of the operation.
//
//*****************************************************************************
static uint32_t
mspi_cq_disable(void *pHandle)
{
    am_hal_mspi_state_t  *pMSPIState = (am_hal_mspi_state_t *)pHandle;

    //
    // Disable the Command Queue operation
    //
    return am_hal_cmdq_disable(pMSPIState->CQ.pCmdQHdl);
}

static uint32_t
mspi_cq_pause(am_hal_mspi_state_t *pMSPIState)
{
    uint32_t status = AM_HAL_STATUS_SUCCESS;
    uint32_t ui32usMaxDelay = AM_HAL_MSPI_MAX_PAUSE_DELAY;

    MSPIn(pMSPIState->ui32Module)->CQSETCLEAR = AM_HAL_MSPI_SC_PAUSE_CQ;
    // It is possible that CQ is disabled once the last transaction is processed
    while ( MSPIn(pMSPIState->ui32Module)->CQCFG_b.CQEN )
    {
        // Need to make sure we're paused at a designated pause point
        if ( MSPIn(pMSPIState->ui32Module)->CQSTAT_b.CQPAUSED && (MSPIn(pMSPIState->ui32Module)->CQPAUSE & AM_HAL_MSPI_PAUSE_FLAG_CQ))
        {
            break;
        }
        if ( ui32usMaxDelay-- )
        {
            //
            // Call the BOOTROM cycle function to delay for about 1 microsecond.
            //
            am_hal_flash_delay( FLASH_CYCLES_US(1) );
        }
        else
        {
            return AM_HAL_STATUS_TIMEOUT;
        }
    }
    if (status == AM_HAL_STATUS_SUCCESS)
    {
        // Now that CQ is guaranteed to not progress further - we need to still wait in case the current CQ entry
        // resulted in a DMA state....need to make sure we finish the current DMA
        status = am_hal_flash_delay_status_check(AM_HAL_MSPI_MAX_PAUSE_DELAY,
                                             (uint32_t)&MSPIn(pMSPIState->ui32Module)->DMASTAT,
                                             MSPI0_DMASTAT_DMATIP_Msk,
                                             _VAL2FLD(MSPI0_DMASTAT_DMATIP, 0),
                                             true);

    }
    return status;
}

static void
program_dma(void *pHandle)
{
    am_hal_mspi_state_t         *pMSPIState = (am_hal_mspi_state_t *)pHandle;
    uint32_t                    ui32Module = pMSPIState->ui32Module;
    uint32_t                    index = (pMSPIState->ui32LastHPIdxProcessed + 1) % pMSPIState->ui32MaxHPTransactions;
    am_hal_mspi_dma_entry_t     *pDMAEntry = &pMSPIState->pHPTransactions[index];

    // Need to make sure we disable DMA before we can reprogram
    MSPIn(ui32Module)->DMACFG = _VAL2FLD(MSPI0_DMACFG_DMAEN, 0);
    //
    // set the DMATARGADDR
    //
    MSPIn(ui32Module)->DMATARGADDR = pDMAEntry->ui32DMATARGADDRVal;

    //
    // set the DMADEVADDR
    //
    MSPIn(ui32Module)->DMADEVADDR = pDMAEntry->ui32DMADEVADDRVal;

    //
    // set the DMATOTALCOUNT
    //
    MSPIn(ui32Module)->DMATOTCOUNT = pDMAEntry->ui32DMATOTCOUNTVal;

    //
    // set the DMACFG to start DMA.
    //
    MSPIn(ui32Module)->DMACFG = pDMAEntry->ui32DMACFG1Val;
}

//*****************************************************************************
//! @brief schedule a hi priority transaction
//!
//! @param pMSPIState
//! @param numTrans
//! @return uint32_t
//*****************************************************************************
static uint32_t
sched_hiprio(am_hal_mspi_state_t *pMSPIState, uint32_t numTrans)
{
    uint32_t ui32NumPend;
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    //
    // Start a critical section.
    //
    AM_CRITICAL_BEGIN

    ui32NumPend = pMSPIState->ui32NumHPEntries;
    pMSPIState->ui32NumHPEntries += numTrans;

    //
    // End the critical section.
    //
    AM_CRITICAL_END


    if (0 == ui32NumPend)
    {
        // Force CQ to Pause
        ui32Status = mspi_cq_pause(pMSPIState);
        if (ui32Status != AM_HAL_STATUS_SUCCESS)
        {
            return ui32Status;
        }
        pMSPIState->ui32TxnInt = 0;
        // Clear DMACMP interrupt
        MSPIn(pMSPIState->ui32Module)->INTCLR = AM_HAL_MSPI_INT_DMACMP;
        // Enable DMACMP interrupt
        MSPIn(pMSPIState->ui32Module)->INTEN |= AM_HAL_MSPI_INT_DMACMP;
        pMSPIState->bHP = true;
        //
        // Program the DMA
        //
        program_dma(pMSPIState);
    }
    return ui32Status;
}

//*****************************************************************************
//! @brief add a high priority transaction
//!
//! @param pHandle
//! @param pDMATrans
//! @param pfnCallback
//! @param pCallbackCtxt
//! @return uint32_t
//*****************************************************************************
static uint32_t
mspi_add_hp_transaction(void *pHandle,
                        am_hal_mspi_dma_transfer_t *pDMATrans,
                        am_hal_mspi_callback_t pfnCallback,
                        void *pCallbackCtxt)
{
    am_hal_mspi_state_t         *pMSPIState = (am_hal_mspi_state_t *)pHandle;
    am_hal_mspi_dma_entry_t     *pDMAEntry;

    uint32_t  index = pMSPIState->ui32NextHPIdx % pMSPIState->ui32MaxHPTransactions;

    //
    // Check to see if there is enough room in the queue
    //
    if ( pMSPIState->ui32NumHPEntries == pMSPIState->ui32MaxHPTransactions )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    pDMAEntry = &pMSPIState->pHPTransactions[index];
    pDMAEntry->ui32DMATARGADDRVal = pDMATrans->ui32SRAMAddress;
    pDMAEntry->ui32DMADEVADDRVal   = pDMATrans->ui32DeviceAddress;
    pDMAEntry->ui32DMATOTCOUNTVal  = pDMATrans->ui32TransferCount;
    pDMAEntry->ui32DMACFG1Val      =
        _VAL2FLD(MSPI0_DMACFG_DMAPWROFF, 0)   |  // DMA Auto Power-off not supported!
        _VAL2FLD(MSPI0_DMACFG_DMAPRI, pDMATrans->ui8Priority)    |
        _VAL2FLD(MSPI0_DMACFG_DMADIR, pDMATrans->eDirection)     |
        _VAL2FLD(MSPI0_DMACFG_DMAEN, 3);
    pDMAEntry->pfnCallback = pfnCallback;
    pDMAEntry->pCallbackCtxt = pCallbackCtxt;

    pMSPIState->ui32NextHPIdx++;
    return AM_HAL_STATUS_SUCCESS;
} // am_hal_mspi_DmaAddTransaction()

//*****************************************************************************
//
//! @brief Configure the device config, seperate I/O, mixed mode, and internal
//!        PADs based on the virtual device configuration passed in.
//!
//! @param pHandle       - handle for the interface.
//! @param eMSPIDevice  - external device configuration for MSPI
//!
//! @return HAL status of the operation.
//
//*****************************************************************************
static uint32_t
mspi_device_configure(void *pHandle, uint32_t ui32MSPIDevice)
{
    am_hal_mspi_state_t           *pMSPIState = (am_hal_mspi_state_t *)pHandle;
    uint32_t                      ui32Module = pMSPIState->ui32Module;

    switch ( ui32MSPIDevice )
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
            MSPIn(ui32Module)->CFG_b.DEVCFG     = MSPI0_CFG_DEVCFG_SERIAL0;
            MSPIn(ui32Module)->CFG_b.SEPIO      = 1;
            MSPIn(ui32Module)->FLASH_b.XIPMIXED = MSPI0_FLASH_XIPMIXED_NORMAL;
            MSPIn(ui32Module)->PADCFG           = 0;
            MSPIn(ui32Module)->PADOUTEN         = MSPI0_PADOUTEN_OUTEN_SERIAL0;
            break;
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
            MSPIn(ui32Module)->CFG_b.DEVCFG     = MSPI0_CFG_DEVCFG_SERIAL1;
            MSPIn(ui32Module)->CFG_b.SEPIO      = 1;
            MSPIn(ui32Module)->FLASH_b.XIPMIXED = MSPI0_FLASH_XIPMIXED_NORMAL;
            MSPIn(ui32Module)->PADCFG           = 0;
            MSPIn(ui32Module)->PADOUTEN         = MSPI0_PADOUTEN_OUTEN_SERIAL1;
            break;
        case AM_HAL_MSPI_FLASH_DUAL_CE0:
            MSPIn(ui32Module)->CFG_b.DEVCFG     = MSPI0_CFG_DEVCFG_DUAL0;
            MSPIn(ui32Module)->CFG_b.SEPIO      = 0;
            MSPIn(ui32Module)->FLASH_b.XIPMIXED = MSPI0_FLASH_XIPMIXED_NORMAL;
            MSPIn(ui32Module)->PADCFG = 0;
            MSPIn(ui32Module)->PADOUTEN = MSPI0_PADOUTEN_OUTEN_SERIAL0;
            break;
        case AM_HAL_MSPI_FLASH_DUAL_CE1:
            MSPIn(ui32Module)->CFG_b.DEVCFG     = MSPI0_CFG_DEVCFG_DUAL1;
            MSPIn(ui32Module)->CFG_b.SEPIO      = 0;
            MSPIn(ui32Module)->FLASH_b.XIPMIXED = MSPI0_FLASH_XIPMIXED_NORMAL;
            MSPIn(ui32Module)->PADCFG = 0;
            MSPIn(ui32Module)->PADOUTEN = MSPI0_PADOUTEN_OUTEN_SERIAL1;
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
            MSPIn(ui32Module)->CFG_b.DEVCFG     = MSPI0_CFG_DEVCFG_QUAD0;
            MSPIn(ui32Module)->CFG_b.SEPIO      = 0;
            MSPIn(ui32Module)->FLASH_b.XIPMIXED = MSPI0_FLASH_XIPMIXED_NORMAL;
            MSPIn(ui32Module)->PADCFG = 0;
            MSPIn(ui32Module)->PADOUTEN = MSPI0_PADOUTEN_OUTEN_QUAD0;
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
            MSPIn(ui32Module)->CFG_b.DEVCFG     = MSPI0_CFG_DEVCFG_QUAD1;
            MSPIn(ui32Module)->CFG_b.SEPIO      = 0;
            MSPIn(ui32Module)->FLASH_b.XIPMIXED = MSPI0_FLASH_XIPMIXED_NORMAL;
            MSPIn(ui32Module)->PADCFG = 0;
            MSPIn(ui32Module)->PADOUTEN = MSPI0_PADOUTEN_OUTEN_QUAD1;
            break;
        case AM_HAL_MSPI_FLASH_OCTAL_CE0:
            MSPIn(ui32Module)->CFG_b.DEVCFG     = MSPI0_CFG_DEVCFG_OCTAL0;
            MSPIn(ui32Module)->CFG_b.SEPIO      = 0;
            MSPIn(ui32Module)->FLASH_b.XIPMIXED = MSPI0_FLASH_XIPMIXED_NORMAL;
            MSPIn(ui32Module)->PADCFG           = 0;
            MSPIn(ui32Module)->PADOUTEN         = MSPI0_PADOUTEN_OUTEN_OCTAL;
            break;
        case AM_HAL_MSPI_FLASH_OCTAL_CE1:
            MSPIn(ui32Module)->CFG_b.DEVCFG     = MSPI0_CFG_DEVCFG_OCTAL1;
            MSPIn(ui32Module)->CFG_b.SEPIO      = 0;
            MSPIn(ui32Module)->FLASH_b.XIPMIXED = MSPI0_FLASH_XIPMIXED_NORMAL;
            MSPIn(ui32Module)->PADCFG           = 0;
            MSPIn(ui32Module)->PADOUTEN         = MSPI0_PADOUTEN_OUTEN_OCTAL;
            break;
        case AM_HAL_MSPI_FLASH_QUADPAIRED:
        case AM_HAL_MSPI_FLASH_QUADPAIRED_SERIAL:
            return AM_HAL_STATUS_INVALID_ARG;
            //break;  // This break results in 'statement unreachable' warning
        case AM_HAL_MSPI_FLASH_DUAL_CE0_1_1_2:
            MSPIn(ui32Module)->CFG_b.DEVCFG     = MSPI0_CFG_DEVCFG_SERIAL0;
            MSPIn(ui32Module)->CFG_b.SEPIO      = 0;
            MSPIn(ui32Module)->FLASH_b.XIPMIXED = MSPI0_FLASH_XIPMIXED_D2;
            MSPIn(ui32Module)->PADCFG           = 0;
            MSPIn(ui32Module)->PADOUTEN         = MSPI0_PADOUTEN_OUTEN_SERIAL0;
            break;
        case AM_HAL_MSPI_FLASH_DUAL_CE1_1_1_2:
            MSPIn(ui32Module)->CFG_b.DEVCFG     = MSPI0_CFG_DEVCFG_SERIAL1;
            MSPIn(ui32Module)->CFG_b.SEPIO      = 0;
            MSPIn(ui32Module)->FLASH_b.XIPMIXED = MSPI0_FLASH_XIPMIXED_D2;
            MSPIn(ui32Module)->PADCFG           = 0;
            MSPIn(ui32Module)->PADOUTEN         = MSPI0_PADOUTEN_OUTEN_SERIAL1;
            break;
        case AM_HAL_MSPI_FLASH_DUAL_CE0_1_2_2:
            MSPIn(ui32Module)->CFG_b.DEVCFG     = MSPI0_CFG_DEVCFG_SERIAL0;
            MSPIn(ui32Module)->CFG_b.SEPIO      = 0;
            MSPIn(ui32Module)->FLASH_b.XIPMIXED = MSPI0_FLASH_XIPMIXED_AD2;
            MSPIn(ui32Module)->PADCFG           = 0;
            MSPIn(ui32Module)->PADOUTEN         = MSPI0_PADOUTEN_OUTEN_SERIAL0;
            break;
        case AM_HAL_MSPI_FLASH_DUAL_CE1_1_2_2:
            MSPIn(ui32Module)->CFG_b.DEVCFG     = MSPI0_CFG_DEVCFG_SERIAL1;
            MSPIn(ui32Module)->CFG_b.SEPIO      = 0;
            MSPIn(ui32Module)->FLASH_b.XIPMIXED = MSPI0_FLASH_XIPMIXED_AD2;
            MSPIn(ui32Module)->PADCFG           = 0;
            MSPIn(ui32Module)->PADOUTEN         = MSPI0_PADOUTEN_OUTEN_SERIAL1;
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE0_1_1_4:
            MSPIn(ui32Module)->CFG_b.DEVCFG     = MSPI0_CFG_DEVCFG_SERIAL0;
            MSPIn(ui32Module)->CFG_b.SEPIO      = 0;
            MSPIn(ui32Module)->FLASH_b.XIPMIXED = MSPI0_FLASH_XIPMIXED_D4;
            MSPIn(ui32Module)->PADCFG           = 0;
            MSPIn(ui32Module)->PADOUTEN         = MSPI0_PADOUTEN_OUTEN_QUAD0;
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE1_1_1_4:
            MSPIn(ui32Module)->CFG_b.DEVCFG     = MSPI0_CFG_DEVCFG_SERIAL1;
            MSPIn(ui32Module)->CFG_b.SEPIO      = 0;
            MSPIn(ui32Module)->FLASH_b.XIPMIXED = MSPI0_FLASH_XIPMIXED_D4;
            MSPIn(ui32Module)->PADCFG           = 0;
            MSPIn(ui32Module)->PADOUTEN         = MSPI0_PADOUTEN_OUTEN_QUAD1;
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4:
            MSPIn(ui32Module)->CFG_b.DEVCFG     = MSPI0_CFG_DEVCFG_SERIAL0;
            MSPIn(ui32Module)->CFG_b.SEPIO      = 0;
            MSPIn(ui32Module)->FLASH_b.XIPMIXED = MSPI0_FLASH_XIPMIXED_AD4;
            MSPIn(ui32Module)->PADCFG           = 0;
            MSPIn(ui32Module)->PADOUTEN         = MSPI0_PADOUTEN_OUTEN_QUAD0;
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE1_1_4_4:
            MSPIn(ui32Module)->CFG_b.DEVCFG     = MSPI0_CFG_DEVCFG_SERIAL1;
            MSPIn(ui32Module)->CFG_b.SEPIO      = 0;
            MSPIn(ui32Module)->FLASH_b.XIPMIXED = MSPI0_FLASH_XIPMIXED_AD4;
            MSPIn(ui32Module)->PADCFG           = 0;
            MSPIn(ui32Module)->PADOUTEN         = MSPI0_PADOUTEN_OUTEN_QUAD1;
            break;
        case AM_HAL_MSPI_FLASH_SERIAL_CE0_3WIRE:
            MSPIn(ui32Module)->CFG_b.DEVCFG     = MSPI0_CFG_DEVCFG_SERIAL0;
            MSPIn(ui32Module)->CFG_b.SEPIO      = 0;
            MSPIn(ui32Module)->FLASH_b.XIPMIXED = MSPI0_FLASH_XIPMIXED_NORMAL;
            MSPIn(ui32Module)->PADCFG           = 0;
            // Enable both D0 and D1 - as D1 might be getting used for DCX
            MSPIn(ui32Module)->PADOUTEN         = MSPI0_PADOUTEN_OUTEN_SERIAL0;
            break;
        case AM_HAL_MSPI_FLASH_SERIAL_CE1_3WIRE:
            MSPIn(ui32Module)->CFG_b.DEVCFG     = MSPI0_CFG_DEVCFG_SERIAL1;
            MSPIn(ui32Module)->CFG_b.SEPIO      = 0;
            MSPIn(ui32Module)->FLASH_b.XIPMIXED = MSPI0_FLASH_XIPMIXED_NORMAL;
            MSPIn(ui32Module)->PADCFG = 0;
            // Enable both D0 and D1 - as D1 might be getting used for DCX
            MSPIn(ui32Module)->PADOUTEN         = MSPI0_PADOUTEN_OUTEN_SERIAL1;
            break;
        default:
            break;
    }

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

static void mspi_dummy_callback(void *pCallbackCtxt, uint32_t status)
{
    // Dummy - Do nothing
}

static void mspi_seq_loopback(void *pCallbackCtxt, uint32_t status)
{
    // Reset the state to allow serving callbacks for next set
    am_hal_mspi_state_t *pMSPIState = (am_hal_mspi_state_t *)pCallbackCtxt;
    pMSPIState->ui32NumCQEntries = pMSPIState->ui32NumTransactions + 1;
    pMSPIState->ui32LastIdxProcessed = 0;
    pMSPIState->bRestart = true;
    // Now resume the CQ - to finish loopback
    // Resume the CQ
    MSPIn(pMSPIState->ui32Module)->CQSETCLEAR = AM_HAL_MSPI_SC_UNPAUSE_SEQLOOP;
}

//*****************************************************************************
//
// External Functions.
//
//*****************************************************************************

//
// MSPI initialization function
//
uint32_t
am_hal_mspi_initialize(uint32_t ui32Module, void **ppHandle)
{
    // Compile time check to ensure ENTRY_SIZE macros are defined correctly
    // incorrect definition will cause divide by 0 error at build time
    am_ct_assert((sizeof(am_hal_mspi_cq_dma_entry_t) + 8) == AM_HAL_MSPI_CQ_ENTRY_SIZE);
    am_ct_assert(sizeof(am_hal_mspi_dma_entry_t) == AM_HAL_MSPI_HIPRIO_ENTRY_SIZE);

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check that the request module is in range.
    //
    if (ui32Module >= AM_REG_MSPI_NUM_MODULES )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    //
    // Check for valid arguements.
    //
    if (!ppHandle)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Check if the handle is unallocated.
    //
    if (g_MSPIState[ui32Module].prefix.s.bInit)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Initialize the handle.
    //
    g_MSPIState[ui32Module].prefix.s.bInit = true;
    g_MSPIState[ui32Module].prefix.s.magic = AM_HAL_MAGIC_MSPI;
    g_MSPIState[ui32Module].ui32Module = ui32Module;

    //
    // Return the handle.
    //
    *ppHandle = (void *)&g_MSPIState[ui32Module];

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//
// MSPI Disable function
//
uint32_t
am_hal_mspi_disable(void *pHandle)
{
    am_hal_mspi_state_t           *pMSPIState = (am_hal_mspi_state_t *)pHandle;
    uint32_t                       ui32Status;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if (!AM_HAL_MSPI_CHK_HANDLE(pHandle))
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    if (!pMSPIState->prefix.s.bEnable)
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    if (pMSPIState->pTCB)
    {
      //
      // Disable the Command Queue.
      //
      ui32Status = mspi_cq_disable(pHandle);
      if (AM_HAL_STATUS_SUCCESS != ui32Status)
      {
        return ui32Status;
      }

      //
      // Reset the Command Queue.
      //
      mspi_cq_term(pHandle);
    }

    pMSPIState->prefix.s.bEnable = false;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}


//
// MSPI Deinitialize function
//
uint32_t
am_hal_mspi_deinitialize(void *pHandle)
{
    am_hal_mspi_state_t           *pMSPIState = (am_hal_mspi_state_t *)pHandle;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if (!AM_HAL_MSPI_CHK_HANDLE(pHandle))
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    if (pMSPIState->prefix.s.bEnable)
    {
        am_hal_mspi_disable(pHandle);
    }

    //
    // Reset the handle.
    //
    pMSPIState->prefix.s.bInit = false;
    pMSPIState->ui32Module = 0;
    pMSPIState->eDeviceConfig = (am_hal_mspi_device_e)0;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//
// MSPI device configuration function
//
uint32_t
am_hal_mspi_device_configure(void *pHandle,
                             am_hal_mspi_dev_config_t *pConfig)
{
    am_hal_mspi_state_t           *pMSPIState = (am_hal_mspi_state_t *)pHandle;
    uint32_t                      ui32Module;
    uint32_t                      ui32Config = 0;

    ui32Module = pMSPIState->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if (!AM_HAL_MSPI_CHK_HANDLE(pHandle))
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    //
    // Configure not allowed in Enabled state
    //
    if (pMSPIState->prefix.s.bEnable)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    //
    // Check if Device Configuration is allowed.  Only Module #1 supports Octal/Paired.
    //
    if ((pConfig->eDeviceConfig > AM_HAL_MSPI_FLASH_QUAD_CE1) &&
        (pConfig->eDeviceConfig < AM_HAL_MSPI_FLASH_DUAL_CE0_1_1_2) &&
        (ui32Module != AM_HAL_MSPI_OCTAL_MODULE))
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    //
    // Check for DMA to/from DTCM.
    //
    if ( ((uint32_t)pConfig->pTCB >= AM_HAL_FLASH_DTCM_START) &&
      ((uint32_t)pConfig->pTCB <= AM_HAL_FLASH_DTCM_END) )
    {
      return AM_HAL_STATUS_OUT_OF_RANGE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Set the clock polarity and phase based on SPI mode.
    //
    switch(pConfig->eSpiMode)
    {
        case AM_HAL_MSPI_SPI_MODE_0:                  // CPOL = 0; CPHA = 0
            ui32Config |= _VAL2FLD(MSPI0_CFG_CPOL, 0)    |
                          _VAL2FLD(MSPI0_CFG_CPHA, 0);
            break;
        case AM_HAL_MSPI_SPI_MODE_2:                  // CPOL = 1; CPHA = 0
            ui32Config |= _VAL2FLD(MSPI0_CFG_CPOL, 1)    |
                          _VAL2FLD(MSPI0_CFG_CPHA, 0);
            break;
        case AM_HAL_MSPI_SPI_MODE_1:                  // CPOL = 0; CPHA = 1
            ui32Config |= _VAL2FLD(MSPI0_CFG_CPOL, 0)    |
                          _VAL2FLD(MSPI0_CFG_CPHA, 1);
            break;
        case AM_HAL_MSPI_SPI_MODE_3:                  // CPOL = 1; CPHA = 1
            ui32Config |= _VAL2FLD(MSPI0_CFG_CPOL, 1)    |
                          _VAL2FLD(MSPI0_CFG_CPHA, 1);
            break;
    }

    //
    // Set the number of turn-around cycles.
    //
    ui32Config |= _VAL2FLD(MSPI0_CFG_TURNAROUND, pConfig->ui8TurnAround);

    //
    // Set the address configuration.
    //
    ui32Config |= _VAL2FLD(MSPI0_CFG_ASIZE, pConfig->eAddrCfg);

    //
    // Set the instruction configuration.
    //
    ui32Config |= _VAL2FLD(MSPI0_CFG_ISIZE, pConfig->eInstrCfg);

    //
    // Set the write latency value.
    //
    ui32Config |= _VAL2FLD(MSPI0_CFG_WRITELATENCY, pConfig->ui8WriteLatency);

    //
    // Set the configuration in the MSPI peripheral.
    //
    MSPIn(ui32Module)->CFG = ui32Config;

    //
    // Set the clock divisor to get the desired MSPI clock frequency.
    //
    MSPIn(ui32Module)->MSPICFG_b.CLKDIV = pConfig->eClockFreq;

    //
    // Adjust the clock edge configuration depending upon the clock frequency.
    //
    if ( pConfig->eClockFreq == AM_HAL_MSPI_CLK_48MHZ )
    {
        MSPIn(ui32Module)->MSPICFG_b.TXNEG = 1;
        MSPIn(ui32Module)->MSPICFG_b.RXNEG = 0;
        MSPIn(ui32Module)->MSPICFG_b.RXCAP = 1;
        MSPIn(ui32Module)->MSPIDDR = 0;
    }
    else
    {
        MSPIn(ui32Module)->MSPICFG_b.TXNEG = 0;
        MSPIn(ui32Module)->MSPICFG_b.RXNEG = 0;
        MSPIn(ui32Module)->MSPICFG_b.RXCAP = 1;
        MSPIn(ui32Module)->MSPIDDR  = 0;
    }

    //
    // Set the APBCLK for continuous operation.
    //
    MSPIn(ui32Module)->MSPICFG_b.APBCLK = 0;

    //
    // Reset the register storage for next write.
    //
    ui32Config = 0;

    //
    // Set whether to send an instruction.
    //
    if ( pConfig->bSendInstr )
    {
        ui32Config |= _VAL2FLD(MSPI0_FLASH_XIPSENDI, 1);
    }

    //
    // Set whether to send an address.
    //
    if ( pConfig->bSendAddr )
    {
        ui32Config |= _VAL2FLD(MSPI0_FLASH_XIPSENDA, 1);
    }

    //
    // Set whether to enable the TX to RX turnaround.
    //
    if ( pConfig->bTurnaround )
    {
        ui32Config |= _VAL2FLD(MSPI0_FLASH_XIPENTURN, 1);
    }

    //
    // Set to Little Endian mode by default.
    //
    ui32Config |= _VAL2FLD(MSPI0_FLASH_XIPBIGENDIAN, pMSPIState->bBigEndian);

    //
    // Set the XIP ACK value to default to 1's during latency period.
    //
    ui32Config |= _VAL2FLD(MSPI0_FLASH_XIPACK, MSPI0_FLASH_XIPACK_TERMINATE);


    //
    // Set the XIPENWLAT mode.
    //
    ui32Config |= _VAL2FLD(MSPI0_FLASH_XIPENWLAT, pConfig->bEnWriteLatency);

    //
    // Set the configuration in the MSPI peripheral.
    //
    MSPIn(ui32Module)->FLASH = ui32Config;

    //
    // Reset the register storage for next write.
    //
    ui32Config = 0;

    //
    // Set the read instruction.
    //
    ui32Config |= _VAL2FLD(MSPI0_XIPINSTR_READINSTR, pConfig->ui8ReadInstr);

    //
    // Set the write instruction.
    //
    ui32Config |= _VAL2FLD(MSPI0_XIPINSTR_WRITEINSTR, pConfig->ui8WriteInstr);

    //
    // Set the configuration in the MSPI peripheral.
    //
    MSPIn(ui32Module)->XIPINSTR = ui32Config;

    //
    // Reset the register storage for next write.
    //
    ui32Config = 0;

    //
    // Set up the DMA Boundary configuration.
    //
    ui32Config |= _VAL2FLD(MSPI0_DMABOUNDARY_DMABOUND, pConfig->eDMABoundary);

    //
    // Set the DMA time limit.
    //
    ui32Config |= _VAL2FLD(MSPI0_DMABOUNDARY_DMATIMELIMIT, pConfig->ui16DMATimeLimit);

    //
    // Set the DMA
    //
    // Set the configuration in the MSPI peripheral.
    //
    MSPIn(ui32Module)->DMABOUNDARY = ui32Config;

    g_MSPIState[ui32Module].pTCB = pConfig->pTCB;
    g_MSPIState[ui32Module].ui32TCBSize = pConfig->ui32TCBSize;

    if (pConfig->pTCB)
    {
      // set the DMABCOUNT
      MSPIn(ui32Module)->DMABCOUNT = AM_HAL_MSPI_DEFAULT_BURST_COUNT;

      if ( pConfig->eClockFreq == AM_HAL_MSPI_CLK_48MHZ )
      {

        // set the DMATHRESHs asymmetrically for 48MHz.
        MSPIn(ui32Module)->DMATHRESH_b.DMATXTHRESH = AM_HAL_MSPI_MAX_FIFO_SIZE - AM_HAL_MSPI_HIGH_SPEED_OFFSET;
        MSPIn(ui32Module)->DMATHRESH_b.DMARXTHRESH = AM_HAL_MSPI_HIGH_SPEED_OFFSET;
      }
      else
      {
        // set the DMATHRESHs symmetrically for all other clock speeds.
        MSPIn(ui32Module)->DMATHRESH_b.DMATXTHRESH = (AM_HAL_MSPI_DEFAULT_BURST_COUNT >> 2);
        MSPIn(ui32Module)->DMATHRESH_b.DMARXTHRESH = (AM_HAL_MSPI_DEFAULT_BURST_COUNT >> 2);
      }
      // Worst case minimum CQ entries that can be accomodated in provided buffer
      // Need to account for the wrap
      g_MSPIState[ui32Module].ui32MaxPending = ((pConfig->ui32TCBSize - 8) * 4 / AM_HAL_MSPI_CQ_ENTRY_SIZE);
      if (g_MSPIState[ui32Module].ui32MaxPending > AM_HAL_MSPI_MAX_CQ_ENTRIES)
      {
        g_MSPIState[ui32Module].ui32MaxPending = AM_HAL_MSPI_MAX_CQ_ENTRIES;
      }
    }
    //
    // Reset the register storage for next write.
    //
    ui32Config = 0;

    //
    // Set the scrambling start and end addresses aligned to 64K region.
    //
    MSPIn(ui32Module)->SCRAMBLING =
        _VAL2FLD(MSPI0_SCRAMBLING_SCRSTART, pConfig->scramblingStartAddr >> 16) |
        _VAL2FLD(MSPI0_SCRAMBLING_SCREND, pConfig->scramblingEndAddr >> 16);

    //
    // Set the selected IOM to disable.
    //
    MSPIn(ui32Module)->MSPICFG_b.IOMSEL = AM_HAL_MSPI_LINK_NONE;

    {
      //
      // Configure the MSPI for a specific device configuration.
      // This function sets the following registers/fields:
      //    CFG.DEVCFG
      //    CFG.SEPIO
      //    FLASH.XIPMIXED
      //    PADCFG
      //    PADOUTEN
      //
      mspi_device_configure(pHandle, pConfig->eDeviceConfig);
    }

    //
    // Set the default endianess for the FIFO.
    //
    pMSPIState->bBigEndian = false;

    //
    // Store the clock frequency for later SW workarounds.
    //
    pMSPIState->eClockFreq = pConfig->eClockFreq;

    //
    // Set the default maximum delay timeout value.
    //
    pMSPIState->waitTimeout = 10000;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//
// MSPI device configuration function
//
uint32_t
am_hal_mspi_enable(void *pHandle)
{
    am_hal_mspi_state_t           *pMSPIState = (am_hal_mspi_state_t *)pHandle;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if (!AM_HAL_MSPI_CHK_HANDLE(pHandle))
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION


    if (pMSPIState->pTCB)
    {
        pMSPIState->ui32LastIdxProcessed = 0;
        pMSPIState->ui32NumCQEntries = 0;


        //
        // Initialize the Command Queue service with memory supplied by the application.
        //
        mspi_cq_init(pMSPIState->ui32Module, pMSPIState->ui32TCBSize, pMSPIState->pTCB);
        // Initialize Flags used to force CQ Pause
        MSPIn(pMSPIState->ui32Module)->CQSETCLEAR = AM_HAL_MSPI_SC_UNPAUSE_CQ | AM_HAL_MSPI_SC_PAUSE_SEQLOOP;
        pMSPIState->pHPTransactions = NULL;
        pMSPIState->bHP = false;
        pMSPIState->ui32NumHPPendingEntries = 0;
        pMSPIState->block = 0;
        pMSPIState->ui32NumHPEntries = 0;
        pMSPIState->eSeq = AM_HAL_MSPI_SEQ_NONE;
        pMSPIState->ui32NumTransactions = 0;
        pMSPIState->bAutonomous = true;
        pMSPIState->ui32NumUnSolicited = 0;
    }

    pMSPIState->prefix.s.bEnable = true;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//
// MSPI device specific control function.
//
uint32_t am_hal_mspi_control(void *pHandle,
                             am_hal_mspi_request_e eRequest,
                             void *pConfig)
{
    am_hal_mspi_state_t     *pMSPIState = (am_hal_mspi_state_t *)pHandle;
    uint32_t                ui32Module;
    uint32_t                ui32Status = AM_HAL_STATUS_SUCCESS;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_MSPI_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    //
    // Validate the parameters
    //
    if (eRequest > AM_HAL_MSPI_REQ_MAX)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION
    ui32Module = pMSPIState->ui32Module;

    switch(eRequest)
    {
        case AM_HAL_MSPI_REQ_APBCLK:
#ifndef AM_HAL_DISABLE_API_VALIDATION
            if (!pConfig)
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
#endif // AM_HAL_DISABLE_API_VALIDATION

            //
            // Enable/Disable APBCLK.
            //
            MSPIn(ui32Module)->MSPICFG_b.APBCLK = *((uint32_t *)pConfig);
            break;

        case AM_HAL_MSPI_REQ_FLAG_SETCLR:
#ifndef AM_HAL_DISABLE_API_VALIDATION
            if (!pConfig)
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
            if (*((uint32_t *)pConfig) & AM_HAL_MSPI_SC_RESV_MASK)
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
#endif // AM_HAL_DISABLE_API_VALIDATION

            MSPIn(ui32Module)->CQSETCLEAR = *((uint32_t *)pConfig);
            break;

        case AM_HAL_MSPI_REQ_LINK_IOM:
#ifndef AM_HAL_DISABLE_API_VALIDATION
            if (!pConfig)
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
            if (( *((uint32_t *)pConfig) >= AM_REG_IOM_NUM_MODULES ) && ( *((uint32_t *)pConfig) != AM_HAL_MSPI_LINK_NONE ))
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
#endif // AM_HAL_DISABLE_API_VALIDATION

            //
            // Set the Linked IOM
            //
            MSPIn(ui32Module)->MSPICFG_b.IOMSEL = *((uint32_t *)pConfig);
            break;

        case AM_HAL_MSPI_REQ_LINK_MSPI:
#ifndef AM_HAL_DISABLE_API_VALIDATION
            if (!pConfig)
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
            if (( *((uint32_t *)pConfig) >= AM_REG_MSPI_NUM_MODULES ) && ( *((uint32_t *)pConfig) != AM_HAL_MSPI_LINK_NONE ))
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
#endif // AM_HAL_DISABLE_API_VALIDATION

            //
            // Set the Linked MSPI
            //
            MSPIn(ui32Module)->MSPICFG_b.IOMSEL = *((uint32_t *)pConfig) + AM_HAL_MSPI_LINK_BASE;
            break;

        case AM_HAL_MSPI_REQ_DCX_DIS:
            //
            // Disable DCX.
            //
            MSPIn(ui32Module)->FLASH_b.XIPENDCX = 0;
            break;

        case AM_HAL_MSPI_REQ_DCX_EN:
            //
            // Enable DCX.
            //
            MSPIn(ui32Module)->FLASH_b.XIPENDCX = 1;
            break;

        case AM_HAL_MSPI_REQ_SCRAMB_DIS:
            //
            // Disable scrambling.
            //
            MSPIn(ui32Module)->SCRAMBLING_b.SCRENABLE = 0;
            break;

        case AM_HAL_MSPI_REQ_SCRAMB_EN:
            //
            // Enable scrambling.
            //
            MSPIn(ui32Module)->SCRAMBLING_b.SCRENABLE = 1;
            break;

        case AM_HAL_MSPI_REQ_XIPACK:
#ifndef AM_HAL_DISABLE_API_VALIDATION
            if (!pConfig)
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
#endif // AM_HAL_DISABLE_API_VALIDATION
            //
            // Enable/Disable XIPACK.
            //
            MSPIn(ui32Module)->FLASH_b.XIPACK = *((uint32_t *)pConfig);
            break;

        case AM_HAL_MSPI_REQ_DDR_DIS:
            //
            // Disable DDR.
            //
            MSPIn(ui32Module)->MSPIDDR_b.EMULATEDDR = 0;
            break;

        case AM_HAL_MSPI_REQ_DDR_EN:
            //
            // DDR emulation is only supported for 24MHz/48MHz.
            //
            if (pMSPIState->eClockFreq > AM_HAL_MSPI_CLK_24MHZ)
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
            //
            // Enable DDR.
            //
            MSPIn(ui32Module)->MSPIDDR_b.EMULATEDDR = 1;
            break;

        case AM_HAL_MSPI_REQ_DQS:
        {
            uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
            am_hal_mspi_dqs_t *pDQS = (am_hal_mspi_dqs_t *)pConfig;
#ifndef AM_HAL_DISABLE_API_VALIDATION
            if (!pConfig)
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
#endif
            MSPIn(ui32Module)->MSPIDDR_b.ENABLEDQS = pDQS->bDQSEnable;
            MSPIn(ui32Module)->MSPIDDR_b.OVERRIDERXDQSDELAY = pDQS->bOverrideRXDQSDelay;
            MSPIn(ui32Module)->MSPIDDR_b.RXDQSDELAY = pDQS->ui8RxDQSDelay;
            MSPIn(ui32Module)->MSPIDDR_b.OVERRIDEDDRCLKOUTDELAY = pDQS->bOverrideTXDQSDelay;
            MSPIn(ui32Module)->MSPIDDR_b.TXDQSDELAY = pDQS->ui8TxDQSDelay;
            MSPIn(ui32Module)->MSPIDDR_b.DQSSYNCNEG = pDQS->bDQSSyncNeg;
            return ui32Status;
        }
        //break;

        case AM_HAL_MSPI_REQ_XIP_DIS:
            //
            // Disable XIP.
            //
            MSPIn(ui32Module)->FLASH_b.XIPEN = 0;
            break;

        case AM_HAL_MSPI_REQ_XIP_EN:
            //
            // Enable XIP.
            //
            MSPIn(ui32Module)->FLASH_b.XIPEN = 1;
            break;

        case AM_HAL_MSPI_REQ_DEVICE_CONFIG:
#ifndef AM_HAL_DISABLE_API_VALIDATION
            if (!pConfig)
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
#endif // AM_HAL_DISABLE_API_VALIDATION
            {
              am_hal_mspi_device_e  deviceConfig;

              deviceConfig = *((am_hal_mspi_device_e *)pConfig);

              //
              // Configure the MSPI for a specific device configuration.
              // This function sets the following registers/fields:
              //    CFG.DEVCFG
              //    CFG.SEPIO
              //    FLASH.XIPMIXED
              //    PADCFG
              //    PADOUTEN
              //
              ui32Status = mspi_device_configure(pHandle, deviceConfig);
              if (AM_HAL_STATUS_SUCCESS != ui32Status)
              {
                return ui32Status;
              }
            }
            break;

        case AM_HAL_MSPI_REQ_CLOCK_CONFIG:
            {
                am_hal_mspi_clock_e  eClockFreq;
                eClockFreq = *((am_hal_mspi_clock_e *)pConfig);
                //
                // Set the clock divisor to get the desired MSPI clock frequency.
                //
                MSPIn(ui32Module)->MSPICFG_b.CLKDIV = eClockFreq;
                //
                // Adjust the clock edge configuration depending upon the clock frequency.
                //
                if ( eClockFreq == AM_HAL_MSPI_CLK_48MHZ )
                {
                    MSPIn(ui32Module)->MSPICFG_b.TXNEG = 1;
                    MSPIn(ui32Module)->MSPICFG_b.RXNEG = 0;
                    MSPIn(ui32Module)->MSPICFG_b.RXCAP = 1;
                    MSPIn(ui32Module)->MSPIDDR = 0;
                }
                else
                {
                    MSPIn(ui32Module)->MSPICFG_b.TXNEG = 0;
                    MSPIn(ui32Module)->MSPICFG_b.RXNEG = 0;
                    MSPIn(ui32Module)->MSPICFG_b.RXCAP = 1;
                    MSPIn(ui32Module)->MSPIDDR  = 0;
                }
                // set the DMABCOUNT
                if ( pMSPIState->pTCB )
                {
                    if ( eClockFreq == AM_HAL_MSPI_CLK_48MHZ )
                    {
                      // set the DMATHRESHs asymmetrically for 48MHz.
                      MSPIn(ui32Module)->DMATHRESH_b.DMATXTHRESH = AM_HAL_MSPI_MAX_FIFO_SIZE - AM_HAL_MSPI_HIGH_SPEED_OFFSET;
                      MSPIn(ui32Module)->DMATHRESH_b.DMARXTHRESH = AM_HAL_MSPI_HIGH_SPEED_OFFSET;
                    }
                    else
                    {
                      // set the DMATHRESHs symmetrically for all other clock speeds.
                      MSPIn(ui32Module)->DMATHRESH_b.DMATXTHRESH = (AM_HAL_MSPI_DEFAULT_BURST_COUNT >> 2);
                      MSPIn(ui32Module)->DMATHRESH_b.DMARXTHRESH = (AM_HAL_MSPI_DEFAULT_BURST_COUNT >> 2);
                    }
                }
                pMSPIState->eClockFreq = eClockFreq;
            }
            break;

        case AM_HAL_MSPI_REQ_PAUSE:
            ui32Status = mspi_cq_pause(pMSPIState);
            break;

        case AM_HAL_MSPI_REQ_UNPAUSE:
            // Resume the CQ
            MSPIn(ui32Module)->CQSETCLEAR = AM_HAL_MSPI_SC_UNPAUSE_CQ;
            break;

        case AM_HAL_MSPI_REQ_SET_SEQMODE:
        {
            am_hal_mspi_seq_e eSeq;
#ifndef AM_HAL_DISABLE_API_VALIDATION
            if (!pConfig)
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
            if (!pMSPIState->pTCB)
            {
                // No space for CMDQ
                return AM_HAL_STATUS_INVALID_OPERATION;
            }
#endif // AM_HAL_DISABLE_API_VALIDATION
            eSeq = *((bool *)pConfig) ? AM_HAL_MSPI_SEQ_UNDER_CONSTRUCTION: AM_HAL_MSPI_SEQ_NONE;
            if (eSeq == pMSPIState->eSeq)
            {
                // Nothing to do
                return AM_HAL_STATUS_SUCCESS;
            }
#if 0 // We should be able to operate on sequence even if there are HP transactions in progress
            // Make sure there is no high priority transaction in progress
            if (pMSPIState->ui32NumHPEntries)
            {
                return AM_HAL_STATUS_INVALID_OPERATION;
            }
#endif
            switch (pMSPIState->eSeq)
            {
                case AM_HAL_MSPI_SEQ_RUNNING:
                {
                    ui32Status = mspi_cq_pause(pMSPIState);
                    break;
                }
                case AM_HAL_MSPI_SEQ_NONE:
                {
                    // Make sure there is no non-blocking transaction in progress
                    if (pMSPIState->ui32NumCQEntries)
                    {
                        return AM_HAL_STATUS_INVALID_OPERATION;
                    }
                    break;
                }
                default:
                ;
            }
            if (ui32Status == AM_HAL_STATUS_SUCCESS)
            {
                // Reset the cmdq
                am_hal_cmdq_reset(pMSPIState->CQ.pCmdQHdl);
                pMSPIState->ui32LastIdxProcessed = 0;
                pMSPIState->ui32NumTransactions = 0;
                pMSPIState->ui32NumCQEntries = 0;
                pMSPIState->eSeq = eSeq;
                pMSPIState->bAutonomous = true;
                pMSPIState->ui32NumUnSolicited = 0;
            }
            break;
        }

        case AM_HAL_MSPI_REQ_SEQ_END:
        {
            uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
            am_hal_cmdq_entry_t     *pCQBlock;
            uint32_t                index;
            am_hal_mspi_seq_end_t *pLoop = (am_hal_mspi_seq_end_t *)pConfig;
            uint32_t pause = 0;
            uint32_t scUnpause = 0;
            uint32_t            ui32Critical = 0;
#ifndef AM_HAL_DISABLE_API_VALIDATION
            if (!pConfig)
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
            if (pLoop->ui32PauseCondition & AM_HAL_MSPI_PAUSE_FLAG_RESV)
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
            if (pLoop->ui32StatusSetClr & AM_HAL_MSPI_SC_RESV_MASK)
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
            if (pMSPIState->eSeq != AM_HAL_MSPI_SEQ_UNDER_CONSTRUCTION)
            {
                return AM_HAL_STATUS_INVALID_OPERATION;
            }
#endif // AM_HAL_DISABLE_API_VALIDATION
            if (pMSPIState->block)
            {
                // End the block if the sequence is ending
                pMSPIState->block = 0;
                // Unblock the whole batch of commands in this block
                MSPIn(ui32Module)->CQSETCLEAR = AM_HAL_MSPI_SC_UNPAUSE_BLOCK;
            }
            if ((pLoop->bLoop) && (!pMSPIState->bAutonomous))
            {
                // Need to insert special element in CQ to cause a callback
                // This is to reset internal state
                ui32Status = am_hal_cmdq_alloc_block(pMSPIState->CQ.pCmdQHdl, 1, &pCQBlock, &index);
                if (ui32Status != AM_HAL_STATUS_SUCCESS)
                {
                    return ui32Status;
                }
                else
                {
                    //
                    // Store the callback function pointer.
                    //
                    pMSPIState->pfnCallback[index & (AM_HAL_MSPI_MAX_CQ_ENTRIES - 1)] = mspi_seq_loopback;
                    pMSPIState->pCallbackCtxt[index & (AM_HAL_MSPI_MAX_CQ_ENTRIES - 1)] = (void *)pMSPIState;

                    // Dummy Entry
                    pCQBlock->address = (uint32_t)&MSPIn(ui32Module)->CQSETCLEAR;
                    pCQBlock->value = 0;

                    //
                    // Need to protect access of ui32NumPendTransactions as it is accessed
                    // from ISR as well
                    //
                    // Start a critical section.
                    //
                    ui32Critical = am_hal_interrupt_master_disable();
                    //
                    // Post to the CQ.
                    //
                    ui32Status = am_hal_cmdq_post_block(pMSPIState->CQ.pCmdQHdl, true);

                    if (AM_HAL_STATUS_SUCCESS != ui32Status)
                    {
                        //
                        // End the critical section.
                        //
                        am_hal_interrupt_master_set(ui32Critical);

                        am_hal_cmdq_release_block(pMSPIState->CQ.pCmdQHdl);
                        return ui32Status;
                    }
                    else
                    {
                        uint32_t ui32NumPend = pMSPIState->ui32NumCQEntries++;
                        //
                        // End the critical section.
                        //
                        am_hal_interrupt_master_set(ui32Critical);
                        if (ui32NumPend == 0)
                        {
                            //
                            // Enable the Command Queue
                            //
                            ui32Status = mspi_cq_enable(pHandle);
                            if (AM_HAL_STATUS_SUCCESS != ui32Status)
                            {
                                return ui32Status;
                            }
                        }
                    }
                    // Use SWFLAG6 to cause a pause
                    pause = AM_HAL_MSPI_PAUSE_FLAG_SEQLOOP;
                    // Revert back the flag after SW callback unpauses it
                    scUnpause = AM_HAL_MSPI_SC_PAUSE_SEQLOOP;
                }
            }
            // Insert the loopback
            ui32Status = am_hal_cmdq_alloc_block(pMSPIState->CQ.pCmdQHdl, sizeof(am_hal_mspi_cq_loop_entry_t) / 8, &pCQBlock, &index);
            if (ui32Status != AM_HAL_STATUS_SUCCESS)
            {
                return ui32Status;
            }
            else
            {
                am_hal_mspi_cq_loop_entry_t *pLoopEntry = (am_hal_mspi_cq_loop_entry_t *)pCQBlock;
                pLoopEntry->ui32PAUSENAddr = pLoopEntry->ui32PAUSEN2Addr = (uint32_t)&MSPIn(ui32Module)->CQPAUSE;
                pLoopEntry->ui32SETCLRAddr = (uint32_t)&MSPIn(ui32Module)->CQSETCLEAR;
                pLoopEntry->ui32PAUSEENVal = get_pause_val(pMSPIState, pLoop->ui32PauseCondition | pause);
                pLoopEntry->ui32PAUSEEN2Val = AM_HAL_MSPI_PAUSE_DEFAULT;
                pLoopEntry->ui32SETCLRVal = pLoop->ui32StatusSetClr | scUnpause;

                //
                // Need to protect access of ui32NumPendTransactions as it is accessed
                // from ISR as well
                //
                // Start a critical section.
                //
                ui32Critical = am_hal_interrupt_master_disable();

                //
                // Post to the CQ.
                //
                if (pLoop->bLoop)
                {
                    ui32Status = am_hal_cmdq_post_loop_block(pMSPIState->CQ.pCmdQHdl, false);
                }
                else
                {
                    ui32Status = am_hal_cmdq_post_block(pMSPIState->CQ.pCmdQHdl, false);
                }

                if (AM_HAL_STATUS_SUCCESS != ui32Status)
                {
                    //
                    // End the critical section.
                    //
                    am_hal_interrupt_master_set(ui32Critical);
                    am_hal_cmdq_release_block(pMSPIState->CQ.pCmdQHdl);
                }
                else
                {
                    uint32_t ui32NumPend = pMSPIState->ui32NumCQEntries++;
                    pMSPIState->eSeq = (pLoop->bLoop) ? AM_HAL_MSPI_SEQ_RUNNING : AM_HAL_MSPI_SEQ_NONE;
                    //
                    // End the critical section.
                    //
                    am_hal_interrupt_master_set(ui32Critical);
                    if (ui32NumPend == 0)
                    {
                        //
                        // Enable the Command Queue
                        //
                        ui32Status = mspi_cq_enable(pHandle);
                        if (AM_HAL_STATUS_SUCCESS != ui32Status)
                        {
                            return ui32Status;
                        }
                    }
                }
            }
            return ui32Status;
            //break;
        }

        case AM_HAL_MSPI_REQ_INIT_HIPRIO:
        {
            am_hal_mspi_hiprio_cfg_t *pHPCfg = (am_hal_mspi_hiprio_cfg_t *)pConfig;
#ifndef AM_HAL_DISABLE_API_VALIDATION
            if (!pConfig)
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
            if (pMSPIState->pHPTransactions)
            {
                return AM_HAL_STATUS_INVALID_OPERATION;
            }
#endif // AM_HAL_DISABLE_API_VALIDATION
            pMSPIState->ui32NumHPEntries = pMSPIState->ui32LastHPIdxProcessed = 0;
            pMSPIState->ui32NextHPIdx = pMSPIState->ui32LastHPIdxProcessed + 1;
            pMSPIState->pHPTransactions = (am_hal_mspi_dma_entry_t *)pHPCfg->pBuf;
            pMSPIState->ui32MaxHPTransactions = pHPCfg->size / sizeof(am_hal_mspi_dma_entry_t);
            break;
        }

        case AM_HAL_MSPI_REQ_START_BLOCK:
            // Pause the next block from proceeding till whole block is finished
            MSPIn(ui32Module)->CQSETCLEAR = AM_HAL_MSPI_SC_PAUSE_BLOCK;
            pMSPIState->block = 1;
            pMSPIState->ui32NumHPPendingEntries = 0;
            break;

        case AM_HAL_MSPI_REQ_END_BLOCK:
            // Unblock the whole batch of commands in this block
            MSPIn(ui32Module)->CQSETCLEAR = AM_HAL_MSPI_SC_UNPAUSE_BLOCK;
            pMSPIState->block = 0;
            if (!pMSPIState->ui32NumHPPendingEntries)
            {
                // Now it is okay to let go of the block of HiPrio transactions
                ui32Status = sched_hiprio(pMSPIState, pMSPIState->ui32NumHPPendingEntries);
                if (ui32Status == AM_HAL_STATUS_SUCCESS)
                {
                    pMSPIState->ui32NumHPPendingEntries = 0;
                }
            }
            break;

        case AM_HAL_MSPI_REQ_CQ_RAW:
        {
            am_hal_mspi_cq_raw_t *pCqRaw = (am_hal_mspi_cq_raw_t *)pConfig;
            am_hal_cmdq_entry_t *pCQBlock;
            uint32_t            ui32Critical = 0;
            uint32_t            ui32NumPend;
            uint32_t            index;
            am_hal_mspi_callback_t pfnCallback1;
#ifndef AM_HAL_DISABLE_API_VALIDATION
            if (!pCqRaw)
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
            if (!pMSPIState->CQ.pCmdQHdl)
            {
                return AM_HAL_STATUS_INVALID_OPERATION;
            }
#endif // AM_HAL_DISABLE_API_VALIDATION
            //
            // Check to see if there is enough room in the CQ
            //
            if ((pMSPIState->ui32NumCQEntries == AM_HAL_MSPI_MAX_CQ_ENTRIES) ||
                (am_hal_cmdq_alloc_block(pMSPIState->CQ.pCmdQHdl, pCqRaw->numEntries + 3, &pCQBlock, &index)))
            {
                return AM_HAL_STATUS_OUT_OF_RANGE;
            }

            pCQBlock->address = (uint32_t)&MSPIn(ui32Module)->CQPAUSE;
            pCQBlock->value = get_pause_val(pMSPIState, pCqRaw->ui32PauseCondition);
            pCQBlock++;
            // Copy the CQ Entry contents
            for (uint32_t i = 0; i < pCqRaw->numEntries; i++, pCQBlock++)
            {
                pCQBlock->address = pCqRaw->pCQEntry[i].address;
                pCQBlock->value = pCqRaw->pCQEntry[i].value;
            }
            // If there is a need - populate the jump back address
            if (pCqRaw->pJmpAddr)
            {
                *(pCqRaw->pJmpAddr) = (uint32_t)pCQBlock;
            }
            pCQBlock->address = (uint32_t)&MSPIn(ui32Module)->CQPAUSE;
            pCQBlock->value = AM_HAL_MSPI_PAUSE_DEFAULT;
            pCQBlock++;
            pCQBlock->address = (uint32_t)&MSPIn(ui32Module)->CQSETCLEAR;
            pCQBlock->value = pCqRaw->ui32StatusSetClr;

            pfnCallback1 = pCqRaw->pfnCallback;
            if ( !pfnCallback1 && !pMSPIState->block && (pMSPIState->eSeq == AM_HAL_MSPI_SEQ_NONE) &&
                 (pMSPIState->ui32NumUnSolicited >= (pMSPIState->ui32MaxPending / 2)) )
            {
                // Need to schedule a dummy callback, to ensure ui32NumCQEntries get updated in ISR
                pfnCallback1 = mspi_dummy_callback;
            }
            //
            // Store the callback function pointer.
            //
            pMSPIState->pfnCallback[index & (AM_HAL_MSPI_MAX_CQ_ENTRIES - 1)] = pfnCallback1;
            pMSPIState->pCallbackCtxt[index & (AM_HAL_MSPI_MAX_CQ_ENTRIES - 1)] = pCqRaw->pCallbackCtxt;

            //
            // Need to protect access of ui32NumPendTransactions as it is accessed
            // from ISR as well
            //
            // Start a critical section.
            //
            ui32Critical = am_hal_interrupt_master_disable();

            //
            // Post the transaction to the CQ.
            // Register for interrupt only if there is a callback
            //
            ui32Status = am_hal_cmdq_post_block(pMSPIState->CQ.pCmdQHdl, pfnCallback1);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                //
                // End the critical section.
                //
                am_hal_interrupt_master_set(ui32Critical);

                am_hal_cmdq_release_block(pMSPIState->CQ.pCmdQHdl);
            }
            else
            {
                ui32NumPend = pMSPIState->ui32NumCQEntries++;
                pMSPIState->ui32NumTransactions++;
                if (pCqRaw->pfnCallback)
                {
                    pMSPIState->bAutonomous = false;
                    pMSPIState->ui32NumUnSolicited = 0;
                }
                else
                {
                    if (pfnCallback1)
                    {
                        // This implies we have already scheduled a dummy callback
                        pMSPIState->ui32NumUnSolicited = 0;
                    }
                    else
                    {
                        pMSPIState->ui32NumUnSolicited++;
                    }
                }
                //
                // End the critical section.
                //
                am_hal_interrupt_master_set(ui32Critical);
                if (ui32NumPend == 0)
                {
                    //
                    // Enable the Command Queue
                    //
                    ui32Status = mspi_cq_enable(pHandle);
                    if (AM_HAL_STATUS_SUCCESS != ui32Status)
                    {
                        return ui32Status;
                    }
                }
            }
            break;
        }
        case AM_HAL_MSPI_REQ_ASIZE_SET:
        {
            uint8_t olen, nlen;

            nlen = *((uint8_t *)pConfig);

            //
            //preventing the invalid case
            //
            if (nlen == 0)
            {
                break;
            }

            olen = MSPIn(pMSPIState->ui32Module)->CFG_b.ASIZE + 1;
            if (olen != nlen)
            {
                MSPIn(pMSPIState->ui32Module)->CFG_b.ASIZE = nlen - 1;
            }
            *((uint8_t *)pConfig) = olen;
            break;
        }

        case AM_HAL_MSPI_REQ_NAND_FLASH_SET_WLAT:
        {
#ifndef AM_HAL_DISABLE_API_VALIDATION
            if (!pConfig)
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
#endif // AM_HAL_DISABLE_API_VALIDATION

            am_hal_mspi_dev_config_t *pDevConfig = (am_hal_mspi_dev_config_t*)pConfig;

            MSPIn(ui32Module)->CFG_b.WRITELATENCY = pDevConfig->ui8WriteLatency;
            MSPIn(ui32Module)->FLASH_b.XIPMIXED = MSPI0_FLASH_XIPMIXED_AD4;
            MSPIn(ui32Module)->FLASH_b.XIPENWLAT = pDevConfig->bEnWriteLatency;
            ui32Status = AM_HAL_STATUS_SUCCESS;

            break;
        }
        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Return the status.
    //
    return ui32Status;
}

//
// MSPI get capabilities
//
uint32_t am_hal_mspi_capabilities_get(void *pHandle,
                                      am_hal_mspi_capabilities_t **pCapabilities)
{
  am_hal_mspi_state_t           *pMSPIState = (am_hal_mspi_state_t *)pHandle;
  uint32_t                      ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if (!AM_HAL_MSPI_CHK_HANDLE(pHandle))
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Module = pMSPIState->ui32Module;
    //
    // copy the pointer the MSPI instance capabilities into the passed pointer
    //
    *pCapabilities = &g_MSPIState[ui32Module].capabilities;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//
// MSPI blocking transfer function
//
uint32_t am_hal_mspi_blocking_transfer(void *pHandle,
                                       am_hal_mspi_pio_transfer_t *pTransaction,
                                       uint32_t ui32Timeout)
{
    am_hal_mspi_state_t           *pMSPIState = (am_hal_mspi_state_t *)pHandle;
    uint32_t                      ui32Module;
    uint32_t                      ui32Control = 0;
    uint32_t                      ui32Status = AM_HAL_STATUS_SUCCESS;
    uint32_t                      intMask;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if (!AM_HAL_MSPI_CHK_HANDLE(pHandle))
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
    //
    // Check that the interface is enabled.
    //
    if (!pMSPIState->prefix.s.bEnable)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION
    ui32Module = pMSPIState->ui32Module;

    // Make sure there is no non-blocking transaction in progress
    if (pMSPIState->ui32NumCQEntries || pMSPIState->ui32NumHPEntries)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
    if (pMSPIState->eSeq == AM_HAL_MSPI_SEQ_RUNNING)
    {
        // Dynamic additions to sequence not allowed
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    //
    // Set the number of bytes to transfer.
    //
    ui32Control |= _VAL2FLD(MSPI0_CTRL_XFERBYTES, pTransaction->ui32NumBytes);

    //
    // Set the PIO default to scrambling disabled.
    //
    ui32Control |= _VAL2FLD(MSPI0_CTRL_PIOSCRAMBLE, pTransaction->bScrambling);

    //
    // Set transmit or receive operation.
    //
    ui32Control |= _VAL2FLD(MSPI0_CTRL_TXRX, pTransaction->eDirection);

    //
    // Set the indication to send an instruction and set the instruction value if
    // we have a valid instruction.
    //
    if ( pTransaction->bSendInstr )
    {
        ui32Control |= _VAL2FLD(MSPI0_CTRL_SENDI, 1);
        MSPIn(ui32Module)->INSTR =
            _VAL2FLD(MSPI0_INSTR_INSTR, pTransaction->ui16DeviceInstr);
    }

    //
    // Set the inidication to send an address and set the address value if we have
    // a valid address.
    //
    if ( pTransaction->bSendAddr )
    {
        ui32Control |= _VAL2FLD(MSPI0_CTRL_SENDA, 1);
        MSPIn(ui32Module)->ADDR =
            _VAL2FLD(MSPI0_ADDR_ADDR, pTransaction->ui32DeviceAddr);
    }

    //
    // Set the turn-around if needed.
    //
    if ( pTransaction->bTurnaround )
    {
        ui32Control |= _VAL2FLD(MSPI0_CTRL_ENTURN, 1);
    }

    //
    // Set the default FIFO Little Endian format.
    //
    ui32Control |= _VAL2FLD(MSPI0_CTRL_BIGENDIAN, pMSPIState->bBigEndian);

    //
    // Set for default of no continuation.
    //
    ui32Control |= _VAL2FLD(MSPI0_CTRL_CONT, pTransaction->bContinue);

    //
    // Set the write latency counter enable if needed.
    //
    ui32Control |= _VAL2FLD(MSPI0_CTRL_ENWLAT, pTransaction->bEnWRLatency);

    //
    // Set the Quad Command if this is transmit and the device is configured
    // for Dual Quad mode.
    //
    if ( pTransaction->bQuadCmd )
    {
        ui32Control |= _VAL2FLD(MSPI0_CTRL_QUADCMD, 1);
    }

    //
    // Enabled DCX if requested.
    //
    if ( pTransaction->bDCX)
    {
        ui32Control |= _VAL2FLD(MSPI0_CTRL_ENDCX, 1);
    }

    //
    // Start the Transfer.
    //
    ui32Control |= _VAL2FLD(MSPI0_CTRL_START, 1);

    // Disable all interrupts
    intMask = MSPIn(ui32Module)->INTEN;
    MSPIn(ui32Module)->INTEN = 0;
    MSPIn(ui32Module)->INTCLR = AM_HAL_MSPI_INT_ALL;

    //
    // Initiate the Transfer.
    //
    MSPIn(ui32Module)->CTRL = ui32Control;

    //
    // Read or Feed the FIFOs.
    //
    if ( AM_HAL_MSPI_RX == pTransaction->eDirection )
    {
        ui32Status = mspi_fifo_read(ui32Module, pTransaction->pui32Buffer,
                                    pTransaction->ui32NumBytes, pMSPIState->waitTimeout);
    }
    else if ( AM_HAL_MSPI_TX == pTransaction->eDirection )
    {
        ui32Status = mspi_fifo_write(ui32Module, pTransaction->pui32Buffer,
                                     pTransaction->ui32NumBytes, pMSPIState->waitTimeout );
    }

    //
    // Check status.
    //
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        //
        // Restore interrupts
        //
        MSPIn(ui32Module)->INTCLR = AM_HAL_MSPI_INT_ALL;
        MSPIn(ui32Module)->INTEN = intMask;
        return ui32Status;
    }

    //
    // Wait for the command to complete.
    //
    ui32Status = am_hal_flash_delay_status_check(ui32Timeout,
                                                 (uint32_t)&MSPIn(ui32Module)->CTRL,
                                                 MSPI0_CTRL_STATUS_Msk,
                                                 _VAL2FLD(MSPI0_CTRL_STATUS, 1),
                                                 true);

    //
    // Restore interrupts
    //
    MSPIn(ui32Module)->INTCLR = AM_HAL_MSPI_INT_ALL;
    MSPIn(ui32Module)->INTEN = intMask;

    //
    // Return the status.
    //
    return ui32Status;
}

//
// MSPI Non-Blocking transfer function
//
uint32_t am_hal_mspi_nonblocking_transfer(void *pHandle,
                                          void *pTransfer,
                                          am_hal_mspi_trans_e eMode,
                                          am_hal_mspi_callback_t pfnCallback,
                                          void *pCallbackCtxt)
{
    am_hal_mspi_state_t           *pMSPIState = (am_hal_mspi_state_t *)pHandle;
    uint32_t                      ui32Status = AM_HAL_STATUS_SUCCESS;
    uint32_t ui32NumPend;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if (!AM_HAL_MSPI_CHK_HANDLE(pHandle))
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
    if (!pMSPIState->pTCB)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
    //
    // Check that the interface is enabled.
    //
    if (!pMSPIState->prefix.s.bEnable)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

#if 0 // We should be able to queue up the CQ even if high priority transaction is in progress
    if (pMSPIState->ui32NumHPEntries)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif
    if (pMSPIState->eSeq == AM_HAL_MSPI_SEQ_RUNNING)
    {
        // Dynamic additions to sequence not allowed
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
    //
    // Check for DMA to/from DTCM.
    //
    if ( AM_HAL_MSPI_TRANS_DMA == eMode)
    {
      am_hal_mspi_dma_transfer_t *pDMATransaction = (am_hal_mspi_dma_transfer_t *)pTransfer;
      if ( (pDMATransaction->ui32SRAMAddress >= AM_HAL_FLASH_DTCM_START) &&
          (pDMATransaction->ui32SRAMAddress <= AM_HAL_FLASH_DTCM_END) )
      {
        return AM_HAL_STATUS_OUT_OF_RANGE;
      }
    }
#endif // AM_HAL_DISABLE_API_VALIDATION
    am_hal_mspi_callback_t pfnCallback1 = pfnCallback;
    if ( !pfnCallback1 && !pMSPIState->block && (pMSPIState->eSeq == AM_HAL_MSPI_SEQ_NONE) &&
         (pMSPIState->ui32NumUnSolicited >= (pMSPIState->ui32MaxPending / 2)) )
    {
        // Need to schedule a dummy callback, to ensure ui32NumCQEntries get updated in ISR
        pfnCallback1 = mspi_dummy_callback;
    }

    //
    // DMA defaults to using the Command Queue
    //
    ui32Status = mspi_cq_add_transaction(pHandle, pTransfer, eMode, pfnCallback1, pCallbackCtxt);

    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return ui32Status;
    }
    else
    {

        uint32_t ui32Critical = 0;

        //
        // Start a critical section.
        //
        ui32Critical = am_hal_interrupt_master_disable();

        //
        // Post the transaction to the CQ.
        //
        ui32Status = am_hal_cmdq_post_block(pMSPIState->CQ.pCmdQHdl, pfnCallback1);

        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            //
            // End the critical section.
            //
            am_hal_interrupt_master_set(ui32Critical);

            am_hal_cmdq_release_block(pMSPIState->CQ.pCmdQHdl);
        }
        else
        {
            ui32NumPend = pMSPIState->ui32NumCQEntries++;
            pMSPIState->ui32NumTransactions++;
            if (pfnCallback)
            {
                pMSPIState->bAutonomous = false;
                pMSPIState->ui32NumUnSolicited = 0;
            }
            else
            {
                if (pfnCallback1)
                {
                    // This implies we have already scheduled a dummy callback
                    pMSPIState->ui32NumUnSolicited = 0;
                }
                else
                {
                    pMSPIState->ui32NumUnSolicited++;
                }
            }
            //
            // End the critical section.
            //
            am_hal_interrupt_master_set(ui32Critical);
            if (ui32NumPend == 0)
            {
                //
                // Enable the Command Queue
                //
                ui32Status = mspi_cq_enable(pHandle);
                if (AM_HAL_STATUS_SUCCESS != ui32Status)
                {
                    return ui32Status;
                }
            }
        }
    }

    //
    // Return the status.
    //
    return ui32Status;
}

//
// MSPI status function
//
uint32_t am_hal_mspi_status_get(void *pHandle,
                                am_hal_mspi_status_t *pStatus )
{
    am_hal_mspi_state_t           *pMSPIState = (am_hal_mspi_state_t *)pHandle;
    uint32_t                      ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if (!AM_HAL_MSPI_CHK_HANDLE(pHandle))
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Module = pMSPIState->ui32Module;
    //
    // Get the Command Complete status.
    //

    //
    // Get the FIFO status.
    //

    //
    //  Get the DMA status.
    //
    pStatus->bErr = ((MSPIn(ui32Module)->DMASTAT & MSPI0_DMASTAT_DMAERR_Msk) > 0);
    pStatus->bCmp = ((MSPIn(ui32Module)->DMASTAT & MSPI0_DMASTAT_DMACPL_Msk) > 0);
    pStatus->bTIP = ((MSPIn(ui32Module)->DMASTAT & MSPI0_DMASTAT_DMATIP_Msk) > 0);

    //
    // Get the CQ status.
    //
    pStatus->ui32NumCQEntries = pMSPIState->ui32NumCQEntries;

    //
    // Get the scrambling status.
    //

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//
// MSPI enable interrupts function
//
uint32_t am_hal_mspi_interrupt_enable(void *pHandle,
                                      uint32_t ui32IntMask)
{
    am_hal_mspi_state_t           *pMSPIState = (am_hal_mspi_state_t *)pHandle;
    uint32_t                      ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if (!AM_HAL_MSPI_CHK_HANDLE(pHandle))
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Module = pMSPIState->ui32Module;
    //
    // Set the interrupt enables according to the mask.
    //
    MSPIn(ui32Module)->INTEN |= ui32IntMask;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//
// MSPI disable interrupts function
//
uint32_t am_hal_mspi_interrupt_disable(void *pHandle,
                                       uint32_t ui32IntMask)
{
    am_hal_mspi_state_t           *pMSPIState = (am_hal_mspi_state_t *)pHandle;
    uint32_t                      ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if (!AM_HAL_MSPI_CHK_HANDLE(pHandle))
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Module = pMSPIState->ui32Module;
    //
    // Clear the interrupt enables according to the mask.
    //
    MSPIn(ui32Module)->INTEN &= ~ui32IntMask;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//
// MSPI interrupt status function
//
uint32_t am_hal_mspi_interrupt_status_get(void *pHandle,
                                          uint32_t  *pui32Status,
                                          bool bEnabledOnly)
{
    am_hal_mspi_state_t           *pMSPIState = (am_hal_mspi_state_t *)pHandle;
    uint32_t                      ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if (!AM_HAL_MSPI_CHK_HANDLE(pHandle))
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Module = pMSPIState->ui32Module;
    //
    // if requested, only return the interrupts that are enabled.
    //
    if ( bEnabledOnly )
    {
        uint32_t ui32RetVal = MSPIn(ui32Module)->INTSTAT;
        *pui32Status = ui32RetVal & MSPIn(ui32Module)->INTEN;
    }
    else
    {
        *pui32Status = MSPIn(ui32Module)->INTSTAT;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//
// MSPI interrupt clear
//
uint32_t am_hal_mspi_interrupt_clear(void *pHandle,
                                     uint32_t ui32IntMask)
{
    am_hal_mspi_state_t           *pMSPIState = (am_hal_mspi_state_t *)pHandle;
    uint32_t                      ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_MSPI_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Module = pMSPIState->ui32Module;
    //
    // clear the requested interrupts.
    //
    MSPIn(ui32Module)->INTCLR = ui32IntMask;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//
// MSPI interrupt service routine
//
uint32_t am_hal_mspi_interrupt_service(void *pHandle, uint32_t ui32IntStatus)
{
    am_hal_mspi_state_t           *pMSPIState = (am_hal_mspi_state_t *)pHandle;
    uint32_t                      ui32Module;
    uint32_t                      ui32Status;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if (!AM_HAL_MSPI_CHK_HANDLE(pHandle))
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Module = pMSPIState->ui32Module;
    //
    // Add a delay to help make the service function work.
    //
    // am_hal_flash_delay(FLASH_CYCLES_US(10));

    if (pMSPIState->bHP)
    {
#if 0
        if (ui32IntStatus & AM_HAL_MSPI_INT_CQUPD)
        {
            while(1);
        }
#endif
        //
        // Accumulate the INTSTAT for this transaction
        //
        pMSPIState->ui32TxnInt |= ui32IntStatus;

        //
        // We need to wait for the DMA complete as well
        //
        if (pMSPIState->ui32TxnInt & (AM_HAL_MSPI_INT_DMACMP | AM_HAL_MSPI_INT_ERR))
        {
            uint32_t index;

            //
            // Wait for the command completion.
            // In Apollo3P - we cannot rely on CMDCMP, as hardware may be splitting
            // the transactions internally, and each split transaction generates CMDCMP
            // DMATIP is guaranteed to deassert only once the bus transaction is done
            //
            while (MSPIn(pMSPIState->ui32Module)->DMASTAT_b.DMATIP);
            pMSPIState->ui32TxnInt |= MSPIn(ui32Module)->INTSTAT;

            //
            // Clear the interrupt status
            //
            MSPIn(ui32Module)->INTCLR = AM_HAL_MSPI_INT_ALL;

            //
            // Need to determine the error, call the callback with proper status
            //
            if (pMSPIState->ui32TxnInt & AM_HAL_MSPI_INT_ERR)
            {
                ui32Status = AM_HAL_STATUS_FAIL;

                //
                // Disable DMA
                //
                MSPIn(ui32Module)->DMACFG_b.DMAEN = 0;

                //
                // Must reset xfer block
                //
                MSPIn(ui32Module)->MSPICFG_b.IPRSTN = 0;  // in reset
                MSPIn(ui32Module)->MSPICFG_b.IPRSTN = 1;  // back out -- clears current transfer
            }
            else
            {
                ui32Status = AM_HAL_STATUS_SUCCESS;
            }

            pMSPIState->ui32LastHPIdxProcessed++;
            pMSPIState->ui32NumHPEntries--;
            index = pMSPIState->ui32LastHPIdxProcessed % pMSPIState->ui32MaxHPTransactions;
            am_hal_mspi_dma_entry_t *pDMAEntry = &pMSPIState->pHPTransactions[index];

            //
            // Call the callback
            //
            if ( pDMAEntry->pfnCallback != NULL )
            {
                pDMAEntry->pfnCallback(pDMAEntry->pCallbackCtxt, ui32Status);
                pDMAEntry->pfnCallback = NULL;
            }

            //
            // Post next transaction if queue is not empty
            //
            if (pMSPIState->ui32NumHPEntries)
            {
                pMSPIState->ui32TxnInt = 0;
                program_dma(pMSPIState);
            }
            else
            {
                pMSPIState->bHP = false;
                // Unpause the CQ
                //
                // Command to set the DMACFG to disable DMA.
                // Need to make sure we disable DMA before we can reprogram
                //
                MSPIn(ui32Module)->DMACFG = _VAL2FLD(MSPI0_DMACFG_DMAEN, 0);
                // Restore interrupts
                MSPIn(ui32Module)->INTEN &= ~(AM_HAL_MSPI_INT_DMACMP);
                // Resume the CQ
                MSPIn(ui32Module)->CQSETCLEAR = AM_HAL_MSPI_SC_UNPAUSE_CQ;
            }
        }
        return AM_HAL_STATUS_SUCCESS;
    }
    //
    // Need to check if there is an ongoing transaction
    // This is needed because we may get interrupts even for the XIP transactions
    //
    if (pMSPIState->ui32NumCQEntries)
    {
        am_hal_cmdq_status_t status;
        uint32_t index;
        am_hal_mspi_CQ_t *pCQ = &g_MSPIState[ui32Module].CQ;

        //
        // Get the current and last indexes.
        //
        if (pCQ->pCmdQHdl)
        {
            ui32Status = am_hal_cmdq_get_status(pCQ->pCmdQHdl, &status);

            if (AM_HAL_STATUS_SUCCESS == ui32Status)
            {
                // For Sequence - this can be updated in the callback
                pMSPIState->bRestart = false;
                //
                // Figure out which callbacks need to be handled.
                //
                while (!pMSPIState->bRestart && (pMSPIState->ui32LastIdxProcessed != status.lastIdxProcessed))
                {

                    pMSPIState->ui32LastIdxProcessed++;
                    pMSPIState->ui32NumCQEntries--;
                    index = pMSPIState->ui32LastIdxProcessed & (AM_HAL_MSPI_MAX_CQ_ENTRIES - 1);
                    if ( pMSPIState->pfnCallback[index] != NULL )
                    {
                        pMSPIState->pfnCallback[index](pMSPIState->pCallbackCtxt[index], AM_HAL_STATUS_SUCCESS);
                        if (pMSPIState->eSeq != AM_HAL_MSPI_SEQ_RUNNING)
                        {
                            pMSPIState->pfnCallback[index] = NULL;
                        }
                    }
                }

                // For Sequence - this can be updated in the callback
                if (!pMSPIState->bRestart)
                {
                    //
                    // Process one extra callback if there was an error.
                    //
                    if ( (ui32IntStatus & AM_HAL_MSPI_INT_ERR) || (status.bErr) )
                    {
                        pMSPIState->ui32LastIdxProcessed++;
                        pMSPIState->ui32NumCQEntries--;
                        index = pMSPIState->ui32LastIdxProcessed & (AM_HAL_MSPI_MAX_CQ_ENTRIES - 1);
                        if ( pMSPIState->pfnCallback[index] != NULL )
                        {
                            pMSPIState->pfnCallback[index](pMSPIState->pCallbackCtxt[index], AM_HAL_STATUS_FAIL);
                            if (pMSPIState->eSeq != AM_HAL_MSPI_SEQ_RUNNING)
                            {
                                pMSPIState->pfnCallback[index] = NULL;
                            }
                        }
                        // Disable CQ
                        ui32Status = mspi_cq_disable(pMSPIState);
                        if (AM_HAL_STATUS_SUCCESS != ui32Status)
                        {
                          return ui32Status;
                        }
                        // Disable DMA
                        MSPIn(ui32Module)->DMACFG_b.DMAEN = 0;

                        // Must reset xfer block
                        MSPIn(ui32Module)->MSPICFG_b.IPRSTN = 0;  // in reset
                        MSPIn(ui32Module)->MSPICFG_b.IPRSTN = 1;  // back out -- clears current transfer

                        // Clear the CQ error.
                        MSPIn(ui32Module)->CQSTAT |= _VAL2FLD(MSPI0_CQSTAT_CQERR, 0);
                        am_hal_cmdq_error_resume(pCQ->pCmdQHdl);
                        if (pMSPIState->ui32NumCQEntries)
                        {
                          // Re-enable CQ
                          ui32Status = mspi_cq_enable(pMSPIState);
                          if (AM_HAL_STATUS_SUCCESS != ui32Status)
                          {
                            return ui32Status;
                          }
                        }
                    }
                    if (pMSPIState->ui32NumCQEntries == 0)
                    {
                        // Disable CQ
                        ui32Status = mspi_cq_disable(pMSPIState);
                        if (AM_HAL_STATUS_SUCCESS != ui32Status)
                        {
                          return ui32Status;
                        }
                    }
                }
            }
        }

        if (pMSPIState->ui32NumCQEntries == 0)
        {
            // Disable DMA
            MSPIn(ui32Module)->DMACFG_b.DMAEN = 0;
        }
    }

  //
  // Return the status.
  //
  return AM_HAL_STATUS_SUCCESS;
}

//
// MSPI power control function
//
uint32_t am_hal_mspi_power_control(void *pHandle,
                                   am_hal_sysctrl_power_state_e ePowerState,
                                   bool bRetainState)
{
    am_hal_mspi_state_t           *pMSPIState = (am_hal_mspi_state_t *)pHandle;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if (!AM_HAL_MSPI_CHK_HANDLE(pHandle))
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Decode the requested power state and update MSPI operation accordingly.
    //
    switch (ePowerState)
    {
        case AM_HAL_SYSCTRL_WAKE:

            if (bRetainState && !pMSPIState->registerState.bValid)
            {
                return AM_HAL_STATUS_INVALID_OPERATION;
            }

            //
            // Enable power control.
            //
            am_hal_pwrctrl_periph_enable((am_hal_pwrctrl_periph_e)(AM_HAL_PWRCTRL_PERIPH_MSPI0 + pMSPIState->ui32Module));

            if (bRetainState)
            {
                //
                // Restore MSPI registers
                //
                MSPIn(pMSPIState->ui32Module)->CFG        = pMSPIState->registerState.regCFG;
                MSPIn(pMSPIState->ui32Module)->MSPICFG    = pMSPIState->registerState.regMSPICFG;
                MSPIn(pMSPIState->ui32Module)->PADCFG     = pMSPIState->registerState.regPADCFG;
                MSPIn(pMSPIState->ui32Module)->PADOUTEN   = pMSPIState->registerState.regPADOUTEN;
                MSPIn(pMSPIState->ui32Module)->FLASH      = pMSPIState->registerState.regFLASH;
                MSPIn(pMSPIState->ui32Module)->SCRAMBLING = pMSPIState->registerState.regSCRAMBLING;
                MSPIn(pMSPIState->ui32Module)->CQADDR     = pMSPIState->registerState.regCQADDR;
                MSPIn(pMSPIState->ui32Module)->CQPAUSE    = pMSPIState->registerState.regCQPAUSE;
                MSPIn(pMSPIState->ui32Module)->CQCURIDX   = pMSPIState->registerState.regCQCURIDX;
                MSPIn(pMSPIState->ui32Module)->CQENDIDX   = pMSPIState->registerState.regCQENDIDX;
                MSPIn(pMSPIState->ui32Module)->INTEN      = pMSPIState->registerState.regINTEN;
                MSPIn(pMSPIState->ui32Module)->XIPINSTR   = pMSPIState->registerState.regXIPINSTR;
                MSPIn(pMSPIState->ui32Module)->THRESHOLD   = pMSPIState->registerState.regTHRESHOLD;

                MSPIn(pMSPIState->ui32Module)->DMABCOUNT  = pMSPIState->registerState.regDMABCOUNT;
                MSPIn(pMSPIState->ui32Module)->DMATHRESH  = pMSPIState->registerState.regDMATHRESH;

                // CQFGLAGS are Read-Only and hence can not be directly restored.
                // We can try to restore the SWFlags here. Hardware flags depend on external conditions
                // and hence can not be restored (assuming the external conditions remain the same, it should be set automatically.
                MSPIn(pMSPIState->ui32Module)->CQSETCLEAR = AM_HAL_MSPI_SC_SET(pMSPIState->registerState.regCQFLAGS & 0xFF);

                //
                // Set the CQCFG last
                //
                MSPIn(pMSPIState->ui32Module)->CQCFG      = pMSPIState->registerState.regCQCFG & ~_VAL2FLD(MSPI0_CQCFG_CQEN, MSPI0_CQCFG_CQEN_EN);

                if (pMSPIState->registerState.regCQCFG & _VAL2FLD(MSPI0_CQCFG_CQEN, MSPI0_CQCFG_CQEN_EN))
                {
                    mspi_cq_enable(pMSPIState);
                }

                pMSPIState->registerState.bValid = false;
            }
            break;

        case AM_HAL_SYSCTRL_NORMALSLEEP:
        case AM_HAL_SYSCTRL_DEEPSLEEP:
            // Make sure MPSI is not active currently
            if (pMSPIState->prefix.s.bEnable &&
                ((MSPIn(pMSPIState->ui32Module)->DMASTAT_b.DMATIP) ||
                   pMSPIState->ui32NumHPPendingEntries))
            {
                return AM_HAL_STATUS_IN_USE;
            }
            if (bRetainState)
            {
                //
                // Save MSPI Registers
                //
                pMSPIState->registerState.regCFG        = MSPIn(pMSPIState->ui32Module)->CFG;
                pMSPIState->registerState.regMSPICFG    = MSPIn(pMSPIState->ui32Module)->MSPICFG;
                pMSPIState->registerState.regPADCFG     = MSPIn(pMSPIState->ui32Module)->PADCFG;
                pMSPIState->registerState.regPADOUTEN   = MSPIn(pMSPIState->ui32Module)->PADOUTEN;
                pMSPIState->registerState.regFLASH      = MSPIn(pMSPIState->ui32Module)->FLASH;
                pMSPIState->registerState.regSCRAMBLING = MSPIn(pMSPIState->ui32Module)->SCRAMBLING;
                pMSPIState->registerState.regCQADDR     = MSPIn(pMSPIState->ui32Module)->CQADDR;
                pMSPIState->registerState.regCQPAUSE    = MSPIn(pMSPIState->ui32Module)->CQPAUSE;
                pMSPIState->registerState.regCQCURIDX   = MSPIn(pMSPIState->ui32Module)->CQCURIDX;
                pMSPIState->registerState.regCQENDIDX   = MSPIn(pMSPIState->ui32Module)->CQENDIDX;
                pMSPIState->registerState.regINTEN      = MSPIn(pMSPIState->ui32Module)->INTEN;
                pMSPIState->registerState.regCQFLAGS  = MSPIn(pMSPIState->ui32Module)->CQFLAGS;
                pMSPIState->registerState.regXIPINSTR = MSPIn(pMSPIState->ui32Module)->XIPINSTR;
                pMSPIState->registerState.regTHRESHOLD  = MSPIn(pMSPIState->ui32Module)->THRESHOLD;

                pMSPIState->registerState.regDMABCOUNT  = MSPIn(pMSPIState->ui32Module)->DMABCOUNT;
                pMSPIState->registerState.regDMATHRESH  = MSPIn(pMSPIState->ui32Module)->DMATHRESH;

                //
                // Set the CQCFG last
                //
                pMSPIState->registerState.regCQCFG    = MSPIn(pMSPIState->ui32Module)->CQCFG;
                pMSPIState->registerState.bValid      = true;

                if (MSPIn(pMSPIState->ui32Module)->CQCFG & _VAL2FLD(MSPI0_CQCFG_CQEN, MSPI0_CQCFG_CQEN_EN))
                {
                    mspi_cq_disable(pMSPIState);
                }
            }

            //
            // Disable all the interrupts.
            //
            am_hal_mspi_interrupt_disable(pHandle, MSPI0_INTEN_SCRERR_Msk |
                                          MSPI0_INTEN_CQERR_Msk |
                                            MSPI0_INTEN_CQPAUSED_Msk |
                                              MSPI0_INTEN_CQUPD_Msk |
                                                MSPI0_INTEN_CQCMP_Msk |
                                                  MSPI0_INTEN_DERR_Msk |
                                                    MSPI0_INTEN_DCMP_Msk |
                                                      MSPI0_INTEN_RXF_Msk |
                                                        MSPI0_INTEN_RXO_Msk |
                                                          MSPI0_INTEN_RXU_Msk |
                                                            MSPI0_INTEN_TXO_Msk |
                                                              MSPI0_INTEN_TXE_Msk |
                                                                MSPI0_INTEN_CMDCMP_Msk);

            //
            // Disable power control.
            //
            am_hal_pwrctrl_periph_disable((am_hal_pwrctrl_periph_e)(AM_HAL_PWRCTRL_PERIPH_MSPI0 + pMSPIState->ui32Module));
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//
// MSPI High Priority transfer function
//
uint32_t am_hal_mspi_highprio_transfer(void *pHandle,
                                       am_hal_mspi_dma_transfer_t *pTransfer,
                                       am_hal_mspi_trans_e eMode,
                                       am_hal_mspi_callback_t pfnCallback,
                                       void *pCallbackCtxt)
{
    am_hal_mspi_state_t           *pMSPIState = (am_hal_mspi_state_t *)pHandle;
    uint32_t                      ui32Status = AM_HAL_STATUS_SUCCESS;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if (!AM_HAL_MSPI_CHK_HANDLE(pHandle))
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
    if (!pMSPIState->pTCB)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
    if (!pMSPIState->pHPTransactions)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
    if (pTransfer->ui32PauseCondition != 0)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    if (pTransfer->ui32StatusSetClr != 0)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    //
    // Check for DMA to/from DTCM.
    //
    if ( AM_HAL_MSPI_TRANS_DMA == eMode)
    {
      am_hal_mspi_dma_transfer_t *pDMATransaction = (am_hal_mspi_dma_transfer_t *)pTransfer;
      if ( (pDMATransaction->ui32SRAMAddress >= AM_HAL_FLASH_DTCM_START) &&
          (pDMATransaction->ui32SRAMAddress <= AM_HAL_FLASH_DTCM_END) )
      {
        return AM_HAL_STATUS_OUT_OF_RANGE;
      }
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Status = mspi_add_hp_transaction(pHandle, pTransfer, pfnCallback, pCallbackCtxt);

    if (ui32Status == AM_HAL_STATUS_SUCCESS)
    {
        if (!(pMSPIState->block))
        {
            ui32Status = sched_hiprio(pMSPIState, 1);
        }
        else
        {
            pMSPIState->ui32NumHPPendingEntries++;
        }
    }

    //
    // Return the status.
    //
    return ui32Status;
}


//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
