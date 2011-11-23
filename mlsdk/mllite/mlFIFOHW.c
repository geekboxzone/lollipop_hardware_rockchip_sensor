/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: mlFIFOHW.c 4605 2011-01-26 01:20:31Z nroyer $
 *
 *******************************************************************************/

/** 
 *  @defgroup MLFIFO_HW 
 *  @brief  Motion Library - FIFO HW Driver.
 *          Provides facilities to interact with the FIFO.
 *
 *  @{
 *      @file   mlFIFOHW.c
 *      @brief  The Motion Library Fifo Hardware Layer.
 *
 */

#include <string.h>

#include "mlFIFOHW.h"
#include "ml.h"
#include "mldl.h"
#include "mpu.h"
#include "mldl_cfg.h"

#include "mlsl.h"

#include "log.h"
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-fifo"

/*
    Defines
*/

#define _fifoDebug(x) //{x}

/*
    Typedefs
*/

typedef struct _tFIFODataHWHW {
    short             fifoCount;
    tMLError          fifoError;
    unsigned char     fifoOverflow;
    unsigned char     fifoResetOnOverflow;
} tFIFODataHW;

/*
    Global variables
*/
const unsigned char gFifoFooter[FIFO_FOOTER_SIZE] = {0xB2 , 0x6A};

/*
    Static variables
*/
static tFIFODataHW FIFODataHW;


/*
    Definitions
*/


/**
 *  @brief  Initializes the internal FIFO data structure.
 */
void FIFOHWInit(void)
{
    memset( &FIFODataHW, 0, sizeof(FIFODataHW) );
    FIFODataHW.fifoResetOnOverflow = TRUE;
}

/**
 *  @internal
 *  @brief  used to get the FIFO data.
 *  @param  length  
 *              Number of bytes to read from the FIFO.
 *  @param  buffer  
 *              the bytes of FIFO data.
 *              Note that this buffer <b>must</b> be large enough
 *              to store and additional trailing FIFO footer when 
 *              expected.  The callers must make sure enough space
 *              is allocated.
 *  @return number of valid bytes of data.
**/
uint_fast16_t MPUGetFIFO(uint_fast16_t length, unsigned char* buffer)
{
    INVENSENSE_FUNC_START;
    tMLError result;
    uint_fast16_t inFifo;
    uint_fast16_t toRead;
    int_fast8_t kk;

    toRead = length - FIFO_FOOTER_SIZE + FIFODataHW.fifoCount;
    /*---- make sure length is correct ----*/
    if( length > MAX_FIFO_LENGTH || toRead > length || NULL == buffer) {
        FIFODataHW.fifoError = ML_ERROR_INVALID_PARAMETER;
        return 0;
    }

    result = MLDLGetFifoLength(&inFifo);
    if (ML_SUCCESS != result) {
        FIFODataHW.fifoError = result;
        return 0;
    }

    // FIFODataHW.fifoCount is the footer size left in the buffer, or 
    //      0 if this is the first time reading the fifo since it was reset
    if ( inFifo < length + FIFODataHW.fifoCount) {
        FIFODataHW.fifoError = ML_SUCCESS;
        return 0;
    }

    // if a trailing fifo count is expected - start storing data 2 bytes before
    result = MLDLReadFifo(
        FIFODataHW.fifoCount>0 ? buffer : buffer+FIFO_FOOTER_SIZE, 
        toRead
    ); 
    if (ML_SUCCESS != result) {
        FIFODataHW.fifoError = result;
        return 0;
    }

    // Make sure the fifo didn't overflow before or during the read
    result = MLSLSerialRead(MLSerialGetHandle(), MLDLGetMPUSlaveAddr(),
                            MPUREG_INT_STATUS, 1, &FIFODataHW.fifoOverflow);
    if (ML_SUCCESS != result) {
        FIFODataHW.fifoError = result;
        return 0;
    }
    
    if (FIFODataHW.fifoOverflow & BIT_INT_STATUS_FIFO_OVERLOW) {
        MPL_LOGV("Resetting Fifo : Overflow\n");
        FIFOReset();
        FIFODataHW.fifoError = ML_ERROR_FIFO_OVERFLOW;        
        return 0;
    }

    /* Check the Footer value to give us a chance at making sure data 
     * didn't get corrupted */
    for (kk=0; kk<FIFODataHW.fifoCount; ++kk) {
        if (buffer[kk] != gFifoFooter[kk]) {
            MPL_LOGV("Resetting Fifo : Invalid footer : 0x%02x 0x%02x\n",
                        buffer[0], buffer[1]);
            _fifoDebug(
                char out[200];
                MPL_LOGW("fifoCount : %d\n", FIFODataHW.fifoCount);
                sprintf(out, "0x");
                for(kk=0; kk<(int)toRead; kk++) {
                    sprintf(out, "%s%02X", out, buffer[kk]);
                }
                MPL_LOGW("%s\n", out);
            )
            FIFOReset();
            FIFODataHW.fifoError = ML_ERROR_FIFO_FOOTER;
            return 0;
        }    
    }

    if ( FIFODataHW.fifoCount==0 ) {
        FIFODataHW.fifoCount = FIFO_FOOTER_SIZE;
    }

    return length-FIFO_FOOTER_SIZE;
}

/**
 *  @brief  Used to query the status of the FIFO.
 *  @return ML_SUCCESS if the fifo is OK. An error code otherwise.
**/
tMLError MLDLGetFIFOStatus( void )
{
    tMLError fifoError = FIFODataHW.fifoError;
    FIFODataHW.fifoError = 0;
    return fifoError;
}

/** 
 * @internal
 * @brief   Get the length from the fifo
 * 
 * @param[out] len amount of data currently stored in the fifo.
 * 
 * @return ML_SUCCESS or non-zero error code.
**/
tMLError MLDLGetFifoLength(uint_fast16_t * len)
{
    INVENSENSE_FUNC_START;
    unsigned char fifoBuf[2];
    tMLError result;
 
    if (NULL == len)
        return ML_ERROR_INVALID_PARAMETER;

    /*---- read the 2 'count' registers and 
      burst read the data from the FIFO ----*/
    result = MLSLSerialRead(MLSerialGetHandle(), MLDLGetMPUSlaveAddr(),
                            MPUREG_FIFO_COUNTH, 2, fifoBuf );
    if (ML_SUCCESS != result) {
        MPL_LOGE("ReadBurst failed %d\n", result);
        FIFOReset();
        FIFODataHW.fifoError = ML_ERROR_FIFO_READ_COUNT;
        *len = 0;
        return result;
    }

    *len  = (uint_fast16_t)(fifoBuf[0] << 8);
    *len += (uint_fast16_t)(fifoBuf[1]     );
    return result;
}

/**
 *  @brief  MLDLGetFIFOCount is used to get the number of bytes left in the FIFO.
 *          This function returns the stored value and does not access the hardware.  
 *          See MLDLGetFifoLength().
 *  @return the number of bytes left in the FIFO
**/
short MLDLGetFIFOCount(void)
{
    return FIFODataHW.fifoCount;
}

/** 
 *  @internal
 *  @brief  Read data from the fifo
 * 
 *  @param[out] data Location to store the date read from the fifo
 *  @param[in] len   Amount of data to read out of the fifo
 * 
 *  @return ML_SUCCESS or non-zero error code
**/
tMLError MLDLReadFifo(unsigned char *data, uint_fast16_t len)
{
    INVENSENSE_FUNC_START;
    tMLError result;
    result = MLSLSerialReadFifo(MLSerialGetHandle(),
                                MLDLGetMPUSlaveAddr(), 
                                (unsigned short)len, data);
    if (ML_SUCCESS != result) {
        MPL_LOGE("MLSLSerialReadBurst failed %d\n", result);
        FIFOReset();
        FIFODataHW.fifoError = ML_ERROR_FIFO_READ_DATA;
        return result;
    }
    return result;
}

/**
 *  @brief  Clears the FIFO status and its content. 
 *  @note   Halt the DMP writing into the FIFO for the time 
 *          needed to reset the FIFO.
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError FIFOReset(void)
{
    INVENSENSE_FUNC_START;
    int len = FIFO_HW_SIZE;
    unsigned char fifoBuf[2];
    unsigned char tries = 0;
    unsigned char userCtrlReg;
    tMLError result;
    struct mldl_cfg *mldl_cfg = MLDLGetCfg();

    FIFODataHW.fifoCount = 0;
    if (mldl_cfg->gyro_is_suspended)
        return ML_SUCCESS;
    
    result = MLSLSerialRead(MLSerialGetHandle(), MLDLGetMPUSlaveAddr(), 
                            MPUREG_USER_CTRL, 1, &userCtrlReg);
    ERROR_CHECK(result);

    while(len != 0 && tries < 6) {
        result = 
            MLSLSerialWriteSingle(
                MLSerialGetHandle(), MLDLGetMPUSlaveAddr(), MPUREG_USER_CTRL,
                ((userCtrlReg & (~BIT_FIFO_EN)) | BIT_FIFO_RST));
        ERROR_CHECK(result);
        result = 
            MLSLSerialRead(MLSerialGetHandle(), MLDLGetMPUSlaveAddr(),
                           MPUREG_FIFO_COUNTH, 2, fifoBuf);
        ERROR_CHECK(result);
        len = (unsigned short)fifoBuf[0] * 256 + (unsigned short)fifoBuf[1];
        tries++;
    }

    result = MLSLSerialWriteSingle(MLSerialGetHandle(), MLDLGetMPUSlaveAddr(),
                                   MPUREG_USER_CTRL, userCtrlReg);
    ERROR_CHECK(result);

    return ML_SUCCESS;
}


/**
 * @}
**/
