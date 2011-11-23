/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: compass.c 5043 2011-03-18 22:08:25Z nroyer $
 *
 *******************************************************************************/

/** 
 *  @defgroup COMPASSDL 
 *  @brief  Motion Library - Compass Driver Layer.
 *          Provides the interface to setup and handle an compass
 *          connected to either the primary or the seconday I2C interface 
 *          of the gyroscope.
 *
 *  @{
 *      @file   compass.c
 *      @brief  Compass setup and handling methods.
**/

/* ------------------ */
/* - Include Files. - */
/* ------------------ */

#include <string.h>

#include "compass.h"

#include "ml.h"
#include "mlinclude.h"
#include "dmpKey.h"
#include "mlFIFO.h"
#include "mldl.h"
#include "mldl_cfg.h"
#include "mlMathFunc.h"
#include "mlsl.h"
#include "mlos.h"

#include "log.h"
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-compass"

#define COMPASS_DEBUG 0

/* --------------------- */
/* - Global Variables. - */
/* --------------------- */

/* --------------------- */
/* - Static Variables. - */
/* --------------------- */

/* --------------- */
/* - Prototypes. - */
/* --------------- */

/* -------------- */
/* - Externs.   - */
/* -------------- */

/* -------------- */
/* - Functions. - */
/* -------------- */

/** 
 *  @brief  Is a compass configured and used by MPL?
 *  @return ML_SUCCESS if the compass is present.
 */
unsigned char CompassGetPresent( void ) 
{
    INVENSENSE_FUNC_START;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();
    if (NULL != mldl_cfg->compass && 
        NULL != mldl_cfg->compass->resume &&
        mldl_cfg->requested_sensors & ML_THREE_AXIS_COMPASS)
        return TRUE;
    else 
        return FALSE;
}

/**
 *  @brief   Query the compass slave address.
 *  @return  The 7-bit compass slave address.
 */
unsigned char CompassGetSlaveAddr(void)
{
    INVENSENSE_FUNC_START;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();
    if (NULL != mldl_cfg->pdata)
        return mldl_cfg->pdata->compass.address;
    else 
        return 0;
}


/**
 *  @brief   Get the ID of the compass in use.
 *  @return  ID of the compass in use.
 */
unsigned short CompassGetId( void ) 
{
    INVENSENSE_FUNC_START;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();
    if (NULL != mldl_cfg->compass) {
        return mldl_cfg->compass->id;
    }
    return ID_INVALID;
}



/**
 *  @brief  Get a sample of compass data from the device.
 *  @param  data
 *              the buffer to store the compass raw data for
 *              X, Y, and Z axes.
 *  @return ML_SUCCESS or a non-zero error code.
 */
tMLError CompassGetData(long* data)
{
    tMLError result;
    int ii;
    unsigned char tmp[6];
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();

#ifndef M_HW
    /*--- read the compass sensor data.
          The compass read function may return an ML_ERROR_COMPASS_* errors
          when the data is not ready (read/refresh frequency mismatch) or 
          the internal data sampling timing of the device was not respected. 
          Returning the error code will make the sensor fusion supervisor 
          ignore this compass data sample. ---*/
    result = (tMLError)mpu3050_read_compass(mldl_cfg, MLSerialGetHandle(), tmp);
    if (result) {
        if (COMPASS_DEBUG) {
            MPL_LOGV("mpu3050_read_compass returned %d\n", result);
        }
        return result;
    }
    for (ii = 0 ; ii < 3 ; ii++) {
        if (EXT_SLAVE_BIG_ENDIAN == mldl_cfg->compass->endian)
            data[ii] = ((long)((signed char)tmp[2*ii]) << 8) + tmp[2*ii+1];
        else
            data[ii] = ((long)((signed char)tmp[2*ii+1]) << 8) + tmp[2*ii];
    }

#else // M_HW

    if ( mldl_cfg->pdata->compass.bus == EXT_SLAVE_BUS_SECONDARY ) {
        result = FIFOGetExternalSensorData(data);
        ERROR_CHECK(result);
        {
            static unsigned char first = TRUE;
            // one-off write to AKM
            if (first) {
                unsigned char regs[] = {
                    MPUREG_I2C_SLV4_ADDR,                      // beginning Mantis register for one-off slave R/W
                    mldl_cfg->pdata->compass.address,          // the slave to write to
                    /*mldl_cfg->compass->trigger->reg*/0x0A,   // the register to write to
                    /*mldl_cfg->compass->trigger->value*/0x01, // the value to write
                    0xC0                                       // enable the write
                };
                result = MLSLSerialWrite(MLSerialGetHandle(), mldl_cfg->addr, DIM(regs), regs);
                first = FALSE;
            }
            else {
                unsigned char regs[] = {
                    MPUREG_I2C_SLV4_CTRL, 
                    0xC0
                };
                result = MLSLSerialWrite(MLSerialGetHandle(), mldl_cfg->addr, DIM(regs), regs);
            }
        }
    } else {
        result = (tMLError)mpu3050_read_compass(mldl_cfg, MLSerialGetHandle(), tmp);
        if (result) {
            if (COMPASS_DEBUG) {
                MPL_LOGV("mpu3050_read_compass returned %d\n", result);
            }
            return result;
        }
        for (ii = 0 ; ii < 3 ; ii++) {
            if (EXT_SLAVE_BIG_ENDIAN == mldl_cfg->compass->endian)
                data[ii] = ((long)((signed char)tmp[2*ii]) << 8) + tmp[2*ii+1];
            else
                data[ii] = ((long)((signed char)tmp[2*ii+1]) << 8) + tmp[2*ii];
        }
    }
#endif // M_HW
    return ML_SUCCESS;
}

/** Sets the compass bias.
* @param bias Compass bias, length 3. Scale is micro Tesla's * 2^16. Frame
*             is mount frame which may be different from body frame.
*/
tMLError CompassSetBias(long *bias)
{
    tMLError result = ML_SUCCESS;
    long biasC[3];
    struct mldl_cfg * mldlCfg = MLDLGetCfg();

    mlxData.mlMagBias[0] = bias[0];
    mlxData.mlMagBias[1] = bias[1];
    mlxData.mlMagBias[2] = bias[2];

    // Find Bias in units of the raw data scaled by 2^16 in chip mounting frame
    biasC[0] = (long)(bias[0]*(1LL<<30)/mlxData.mlMagSens) + mlxData.mlInitMagBias[0]*(1L<<16);
    biasC[1] = (long)(bias[1]*(1LL<<30)/mlxData.mlMagSens) + mlxData.mlInitMagBias[1]*(1L<<16);
    biasC[2] = (long)(bias[2]*(1LL<<30)/mlxData.mlMagSens) + mlxData.mlInitMagBias[2]*(1L<<16);

    if ( DmpFeatureSupported(KEY_CPASS_BIAS_X) ) {
        unsigned char reg[4];
        long biasB[3];
        signed char *orC = mldlCfg->pdata->compass.orientation;

        // Now transform to body frame
        biasB[0] = biasC[0]*orC[0] + biasC[1]*orC[1] + biasC[2]*orC[2];
        biasB[1] = biasC[0]*orC[3] + biasC[1]*orC[4] + biasC[2]*orC[5];
        biasB[2] = biasC[0]*orC[6] + biasC[1]*orC[7] + biasC[2]*orC[8];

        result = MLDLSetMemoryMPU( KEY_CPASS_BIAS_X, 4, Long32ToBig8(biasB[0], reg) );
        result = MLDLSetMemoryMPU( KEY_CPASS_BIAS_Y, 4, Long32ToBig8(biasB[1], reg) );
        result = MLDLSetMemoryMPU( KEY_CPASS_BIAS_Z, 4, Long32ToBig8(biasB[2], reg) );
    }
    return result;
}

tMLError CompassWriteReg(unsigned char reg, unsigned char val)
{
    struct ext_slave_config config;
    unsigned char data[2];
    tMLError result;

    data[0] = reg;
    data[1] = val;

    config.key = MPU_SLAVE_WRITE_REGISTERS;
    config.len = 2;
    config.apply = TRUE;
    config.data = data;

    result = mpu3050_config_compass(MLDLGetCfg(), MLSerialGetHandle(), &config);
    ERROR_CHECK(result);
    return result;
}

tMLError CompassReadReg(unsigned char reg, unsigned char *val)
{
    struct ext_slave_config config;
    unsigned char data[2];
    tMLError result;

    data[0] = reg;

    config.key = MPU_SLAVE_READ_REGISTERS;
    config.len = 2;
    config.apply = TRUE;
    config.data = data;

    result = mpu3050_get_config_compass(MLDLGetCfg(), MLSerialGetHandle(),
                                        &config);
    ERROR_CHECK(result);
    *val = data[1];
    return result;
}

/**
 *  @}
 */
