/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: accel.c 4595 2011-01-25 01:43:03Z mcaramello $
 *
 *******************************************************************************/

/** 
 *  @defgroup ACCELDL 
 *  @brief  Motion Library - Accel Driver Layer.
 *          Provides the interface to setup and handle an accel
 *          connected to either the primary or the seconday I2C interface 
 *          of the gyroscope.
 *
 *  @{
 *      @file   accel.c
 *      @brief  Accel setup and handling methods.
**/

/* ------------------ */
/* - Include Files. - */
/* ------------------ */

#include <string.h>

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
#define MPL_LOG_TAG "MPL-accel"

#define ACCEL_DEBUG 0

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
 *  @brief  Is a accel configured and used by MPL?
 *  @return ML_SUCCESS if the accel is present.
 */
unsigned char AccelGetPresent( void ) 
{
    INVENSENSE_FUNC_START;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();
    if (NULL != mldl_cfg->accel && 
        NULL != mldl_cfg->accel->resume &&
        mldl_cfg->requested_sensors & ML_THREE_AXIS_ACCEL)
        return TRUE;
    else 
        return FALSE;
}

/**
 *  @brief   Query the accel slave address.
 *  @return  The 7-bit accel slave address.
 */
unsigned char AccelGetSlaveAddr(void)
{
    INVENSENSE_FUNC_START;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();
    if (NULL != mldl_cfg->pdata)
        return mldl_cfg->pdata->accel.address;
    else 
        return 0;
}


/**
 *  @brief   Get the ID of the accel in use.
 *  @return  ID of the accel in use.
 */
unsigned short AccelGetId( void ) 
{
    INVENSENSE_FUNC_START;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();
    if (NULL != mldl_cfg->accel) {
        return mldl_cfg->accel->id;
    }
    return ID_INVALID;
}



/**
 *  @brief  Get a sample of accel data from the device.
 *  @param  data
 *              the buffer to store the accel raw data for
 *              X, Y, and Z axes.
 *  @return ML_SUCCESS or a non-zero error code.
 */
tMLError AccelGetData(long *data)
{
    struct mldl_cfg* mldl_cfg = MLDLGetCfg();
    tMLError result;
    unsigned char raw_data[2*ACCEL_NUM_AXES];
    long tmp[ACCEL_NUM_AXES];
    int ii;
    signed char *mtx = mldl_cfg->pdata->accel.orientation;
    char accelId = mldl_cfg->accel->id;

    if (NULL == data)
        return ML_ERROR_INVALID_PARAMETER;

    if (mldl_cfg->accel->len > sizeof(raw_data))
        return ML_ERROR_ASSERTION_FAILURE;

    result = (tMLError)mpu3050_read_accel(mldl_cfg,
                                          MLSerialGetHandle(),
                                          raw_data);
    if (result == ML_ERROR_ACCEL_DATA_NOT_READY) {
        return result;
    }
    ERROR_CHECK(result);
    
    for (ii = 0; ii < DIM(tmp); ii++) {
        if (EXT_SLAVE_LITTLE_ENDIAN == mldl_cfg->accel->endian) {
            tmp[ii]  = (long)  ((signed char)raw_data[2 * ii + 1]) * 256;
            tmp[ii] += (long)((unsigned char)raw_data[2 * ii    ]);
        } else if ((EXT_SLAVE_BIG_ENDIAN == mldl_cfg->accel->endian) ||
                   (EXT_SLAVE_FS16_BIG_ENDIAN == mldl_cfg->accel->endian)) {
            tmp[ii]  = (long)  ((signed char)raw_data[2 * ii    ]) * 256;
            tmp[ii] += (long)((unsigned char)raw_data[2 * ii + 1]);
            if (accelId == ACCEL_ID_KXSD9) {
                tmp[ii] = (long)((short)(((unsigned short)tmp[ii]) 
                                       + ((unsigned short)0x8000)));
            }
        } else if (EXT_SLAVE_FS8_BIG_ENDIAN == mldl_cfg->accel->endian) {
            tmp[ii] = (long)((signed char)raw_data[ii]) * 256;
        } else {
            result = ML_ERROR_FEATURE_NOT_IMPLEMENTED;
        }
    }

    for (ii = 0; ii < DIM(tmp); ii++) {
        data[ii] = ((long)tmp[0] * mtx[3 * ii    ] +
                    (long)tmp[1] * mtx[3 * ii + 1] +
                    (long)tmp[2] * mtx[3 * ii + 2]);
    }

    //MPL_LOGI("ACCEL: %8ld, %8ld, %8ld\n", data[0], data[1], data[2]);
    return result;
}

/**
 *  @}
 */
