/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: pressure.c 4120 2010-11-21 19:56:16Z mcaramello $
 *
 *******************************************************************************/

/** 
 *  @defgroup PRESSUREDL 
 *  @brief  Motion Library - Pressure Driver Layer.
 *          Provides the interface to setup and handle a pressure sensor
 *          connected to either the primary or the seconday I2C interface 
 *          of the gyroscope.
 *
 *  @{
 *      @file   pressure.c
 *      @brief  Pressure setup and handling methods.
**/

/* ------------------ */
/* - Include Files. - */
/* ------------------ */

#include <string.h>

#include "pressure.h"

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
#define MPL_LOG_TAG "MPL-pressure"

#define _pressureDebug(x) //{x}

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
 *  @brief  Is a pressure configured and used by MPL?
 *  @return ML_SUCCESS if the pressure is present.
 */
unsigned char PressureGetPresent( void ) 
{
    INVENSENSE_FUNC_START;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();
    if (NULL != mldl_cfg->pressure && 
        NULL != mldl_cfg->pressure->resume &&
        mldl_cfg->requested_sensors & ML_THREE_AXIS_PRESSURE)
        return TRUE;
    else 
        return FALSE;
}

/**
 *  @brief   Query the pressure slave address.
 *  @return  The 7-bit pressure slave address.
 */
unsigned char PressureGetSlaveAddr(void)
{
    INVENSENSE_FUNC_START;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();
    if (NULL != mldl_cfg->pdata)
        return mldl_cfg->pdata->pressure.address;
    else 
        return 0;
}


/**
 *  @brief   Get the ID of the pressure in use.
 *  @return  ID of the pressure in use.
 */
unsigned short PressureGetId( void ) 
{
    INVENSENSE_FUNC_START;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();
    if (NULL != mldl_cfg->pressure) {
        return mldl_cfg->pressure->id;
    }
    return ID_INVALID;
}



/**
 *  @brief  Get a sample of pressure data from the device.
 *  @param  data
 *              the buffer to store the pressure raw data for
 *              X, Y, and Z axes.
 *  @return ML_SUCCESS or a non-zero error code.
 */
tMLError PressureGetData(long* data)
{
    tMLError result = ML_SUCCESS;
    unsigned char tmp[3];
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();

    /*--- read the pressure sensor data.
          The pressure read function may return an ML_ERROR_PRESSURE_* errors
          when the data is not ready (read/refresh frequency mismatch) or 
          the internal data sampling timing of the device was not respected. 
          Returning the error code will make the sensor fusion supervisor 
          ignore this pressure data sample. ---*/
    result = (tMLError)mpu3050_read_pressure(mldl_cfg, MLSerialGetHandle(), tmp);
    if (result) {
        _pressureDebug( MPL_LOGV("mpu3050_read_pressure returned %d (%s)\n", result, MLErrorCode(result)) );
        return result;
    }
    if (EXT_SLAVE_BIG_ENDIAN == mldl_cfg->pressure->endian)
        data[0] = (((long)((signed char)tmp[0])) << 16) + (((long)tmp[1]) << 8) + ((long)tmp[2]);
    else
        data[0] = (((long)((signed char)tmp[2])) << 16) + (((long)tmp[1]) << 8) + ((long)tmp[0]);

    return ML_SUCCESS;
}

/**
 *  @}
 */
