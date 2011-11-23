/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */

/******************************************************************************
 *
 * $Id: ml_mputest.c 5125 2011-04-01 22:04:21Z mcaramello $
 *
 *****************************************************************************/

/**
 *  @defgroup MPU_SELF_TEST
 *  @brief  C wrapper to integrate the MPU Self Test wrapper in MPL.
 *          Provides ML name compliant naming and an additional API that
 *          automates the suspension of normal MPL operations, runs the test,
 *          and resume.
 *
 *  @{
 *      @file   ml_mputest.c
 *      @brief  C wrapper to integrate the MPU Self Test wrapper in MPL.
 *              The main logic of the test and APIs can be found in mputest.c
 */

#include <stdio.h>
#include <time.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "ml_mputest.h"

#include "mlmath.h"
#include "mlinclude.h"
#include "ml.h"
#include "mlstates.h"
#include "mldl.h"
#include "mldl_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
    Globals
*/
extern struct mldl_cfg *mputestCfgPtr;
extern signed char g_zSign;

/*
    Prototypes
*/
extern tMLError FactoryCalibrate(void *mlsl_handle);

/**
 *  @brief  An MPL wrapper for the main MPU Self Test API FactoryCalibrate(). 
 *          See FactoryCalibrate() function for more details.
 *
 *  @pre    MLDmpOpen() <b>must</b> have been called to populate the mldl_cfg
 *          data structure.
 *          On Windows, SetupPlatform() is also a madatory pre condition to 
 *          ensure the accelerometer is properly configured before running the 
 *          test.
 *
 *  @param  mlsl_handle
 *              serial interface handle to allow serial communication with the 
 *              device, both gyro and accelerometer.
 *
 *  @return ML_SUCCESS on success or a bitmask error code from the Self Test.
 */
tMLError MLSelfTestFactoryCalibrate(void *mlsl_handle) 
{    
#if M_HW
    MPL_LOGI("%s : SelfTest APIs are unavailable for MPU6000\n", __func__);
    return ML_SUCCESS;
#else
    mputestCfgPtr = MLDLGetCfg();
    return FactoryCalibrate(mlsl_handle);
#endif
}

/**
 *  @brief  Runs the MPU test at MPL runtime.
 *          If the DMP is operating, stops the DMP temporarely,
 *          runs the MPU Self Test, and re-starts the DMP.
 *
 *  @return ML_SUCCESS or first non-zero error code otherwise.
 */
tMLError MLSelfTestRun(void)
{
    INVENSENSE_FUNC_START;
    tMLError firstError = ML_SUCCESS;
#if M_HW
    MPL_LOGI("%s : SelfTest APIs are unavailable for MPU6000\n", __func__);
#else
    tMLError result;
    unsigned char initState = MLGetState();

    if(initState == ML_STATE_DMP_STARTED) {
        result = MLDmpStop();
        ERROR_CHECK_FIRST(firstError, result);
    }

    result = MLSelfTestFactoryCalibrate(MLSerialGetHandle());
    ERROR_CHECK_FIRST(firstError, result);

    if(initState == ML_STATE_DMP_STARTED) {
        result = MLDmpStart();
        ERROR_CHECK_FIRST(firstError, result);
    }
#endif
    return firstError;
}

/**
 *  @brief  Set the orientation of the acceleroemter Z axis as it will be 
 *          expected when running the MPU Self Test.
 *          Specifies the orientation of the accelerometer Z axis : Z axis
 *          pointing upwards or downwards.
 *  @param  zSign
 *              The sign of the accelerometer Z axis; valid values are +1 and
 *              -1 for +Z and -Z respectively.  Any other value will cause the 
 *              setting to be ignored and an error code to be returned.
 *  @return ML_SUCCESS or a non-zero error code.
 */
tMLError MLSelfTestSetAccelZOrient(signed char zSign)
{
#if M_HW
    MPL_LOGI("%s : SelfTest APIs are unavailable for MPU6000\n", __func__);
#else
    if (zSign != +1 && zSign != -1) {
        return ML_ERROR_INVALID_PARAMETER;
    }
    g_zSign = zSign;
#endif
    return ML_SUCCESS;
}


#ifdef __cplusplus
}
#endif

/**
 *  @}
 */

