/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */

/******************************************************************************
 *
 * $Id: mlsupervisor.h 5158 2011-04-07 02:28:54Z mcaramello $
 *
 *****************************************************************************/

#include "mltypes.h"

// The value of getGyroMagSqrd is scaled such the (1 dps)^2 = 2^this_number
// this number must be >=0 and even.
#define GYRO_MAG_SQR_SHIFT 6
// The value of getAccMagSqrd is scaled such that (1g)^2 = 2^this_number
#define ACC_MAG_SQR_SHIFT 16

#define CAL_RUN             0
#define CAL_RESET           1
#define CAL_CHANGED_DATA    2
#define CAL_RESET_TIME      3
#define CAL_ADD_DATA        4
#define CAL_COMBINE         5

#define P_INIT  100000

#define SF_NORMAL           0
#define SF_DISTURBANCE      1
#define SF_FAST_SETTLE      2
#define SF_SLOW_SETTLE      3
#define SF_STARTUP_SETTLE   4
#define SF_UNCALIBRATED     5


typedef struct {
    void (*MLAccelCompassFusion)(double magFB);
    tMLError (*MLGyroBiasSupervisor)(void);
    tMLError (*MLTempCompSupervisor)(unsigned long deltaTime);
    tMLError (*MLProgressiveNoMotionSupervisor)(unsigned long deltaTime);
    tMLError (*MLSensorFusionMagCalAdvanced)(
            double *magFB, unsigned long deltaTime);
    void (*MLResetMagCalAdvanced)(void);
    void (*MLSupervisorReset)(void);
} tMLSupervisorCB;

tMLError MLResetMagCalibration(void);
void MLSensorFusionSupervisorInit(void);
tMLError MLAccelCompassSupervisor(void);
tMLError MLPressureSupervisor(void);




