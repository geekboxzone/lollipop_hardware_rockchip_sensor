
/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: pedometerStandAlone.h 3863 2010-10-08 22:05:31Z nroyer $
 *
 ******************************************************************************/

#ifndef MLPEDOMETER_FULLPOWER_H
#define MLPEDOMETER_FULLPOWER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mltypes.h"
#include "mlpedometer_lowpower.h"

    /* ------------ */
    /* - Defines. - */
    /* ------------ */

    tMLError MLEnablePedometerFullPower(void);
    tMLError MLDisablePedometerFullPower(void);
    tMLError MLSetPedometerFullPowerStepCount(unsigned long steps);
    tMLError MLGetPedometerFullPowerStepCount(unsigned long *steps);
    tMLError MLSetPedometerFullPowerWalkTime(unsigned long timeMs);
    tMLError MLGetPedometerFullPowerWalkTime(unsigned long *timeMs);
    tMLError MLSetPedometerFullPowerStepCallback(
        void (*func) (unsigned long stepNum, unsigned long walkTimeMs) );

    tMLError MLSetPedometerFullPowerParams(const struct stepParams *params);
    tMLError MLSetPedometerFullPowerStepBuffer(unsigned short minSteps);
    tMLError MLSetPedometerFullPowerStepBufferResetTime(unsigned int timeMs);

#ifdef __cplusplus
}
#endif

#endif /* MLPEDOMETER_FULLPOWER_H */
