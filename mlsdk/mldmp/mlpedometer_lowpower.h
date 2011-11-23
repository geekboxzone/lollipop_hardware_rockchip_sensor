
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

#ifndef MLPEDOMETER_LOWPOWER_H
#define MLPEDOMETER_LOWPOWER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mltypes.h"

    /* ------------ */
    /* - Defines. - */
    /* ------------ */

struct stepParams {
    long threshold;
    long clipThreshold;
    long minEnergy;
    unsigned short minUpTime;
    unsigned short maxUpTime;
    unsigned short minSteps;
    unsigned short maxStepBufferTime;
};

    tMLError MLPedometerStandAloneSetParams(const struct stepParams *params );
    tMLError MLPedometerStandAloneSetStepBuffer(unsigned short minSteps);
    tMLError MLPedometerStandAloneSetStepBufferResetTime(unsigned int timeMs);

/******************************************************************************/
/*  Pedometer Standalone API Features                                         */
/******************************************************************************/

    /* --------------------- */
    /* - Function p-types. - */
    /* --------------------- */

    tMLError MLDmpPedometerStandAloneOpen(void);
    tMLError MLDmpPedometerStandAloneClose(void);
    tMLError MLDmpPedometerStandAloneStart(void);
    tMLError MLDmpPedometerStandAloneStop(void);
    tMLError MLPedometerStandAloneGetNumOfSteps(unsigned long *steps);
    tMLError MLPedometerStandAloneSetNumOfSteps(unsigned long steps);
    tMLError MLPedometerStandAloneGetWalkTime(unsigned long *time);
    tMLError MLPedometerStandAloneSetWalkTime(unsigned long time);

    tMLError MLPedometerStandAloneGetNumOfCalories(unsigned short *calories);
    tMLError MLPedometerStandAloneClearNumOfCalories();
    tMLError MLPedometerStandAloneSetWeight(unsigned short weight);
    tMLError MLPedometerStandAloneSetStrideLength(unsigned short strideLength);

#ifdef __cplusplus
}
#endif

#endif /* PEDOMETER_STAND_ALONE_H */
