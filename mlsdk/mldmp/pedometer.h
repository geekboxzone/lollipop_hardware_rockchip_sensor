/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: pedometer.h 4682 2011-02-04 22:28:52Z kkeal $
 *
 *******************************************************************************/

#ifndef PEDOMETER_H
#define PEDOMETER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mltypes.h"

    /* ------------ */
    /* - Defines. - */
    /* ------------ */


    /* --------------------- */
    /* - Function p-types. - */
    /* --------------------- */
    tMLError  MLEnablePedometer();
    tMLError  MLDisablePedometer();
    tMLError  MLSetStepCallback(void (*func) (unsigned short stepNum) );
    tMLError  MLGetNumOfSteps(unsigned int *steps);
    tMLError  MLClearNumOfSteps();
    tMLError  MLSetStrideLength(unsigned short strideLength);
    tMLError  MLPedometerSetDataRate( int dataRate );
    tMLError  MLPedometerSetDirection(int dir);

    float PedometerHeadingToVelocity();
    float PedometerHeading();

    /* Internal functions */
    void startPedometerLog();
    void endPedometerLog();

#ifdef __cplusplus
}
#endif

#endif /* PEDOMETER_H */
