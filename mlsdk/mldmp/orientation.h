/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
#ifndef MLDMP_ORIENTATION_H__
#define MLDMP_ORIENTATION_H__

#include "mltypes.h"
/*******************************************************************************/
/*    Orientations                                                             */
/*******************************************************************************/

#define ML_X_UP                          0x01
#define ML_X_DOWN                        0x02
#define ML_Y_UP                          0x04
#define ML_Y_DOWN                        0x08
#define ML_Z_UP                          0x10
#define ML_Z_DOWN                        0x20
#define ML_ORIENTATION_ALL               0x3F

#ifdef __cplusplus
extern "C" {
#endif

    tMLError MLEnableOrientation(void);
    tMLError MLDisableOrientation(void);
    tMLError MLSetOrientations(int orientation);
    tMLError MLSetOrientationCallback(void (*callback)(unsigned short));
    tMLError MLGetOrientation(int *orientation);
    tMLError MLGetOrientationState(int * state);
    tMLError MLSetOrientationInterrupt(unsigned char on);
    tMLError MLSetOrientationThreshold(float angle,
                                       float hysteresis,
                                       unsigned long time, 
                                       unsigned int axis);

#ifdef __cplusplus
}
#endif

#endif // MLDMP_ORIENTATION_H__
