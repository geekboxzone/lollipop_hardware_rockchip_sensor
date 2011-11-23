/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */

/******************************************************************************
 *
 * $Id:$
 *
 *****************************************************************************/

#ifndef MLDMP_PROG_NO_MOTION_H__
#define MLDMP_PROG_NO_MOTION_H__

#include "mltypes.h"

#define PROG_NO_MOTION  1
#define PROG_MOTION     2

#ifdef __cplusplus
extern "C" {
#endif

/* APIs */
tMLError MLEnableProgressiveNoMotion(void);
tMLError MLDisableProgressiveNoMotion(void);

/* internal use */
int ProgressiveNoMotionIsEnabled(void);
void ProgressiveNoMotionBiasChanged(void);
int ProgressiveNoMotionGetState(void);

#ifdef __cplusplus
}
#endif


#endif // MLDMP_PROG_NO_MOTION_H__
