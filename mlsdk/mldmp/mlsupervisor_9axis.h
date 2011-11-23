/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */

/******************************************************************************
 *
 * $Id: mlsupervisor_9axis.h 5079 2011-03-25 22:30:18Z mcaramello $
 *
 *****************************************************************************/

#ifndef MLDMP_MLSUPERVISOR_H__
#define MLDMP_MLSUPERVISOR_H__

#include "mltypes.h"

#include "tempComp.h"
#include "progressiveNoMotion.h"
#include "gyroBias.h"

#ifdef __cplusplus
extern "C" {
#endif

tMLError MLEnable9axisFusion(void);
tMLError MLDisable9axisFusion(void);
long SetCompassState(long compassState, long accState, unsigned long deltaTime,
                     int magDisturb, int gotBias);

#ifdef __cplusplus
}
#endif

#endif // MLDMP_MLSUPERVISOR_H__
