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

#ifndef MLDMP_GYROBIAS_H__
#define MLDMP_GYROBIAS_H__

#include "mltypes.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GYROBIAS_TRACKER_TEMPCOMP               (0x01)
#define GYROBIAS_TRACKER_PROGRESSIVENOMOTION    (0x02)

#define GYROBIAS_TRACKER_ALL                    \
            (GYROBIAS_TRACKER_TEMPCOMP | GYROBIAS_TRACKER_PROGRESSIVENOMOTION)


/* internal use */
tMLError MLSetGyroBias(void);
tMLError MLSetGyroBiasReset(void);
tMLError GyroBiasRegisterTracker(int add, unsigned short trackerMask);

#ifdef __cplusplus
}
#endif


#endif // MLDMP_GYROBIAS_H__

