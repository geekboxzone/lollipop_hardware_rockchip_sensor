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

#ifndef MLDMP_TEMPCOMP_H__
#define MLDMP_TEMPCOMP_H__

#include "mltypes.h"

/* temperature compensation bins */
#define BINS            (25)
#define PTS_PER_BIN     (5)
#define MIN_TEMP        (-40)
#define MAX_TEMP        (+85)
#define TEMP_PER_BIN    ((MAX_TEMP - MIN_TEMP) / BINS)

#ifdef __cplusplus
extern "C" {
#endif

/* APIs */
tMLError MLEnableTempComp(void);
tMLError MLDisableTempComp(void);

/* internal use */
int   TempCompIsEnabled(void);
int   TempCompHaveSlope(void);
int   TempCompHaveOffset(void);
int   TempCompFindTempBin(float temp);
void  TempCompReset(void);
float TempCompGetTempDifference(void);

#ifdef __cplusplus
}
#endif


#endif // MLDMP_TEMPCOMP_H__
