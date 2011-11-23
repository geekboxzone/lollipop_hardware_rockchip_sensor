/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
 
/*******************************************************************************
 *
 * $Id:$
 *
 ******************************************************************************/

#ifndef ML_STORED_DATA_H
#define ML_STORED_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

/*
    Includes.
*/

#include "mltypes.h"

/*
    Defines
*/
#define ML_CAL_ACCEL_LEN    (12)
#define ML_CAL_COMPASS_LEN  (555)
#define ML_CAL_HDR_LEN      (6)
#define ML_CAL_CHK_LEN      (4)

/* 
    APIs 
*/
tMLError MLLoadCalibration(void);
tMLError MLStoreCalibration(void);
#define LoadCalibration  MLLoadCalibration
#define StoreCalibration MLStoreCalibration

/*
    Other prototypes
*/
tMLError MLLoadCal(unsigned char *calData);
tMLError MLStoreCal(unsigned char *calData, int length);
tMLError MLGetCalLength(unsigned int *length);

#ifdef __cplusplus
}
#endif

#endif /* ML_STORED_DATA_H */


