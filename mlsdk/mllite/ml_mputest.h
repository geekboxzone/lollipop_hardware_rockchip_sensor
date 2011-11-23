/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
 
/******************************************************************************
 *
 * $Id: ml_mputest.h 4894 2011-02-28 23:01:53Z prao $
 *
 *****************************************************************************/

#ifndef _ML_MPUTEST_H_
#define _ML_MPUTEST_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mltypes.h"

/* user APIs */
tMLError MLSelfTestFactoryCalibrate(void *mlsl_handle);
tMLError MLSelfTestSetAccelZOrient(signed char zSign);
tMLError MLSelfTestRun(void);
#define  MLSelfTestSetParameters TestSetParameters

/* other functions */
void TestSetParameters(unsigned int slaveAddr, float sensitivity, 
                       int pThresh, float totalTimeTol,
                       int biasThresh, float rmsThresh, float SPShiftThresh);

#ifdef __cplusplus
}
#endif

#endif /* _ML_MPUTEST_H_ */

