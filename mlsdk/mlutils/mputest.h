/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
 
/******************************************************************************
 *
 * $Id: mputest.h 4051 2010-11-19 04:51:58Z mcaramello $
 *
 *****************************************************************************/

#ifndef MPUTEST_H
#define MPUTEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mlsl.h"
#include "mldl_cfg.h"

/* user facing APIs */
tMLError FactoryCalibrate(void *mlsl_handle);
void TestSetupAccel(unsigned char accelId, 
                    unsigned char accelAddr,
                    char zSign);
void TestSetParameters(unsigned int slaveAddr,
                       float sensitivity,
                       int pThresh,
                       float totalTimeTol,
                       int biasThresh,
                       float rmsThresh,
                       float SPShiftThresh);

/* additional functions */
int  MPUTest(void *mlsl_handle);
int  Testyro(void *mlsl_handle, short gyro_biases[3], short *temp_avg);
int  TestAccel(void *mlsl_handle, short accel_biases[3]);


#ifdef __cplusplus
}
#endif

#endif /* MPUTEST_H */

