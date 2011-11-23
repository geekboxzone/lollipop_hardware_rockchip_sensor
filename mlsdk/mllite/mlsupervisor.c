/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */

/******************************************************************************
 *
 * $Id: mlsupervisor.c 5158 2011-04-07 02:28:54Z mcaramello $
 *
 *****************************************************************************/

/**
 *  @defgroup   ML_SUPERVISOR
 *  @brief      Basic sensor fusion supervisor functionalities.
 *  
 *  @{  
 *      @file   mlsupervisor.c
 *      @brief  Basic sensor fusion supervisor functionalities.
 */

#include "ml.h"
#include "mldl.h"
#include "mldl_cfg.h"
#include "mltypes.h"
#include "mlinclude.h"
#include "compass.h"
#include "pressure.h"
#include "dmpKey.h"
#include "dmpDefault.h"
#include "mlstates.h"
#include "mlFIFO.h"
#include "mlFIFOHW.h"
#include "mlMathFunc.h"
#include "mlsupervisor.h"
#include "mlmath.h"

#include "mlsl.h"
#include "mlos.h"

#include <log.h>
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-sup"

static unsigned long lastCompassTime = 0;
static unsigned long polltime = 0;
static int accCount = 0;    
static int compassCalStableCount = 0;
static int compassCalCount = 0;

#define SUPERVISOR_DEBUG 0

tMLSupervisorCB ml_supervisor_cb = {0};

/** 
 *  @brief  This initializes all variables that should be reset on 
 */
void MLSensorFusionSupervisorInit(void)
{
    lastCompassTime = 0;
    polltime = 0;
    mlxData.mlAccState = SF_STARTUP_SETTLE;
    accCount = 0;
    compassCalStableCount = 0;
    compassCalCount = 0;
#ifdef M_HW
    if (CompassGetPresent()) {
        struct mldl_cfg *mldl_cfg = MLDLGetCfg();
        if ( mldl_cfg->pdata->compass.bus == EXT_SLAVE_BUS_SECONDARY ) {
            (void)FIFOSendRawExternal(ML_ALL,ML_16_BIT);
        }
    }
#endif

    if (ml_supervisor_cb.MLSupervisorReset != NULL) {
        ml_supervisor_cb.MLSupervisorReset();
    }
}

static int MLUpdateCompassCalibration3DOF(
            int command, long *data, 
            unsigned long deltaTime)
{
    INVENSENSE_FUNC_START;
    int retValue = ML_SUCCESS;
    static float m[10][10] = {{0}};
    float mInv[10][10] = {{0}};
    float mTmp[10][10] = {{0}};
    static float xTransY[4] = {0};
    float magSqr = 0;    
    float inpData[3]= {0};
    int i, j; 
    int a, b;
    float d;

    switch(command) {
        case CAL_ADD_DATA:   
            inpData[0] = (float)data[0];
            inpData[1] = (float)data[1];
            inpData[2] = (float)data[2];
            m[0][0] += (-2*inpData[0])*(-2*inpData[0]);
            m[0][1] += (-2*inpData[0])*(-2*inpData[1]);
            m[0][2] += (-2*inpData[0])*(-2*inpData[2]);
            m[0][3] += (-2*inpData[0]);
            m[1][0] += (-2*inpData[1])*(-2*inpData[0]);
            m[1][1] += (-2*inpData[1])*(-2*inpData[1]);
            m[1][2] += (-2*inpData[1])*(-2*inpData[2]);
            m[1][3] += (-2*inpData[1]);
            m[2][0] += (-2*inpData[2])*(-2*inpData[0]);
            m[2][1] += (-2*inpData[2])*(-2*inpData[1]);
            m[2][2] += (-2*inpData[2])*(-2*inpData[2]);
            m[2][3] += (-2*inpData[2]);
            m[3][0] += (-2*inpData[0]);
            m[3][1] += (-2*inpData[1]);
            m[3][2] += (-2*inpData[2]);
            m[3][3] += 1.0f;            
            magSqr = inpData[0]*inpData[0] + inpData[1]*inpData[1] + inpData[2]*inpData[2];            
            xTransY[0] += (-2*inpData[0])*magSqr;
            xTransY[1] += (-2*inpData[1])*magSqr;
            xTransY[2] += (-2*inpData[2])*magSqr;
            xTransY[3] += magSqr;                
            break;
        case CAL_RUN:                                            
            a = 4;
            b=a;  
            for(i=0;i<b;i++) {
                for(j=0;j<b;j++) {
                    a=b;
                    matDetInc(&m[0][0],&mTmp[0][0],&a,i,j);
                    mInv[j][i]=SIGNM(i+j)*matDet(&mTmp[0][0],&a);
                }
            }                
            a=b;
            d=matDet(&m[0][0],&a);                 
            if(d==0) {                
                return ML_ERROR;
            }                                
            for (i=0; i<3; i++) {
                float tmp = 0;                
                for (j=0; j<4; j++) {
                    tmp += mInv[j][i]/d*xTransY[j];                    
                }
                mlxData.mlMagTestBias[i] = -(long)(tmp*mlxData.mlMagSens/16384.0f);
            }
            break;
        case CAL_RESET:
            for (i=0; i<4; i++) {
                for (j=0; j<4; j++) {
                    m[i][j] = 0;
                }
                xTransY[i] = 0;
            }                        
        default:
            break;
    }
    return retValue;
}

/**
 * Entry point for Sensor Fusion operations
 * @internal
 * @param magFB magnetormeter FB
 * @param accSF accelerometer SF
 */
static tMLError MLSensorFusionSupervisor(double *magFB, long *accSF, unsigned long deltaTime)
{   
    static long prevCompassBias[3] = {0};    
    static long magMax[3] = {
        -1073741824L, 
        -1073741824L, 
        -1073741824L
    };
    static long magMin[3] = {
        1073741824L, 
        1073741824L, 
        1073741824L
    };
    int gyroMag;
    long accMag;        
    tMLError result;
    int i;
    
    if(ml_supervisor_cb.MLTempCompSupervisor != NULL) {
        ml_supervisor_cb.MLTempCompSupervisor(deltaTime);
    }
    if(ml_supervisor_cb.MLProgressiveNoMotionSupervisor != NULL) {
        ml_supervisor_cb.MLProgressiveNoMotionSupervisor(deltaTime);
    }
    if(ml_supervisor_cb.MLGyroBiasSupervisor != NULL) {
        ml_supervisor_cb.MLGyroBiasSupervisor();
    }

    gyroMag = getGyroMagSqrd() >> GYRO_MAG_SQR_SHIFT;
    accMag  = getAccMagSqrd();

    // Scaling below assumes certain scaling.
#if ACC_MAG_SQR_SHIFT != 16
#error
#endif

    if(ml_supervisor_cb.MLSensorFusionMagCalAdvanced != NULL) {
        result = ml_supervisor_cb.MLSensorFusionMagCalAdvanced(
                    magFB, deltaTime);
        ERROR_CHECK(result);
    } else if (mlParams.biasUpdateFunc & ML_MAG_BIAS_FROM_MOTION) {
        //Most basic compass calibration, used only with lite MPL
        if (mlxData.mlResettingCompass == 1) {
            for (i=0; i<3; i++) {
                magMax[i]= -1073741824L;
                magMin[i]= 1073741824L;
                prevCompassBias[i] = 0;
            }
            compassCalStableCount = 0;
            compassCalCount = 0;
            mlxData.mlResettingCompass = 0;
        }
        if ((gyroMag>400) || (MLGetGyroPresent()==0)) {            
            if (compassCalStableCount<1000) {
                for (i=0; i<3; i++) {
                    if (mlxData.mlMagSensorData[i]>magMax[i]) {
                        magMax[i] = mlxData.mlMagSensorData[i];                        
                    }
                    if (mlxData.mlMagSensorData[i]<magMin[i]) {
                        magMin[i] = mlxData.mlMagSensorData[i];                        
                    }
                }
                MLUpdateCompassCalibration3DOF(CAL_ADD_DATA, mlxData.mlMagSensorData, deltaTime);                
                compassCalStableCount+=deltaTime;                
                for (i=0; i<3; i++) {
                    if (magMax[i]-magMin[i]<(long long)40*1073741824/mlxData.mlMagSens) {
                        compassCalStableCount = 0;
                    }
                }
            } else { 
                if ( compassCalStableCount>=1000) {                    
                    if (MLUpdateCompassCalibration3DOF(CAL_RUN, mlxData.mlMagSensorData, deltaTime)==ML_SUCCESS) {
                        mlxData.mlGotCompassBias = 1;

                        if (mlxData.mlCompassState == SF_UNCALIBRATED)
                            mlxData.mlCompassState = SF_STARTUP_SETTLE;
                        CompassSetBias( mlxData.mlMagTestBias );                        
                    }
                    compassCalCount = 0;
                    compassCalStableCount = 0;                      
                    for (i=0; i<3; i++) {
                        magMax[i]= -1073741824L;
                        magMin[i]= 1073741824L;
                        prevCompassBias[i] = 0;
                    }
                    MLUpdateCompassCalibration3DOF(CAL_RESET, mlxData.mlMagSensorData, deltaTime);
                }
            }
        }
        compassCalCount+=deltaTime;
        if (compassCalCount>20000) {
            compassCalCount = 0;
            compassCalStableCount = 0;              
            for (i=0; i<3; i++) {
                magMax[i]= -1073741824L;
                magMin[i]= 1073741824L;
                prevCompassBias[i] = 0;
            }
            MLUpdateCompassCalibration3DOF(CAL_RESET, mlxData.mlMagSensorData, deltaTime);
        }
    }

    
    if (mlxData.mlAccState != SF_STARTUP_SETTLE) {
        if (((accMag>260000L) || (accMag<6000)) || (gyroMag>2250000L)) {
            mlxData.mlAccState = SF_DISTURBANCE;//No accels, fast swing
            accCount = 0;
        } else {
            if ((gyroMag<400) && (accMag<200000L) && (accMag>26214)
               && ((mlxData.mlAccState == SF_DISTURBANCE) || (mlxData.mlAccState == SF_SLOW_SETTLE))) {
                    accCount += deltaTime;
                    if (accCount > 0) {
                        mlxData.mlAccState = SF_FAST_SETTLE;
                        accCount = 0;                          
                    }
            }
            else {
                if (mlxData.mlAccState==SF_DISTURBANCE) {
                    accCount += deltaTime ;
                    if (accCount > 500) {
                        mlxData.mlAccState = SF_SLOW_SETTLE;
                        accCount = 0;
                    }
                }
                else if (mlxData.mlAccState == SF_SLOW_SETTLE) {
                    accCount += deltaTime ;
                    if (accCount > 1000) {
                        mlxData.mlAccState = SF_NORMAL;
                        accCount = 0;
                    }
                } else if (mlxData.mlAccState == SF_FAST_SETTLE) {
                    accCount += deltaTime ;
                    if (accCount > 250) {
                        mlxData.mlAccState = SF_NORMAL;
                        accCount = 0;
                    }
                }
            }
        }
    }   
    if (mlxData.mlAccState==SF_STARTUP_SETTLE) {        
        accCount += deltaTime;        
        if (accCount>50) {
            *accSF = 1073741824;//Startup settling
            mlxData.mlAccState = SF_NORMAL;
            accCount = 0;            
        } 
    } else if ((mlxData.mlAccState==SF_NORMAL) || (mlxData.mlAccState==SF_SLOW_SETTLE))  {
        *accSF = mlxData.mlAccelSens*64;//Normal
    } else if ((mlxData.mlAccState==SF_DISTURBANCE)) {
        *accSF = mlxData.mlAccelSens*64;//Don't use accel (should be 0)
    } else if (mlxData.mlAccState==SF_FAST_SETTLE) {
        *accSF = mlxData.mlAccelSens*512;//Amplify accel
    }
    if (!MLGetGyroPresent()) {
        *accSF = mlxData.mlAccelSens*128;
    }
    return ML_SUCCESS;
}

/**
 *  @brief  Entry point for software sensor fusion operations.  
 *          Manages hardware interaction, calls sensor fusion supervisor for 
 *          bias calculation.
 *  @return error code. ML_SUCCESS if no error occurred.
 */
tMLError MLAccelCompassSupervisor(void)
{
    tMLError result;
    int adjustSensorFusion = 0;
    long accSF = 1073741824;
    static double magFB = 0;
    long magSensorData[3];    
    if (CompassGetPresent()) { /* check for compass data */
        int i, j;
        long long tmp[3] = { 0 };
        long long tmp64 = 0;
        unsigned long ctime = MLOSGetTickCount();        
        if (( (CompassGetId() == COMPASS_ID_AKM) && ((ctime - polltime)>20)) || (polltime == 0 || ((ctime - polltime) > 80))) { // every 1/12 of a second
            if (SUPERVISOR_DEBUG) {
                MPL_LOGV("Fetch compass data from MLProcessFIFOData\n");
                MPL_LOGV("delta time = %ld\n", ctime - polltime);
            }
            polltime = ctime;
            result = CompassGetData(magSensorData);
            if (result) {
                if (SUPERVISOR_DEBUG) {
                    MPL_LOGW("CompassGetData returned %d\n", result);
                }
            } else {
                unsigned long compassTime = MLOSGetTickCount();
                unsigned long deltaTime = 1;
                if (lastCompassTime != 0) {
                    deltaTime = compassTime - lastCompassTime; 
                }
                lastCompassTime = compassTime;
                adjustSensorFusion = 1;
                if (mlxData.mlGotInitCompassBias==0) {
                    mlxData.mlGotInitCompassBias = 1;
                    for (i=0; i<3; i++) {
                        mlxData.mlInitMagBias[i] = magSensorData[i];
                    }
                }
                for (i = 0; i < 3; i++) {
                    mlxData.mlMagSensorData[i] = (long) magSensorData[i];  
                    mlxData.mlMagSensorData[i] -= mlxData.mlInitMagBias[i];
                    tmp[i] = ((long long) mlxData.mlMagSensorData[i])
                            * mlxData.mlMagSens / 16384;
                    tmp[i] -= mlxData.mlMagBias[i];
                    tmp[i] = (tmp[i] * mlxData.mlMagScale[i]) / 65536L;
                }
                for (i = 0; i < 3; i++) {
                    tmp64 = 0;
                    for (j = 0; j < 3; j++) {
                        tmp64 += (long long) tmp[j] * 
                                 mlxData.mlMagCal[i * 3 + j];
                    }
                    mlxData.mlMagCalibratedData[i] = (long) (tmp64
                            / mlxData.mlMagSens);
                }                
                if (SUPERVISOR_DEBUG) {
                    MPL_LOGI("RM : %+10.6f %+10.6f %+10.6f\n",
                             (float)mlxData.mlMagCalibratedData[0] / 65536.f,
                             (float)mlxData.mlMagCalibratedData[1] / 65536.f,
                             (float)mlxData.mlMagCalibratedData[2] / 65536.f);
                }
                magFB = 1.0;
                adjustSensorFusion = 1;
                result = MLSensorFusionSupervisor(&magFB, &accSF, deltaTime);
                ERROR_CHECK(result);
            }
        }
    } else {
        //No compass, but still modify accel
        unsigned long ctime = MLOSGetTickCount();
        if (polltime == 0 || ((ctime - polltime) > 80)) { // at the beginning AND every 1/8 second            
            unsigned long deltaTime = 1;
            adjustSensorFusion = 1;
            magFB = 0;
            if (polltime != 0) {
                deltaTime = ctime - polltime; 
            }
            MLSensorFusionSupervisor(&magFB, &accSF, deltaTime);
            polltime = ctime;
        }
    }
    if (adjustSensorFusion == 1) {
        unsigned char regs[4];
        static int prevAccSF = 1;

        if (accSF != prevAccSF) {
            regs[0] = (unsigned char) ((accSF >> 24) & 0xff);
            regs[1] = (unsigned char) ((accSF >> 16) & 0xff);
            regs[2] = (unsigned char) ((accSF >> 8) & 0xff);
            regs[3] = (unsigned char) (accSF & 0xff);
            result = MLDLSetMemoryMPU(KEY_D_0_96, 4, regs);
            ERROR_CHECK(result);
            prevAccSF = accSF;
        }
    }

    if(ml_supervisor_cb.MLAccelCompassFusion != NULL)
        ml_supervisor_cb.MLAccelCompassFusion(magFB);

    return ML_SUCCESS;
}

/**
 *  @brief  Entry point for software sensor fusion operations.
 *          Manages hardware interaction, calls sensor fusion supervisor for 
 *          bias calculation.
 *  @return ML_SUCCESS or non-zero error code on error.
 */
tMLError MLPressureSupervisor(void)
{
    long pressureSensorData[1];
    static unsigned long pressurePolltime = 0;
    if (PressureGetPresent()) { /* check for pressure data */
        unsigned long ctime = MLOSGetTickCount();
        if ((pressurePolltime == 0 || ((ctime - pressurePolltime) > 80))) { //every 1/8 second
            if (SUPERVISOR_DEBUG) {
                MPL_LOGV("Fetch pressure data\n");
                MPL_LOGV("delta time = %ld\n", ctime - pressurePolltime);
            }
            pressurePolltime = ctime;            
            if (PressureGetData(&pressureSensorData[0]) == ML_SUCCESS) {
                mlxData.mlPressure = pressureSensorData[0];
            }
        }
    }
    return ML_SUCCESS;
}


/**
 *  @brief  Resets the magnetometer calibration algorithm.
 *  @return ML_SUCCESS if successful, or non-zero error code otherwise.
 */
tMLError MLResetMagCalibration(void)
{
    if (mlParams.biasUpdateFunc & ML_MAG_BIAS_FROM_GYRO) {
        if(ml_supervisor_cb.MLResetMagCalAdvanced != NULL)
            ml_supervisor_cb.MLResetMagCalAdvanced();
    } 
    MLUpdateCompassCalibration3DOF(CAL_RESET, mlxData.mlMagSensorData, 1);
    
    mlxData.mlGotCompassBias = 0;
    mlxData.mlGotInitCompassBias = 0;    
    mlxData.mlCompassState = SF_UNCALIBRATED;
    mlxData.mlResettingCompass = 1;

    return ML_SUCCESS;
}

/**
 *  @}
 */

