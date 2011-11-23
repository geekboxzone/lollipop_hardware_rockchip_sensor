/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/******************************************************************************
 *
 * $Id: ml.h 5131 2011-04-02 00:34:38Z jbotelho $
 *
 *****************************************************************************/

/** 
 *  @defgroup ML
 *  @brief  The Motion Library processes gyroscopes and accelerometers to 
 *          provide a physical model of the movement of the sensors. 
 *          The results of this processing may be used to control objects 
 *          within a user interface environment, detect gestures, track 3D 
 *          movement for gaming applications, and analyze the blur created 
 *          due to hand movement while taking a picture.
 *  
 *  @{
 *      @file ml.h
 *      @brief Header file for the Motion Library.
**/

#ifndef ML_H
#define ML_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tempComp.h"
#include "mltypes.h"
#include "mldmp.h"
#include "mlsl.h"
#include "mpu.h"

    /* ------------ */
    /* - Defines. - */
    /* ------------ */

    /* - Module defines. - */

    /*************************************************************************/
    /*  Motion Library Vesion                                                */
    /*************************************************************************/

#define ML_VERSION_MAJOR                 3
#define ML_VERSION_MINOR                 4
#define ML_VERSION_SUB_MINOR             0

#define ML_VERSION_MAJOR_STR            "3"
#define ML_VERSION_MINOR_STR            "4"
#define ML_VERSION_SUB_MINOR_STR        "0"

#define ML_VERSION_NONE                 ""
#define ML_VERSION_PROTOTYPE            "ProtoA "
#define ML_VERSION_ENGINEERING          "EngA "
#define ML_VERSION_PRE_ALPHA            "Pre-Alpha "
#define ML_VERSION_ALPHA                "Alpha "
#define ML_VERSION_BETA                 "Beta "
#define ML_VERSION_PRODUCTION           "Prod "

#ifndef ML_VERSION_TYPE
#define ML_VERSION_TYPE ML_VERSION_NONE
#endif

#define ML_VERSION  "InvenSense MPL" " " \
    "v" ML_VERSION_MAJOR_STR "." ML_VERSION_MINOR_STR "." ML_VERSION_SUB_MINOR_STR " " \
    ML_VERSION_TYPE \
    __DATE__ " " __TIME__


    /*************************************************************************/
    /*  Motion processing engines                                            */
    /*************************************************************************/
		
#define ML_MOTION_DETECT				(0x0004)
#define ML_BIAS_UPDATE					(0x0008)
#define ML_GESTURE						(0x0020)
#define ML_CONTROL						(0x0040)
#define ML_ORIENTATION                  (0x0100)
#define ML_PEDOMETER                    (0x0200)
#define ML_BASIC                        (ML_MOTION_DETECT | ML_BIAS_UPDATE)

    /*************************************************************************/
    /*  Data Source - Obsolete                                               */
    /*************************************************************************/

#define ML_DATA_FIFO        (0x1)
#define ML_DATA_POLL        (0x2)

    /*************************************************************************/
    /*  Interrupt Source                                                     */
    /*************************************************************************/
#define ML_INT_MOTION           (0x01)
#define ML_INT_FIFO             (0x02)
#define ML_INT_TAP              (0x04)
#define ML_INT_ORIENTATION      (0x08)
#define ML_INT_SHAKE_PITCH      (0x10)
#define ML_INT_SHAKE_ROLL       (0x20)
#define ML_INT_SHAKE_YAW        (0x40)

    /*************************************************************************/
    /*  Bias update functions                                                */
    /*************************************************************************/

#define ML_BIAS_FROM_NO_MOTION          0x0001
#define ML_BIAS_FROM_GRAVITY            0x0002
#define ML_BIAS_FROM_TEMPERATURE        0x0004
#define ML_BIAS_FROM_LPF                0x0008
#define ML_MAG_BIAS_FROM_MOTION         0x0010
#define ML_MAG_BIAS_FROM_GYRO           0x0020
#define ML_LEARN_BIAS_FROM_TEMPERATURE  0x0040
#define ML_AUTO_RESET_MAG_BIAS          0x0080
#define ML_REJECT_MAG_DISTURBANCE       0x0100
#define ML_PROGRESSIVE_NO_MOTION        0x0200

    /*************************************************************************/
    /*  Euler angles and axis names                                          */
    /*************************************************************************/

#define ML_X_AXIS                       (0x01)
#define ML_Y_AXIS                       (0x02)
#define ML_Z_AXIS                       (0x04)

#define ML_PITCH                        (ML_X_AXIS)
#define ML_ROLL                         (ML_Y_AXIS)
#define ML_YAW                          (ML_Z_AXIS)

    /*************************************************************************/
    /*  Sensor types                                                         */
    /*************************************************************************/

#define ML_GYROS                        0x0001
#define ML_ACCELS                       0x0002

    /*************************************************************************/
    /*  Motion arrays                                                        */
    /*************************************************************************/

#define ML_ROTATION_MATRIX              0x0003
#define ML_QUATERNION                   0x0004
#define ML_EULER_ANGLES                 0x0005
#define ML_LINEAR_ACCELERATION          0x0006
#define ML_LINEAR_ACCELERATION_WORLD    0x0007
#define ML_GRAVITY                      0x0008
#define ML_ANGULAR_VELOCITY             0x0009

#define ML_GYRO_CALIBRATION_MATRIX      0x000B
#define ML_ACCEL_CALIBRATION_MATRIX     0x000C
#define ML_GYRO_BIAS                    0x000D
#define ML_ACCEL_BIAS                   0x000E
#define ML_GYRO_TEMP_SLOPE              0x000F

#define ML_RAW_DATA                     0x0011
#define ML_DMP_TAP                      0x0012
#define ML_DMP_TAP2                     0x0021

#define ML_EULER_ANGLES_X               0x0013
#define ML_EULER_ANGLES_Y               0x0014
#define ML_EULER_ANGLES_Z               0x0015

#define ML_BIAS_UNCERTAINTY             0x0016
#define ML_DMP_PACKET_NUMBER            0x0017
#define ML_FOOTER                       0x0018

#define ML_CONTROL_DATA                 0x0019

#define ML_MAGNETOMETER                 0x001A
#define ML_PEDLBS                       0x001B
#define ML_MAG_RAW_DATA                 0x001C
#define ML_MAG_CALIBRATION_MATRIX       0x001D
#define ML_MAG_BIAS                     0x001E
#define ML_HEADING                      0x001F

#define ML_MAG_BIAS_ERROR               0x0020

#define ML_PRESSURE                     0x0021
#define ML_LOCAL_FIELD                  0x0022
#define ML_MAG_SCALE                    0x0023

#define ML_RELATIVE_QUATERNION          0x0024


#define SET_QUATERNION                                  0x0001
#define SET_GYROS                                       0x0002
#define SET_LINEAR_ACCELERATION                         0x0004
#define SET_GRAVITY                                     0x0008
#define SET_ACCELS                                      0x0010
#define SET_TAP                                         0x0020
#define SET_PEDLBS                                      0x0040
#define SET_LINEAR_ACCELERATION_WORLD                   0x0080
#define SET_CONTROL                                     0x0100
#define SET_PACKET_NUMBER                               0x4000
#define SET_FOOTER                                      0x8000

    /*************************************************************************/
    /*  Integral reset options                                               */
    /*************************************************************************/

#define ML_NO_RESET                     0x0000
#define ML_RESET                        0x0001

    /*************************************************************************/
    /*  Motion states                                                        */
    /*************************************************************************/

#define ML_MOTION                       0x0001
#define ML_NO_MOTION                    0x0002

    /*************************************************************************/
    /* Orientation and Gesture states                                        */
    /*************************************************************************/

#define ML_STATE_IDLE       (0)
#define ML_STATE_RUNNING    (1)

    /*************************************************************************/
    /*  Flags                                                                */
    /*************************************************************************/

#define ML_RAW_DATA_READY               0x0001
#define ML_PROCESSED_DATA_READY         0x0002

#define ML_GOT_GESTURE                  0x0004

#define ML_MOTION_STATE_CHANGE          0x0006

    /*************************************************************************/
    /*  General                                                              */
    /*************************************************************************/

#define ML_NONE              (0x0000)
#define ML_INVALID_FIFO_RATE (0xFFFF)

    /*************************************************************************/
    /*  ML Params Structure Default Values                                   */
    /*************************************************************************/

#define ML_BIAS_UPDATE_FUNC_DEFAULT               ML_BIAS_FROM_NO_MOTION|ML_BIAS_FROM_GRAVITY
#define ML_ORIENTATION_MASK_DEFAULT               0x3f
#define ML_PROCESSED_DATA_CALLBACK_DEFAULT           0
#define ML_ORIENTATION_CALLBACK_DEFAULT              0
#define ML_MOTION_CALLBACK_DEFAULT                   0

    /* ------------ */
    /* - Defines. - */
    /* ------------ */
#define MAX_HIGH_RATE_PROCESSES 8
#define MAX_INTERRUPT_PROCESSES 5
/* Number of quantized accel samples */
#define ML_MAX_NUM_ACCEL_SAMPLES (8)

#define PRECISION 10000.f
#define RANGE_FLOAT_TO_FIXEDPOINT(range, x) {\
    range.mantissa = (long)x; \
    range.fraction = (long)((float)(x-(long)x)*PRECISION); \
}
#define RANGE_FIXEDPOINT_TO_FLOAT(range, x) {\
    x = (float)(range.mantissa); \
    x += ((float)range.fraction/PRECISION); \
}

    /* --------------- */
    /* - Structures. - */
    /* --------------- */

    typedef struct {
        //Sensor data variables
        unsigned short mlEngineMask;

        //Calibration parameters
        long mlBias[6];
        long mlMagBias[3];                
        long mlMagScale[3]; 
        long mlMagTestBias[3];        
        long mlMagTestScale[3];  

        long mlMagBiasError[3];  

        long mlGotNoMotionBias;
        long mlGotCompassBias;
        long mlCompassState;
        long mlLargeField;
        long mlAccState;

        long mlFactoryTempComp;
        long mlGotCoarseHeading;
        long mlGyroTempBias[3];
        long mlNoMotionBias[3];
        long mlProgNoMotionBias[3];

        long mlGyroBiasTest[3];
        long mlAccelCal[9];
        // Deprecated, used mlGyroOrient
        long mlGyroCal[MPU_NUM_AXES * MPU_NUM_AXES];
        long mlGyroOrient[MPU_NUM_AXES * MPU_NUM_AXES];
        long mlAccelSens;
        long mlMagCal[9];        
        long mlGyroSens;
        long mlTempSlope[MPU_NUM_AXES];
        long mlMagSens;
        long mlTempOffset[MPU_NUM_AXES];

        int calLoadedFlag;

        /* temperature compensation */
        float mlXGyroCoeff[3];
        float mlYGyroCoeff[3];
        float mlZGyroCoeff[3];
        float mlXGyroTempData[BINS][PTS_PER_BIN];
        float mlYGyroTempData[BINS][PTS_PER_BIN];
        float mlZGyroTempData[BINS][PTS_PER_BIN];
        float mlTempData[BINS][PTS_PER_BIN];
        int mlTempPtrs[BINS];
        long mlTempValidData[BINS];

        long mlMagCorrection[4];
        long mlMagCorrectionRelative[4];
        long mlMagDisturbCorrection[4];
        long mlMagCorrectionOffset[4];
        long mlRelativeQuat[4];
        long mlLocalField[3];
        long mlNewLocalField;
        long mlGravBodySync[3];
        int mlGyroBiasErr;

        double mlMagBiasP[9];
        double mlMagBiasV[3];
        double mlMagPrevM[36];
        double mlMagPrevXTY[6];

        int mlMagPeaks[18];
        int mlAllSensorsNoMotion;

        long mlInitMagBias[3];

        int mlGotInitCompassBias;
        int mlResettingCompass;

        long mlAngVBody[MPU_NUM_AXES];
        long mlMagSensorData[3];
        long mlMagCalibratedData[3];
        long mlMagTestCalibratedData[3];
        long mlPressure;

        unsigned short mlFlags[7];
        unsigned short suspend;

        long mlNoMotionThreshold;
        unsigned long mlMotionDuration;

        unsigned short mlMotionState;

        unsigned short mlDataMode;
        unsigned short mlInterruptSources;

        unsigned short mlBiasUpdateTime;
        unsigned short mlBiasCalcTime;

        unsigned char internalMotionState;
        long startTime;
        unsigned char saveData[17];

        long accelLPFgain;
        long accelLPF[3];
        unsigned long polltimeNoMotion;
        long mlNoMotionAccelThreshold;
        unsigned long mlNoMotionAccelTime;
        tMLError (*modeChange)(unsigned long, unsigned long);
    } tMLXData;

    typedef tMLError (*tMlxdataFunction)(tMLXData *);

    extern tMLXData mlxData;

    /* --------------------- */
    /* - Params Structure. - */
    /* --------------------- */

    typedef struct {


        unsigned short  biasUpdateFunc;                        // A function or bitwise OR of functions that determine how the gyroscope bias will be automatically updated.
        // Functions include ML_BIAS_FROM_NO_MOTION, ML_BIAS_FROM_GRAVITY, and ML_BIAS_FROM_TEMPERATURE.
        // The engine ML_BIAS_UPDATE must be enabled for these algorithms to run.

        unsigned short  orientationMask;                       // Allows a user to register which orientations will trigger the user defined callback function.
        // The orientations are ML_X_UP, ML_X_DOWN, ML_Y_UP, ML_Y_DOWN, ML_Z_UP, and ML_Z_DOWN.
        // ML_ORIENTATION_ALL is equivalent to ML_X_UP | ML_X_DOWN | ML_Y_UP | ML_Y_DOWN | ML_Z_UP | ML_Z_DOWN.

        void (*processedDataCallback)(void);          // Callback function that triggers when all the processing has been finished by the motion processing engines.

        void (*orientationCallback)(unsigned short orient);    // Callback function that will run when a change of orientation is detected.
        // The new orientation. May be one of ML_X_UP, ML_X_DOWN, ML_Y_UP, ML_Y_DOWN, ML_Z_UP, or ML_Z_DOWN.

        void (*motionCallback)(unsigned short motionState);    // Callback function that will run when a change of motion state is detected.
        // The new motion state. May be one of ML_MOTION, or ML_NO_MOTION.

        unsigned char mlState;

    }   tMLParams,   // new type
        ML_Params_t; // backward-compatibily type

    extern tMLParams mlParams;
    /* --------------------- */
    /* - Function p-types. - */
    /* --------------------- */

    tMLError MLSerialOpen(char const * port);
    tMLError MLSerialClose(void);
    tMLError MLSetMPUSensors(unsigned long sensors);
    void *MLSerialGetHandle(void);

    /*API for handling the buffer*/
    tMLError MLUpdateData( void );

    /*API for handling polling*/
    int MLCheckFlag(int flag);

    /*API for enabling and disabling engines*/
    int MLGetEngines(void);

    /*API for setting bias update function*/
    tMLError MLSetBiasUpdateFunc(unsigned short biasFunction);
#ifdef M_HW
    tMLError MLTurnOnBiasFromNoMotion();
    tMLError MLTurnOffBiasFromNoMotion();
#endif

    /*Functions for handling augmented data*/
    tMLError MLGetArray         (int dataSet, long *data);
    tMLError MLGetFloatArray    (int dataSet, float *data);
    tMLError MLSetArray         (int dataSet, long* data);
    tMLError MLSetFloatArray    (int dataSet, float *data);

    tMLError MLApplyAccelEndian         (void);
    tMLError MLApplyCalibration         (void);
    tMLError MLSetGyroCalibration       (float range, signed char *orientation);
    tMLError MLSetAccelCalibration      (float range, signed char *orientation);
    tMLError MLSetMagCalibration        (float range, signed char *orientation);

    /*API for detecting change of state*/
    tMLError MLSetMotionCallback(void (*func)(unsigned short motionState) );
    int MLGetMotionState(void);

    /*API for getting ML version. */
    tMLError MLVersion(unsigned char **version);

    tMLError MLSetMotionInterrupt(unsigned char on);
    tMLError MLSetFifoInterrupt(unsigned char on);

    int MLGetInterrupts(void);
    tMLError MLSetFIFORate(unsigned short fifoRate);
    unsigned short MLGetFIFORate(void);

    // new
    tMLError MLEnableMotionDetect(void);
    tMLError MLDisableMotionDetect(void);
    
    /* Simulated DMP */
    int MLGetGyroPresent(void);

    tMLError MLSetNoMotionTime(float time);
    tMLError MLSetNoMotionThresh(float thresh);
    tMLError MLSetNoMotionThreshAccel(long thresh);
    tMLError MLResetMotion();

    tMLError MLPollMotionStatus(int newData);
    tMLError MLUpdateBias(void);
    tMLError MLSetDeadZone();
    void MLBiasStart(void);
    void MLBiasStop(void);

    // Private functions shared accross modules
    void MLXInit(void);
    
    tMLError RegisterProcessDmpInterrupt(tMlxdataFunction func);
    tMLError UnRegisterProcessDmpInterrupt(tMlxdataFunction func);
    void RunProcessDmpInterruptFuncs(void);
    void MLSetModeChangeCB( tMLError (*modeChange)(unsigned long, unsigned long) );

#ifdef __cplusplus
}
#endif

#endif // ML_H



  /******************/
 /** @} defgroup  **/
/******************/

