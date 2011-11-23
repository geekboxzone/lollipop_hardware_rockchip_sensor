/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_NDEBUG 0

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <sys/select.h>
#include <dlfcn.h>
#include <pthread.h>

#include <cutils/log.h>

#include "MPLSensor.h"

/*  *****************************
 *  includes for MPL              */
//#include "sm_version.h"
#include "math.h"
#include "ml.h"
#include "mlFIFO.h"
#include "mlsl.h"
#include "mlos.h"
#include "ml_mputest.h"

extern "C" {
#include "mlsupervisor.h"
}
#include "mlsupervisor_9axis.h"
#include "kernel/mpuirq.h"
#include "gesture.h"
#include "mlcontrol.h"
#include "orientation.h"
//#include "mlpedometer_fullpower.h"
#include "pedometer.h"
#include "mlglyph.h"
#include "mldl_cfg.h"
#include "mldl.h"

#define EXTRA_VERBOSE 0
#define FUNC_LOG LOGV("%s", __PRETTY_FUNCTION__)
#define VFUNC_LOG LOGV_IF(EXTRA_VERBOSE, "%s", __PRETTY_FUNCTION__)
/* this mask must turn on only the sensors that are present and managed by the MPL */
#define ALL_MPL_SENSORS_NP (ML_THREE_AXIS_ACCEL | ML_THREE_AXIS_COMPASS | ML_THREE_AXIS_GYRO)

/* ***************************************************************************
 * MPL interface functions
 */

static int mpu_accuracy;                 //global storage for the current accuracy status
static int new_data = 0;                 //flag indicating that the MPL calculated new output values
static pthread_mutex_t mpld_mutex = PTHREAD_MUTEX_INITIALIZER;
static long master_sensor_mask = ML_ALL_SENSORS;
static long local_sensor_mask = ALL_MPL_SENSORS_NP;
static bool dmp_started = false;
static sensors_event_t* output_gesture_list;
static uint32_t* enabledMask;
static uint32_t* pendingMask;

#define PITCH_SHAKE                  0x01
#define ROLL_SHAKE                   0x02
#define YAW_SHAKE                    0x04
#define TAP                          0x08
#define YAW_IMAGE_ROTATE             0x10
#define ORIENTATION                  0x20
#define MOTION                       0x40
#define GRID_NUM                     0x80
#define GRID_CHANGE                 0x100
#define CONTROL_SIG                 0x200
#define STEP                        0x400
#define SNAP                        0x800

#define GY_ENABLED ((1<<ID_GY) & enabled_sensors)
#define A_ENABLED  ((1<<ID_A)  & enabled_sensors)
#define O_ENABLED  ((1<<ID_O)  & enabled_sensors)
#define M_ENABLED  ((1<<ID_M)  & enabled_sensors)
#define LA_ENABLED ((1<<ID_LA) & enabled_sensors)
#define GR_ENABLED ((1<<ID_GR) & enabled_sensors)
#define RV_ENABLED ((1<<ID_RV) & enabled_sensors)
#define GESTURE_MASK ((1<<7) | (1<<8) |(1<<9) |(1<<10) |(1<<11) |(1<<12) |(1<<13) |(1<<14) |(1<<15) |(1<<16))

extern "C" {
    void initMPL();
    void setupCallbacks();
    void setupFIFO();
    void cb_onMotion(uint16_t);
    void cb_onGesture(gesture_t* gesture);
    void cb_onOrientation(uint16_t orientation);
    void cb_onGridMove(unsigned short ctlSig, long *gNum, long *gChg);
    void cb_onStep(uint16_t);
    //void cb_onStep_fp(unsigned long);
    void cb_procData();
    void set_power_states(int);
    void log_sys_api_addr();
}

void set_power_states(int enabled_sensors)
{
    FUNC_LOG;
    LOGV(" enabled_sensors: %d dmp_started: %d", enabled_sensors, (int)dmp_started);

    do {
        if(enabled_sensors & GESTURE_MASK) {
            local_sensor_mask = ALL_MPL_SENSORS_NP;
            break;
        }

        if (LA_ENABLED || GR_ENABLED || RV_ENABLED || O_ENABLED) {
            local_sensor_mask = ALL_MPL_SENSORS_NP;
            break;
        }

        if (!A_ENABLED && !M_ENABLED && !GY_ENABLED) {
            local_sensor_mask = 0;
            break;
        }

        if (GY_ENABLED) {
            local_sensor_mask |= ML_THREE_AXIS_GYRO;
        } else {
            local_sensor_mask &= ~ML_THREE_AXIS_GYRO;
        }

        if (A_ENABLED) {
            local_sensor_mask |= ML_THREE_AXIS_ACCEL;
        } else {
            local_sensor_mask &= ~(ML_THREE_AXIS_ACCEL);
        }

        if (M_ENABLED) {
            local_sensor_mask |= ML_THREE_AXIS_COMPASS;
        } else {
            local_sensor_mask &= ~(ML_THREE_AXIS_COMPASS);
        }
    } while (0);

    //record the new sensor state
    //   ToDo: only call MLSetSensors if there is something to change
    LOGV("MLSetMPUSensors: %lx", (local_sensor_mask & master_sensor_mask));
    tMLError rv = MLSetMPUSensors(local_sensor_mask & master_sensor_mask);
    LOGE_IF(rv != ML_SUCCESS, "error: unable to set MPL sensor power states (sens=%ld retcode = %d)", local_sensor_mask & master_sensor_mask, rv);

    //check if we should start or stop the DMP
    if (!dmp_started && (local_sensor_mask & master_sensor_mask) != 0) {
        LOGV("Starting DMP");
        tMLError r = MLDmpStart();
        LOGE_IF(r != ML_SUCCESS, "error: unable to start DMP (retcode = %d)", r);
        if (r==ML_SUCCESS)
            dmp_started = true;
    } else if (dmp_started && (local_sensor_mask & master_sensor_mask) == 0) {
        LOGV("Stopping DMP");
        rv = MLDmpStop();
        LOGE_IF(rv != ML_SUCCESS, "error: unable to stop DMP (retcode = %d)", rv);
        if(rv==ML_SUCCESS)
            dmp_started = false;
    }
}

/** setup the pedometer engine.
 *  this function should only be called when the mpld thread holds the mpld_mutex
 */
void setupPed()
{
    tMLError result;

    result = MLEnablePedometer();
    if (result != ML_SUCCESS) {
        LOGE("Fatal error: MLEnablePedometer returned %d\n",result);
    }

    result = MLPedometerSetDataRate(6);
    if (result != ML_SUCCESS) {
        LOGE("Fatal error: MLPedometerSetDataRate returned %d\n",result);
    }

    result = MLClearNumOfSteps();
    if(result != ML_SUCCESS) LOGE("MLClearNumOfSteps fail (%d)", result);

    result = MLSetStepCallback( cb_onStep );
    if (result != ML_SUCCESS) {
        LOGE("Fatal error: MLSetStepCallback returned %d\n",result);
    }
}
/*
void setupPed_fp()
{
    tMLError result;
    struct stepParams params;

    params.threshold     = 25000000L;
    params.minUpTime     = 16; // 1 / (0.02 * 10) = 5 Hz maximum
    params.maxUpTime     = 60; // 1 / (0.02 * 25) = 0.83 Hz minimum
    params.minSteps      = 5;
    params.minEnergy     = 0x0d000000L;
    params.maxStepBufferTime = 125;
    params.clipThreshold = 0x06000000L;

    result = MLEnablePedometerFullPower();
    if (result != ML_SUCCESS) {
        LOGD("Fatal error: MLEnablePedometerFullPower returned %d\n",result);
    }

    result = MLSetPedometerFullPowerParams(&params);
    if (result != ML_SUCCESS) {
        LOGD("Fatal error: MLEnablePedometerFullPower returned %d\n",result);
    }

    result = MLSetPedometerFullPowerStepCallback( cb_onStep_fp );
    if (result != ML_SUCCESS) {
        LOGD("Fatal error: MLSetStepCallback returned %d\n",result);
    }
}
*/

/* struct used to store gesture parameters. copied from MPL apps */
struct gparams {
    /* Tap Params */
    int xTapThreshold;
    int yTapThreshold;
    int zTapThreshold;
    int tapTime;
    int nextTapTime;
    int maxTaps;
    float shakeRejectValue;

    /* Shake Params */
    int xShakeThresh;
    int yShakeThresh;
    int zShakeThresh;
    int xSnapThresh;
    int ySnapThresh;
    int zSnapThresh;
    int shakeTime;
    int nextShakeTime;
    int shakeFunction;
    int maxShakes;

    /* Yaw rotate params */
    int yawRotateTime;
    int yawRotateThreshold;
};

struct gparams params = {
    xTapThreshold:  100,
    yTapThreshold:  100,
    zTapThreshold:  100,
    tapTime:  100,
    nextTapTime:  600,
    maxTaps:  2,
    shakeRejectValue:  0.8f,
    xShakeThresh:  750,
    yShakeThresh:  750,
    zShakeThresh:  750,
    xSnapThresh:  160,
    ySnapThresh:  320,
    zSnapThresh:  160,
    shakeTime:  400,
    nextShakeTime:  1000,
    shakeFunction:  0,
    maxShakes:  3,
    yawRotateTime:  150,
    yawRotateThreshold:  60,
};

/**
 * apply the parameters stored in the params struct and enable other 'gesture' data
 * (control, etc.)
 */
void setupGestures() {
    tMLError result;
    result = MLEnableGesture();
    if (result != ML_SUCCESS) {
        LOGE("Fatal error: MLEnableGesture returned %d\n",result);
    }
    result = MLSetGestureCallback(cb_onGesture);
    if (result != ML_SUCCESS) {
        LOGE("Fatal error: MLSetGestureCallback returned %d\n",result);
    }
    result = MLSetGestures(ML_GESTURE_ALL);
    if (result != ML_SUCCESS) {
        LOGE("Fatal error: MLSetGestures returned %d\n",result);
    }

    result = MLSetTapThreshByAxis(ML_TAP_AXIS_X, params.xTapThreshold);
    if (result != ML_SUCCESS) {
        LOGE("MLSetTapThreshByAxis returned :%d\n",result);
    }
    result = MLSetTapThreshByAxis(ML_TAP_AXIS_Y, params.yTapThreshold);
    if (result != ML_SUCCESS) {
        LOGE("MLSetTapThreshByAxis returned :%d\n",result);
    }
    result = MLSetTapThreshByAxis(ML_TAP_AXIS_Z, params.zTapThreshold);
    if (result != ML_SUCCESS) {
        LOGE("MLSetTapThreshByAxis returned :%d\n",result);
    }
    result = MLSetTapTime(params.tapTime);
    if (result != ML_SUCCESS) {
        LOGE("MLSetTapTime returned :%d\n",result);
    }
    result = MLSetNextTapTime(params.nextTapTime);
    if (result != ML_SUCCESS) {
        LOGE("MLSetNextTapTime returned :%d\n",result);
    }
    result = MLSetMaxTaps(params.maxTaps);
    if (result != ML_SUCCESS) {
        LOGE("MLSetMaxTaps returned :%d\n",result);
    }
    result = MLSetTapShakeReject(params.shakeRejectValue);
    if (result != ML_SUCCESS) {
        LOGE("MLSetTapShakeReject returned :%d\n",result);
    }

    //Set up shake gesture
    result = MLSetShakeFunc(params.shakeFunction);
    if (result != ML_SUCCESS) {
        LOGE("MLSetShakeFunc returned :%d\n",result);
    }
    result = MLSetShakeThresh(ML_ROLL_SHAKE, params.xShakeThresh);
    if (result != ML_SUCCESS) {
        LOGE("MLSetShakeThresh returned :%d\n",result);
    }
    result = MLSetShakeThresh(ML_PITCH_SHAKE, params.yShakeThresh);
    if (result != ML_SUCCESS) {
        LOGE("MLSetShakeThresh returned :%d\n",result);
    }
    result = MLSetShakeThresh(ML_YAW_SHAKE, params.zShakeThresh);
    if (result != ML_SUCCESS) {
        LOGE("MLSetShakeThresh returned :%d\n",result);
    }
    result = MLSetShakeTime(params.shakeTime);
    if (result != ML_SUCCESS) {
        LOGE("MLSetShakeTime returned :%d\n",result);
    }
    result = MLSetNextShakeTime(params.nextShakeTime);
    if (result != ML_SUCCESS) {
        LOGE("MLSetNextShakeTime returned :%d\n",result);
    }
    result = MLSetMaxShakes(ML_SHAKE_ALL,params.maxShakes);
    if (result != ML_SUCCESS) {
        LOGE("MLSetMaxShakes returned :%d\n",result);
    }

    // Yaw rotate settings
    result = MLSetYawRotateTime(params.yawRotateTime);
    if (result != ML_SUCCESS) {
        LOGE("MLSetYawRotateTime returned :%d\n",result);
    }
    result = MLSetYawRotateThresh(params.yawRotateThreshold);
    if (result != ML_SUCCESS) {
        LOGE("MLSetYawRotateThresh returned :%d\n",result);
    }

#define CHK_RES {if(result != ML_SUCCESS) LOGE("unexpected mpl failure %d at line %d",result, __LINE__);}
    //Orientation is lumped in with gestures
    result = MLEnableOrientation(); CHK_RES;
    result = MLSetOrientations(ML_ORIENTATION_ALL);CHK_RES;
    result = MLSetOrientationCallback(cb_onOrientation);CHK_RES;

    //Control is also a 'gesture'
    result = MLSetGridCallback(cb_onGridMove);CHK_RES;

    result = MLSetControlData(ML_CONTROL_1, ML_ANGULAR_VELOCITY, ML_ROLL);CHK_RES;
    result = MLSetControlData(ML_CONTROL_2, ML_ANGULAR_VELOCITY, ML_PITCH);CHK_RES;
    result = MLSetControlData(ML_CONTROL_3, ML_ANGULAR_VELOCITY, ML_PITCH);CHK_RES;
    result = MLSetControlData(ML_CONTROL_4, ML_ANGULAR_VELOCITY, ML_YAW);CHK_RES;
    result = MLSetControlSensitivity(ML_CONTROL_1, 50);CHK_RES;
    result = MLSetControlSensitivity(ML_CONTROL_2, 50);CHK_RES;
    result = MLSetControlSensitivity(ML_CONTROL_3, 150);CHK_RES;
    result = MLSetControlSensitivity(ML_CONTROL_4, 150);CHK_RES;
    result = MLSetGridThresh(ML_CONTROL_1, 100000);CHK_RES;
    result = MLSetGridThresh(ML_CONTROL_2, 100000);CHK_RES;
    result = MLSetGridThresh(ML_CONTROL_3, 1000);CHK_RES;
    result = MLSetGridThresh(ML_CONTROL_4, 1000);CHK_RES;

    result = MLSetGridMax(ML_CONTROL_1, 120);CHK_RES;
    result = MLSetGridMax(ML_CONTROL_2, 120);CHK_RES;
    result = MLSetGridMax(ML_CONTROL_3, 3);CHK_RES;
    result = MLSetGridMax(ML_CONTROL_4, 3);CHK_RES;

    result = MLSetControlFunc( ML_GRID | ML_HYSTERESIS);CHK_RES;

    // Enable Control.
    result = MLEnableControl();CHK_RES;
    FIFOSendControlData(ML_ALL,ML_32_BIT); //enable control should do this
}

void setupGlyph()
{
    tMLError result;

    result = MLSetControlData(ML_CONTROL_3, ML_ANGULAR_VELOCITY, ML_PITCH); CHK_RES;
    result = MLSetControlData(ML_CONTROL_4, ML_ANGULAR_VELOCITY, ML_YAW); CHK_RES;
    result = MLSetControlSensitivity(ML_CONTROL_3, 150); CHK_RES;
    result = MLSetControlSensitivity(ML_CONTROL_4, 150); CHK_RES;
    result = MLSetGridThresh(ML_CONTROL_3, 1000); CHK_RES;
    result = MLSetGridThresh(ML_CONTROL_4, 1000); CHK_RES;
    result = MLSetGridMax(ML_CONTROL_3, 3); CHK_RES;
    result = MLSetGridMax(ML_CONTROL_4, 3); CHK_RES;
    result = MLSetControlFunc(  ML_HYSTERESIS | ML_GRID ); CHK_RES;

    //Set up glyph recognition
    result = MLEnableGlyph(); CHK_RES;
    result = MLSetGlyphSpeedThresh(10); CHK_RES;
    result = MLSetGlyphProbThresh(64); CHK_RES;

}

/**
 * utility function for putting mpl data into our local data struct
 */
void store_gest(sensors_event_t* dest, gesture_t* src) {
    dest->data[0]   = src->type;
    dest->data[1]   = src->strength;
    dest->data[2]   = src->speed;
    dest->data[3]   = src->num;
    dest->data[4]   = src->meta;
    dest->data[5]   = src->reserved;
}

/**
 * handle gesture data returned from the MPL.
 */
void cb_onGesture(gesture_t* gesture)
{
    LOGD("gesture callback (type: %d)", gesture->type);

    switch (gesture->type)
    {
    case ML_TAP:
    {
        if(*enabledMask & (1<<MPLSensor::Tap)) {
            store_gest(&(output_gesture_list[MPLSensor::Tap]) ,gesture);
            *pendingMask |= (1<<MPLSensor::Tap);
            LOGD("stored TAP");
        }
    }
    break;
    case ML_ROLL_SHAKE:
    case ML_PITCH_SHAKE:
    case ML_YAW_SHAKE:
    {
        if(gesture->strength == 1 && gesture->num == 1) {
            if(*enabledMask & (1<<MPLSensor::Snap)) {
                store_gest(&(output_gesture_list[MPLSensor::Snap]), gesture);
                *pendingMask |= (1<<MPLSensor::Snap);
            }

        }
        if(*enabledMask & (1<<MPLSensor::Shake)) {
            store_gest(&(output_gesture_list[MPLSensor::Shake]), gesture);
            *pendingMask |= (1<<MPLSensor::Shake);
            LOGD("stored SHAKE");
        } else {
            LOGD("SHAKE ignored");
        }

    }
    break;
    case ML_YAW_IMAGE_ROTATE:
    {
        if(*enabledMask & (1<<MPLSensor::YawIR)) {
            store_gest(&(output_gesture_list[MPLSensor::YawIR]),gesture);
            *pendingMask |= (1<<MPLSensor::YawIR);
        }
    }
    break;
    default:
        LOGE("Unknown Gesture received\n");
        break;
    }

}
/**
 * handle Orientation output from MPL
 */
void cb_onOrientation(uint16_t orientation) {
    if(*enabledMask & (1<<MPLSensor::Orient6)) {
         output_gesture_list[MPLSensor::Orient6].data[0] = ORIENTATION;
         output_gesture_list[MPLSensor::Orient6].data[1] = orientation;
         *pendingMask |= (1<<MPLSensor::Orient6);
         LOGD("stored ORIENTATION");
     }
}

/**
 * handle control events and generate associated output gestures
 */
void cb_onGridMove(unsigned short ctlSig, long *gNum, long *gChg) {
    //LOGD("%s", __FUNCTION__);
    int cs[4];
    int c[4];
    int d[4];

    //MLGetControlData(cs, d, c);

    if(*enabledMask & (1<<MPLSensor::CtrlSig)) {
        output_gesture_list[MPLSensor::CtrlSig].data[0] = CONTROL_SIG;
        output_gesture_list[MPLSensor::CtrlSig].data[1] = ctlSig;//cs[0];
        *pendingMask |= (1<<MPLSensor::CtrlSig);
        //LOGD("stored CONTROL_SIG");
    }

    if(*enabledMask & (1<<MPLSensor::GridNum)) {
        output_gesture_list[MPLSensor::GridNum].data[0] = GRID_NUM;
        output_gesture_list[MPLSensor::GridNum].data[1] = gNum[0]; //d[0];
        output_gesture_list[MPLSensor::GridNum].data[2] = gNum[1]; //d[1];
        output_gesture_list[MPLSensor::GridNum].data[3] = gNum[2]; //d[2];
        output_gesture_list[MPLSensor::GridNum].data[4] = gNum[3]; //d[3];
        *pendingMask |= (1<<MPLSensor::GridNum);
        //LOGD("stored GRID_NUM");
    }

    if(*enabledMask & (1<<MPLSensor::GridDelta)) {
        output_gesture_list[MPLSensor::GridDelta].data[0] = GRID_CHANGE;
        output_gesture_list[MPLSensor::GridDelta].data[1] = gChg[0]; //c[0];
        output_gesture_list[MPLSensor::GridDelta].data[2] = gChg[1]; //c[1];
        output_gesture_list[MPLSensor::GridDelta].data[3] = gChg[2]; //c[2];
        output_gesture_list[MPLSensor::GridDelta].data[4] = gChg[3]; //c[3];
        *pendingMask |= (1<<MPLSensor::GridDelta);
        //LOGD("stored GRID_CHANGE");
    }
}

void cb_onStep(uint16_t val)
{
    if(*enabledMask & (1<<MPLSensor::Step)) {
        output_gesture_list[MPLSensor::Step].data[0] = STEP;
        output_gesture_list[MPLSensor::Step].data[1] = val;
        *pendingMask |= (1<<MPLSensor::Step);
        LOGD("stored STEP");
    }
}

/*
void cb_onStep_fp(unsigned long val)
{
    if(output_gesture_list[8].enabled) {
        output_gesture_list[8].gesture_data.type = STEP;
        output_gesture_list[8].gesture_data.data[0] = val;
        output_gesture_list[8].valid = 1;
        LOGD("stored STEP");
    }
}
*/

/**
 * container function for all the calls we make once to set up the MPL.
 */
void initMPL()
{
    FUNC_LOG;
    char *port = NULL;
    tMLError result;

    if (MLSerialOpen(port) != ML_SUCCESS) {
        LOGE("Fatal Error : could not open MPL serial interface");
    }

    if (MLDmpOpen() != ML_SUCCESS) {
        LOGE("Fatal Error : could not open DMP correctly.\n");
    }

    if (MLEnable9axisFusion() != ML_SUCCESS) {
        LOGE("Warning : 9 axis sensor fusion not available - No compass detected.\n");
    }

    if (MLSetBiasUpdateFunc(0xFFFF) != ML_SUCCESS) {
        LOGE("Error : Bias update function could not be set.\n");
    }

    if (MLSetMotionInterrupt(1) != ML_SUCCESS) {
        LOGE("Error : could not set motion interrupt");
    }

    if (MLSetFifoInterrupt(1) != ML_SUCCESS) {
        LOGE("Error : could not set fifo interrupt");
    }

    result = MLSetFIFORate(6);
    if (result != ML_SUCCESS) {
        LOGE("Fatal error: MLSetFIFORate returned %d\n",result);
    }

    mpu_accuracy = SENSOR_STATUS_ACCURACY_MEDIUM;
    setupCallbacks();
    setupGestures();
    setupPed();
    setupGlyph();
}

/** setup the fifo contents.
 */
void setupFIFO()
{
    FUNC_LOG;
    tMLError result;

    result = FIFOSendQuaternion(ML_32_BIT);
    if (result != ML_SUCCESS) {
        LOGE("Fatal error: FIFOSendQuaternion returned %d\n",result);
    }

    result = FIFOSendLinearAccel(ML_ALL, ML_32_BIT);
    if (result != ML_SUCCESS) {
        LOGE("Fatal error: FIFOSendLinearAccel returned %d\n",result);
    }

    result = FIFOSendGravity(ML_ALL, ML_32_BIT);
    if (result != ML_SUCCESS) {
        LOGE("Fatal error: FIFOSendGravity returned %d\n",result);
    }

    result = FIFOSendGyro(ML_ALL, ML_32_BIT);
    if (result != ML_SUCCESS) {
        LOGE("Fatal error: FIFOSendGyro returned %d\n",result);
    }

    result = FIFOSendAccel(ML_ALL, ML_32_BIT);
    if (result != ML_SUCCESS) {
        LOGE("Fatal error: FIFOSendAccel returned %d\n",result);
    }

}


/**
 *  set up the callbacks that we use in all cases (outside of gestures, etc)
 */
void setupCallbacks()
{
    FUNC_LOG;
    if (MLSetMotionCallback(cb_onMotion) != ML_SUCCESS) {
        LOGE("Error : Motion callback could not be set.\n");

    }

    if (MLSetProcessedDataCallback(cb_procData) != ML_SUCCESS) {
        LOGE("Error : Processed data callback could not be set.");

    }
}

extern "C" { //seems to make the cb work, copied from Ameer's work
/**
 * handle the motion/no motion output from the MPL.
 */
void cb_onMotion(uint16_t val)
{
    FUNC_LOG;
    //after the first no motion, the gyro should be calibrated well
    if (val == 2) {
        mpu_accuracy = SENSOR_STATUS_ACCURACY_HIGH;
    }

    return;
}
}
static int sampleCount = 0;

void cb_procData()
{
    new_data = 1;
    sampleCount++;
    LOGV_IF(EXTRA_VERBOSE, "new data (%d)", sampleCount);
}

//these handlers transform mpl data into one of the Android sensor types
//  scaling and coordinate transforms should be done in the handlers

void gyro_handler(sensors_event_t* s, uint32_t* pending_mask, int index)
{
    VFUNC_LOG;
    tMLError res;
    res = MLGetFloatArray(ML_GYROS, s->gyro.v);
    s->gyro.v[0] = s->gyro.v[0] * M_PI / 180.0;
    s->gyro.v[1] = s->gyro.v[1] * M_PI / 180.0;
    s->gyro.v[2] = s->gyro.v[2] * M_PI / 180.0;
    s->gyro.status = mpu_accuracy;
    if (res == ML_SUCCESS)
        *pending_mask |= (1 << index);
}

void accel_handler(sensors_event_t* s, uint32_t* pending_mask, int index)
{
    VFUNC_LOG;
    tMLError res;
    res = MLGetFloatArray(ML_ACCELS, s->acceleration.v);
    s->acceleration.v[0] = s->acceleration.v[0] * 9.81;
    s->acceleration.v[1] = s->acceleration.v[1] * 9.81;
    s->acceleration.v[2] = s->acceleration.v[2] * 9.81;
    s->acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;
    if (res == ML_SUCCESS)
        *pending_mask |= (1 << index);
}

void compass_handler(sensors_event_t* s, uint32_t* pending_mask, int index)
{
    VFUNC_LOG;
    tMLError res, res2;
    float bias_error[3];
    float total_be;
    static int bias_error_settled = 0;

    res = MLGetFloatArray(ML_MAGNETOMETER, s->magnetic.v);

    if (res != ML_SUCCESS) {
        LOGD("compass_handler MLGetFloatArray(ML_MAGNETOMETER) returned %d", res);
    }

    if (1) { //!bias_error_settled) {
        res2 = MLGetFloatArray(ML_MAG_BIAS_ERROR, bias_error);

        if (res2 == ML_SUCCESS) {
            //use total of bias errors to estimate sensor accuracy
            total_be = bias_error[0] + bias_error[1] + bias_error[2];
            if (total_be > 2700.0)
                s->magnetic.status = SENSOR_STATUS_UNRELIABLE;
            else if (total_be > 1500.0)
                s->magnetic.status = SENSOR_STATUS_ACCURACY_LOW;
            else if (total_be > 300.0)
                s->magnetic.status = SENSOR_STATUS_ACCURACY_MEDIUM;
            else {
                s->magnetic.status = SENSOR_STATUS_ACCURACY_HIGH;
                bias_error_settled = 1;
            }
        } else {
            LOGE("could not get mag bias error");
            s->magnetic.status = SENSOR_STATUS_ACCURACY_HIGH;
        }
    } else {
        s->magnetic.status = SENSOR_STATUS_ACCURACY_HIGH;
    }

    if (res == ML_SUCCESS)
        *pending_mask |= (1 << index);
}

void rv_handler(sensors_event_t* s, uint32_t* pending_mask, int index)
{
    VFUNC_LOG;
    float quat[4];
    float norm = 0;
    float ang = 0;
    tMLError r;

    r = MLGetFloatArray(ML_QUATERNION, quat);

    if (r != ML_SUCCESS) {
        *pending_mask &= ~(1 << index);
        return;
    } else {
        *pending_mask |= (1 << index);
    }

    if (quat[0] < 0.0) {
        quat[1] = -quat[1];
        quat[2] = -quat[2];
        quat[3] = -quat[3];
    }

    s->gyro.v[0] = quat[1];
    s->gyro.v[1] = quat[2];
    s->gyro.v[2] = quat[3];

    s->gyro.status = mpu_accuracy;
}

void la_handler(sensors_event_t* s, uint32_t* pending_mask, int index)
{
    VFUNC_LOG;
    tMLError res;
    res = MLGetFloatArray(ML_LINEAR_ACCELERATION, s->gyro.v);
    s->gyro.v[0] *= 9.81;
    s->gyro.v[1] *= 9.81;
    s->gyro.v[2] *= 9.81;
    s->gyro.status = mpu_accuracy;
    if (res == ML_SUCCESS)
        *pending_mask |= (1 << index);
}

void grav_handler(sensors_event_t* s, uint32_t* pending_mask, int index)
{
    VFUNC_LOG;
    tMLError res;
    res = MLGetFloatArray(ML_GRAVITY, s->gyro.v);
    s->gyro.v[0] *= 9.81;
    s->gyro.v[1] *= 9.81;
    s->gyro.v[2] *= 9.81;
    s->gyro.status = mpu_accuracy;
    if (res == ML_SUCCESS)
        *pending_mask |= (1 << index);
}

//pick up the ComputeAndOrientation function from the mlsdk
#include "mlsdk/mlutils/and_orient_helper.c"

void orien_handler(sensors_event_t* s, uint32_t* pending_mask, int index) //note that this is the handler for the android 'orientation' sensor, not the mpl orientation output
{
    VFUNC_LOG;
    tMLError r1, r2;
    float euler[3];
    float heading[1];

    r1 = MLGetFloatArray(ML_EULER_ANGLES, euler);
    r2 = MLGetFloatArray(ML_HEADING, heading);

    ComputeAndOrientation(heading[0], euler, s->orientation.v);
    s->orientation.status = mpu_accuracy;

    if ((r1 == ML_SUCCESS) && (r2 == ML_SUCCESS))
        *pending_mask |= (1 << index);
    else
        LOGD("orien_handler: data not valid (%d %d)", (int)r1, (int)r2);

}

void noop_handler(sensors_event_t* s, uint32_t* pending_mask, int index) {
    return;
}

/*****************************************************************************/
/* sensor class implementation
 */

MPLSensor::MPLSensor() :
    SensorBase(NULL, NULL),
    mEnabled(0), mPendingMask(0)
{
    FUNC_LOG;
    tMLError rv;
    int mpu_int_fd;

    log_sys_api_addr();

    pthread_mutex_lock(&mpld_mutex);

    mpu_int_fd = open("/dev/mpuirq", O_RDWR);
    if (mpu_int_fd == -1) {
        LOGE("could not open the mpu irq device node");
    }

    accel_fd = open("/dev/accelirq", O_RDWR);
    if(accel_fd == -1) {
        LOGE("could not open the accel irq device node");
    }

    data_fd = mpu_int_fd;

    memset(mPendingEvents, 0, sizeof(mPendingEvents));

    mPendingEvents[RotationVector].version = sizeof(sensors_event_t);
    mPendingEvents[RotationVector].sensor = ID_RV;
    mPendingEvents[RotationVector].type = SENSOR_TYPE_ROTATION_VECTOR;
    mPendingEvents[RotationVector].acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;

    mPendingEvents[LinearAccel].version = sizeof(sensors_event_t);
    mPendingEvents[LinearAccel].sensor = ID_LA;
    mPendingEvents[LinearAccel].type = SENSOR_TYPE_LINEAR_ACCELERATION;
    mPendingEvents[LinearAccel].acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;

    mPendingEvents[Gravity].version = sizeof(sensors_event_t);
    mPendingEvents[Gravity].sensor = ID_GR;
    mPendingEvents[Gravity].type = SENSOR_TYPE_GRAVITY;
    mPendingEvents[Gravity].acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;

    mPendingEvents[Gyro].version = sizeof(sensors_event_t);
    mPendingEvents[Gyro].sensor = ID_GY;
    mPendingEvents[Gyro].type = SENSOR_TYPE_GYROSCOPE;
    mPendingEvents[Gyro].gyro.status = SENSOR_STATUS_ACCURACY_HIGH;

    mPendingEvents[Accelerometer].version = sizeof(sensors_event_t);
    mPendingEvents[Accelerometer].sensor = ID_A;
    mPendingEvents[Accelerometer].type = SENSOR_TYPE_ACCELEROMETER;
    mPendingEvents[Accelerometer].acceleration.status
            = SENSOR_STATUS_ACCURACY_HIGH;

    mPendingEvents[MagneticField].version = sizeof(sensors_event_t);
    mPendingEvents[MagneticField].sensor = ID_M;
    mPendingEvents[MagneticField].type = SENSOR_TYPE_MAGNETIC_FIELD;
    mPendingEvents[MagneticField].magnetic.status = SENSOR_STATUS_ACCURACY_HIGH;

    mPendingEvents[Orientation].version = sizeof(sensors_event_t);
    mPendingEvents[Orientation].sensor = ID_O;
    mPendingEvents[Orientation].type = SENSOR_TYPE_ORIENTATION;
    mPendingEvents[Orientation].orientation.status = SENSOR_STATUS_ACCURACY_HIGH;

    mPendingEvents[Tap].version = sizeof(sensors_event_t);
    mPendingEvents[Tap].sensor = 7;
    mPendingEvents[Tap].type = 0;


    mPendingEvents[Shake].version = sizeof(sensors_event_t);
    mPendingEvents[Shake].sensor = 8;
    mPendingEvents[Shake].type = 0;


    mPendingEvents[YawIR].version = sizeof(sensors_event_t);
    mPendingEvents[YawIR].sensor = 9;
    mPendingEvents[YawIR].type = 0;


    mPendingEvents[Orient6].version = sizeof(sensors_event_t);
    mPendingEvents[Orient6].sensor = 10;
    mPendingEvents[Orient6].type = 0;


    mPendingEvents[GridNum].version = sizeof(sensors_event_t);
    mPendingEvents[GridNum].sensor = 11;
    mPendingEvents[GridNum].type = 0;


    mPendingEvents[GridDelta].version = sizeof(sensors_event_t);
    mPendingEvents[GridDelta].sensor = 12;
    mPendingEvents[GridDelta].type = 0;


    mPendingEvents[GridDelta].version = sizeof(sensors_event_t);
    mPendingEvents[GridDelta].sensor = 13;
    mPendingEvents[GridDelta].type = 0;


    mPendingEvents[Motion].version = sizeof(sensors_event_t);
    mPendingEvents[Motion].sensor = 14;
    mPendingEvents[Motion].type = 0;


    mPendingEvents[Step].version = sizeof(sensors_event_t);
    mPendingEvents[Step].sensor = 15;
    mPendingEvents[Step].type = 0;


    mPendingEvents[Snap].version = sizeof(sensors_event_t);
    mPendingEvents[Snap].sensor = 16;
    mPendingEvents[Snap].type = 0;


    mHandlers[RotationVector] = rv_handler;
    mHandlers[LinearAccel] = la_handler;
    mHandlers[Gravity] = grav_handler;
    mHandlers[Gyro] = gyro_handler;
    mHandlers[Accelerometer] = accel_handler;
    mHandlers[MagneticField] = compass_handler;
    mHandlers[Orientation] = orien_handler;
    mHandlers[7] =  noop_handler;  //gestures don't get handlers as they are done via callbacks
    mHandlers[8] =  noop_handler;
    mHandlers[9] =  noop_handler;
    mHandlers[10] = noop_handler;
    mHandlers[11] = noop_handler;
    mHandlers[12] = noop_handler;
    mHandlers[13] = noop_handler;
    mHandlers[14] = noop_handler;
    mHandlers[15] = noop_handler;
    mHandlers[16] = noop_handler;

    for (int i = 0; i < numSensors; i++)
        mDelays[i] = 30000000LLU; // 30 ms by default

    //initialize library parameters
    initMPL();

    //setup the FIFO contents
    setupFIFO();

    //now start the motion processing
    rv = MLDmpStart();
    if (rv != ML_SUCCESS) {
        LOGE("Fatal error: could not start the DMP correctly. (code = %d)\n", rv);
    } else {
        dmp_started = true;
    }

    output_gesture_list = mPendingEvents;
    enabledMask = &mEnabled;
    pendingMask = &mPendingMask;

    pthread_mutex_unlock(&mpld_mutex);

}

MPLSensor::~MPLSensor()
{
    FUNC_LOG;
    pthread_mutex_lock(&mpld_mutex);
    if (MLDmpStop() != ML_SUCCESS) {
        LOGD("Error: could not stop the DMP correctly.\n");
    }

    if (MLDmpClose() != ML_SUCCESS) {
        LOGD("Error: could not close the DMP");
    }

    if (MLSerialClose() != ML_SUCCESS) {
        LOGD("Error : could not close the serial port");
    }
    pthread_mutex_unlock(&mpld_mutex);
}

int MPLSensor::enable(int32_t handle, int en)
{
    FUNC_LOG;
    LOGV("handle : %d en: %d", handle, en);

    int what = -1;

    switch (handle) {
    case ID_A:
        what = Accelerometer;
        break;
    case ID_M:
        what = MagneticField;
        break;
    case ID_O:
        what = Orientation;
        break;
    case ID_GY:
        what = Gyro;
        break;
    case ID_GR:
        what = Gravity;
        break;
    case ID_RV:
        what = RotationVector;
        break;
    case ID_LA:
        what = LinearAccel;
        break;
    default:
        what = handle;
        break;
    }

    if (uint32_t(what) >= numSensors)
        return -EINVAL;

    int newState = en ? 1 : 0;
    int err = 0;
    LOGV_IF((uint32_t(newState) << what) != (mEnabled & (1 << what)), "sensor state change what=%d",what);

    if (1) { // (uint32_t(newState) << what) != (mEnabled & (1 << what))
        uint32_t sensor_type;
        short flags = newState;
        mEnabled &= ~(1 << what);
        mEnabled |= (uint32_t(flags) << what);
        set_power_states(mEnabled);
        update_delay();
    }
    return err;
}

int MPLSensor::setDelay(int32_t handle, int64_t ns)
{
    FUNC_LOG;
    int what = -1;
    switch (handle) {
    case ID_A:
        what = Accelerometer;
        break;
    case ID_M:
        what = MagneticField;
        break;
    case ID_O:
        what = Orientation;
        break;
    case ID_GY:
        what = Gyro;
        break;
    case ID_GR:
        what = Gravity;
        break;
    case ID_RV:
        what = RotationVector;
        break;
    case ID_LA:
        what = LinearAccel;
        break;
    }

    if (uint32_t(what) >= numSensors)
        return -EINVAL;

    if (ns < 0)
        return -EINVAL;

    mDelays[what] = ns;
    return update_delay();
}

int MPLSensor::update_delay()
{
    FUNC_LOG;
    int rv = 0;
    pthread_mutex_lock(&mpld_mutex);

    if (mEnabled) {
        uint64_t wanted = -1LLU;
        for (int i = 0; i < numSensors; i++) {
            if (mEnabled & (1 << i)) {
                uint64_t ns = mDelays[i];
                wanted = wanted < ns ? wanted : ns;
            }
        }
        int rate = 0;
        if (wanted > 0)
            rate = (wanted - 1) / 5000000LLU; //mpu fifo rate is in increments of 5ms
        if (rate == 0) //KLP disallow fifo rate 0
            rate = 1;
        LOGV("set fifo rate: %d %llu", rate, wanted);
        tMLError res = MLSetFIFORate(rate);
        if (res != ML_SUCCESS) {
            LOGE("error setting fifo rate");
        }
        rv = (res == ML_SUCCESS);
    }
    pthread_mutex_unlock(&mpld_mutex);
    return rv;
}

/* return the current time in nanoseconds */
static int64_t now_ns(void)
{
    //FUNC_LOG;
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    //LOGV("Time %lld", (int64_t)ts.tv_sec * 1000000000 + ts.tv_nsec);
    return (int64_t) ts.tv_sec * 1000000000 + ts.tv_nsec;
}

int MPLSensor::readEvents(sensors_event_t* data, int count)
{
    VFUNC_LOG;
    struct irq_data irqdata;
    int nread;
    tMLError rv;

    if (count < 1)
        return -EINVAL;

    int numEventReceived = 0;

    //read the info from the irq filehandle
    nread = read(data_fd, &irqdata, sizeof(irqdata));
    pthread_mutex_lock(&mpld_mutex);
    rv = MLUpdateData();
    pthread_mutex_unlock(&mpld_mutex);
    if (rv != ML_SUCCESS) {
        LOGE("MLUpdateData error (code %d)", (int)rv);
    }

    if (!new_data) {
        return 0;
    }

    new_data = 0; //reset the flag

    int64_t tt = now_ns();

    pthread_mutex_lock(&mpld_mutex);
    for (int i = 0; i < numSensors; i++) {
        if (mEnabled & (1 << i)) {
            mHandlers[i](mPendingEvents + i, &mPendingMask, i);
            mPendingEvents[i].timestamp = tt;
        }
    }

    for (int j = 0; count && mPendingMask && j < numSensors; j++) {
        if (mPendingMask & (1 << j)) {
            mPendingMask &= ~(1 << j);
            if (mEnabled & (1 << j)) {
                *data++ = mPendingEvents[j];
                count--;
                numEventReceived++;
            }
        }
    }
    pthread_mutex_unlock(&mpld_mutex);
    return numEventReceived;
}

int MPLSensor::getFd() const
{
    LOGV("MPLSensor::getFd returning %d", data_fd);
    return data_fd;
}

int MPLSensor::getAccelFd() const
{
    LOGV("MPLSensor::getAccelFd returning %d", accel_fd);
    return accel_fd;
}

//bool MPLSensor::hasPendingEvents() const {
//    return true;  //KLP hack for polling
//}


extern "C" {

int getBiases(float *b)
{
    FUNC_LOG;
    int rv;
    float err[3];
    LOGV("get biases\n");
    pthread_mutex_lock(&mpld_mutex);
    rv = MLGetFloatArray(ML_ACCEL_BIAS, b);
    rv += MLGetFloatArray(ML_GYRO_BIAS, &(b[3]));
    rv += MLGetFloatArray(ML_MAG_BIAS, &(b[6]));
    MLGetFloatArray(ML_MAG_BIAS_ERROR, err);
    pthread_mutex_unlock(&mpld_mutex);
    LOGV("rpcGetBiases: %f %f %f - %f %f %f - %f %f %f", b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8]);
    LOGV("rpcGetBiases: %f %f %f", err[0], err[1], err[2]);
    return rv;
}

int setBiases(float *b)
{
    FUNC_LOG;
    int rv;
    pthread_mutex_lock(&mpld_mutex);
    rv = MLSetFloatArray(ML_ACCEL_BIAS, b);
    rv += MLSetFloatArray(ML_GYRO_BIAS, b + 3);
    rv += MLSetFloatArray(ML_MAG_BIAS, b + 6);
    pthread_mutex_unlock(&mpld_mutex);
    return rv;
}
int setBiasUpdateFunc(long f)
{
    FUNC_LOG;
    int rv;
    pthread_mutex_lock(&mpld_mutex);
    rv = MLSetBiasUpdateFunc((unsigned short) f);
    LOGE_IF(rv!=ML_SUCCESS, "SysApi :: setBiasUpdateFunc failed (f=%lx rv=%d)", f, rv);
    pthread_mutex_unlock(&mpld_mutex);
    return rv;
}

int setSensors(long s)
{
    FUNC_LOG;
    int rv;

    pthread_mutex_lock(&mpld_mutex);
    master_sensor_mask = s;
    rv = MLSetMPUSensors(s & local_sensor_mask);
    LOGE_IF(rv!=ML_SUCCESS, "SysApi :: setSensors MLSetMPUSensors failed (s=%lx rv=%d)", (s&local_sensor_mask), rv);
    pthread_mutex_unlock(&mpld_mutex);
    return rv;
}

int getSensors(long* s)
{
    FUNC_LOG;
    int rv;
    pthread_mutex_lock(&mpld_mutex);
    *s = MLDLGetCfg()->requested_sensors;
    rv = 0;
    pthread_mutex_unlock(&mpld_mutex);
    return rv;
}

int resetCal()
{
    FUNC_LOG;
    int rv;
    pthread_mutex_lock(&mpld_mutex);
    rv = MLResetMagCalibration();
    pthread_mutex_unlock(&mpld_mutex);
    return rv;
}

int selfTest()
{
    FUNC_LOG;
    int rv;
    pthread_mutex_lock(&mpld_mutex);
    do {
        rv = MLSelfTestSetAccelZOrient(1); //accel z is inverted
        if (rv != ML_SUCCESS) {
            LOGE("error MLSelfTestSetAccelZOrient returned %d", rv);
            break;
        }
        rv = MLSelfTestRun();
        if (rv != ML_SUCCESS) {
            LOGE("error MLSelfTestRun returned %d", rv);
            break;
        }
    } while (0);
    pthread_mutex_unlock(&mpld_mutex);
    return rv;
}

typedef struct {
    int (*fpgetBiases)(float *f);
    int (*fpsetBiases)(float *f);
    int (*fpsetBiasUpdateFunc)(long f);
    int (*fpsetSensors)(long s);
    int (*fpgetSensors)(long* s);
    int (*fpresetCal)();
    int (*fpselfTest)();
} tMplSysApi;

tMplSysApi mplSysApi = { getBiases, setBiases, setBiasUpdateFunc, setSensors,
        getSensors, resetCal, selfTest };

void log_sys_api_addr()
{
    LOGV("sysapi object at %p", &mplSysApi);
    LOGV("  sysapi getBiases func at %p", getBiases);
}


/* GlyphApi functions *********************************************************** */

int addGlyph(unsigned short GlyphID)
{
    int rv;
    pthread_mutex_lock(&mpld_mutex);
    rv = MLAddGlyph(GlyphID);
    pthread_mutex_unlock(&mpld_mutex);
    return rv;
}

int bestGlyph(unsigned short *finalGesture)
{
    int rv;
    pthread_mutex_lock(&mpld_mutex);
    rv = MLBestGlyph(finalGesture);
    pthread_mutex_unlock(&mpld_mutex);
    return rv;
}
int setGlyphSpeedThresh(unsigned short speed)
{
    int rv;
    pthread_mutex_lock(&mpld_mutex);
    rv = MLSetGlyphSpeedThresh(speed);
    pthread_mutex_unlock(&mpld_mutex);
    return rv;
}
int startGlyph(void)
{
    int rv;
    pthread_mutex_lock(&mpld_mutex);
    rv = MLStartGlyph();
    pthread_mutex_unlock(&mpld_mutex);
    return rv;
}
int stopGlyph(void)
{
    int rv;
    pthread_mutex_lock(&mpld_mutex);
    rv = MLStopGlyph();
    pthread_mutex_unlock(&mpld_mutex);
    return rv;
}
int getGlyph(int index, int *x, int *y)
{
    int rv;
    pthread_mutex_lock(&mpld_mutex);
    rv = MLGetGlyph(index, x, y);
    pthread_mutex_unlock(&mpld_mutex);
    return rv;
}
int getGlyphLength(unsigned short *length)
{
    int rv;
    pthread_mutex_lock(&mpld_mutex);
    rv = MLGetGlyphLength(length);
    pthread_mutex_unlock(&mpld_mutex);
    return rv;
}

int clearGlyph(void)
{
    int rv;
    pthread_mutex_lock(&mpld_mutex);
    rv = MLClearGlyph();
    pthread_mutex_unlock(&mpld_mutex);
    return rv;
}

int loadGlyphs(unsigned char *libraryData)
{
    int rv;
    pthread_mutex_lock(&mpld_mutex);
    rv = MLLoadGlyphs(libraryData);
    pthread_mutex_unlock(&mpld_mutex);
    return rv;
}

int storeGlyphs(unsigned char *libraryData, unsigned short *length)
{
    int rv;
    pthread_mutex_lock(&mpld_mutex);
    rv = MLStoreGlyphs(libraryData, length);
    pthread_mutex_unlock(&mpld_mutex);
    return rv;
}

int setGlyphProbThresh(unsigned short prob)
{
    int rv;
    pthread_mutex_lock(&mpld_mutex);
    rv = MLSetGlyphProbThresh(prob);
    pthread_mutex_unlock(&mpld_mutex);
    return rv;
}

int getLibraryLength(unsigned short *length)
{
    int rv;
    //this function is called from another rpc function, so we don't grab the mutex
    rv = MLGetLibraryLength(length);
    return rv;
}

typedef struct {
 int (*fpAddGlyph)(unsigned short GlyphID);
 int (*fpBestGlyph)(unsigned short *finalGesture);
 int (*fpSetGlyphSpeedThresh)(unsigned short speed);
 int (*fpStartGlyph)(void);
 int (*fpStopGlyph)(void);
 int (*fpGetGlyph)(int index, int *x, int *y);
 int (*fpGetGlyphLength)(unsigned short *length);
 int (*fpClearGlyph)(void);
 int (*fpLoadGlyphs)(unsigned char *libraryData);
 int (*fpStoreGlyphs)(unsigned char *libraryData, unsigned short *length);
 int (*fpSetGlyphProbThresh)(unsigned short prob);
 int (*fpGetLibraryLength)(unsigned short *length);
} tMplGlyphApi;

tMplGlyphApi mplGlyphApi = {addGlyph, bestGlyph, setGlyphSpeedThresh, startGlyph, stopGlyph, getGlyph, getGlyphLength,
        clearGlyph, loadGlyphs, storeGlyphs, setGlyphProbThresh, getLibraryLength};

}

