/*
 * Copyright (C) 2011 Invensense, Inc.
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

#include "MPLSensor.h"

#include "ml.h"
#include "mlFIFO.h"
#include "mlsl.h"
#include "mlos.h"
#include "ml_mputest.h"
#include "ml_stored_data.h"
#include "mpu.h"


extern "C" {
#include "mlsupervisor.h"
}
#include "mlsupervisor_9axis.h"
#include "gesture.h"
#include "mlcontrol.h"
#include "orientation.h"
#include "mlpedometer_fullpower.h"
#include "mlpedometer_lowpower.h"
//#include "pedometer.h"
#include "mlglyph.h"
#include "mldl_cfg.h"
#include "mldl.h"
#include "mlFIFO.h"

extern "C" {
    void initMPL();
    void setupCallbacks();
    void setupFIFO();
    void cb_onMotion(uint16_t);
    void cb_onMotion_gest(uint16_t);
    void cb_onGesture(gesture_t* gesture);
    void cb_onOrientation(uint16_t orientation);
    void cb_onGridMove(unsigned short ctlSig, long *gNum, long *gChg);
    void cb_onStep(uint16_t);
    void cb_onStep_fp(unsigned long, unsigned long);
    void cb_procData();
    void set_power_states(int, bool);
    void log_sys_api_addr();
    void setupPed_fp();
    void setupGestures();
    void setupGlyph();
    tMLError MLDisableGlyph(void);
}

#define EXTRA_VERBOSE (1)
#define FUNC_LOG LOGV("%s", __PRETTY_FUNCTION__)
#define VFUNC_LOG LOGV_IF(EXTRA_VERBOSE, "%s", __PRETTY_FUNCTION__)

sensors_event_t* output_gesture_list;
extern uint32_t* s_enabledMask;
extern uint32_t* s_pendingMask;
extern struct stepParams s_step_params;
extern long s_ped_steps;            //pedometer steps recorded
extern long s_ped_wt;
extern bool s_sys_ped_enabled;  //flag indicating if the sys-api ped is enabled

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

/** setup the pedometer engine.
 *  this function should only be called when the mpld thread holds the mpld_mutex
 */
/*void setupPed()
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
}*/





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
    VFUNC_LOG;
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
    VFUNC_LOG;
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
    int *int_ary = (int*)dest->data;
    int_ary[0]   = src->type;
    int_ary[1]   = src->strength;
    int_ary[2]   = src->speed;
    int_ary[3]   = src->num;
    int_ary[4]   = src->meta;
    int_ary[5]   = src->reserved;
}

/**
 * handle gesture data returned from the MPL.
 */
void cb_onGesture(gesture_t* gesture)
{
#ifdef ENABLE_GESTURE_MANAGER
    LOGD("gesture callback (type: %d)", gesture->type);

    switch (gesture->type)
    {
    case ML_TAP:
    {
        if(*s_enabledMask & (1<<MPLSensor::Tap)) {
            store_gest(output_gesture_list+MPLSensor::Tap ,gesture);
            *s_pendingMask |= (1<<MPLSensor::Tap);
            LOGD("stored TAP");
        }
    }
    break;
    case ML_ROLL_SHAKE:
    case ML_PITCH_SHAKE:
    case ML_YAW_SHAKE:
    {
        if(gesture->strength == 1 && gesture->num == 1) {
            if(*s_enabledMask & (1<<MPLSensor::Snap)) {
                store_gest(&(output_gesture_list[MPLSensor::Snap]), gesture);
                *s_pendingMask |= (1<<MPLSensor::Snap);
            }

        }
        if(*s_enabledMask & (1<<MPLSensor::Shake)) {
            store_gest(&(output_gesture_list[MPLSensor::Shake]), gesture);
            *s_pendingMask |= (1<<MPLSensor::Shake);
            LOGD("stored SHAKE");
        } else {
            LOGD("SHAKE ignored");
        }

    }
    break;
    case ML_YAW_IMAGE_ROTATE:
    {
        if(*s_enabledMask & (1<<MPLSensor::YawIR)) {
            store_gest(&(output_gesture_list[MPLSensor::YawIR]),gesture);
            *s_pendingMask |= (1<<MPLSensor::YawIR);
        }
    }
    break;
    default:
        LOGE("Unknown Gesture received\n");
        break;
    }
#endif
}

/**
 * handle Orientation output from MPL
 */
void cb_onOrientation(uint16_t orientation) {
#ifdef ENABLE_GESTURE_MANAGER
    if(*s_enabledMask & (1<<MPLSensor::Orient6)) {
         //((int*)(output_gesture_list[MPLSensor::Orient6].data))[0] = ORIENTATION;
         ((int*)(output_gesture_list[MPLSensor::Orient6].data))[0] = orientation;
         output_gesture_list[MPLSensor::Orient6].sensor = MPLSensor::Orient6;
         *s_pendingMask |= (1<<MPLSensor::Orient6);
         LOGD("stored ORIENTATION");
     }
#endif
}

/**
 * handle control events and generate associated output gestures
 */
void cb_onGridMove(unsigned short ctlSig, long *gNum, long *gChg) {
#ifdef ENABLE_GESTURE_MANAGER
    //LOGD("%s", __FUNCTION__);
    int cs[4];
    int c[4];
    int d[4];

    //MLGetControlData(cs, d, c);

    if(*s_enabledMask & (1<<MPLSensor::CtrlSig)) {
        //((int*)(output_gesture_list[MPLSensor::CtrlSig].data))[0] = CONTROL_SIG;
        ((int*)(output_gesture_list[MPLSensor::CtrlSig].data))[0] = ctlSig;//cs[0];
        output_gesture_list[MPLSensor::CtrlSig].sensor = MPLSensor::CtrlSig;
        *s_pendingMask |= (1<<MPLSensor::CtrlSig);
        LOGD_IF(EXTRA_VERBOSE, "stored CONTROL_SIG");
        LOGV_IF(EXTRA_VERBOSE, "mPendingEvents = %x", *s_pendingMask);
    }

    if(*s_enabledMask & (1<<MPLSensor::GridNum)) {
        //((int*)(output_gesture_list[MPLSensor::GridNum].data))[0] = GRID_NUM;
        ((int*)(output_gesture_list[MPLSensor::GridNum].data))[0] = gNum[0]; //d[0];
        ((int*)(output_gesture_list[MPLSensor::GridNum].data))[1] = gNum[1]; //d[1];
        ((int*)(output_gesture_list[MPLSensor::GridNum].data))[2] = gNum[2]; //d[2];
        ((int*)(output_gesture_list[MPLSensor::GridNum].data))[3] = gNum[3]; //d[3];
        output_gesture_list[MPLSensor::GridNum].sensor = MPLSensor::GridNum;
        *s_pendingMask |= (1<<MPLSensor::GridNum);
        LOGD_IF(EXTRA_VERBOSE, "stored GRID_NUM");
        LOGV_IF(EXTRA_VERBOSE, "mPendingEvents = %x", *s_pendingMask);
    }

    if(*s_enabledMask & (1<<MPLSensor::GridDelta)) {
        //((int*)(output_gesture_list[MPLSensor::GridDelta].data))[0] = GRID_CHANGE;
        ((int*)(output_gesture_list[MPLSensor::GridDelta].data))[0] = gChg[0]; //c[0];
        ((int*)(output_gesture_list[MPLSensor::GridDelta].data))[1] = gChg[1]; //c[1];
        ((int*)(output_gesture_list[MPLSensor::GridDelta].data))[2] = gChg[2]; //c[2];
        ((int*)(output_gesture_list[MPLSensor::GridDelta].data))[3] = gChg[3]; //c[3];
        output_gesture_list[MPLSensor::GridDelta].sensor = MPLSensor::GridDelta;
        *s_pendingMask |= (1<<MPLSensor::GridDelta);
        LOGD_IF(EXTRA_VERBOSE, "stored GRID_CHANGE");
        LOGV_IF(EXTRA_VERBOSE, "mPendingEvents = %x", *s_pendingMask);
    }
#endif
}

void cb_onStep(uint16_t val)
{
#ifdef ENABLE_GESTURE_MANAGER
    if(*s_enabledMask & (1<<MPLSensor::Step)) {
        //((int*)(output_gesture_list[MPLSensor::Step].data))[0] = STEP;
        ((int*)(output_gesture_list[MPLSensor::Step].data))[0] = val;
        output_gesture_list[MPLSensor::Step].sensor = MPLSensor::Step;
        *s_pendingMask |= (1<<MPLSensor::Step);
        LOGD("stored STEP");
    }
#endif
}



void cb_onMotion_gest(uint16_t val)
{
#ifdef ENABLE_GESTURE_MANAGER
    if(*s_enabledMask & (1<<MPLSensor::Motion)) {
        //((int*)(output_gesture_list[MPLSensor::Motion].data))[0] = MOTION;
        ((int*)(output_gesture_list[MPLSensor::Motion].data))[0] = val;
        output_gesture_list[MPLSensor::Motion].sensor = MPLSensor::Motion;
        *s_pendingMask |= (1<<MPLSensor::Motion);
        LOGD("stored MOTION");
    }
#endif
}


