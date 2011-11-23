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
#include <utils/KeyedVector.h>

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
#include "ml_stored_data.h"
#include "mpu.h"
#include "kernel/timerirq.h"
#include "kernel/mpuirq.h"
#include "kernel/slaveirq.h"

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

#define EXTRA_VERBOSE (0)
#define FUNC_LOG LOGV("%s", __PRETTY_FUNCTION__)
#define VFUNC_LOG LOGV_IF(EXTRA_VERBOSE, "%s", __PRETTY_FUNCTION__)
/* this mask must turn on only the sensors that are present and managed by the MPL */
#define ALL_MPL_SENSORS_NP (ML_THREE_AXIS_ACCEL | ML_THREE_AXIS_COMPASS | ML_THREE_AXIS_GYRO)

/* ***************************************************************************
 * MPL interface functions
 */

static int mpu_accuracy;                 //global storage for the current accuracy status
static int new_data = 0;                 //flag indicating that the MPL calculated new output values
static int dmp_started = false;
pthread_mutex_t mpld_mutex = PTHREAD_MUTEX_INITIALIZER;
static long master_sensor_mask = ML_ALL_SENSORS;
static long local_sensor_mask = ALL_MPL_SENSORS_NP;

uint32_t* s_enabledMask;
uint32_t* s_pendingMask;
static int s_poll_time = -1;
static int s_cur_fifo_rate = -1;          //current fifo rate
static bool s_have_good_mpu_cal = false;  //flag indicating that the cal file can be written
//flags for controling step and gesture output
bool s_ped_enabled=false;          //flag indicating if the ped gesture is enabled
bool s_gest_enabled=false;         //flag indicating any gesture is enabled
int s_ped_state=0;                 //current pedometer state (NONE, STANDALONE, FULL)
struct stepParams s_step_params;
long s_ped_steps = 0;            //pedometer steps recorded
long s_ped_wt = 0;
bool s_sys_ped_enabled = false;  //flag indicating if the sys-api ped is enabled
static android::KeyedVector< int, int > irq_fds;
static bool s_use_timerirq_accel = false;
static bool s_use_timerirq_compass = true;
static bool s_use_timerirq = false;
static struct pollfd s_poll_fds[4];

enum PED_STATE {
    PED_NONE,
    PED_STANDALONE,
    PED_FULL,
    PED_SLEEP
};

enum FILEHANDLES {
    MPUIRQ_FD,
    ACCELIRQ_FD,
    COMPASSIRQ_FD,
    TIMERIRQ_FD,
};

#define GY_ENABLED ((1<<ID_GY) & enabled_sensors)
#define A_ENABLED  ((1<<ID_A)  & enabled_sensors)
#define O_ENABLED  ((1<<ID_O)  & enabled_sensors)
#define M_ENABLED  ((1<<ID_M)  & enabled_sensors)
#define LA_ENABLED ((1<<ID_LA) & enabled_sensors)
#define GR_ENABLED ((1<<ID_GR) & enabled_sensors)
#define RV_ENABLED ((1<<ID_RV) & enabled_sensors)

#ifdef ENABLE_GESTURE_MANAGER
#define GESTURE_MASK ((1<<ID_G_TAP) | (1<<ID_G_SHA) |(1<<ID_G_YIR) |(1<<ID_G_OR6) |(1<<ID_G_GRN) |(1<<ID_G_GRD) |(1<<ID_G_CSG) |(1<<ID_G_MOT) |(1<<ID_G_STP) |(1<<ID_G_SNA))
#define G_MASK_NO_PED ((1<<ID_G_TAP) | (1<<ID_G_SHA) |(1<<ID_G_YIR) |(1<<ID_G_OR6) |(1<<ID_G_GRN) |(1<<ID_G_GRD) |(1<<ID_G_CSG) |(1<<ID_G_MOT) |(1<<ID_G_SNA))
#define G_MASK_PED (1<<ID_G_STP)
extern sensors_event_t* output_gesture_list;
#else
#define GESTURE_MASK (0)
#define G_MASK_NO_PED (0)
#define G_MASK_PED (0)
#endif

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

void clearIrqData( bool* irq_set)
{
    unsigned int i;
    int nread;
    struct mpuirq_data irqdata;

    poll(s_poll_fds, ARRAY_SIZE(s_poll_fds), 0); //check which ones need to be cleared

    for (i = 0; i < ARRAY_SIZE(s_poll_fds); i++) {
        int cur_fd = s_poll_fds[i].fd;
        int j = 0;
        if(s_poll_fds[i].revents & POLLIN){
            nread = read(cur_fd, &irqdata, sizeof(irqdata));
            if(nread>0) {
                irq_set[i] = true;
                //LOGV_IF(EXTRA_VERBOSE, "irq: %d %d (%d)", i, irqdata.interruptcount, j++);
            }
        }
        s_poll_fds[i].revents=0;
    }
}

/* this function modifies static state variables.  It must be called with the mpld_mutex held. */
void set_power_states(int enabled_sensors, bool start_sys_ped)
{
    FUNC_LOG;
    bool irq_set[5] = {false, false, false, false, false};

    LOGV(" enabled_sensors: %d dmp_started: %d", enabled_sensors, dmp_started );

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
            local_sensor_mask |= ( ML_THREE_AXIS_ACCEL);
        } else {
            local_sensor_mask &= ~( ML_THREE_AXIS_ACCEL);
        }

        if (M_ENABLED) {
            local_sensor_mask |= ML_THREE_AXIS_COMPASS;
        } else {
            local_sensor_mask &= ~(ML_THREE_AXIS_COMPASS);
        }
        
        if(s_sys_ped_enabled && local_sensor_mask) {
            LOGV_IF(EXTRA_VERBOSE, "sys ped enabled -- bringing up DMP + Accel");
            local_sensor_mask |= (ML_THREE_AXIS_ACCEL + ML_THREE_AXIS_GYRO);
            LOGV_IF(EXTRA_VERBOSE, "local_sensor_mask %ld", local_sensor_mask);
        }
    } while (0);

    //record the new sensor state
    tMLError rv;

    long sen_mask = local_sensor_mask & master_sensor_mask;

    bool enable_ped = (!s_ped_enabled) && ((enabled_sensors & G_MASK_PED) != 0);
    bool disable_ped = s_ped_enabled && ((enabled_sensors & G_MASK_PED) == 0);
    bool enable_gest = (!s_gest_enabled) && ((enabled_sensors & G_MASK_NO_PED) != 0);
    bool disable_gest = s_gest_enabled && ((enabled_sensors & G_MASK_NO_PED) == 0);
    bool changing_sensors = ((MLDLGetCfg()->requested_sensors != sen_mask) || enable_ped || disable_ped || enable_gest || disable_gest ) && (sen_mask != 0);
    bool restart = (!dmp_started) && (sen_mask != 0);

    if ( changing_sensors || restart || start_sys_ped) {

        LOGV_IF(EXTRA_VERBOSE, "cs:%d rs:%d en_ped:%d da_ped:%d en_g:%d da_g:%d", changing_sensors, restart, enable_ped, disable_ped, enable_gest, disable_gest);

        if ((s_ped_state == PED_STANDALONE  || s_ped_state == PED_SLEEP) && restart) {
            unsigned long cursteps;
            unsigned long curwt;
            LOGV_IF(EXTRA_VERBOSE, "sys ped enabled, restarting, switching to full power ped");
            MLPedometerStandAloneGetNumOfSteps(&cursteps);
            MLPedometerStandAloneGetWalkTime(&curwt);
            if(s_ped_state == PED_STANDALONE) {
                MLDmpPedometerStandAloneStop();
            }
            MLDmpPedometerStandAloneClose();
            s_ped_steps = cursteps;
            s_ped_wt = curwt;
            clearIrqData(irq_set);
            initMPL();
            setupFIFO();
            if (s_cur_fifo_rate > 9) {
                MLSetFIFORate(6);
                s_cur_fifo_rate = 6;
            }
            setupPed_fp();
        }
        
        if ( dmp_started || start_sys_ped ) {
            MLDmpStop();
            clearIrqData(irq_set);
            dmp_started = false;
        }

        if(sen_mask != MLDLGetCfg()->requested_sensors) {
            LOGV("MLSetMPUSensors: %lx", sen_mask);
            rv = MLSetMPUSensors(sen_mask);
            LOGE_IF(rv != ML_SUCCESS, "error: unable to set MPL sensor power states (sens=%ld retcode = %d)", sen_mask, rv);
        }

        if(start_sys_ped) {
            setupPed_fp();
        }

        if(enable_ped) {
            LOGV_IF(EXTRA_VERBOSE, "enabling pedometer");
            s_ped_enabled = true;
            if(!s_sys_ped_enabled) {
                LOGV_IF(EXTRA_VERBOSE, "sys ped not enabled");
                setupPed_fp();
            }
        } else if(disable_ped) {
            LOGV_IF(EXTRA_VERBOSE, "disabling pedometer");
            if(!s_sys_ped_enabled) {
                LOGV_IF(EXTRA_VERBOSE, "sys ped not enabled");
                MLDisablePedometerFullPower();
                s_ped_steps = 0;
                s_ped_wt = 0;
                s_ped_state = PED_NONE;
            }
            s_ped_enabled = false;
        }

        if( enable_gest ) {
            LOGV_IF(EXTRA_VERBOSE, "enabling gestures");
            setupGestures();
            setupGlyph();
            s_gest_enabled = true;
        } else if(disable_gest) {
            LOGV_IF(EXTRA_VERBOSE, "disabling gestures");
            MLDisableGlyph();
            MLDisableControl();
            MLDisableOrientation();
            MLDisableGesture();
            s_gest_enabled = false;
        }

        if( (s_use_timerirq_compass && (sen_mask == ML_THREE_AXIS_COMPASS)) 
          || ( s_use_timerirq_accel && (sen_mask & ML_THREE_AXIS_ACCEL)) )
        {
            if( (sen_mask & ML_DMP_PROCESSOR) == 0) {
                LOGV_IF(EXTRA_VERBOSE, "Allowing TimerIRQ");
                s_use_timerirq = true;
            }
        } else {
            if(s_use_timerirq) {
                ioctl(irq_fds.valueFor(TIMERIRQ_FD), TIMERIRQ_STOP, 0);
                clearIrqData(irq_set);
            }
            LOGV_IF(EXTRA_VERBOSE, "Not allowing TimerIRQ");
            s_use_timerirq = false;
        }

        if (!dmp_started) {
            LOGV("Starting DMP");
            rv = MLDmpStart();
            LOGE_IF(rv != ML_SUCCESS, "unable to start dmp");
            dmp_started = true;
        }
    }

    //check if we should stop the DMP
    if (dmp_started && (sen_mask == 0)) {
        LOGV("Stopping DMP");
        rv = MLDmpStop();
        LOGE_IF(rv != ML_SUCCESS, "error: unable to stop DMP (retcode = %d)", rv);
        if(s_use_timerirq) {
            ioctl(irq_fds.valueFor(TIMERIRQ_FD), TIMERIRQ_STOP, 0);
        }
        clearIrqData(irq_set);
        if(s_have_good_mpu_cal) {
            rv = MLStoreCalibration();
            LOGE_IF(rv != ML_SUCCESS, "error: unable to store MPL calibration file");
            s_have_good_mpu_cal = false;
        }
        
        if(s_sys_ped_enabled && s_ped_state == PED_FULL) {
            LOGV_IF(EXTRA_VERBOSE, "sys ped enabled, switching to StandAlone mode");
            MLDmpClose();
            MLDmpPedometerStandAloneOpen();
            MLPedometerStandAloneSetNumOfSteps(s_ped_steps);
            MLPedometerStandAloneSetWalkTime(s_ped_wt);
            s_step_params.threshold = 25000000L;
            s_step_params.minUpTime = 16*20;
            s_step_params.maxUpTime = 60*20;
            s_step_params.minSteps = 5;
            s_step_params.minEnergy = 0x0d000000L;
            s_step_params.maxStepBufferTime = 125*20;
            s_step_params.clipThreshold = 0x06000000L;
            MLPedometerStandAloneSetParams(&s_step_params);
            MLDmpPedometerStandAloneStart();
            s_ped_state = PED_STANDALONE;
        }
        
        dmp_started = false;
        s_poll_time = -1;
	 s_cur_fifo_rate = -1;   // added
    }

}

/**
 * container function for all the calls we make once to set up the MPL.
 */
void initMPL()
{
    FUNC_LOG;
    tMLError result;

    if (MLDmpOpen() != ML_SUCCESS) {
        LOGE("Fatal Error : could not open DMP correctly.\n");
    }

    result = MLSetMPUSensors(ALL_MPL_SENSORS_NP); //default to all sensors, also makes 9axis enable work
    LOGE_IF(result != ML_SUCCESS, "Fatal Error : could not set enabled sensors.");

    if(MLLoadCalibration() != ML_SUCCESS) {
        LOGE("could not open MPL calibration file");
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

}

/** setup the fifo contents.
 */
void setupFIFO()
{
    FUNC_LOG;
    tMLError result;

    result = FIFOSendAccel(ML_ALL, ML_32_BIT);
    if (result != ML_SUCCESS) {
        LOGE("Fatal error: FIFOSendAccel returned %d\n",result);
    }

    result = FIFOSendQuaternion(ML_32_BIT);
    if (result != ML_SUCCESS) {
        LOGE("Fatal error: FIFOSendQuaternion returned %d\n",result);
    }

    result = FIFOSendLinearAccel(ML_ALL, ML_32_BIT);
    if (result != ML_SUCCESS) {
        LOGE("Fatal error: FIFOSendLinearAccel returned %d\n",result);
    }

    result = FIFOSendLinearAccelWorld (ML_ALL, ML_32_BIT);
    if (result != ML_SUCCESS) {
        LOGE("Fatal error: FIFOSendLinearAccelWorld returned %d\n",result);
    }

    result = FIFOSendGravity(ML_ALL, ML_32_BIT);
    if (result != ML_SUCCESS) {
        LOGE("Fatal error: FIFOSendGravity returned %d\n",result);
    }

    result = FIFOSendGyro(ML_ALL, ML_32_BIT);
    if (result != ML_SUCCESS) {
        LOGE("Fatal error: FIFOSendGyro returned %d\n",result);
    }

}

void cb_onStep_fp(unsigned long val, unsigned long wtime)
{
    LOGV_IF(EXTRA_VERBOSE, "onStep cb %ld %ld", val, wtime);
    s_ped_steps = val;
#ifdef ENABLE_GESTURE_MANAGER
    if (*s_enabledMask & (1 << MPLSensor::Step)) {
        //((int*)(output_gesture_list[MPLSensor::Step].data))[0] = STEP;
        ((int*) (output_gesture_list[MPLSensor::Step].data))[0] = val;
        ((int*) (output_gesture_list[MPLSensor::Step].data))[1] = wtime;
        output_gesture_list[MPLSensor::Step].sensor = MPLSensor::Step;
        *s_pendingMask |= (1 << MPLSensor::Step);
        LOGD("stored STEP");
    }
#endif
}

void setupPed_fp()
{
    VFUNC_LOG;
    tMLError result;

    s_ped_state = PED_FULL;

    if(s_cur_fifo_rate > 9) {
        LOGV_IF(EXTRA_VERBOSE, "need to adjust fifo rate (cur = %d)", s_cur_fifo_rate);
        result = MLSetFIFORate(6);
        LOGE_IF(result != ML_SUCCESS, "setupPed_fp : failed to adjust fifo rate");
        s_cur_fifo_rate = 6;
    }

    result = MLEnablePedometerFullPower();
    if (result != ML_SUCCESS) {
        LOGE("Fatal error: MLEnablePedometerFullPower returned %d\n",result);
    }

    s_step_params.threshold = 25000000L;
    s_step_params.minUpTime = 16*20;
    s_step_params.maxUpTime = 60*20;
    s_step_params.minSteps = 5;
    s_step_params.minEnergy = 0x0d000000L;
    s_step_params.maxStepBufferTime = 125*20;
    s_step_params.clipThreshold = 0x06000000L;

    result = MLSetPedometerFullPowerParams(&s_step_params);
    if (result != ML_SUCCESS) {
        LOGE("Fatal error: MLSetPedometerFullPowerParams returned %d\n",result);
    }

    result = MLSetPedometerFullPowerStepCallback( cb_onStep_fp );
    if (result != ML_SUCCESS) {
        LOGE("Fatal error: MLSetPedometerFullPowerStepCallback returned %d\n",result);
    }

    result = MLSetPedometerFullPowerStepCount(s_ped_steps);
    result = MLSetPedometerFullPowerWalkTime(s_ped_wt);

    LOGE_IF(result != ML_SUCCESS, "Fatal error: MLSetPedometerFullPowerStepCount failed (%d)", result);
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
        if(*s_enabledMask & (1<<MPLSensor::Gyro)) {
            //if gyros are on and we got a no motion, set a flag 
            // indicating that the cal file can be written. 
            s_have_good_mpu_cal = true;
        }
    }

#ifdef ENABLE_GESTURE_MANAGER
    cb_onMotion_gest(val);
#endif
    return;
}

} //end of extern "C"

static int sampleCount = 0;

void cb_procData()
{
    new_data = 1;
    sampleCount++;
    //LOGV_IF(EXTRA_VERBOSE, "new data (%d)", sampleCount);
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
    //VFUNC_LOG;
    tMLError res;
    res = MLGetFloatArray(ML_ACCELS, s->acceleration.v);
    //res = FIFOGetAccelFloat(s->acceleration.v);
    s->acceleration.v[0] = s->acceleration.v[0] * 9.81;
    s->acceleration.v[1] = s->acceleration.v[1] * 9.81;
    s->acceleration.v[2] = s->acceleration.v[2] * 9.81;
    //LOGV_IF(EXTRA_VERBOSE, "accel data: %f %f %f", s->acceleration.v[0], s->acceleration.v[1], s->acceleration.v[2]);
    s->acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;
    if (res == ML_SUCCESS)
        *pending_mask |= (1 << index);
}

int estimate_compass_accuracy() {
    tMLError res, res2;
    float total_be;
    float bias_error[3];
    int rv;

    res2 = MLGetFloatArray(ML_MAG_BIAS_ERROR, bias_error);

    if (res2 == ML_SUCCESS) {
        //use total of bias errors to estimate sensor accuracy
        total_be = bias_error[0] + bias_error[1] + bias_error[2];
        if (total_be > 2700.0)
            rv = SENSOR_STATUS_UNRELIABLE;
        else if (total_be > 1500.0)
            rv = SENSOR_STATUS_ACCURACY_LOW;
        else if (total_be > 300.0)
            rv = SENSOR_STATUS_ACCURACY_MEDIUM;
        else {
            rv = SENSOR_STATUS_ACCURACY_HIGH;
        }
    } else {
        LOGE("could not get mag bias error");
        rv = SENSOR_STATUS_ACCURACY_LOW;
    }

    return rv;
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

    s->magnetic.status = estimate_compass_accuracy();

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

	s->gyro.status = ((mpu_accuracy < estimate_compass_accuracy()) ? mpu_accuracy : estimate_compass_accuracy());
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

void calcOrientationSensor(float *R, float *values)
{
    float tmp;
    //Azimuth
    if ((R[7]>0.707) || ((R[8]<0) && (fabs(R[7])>fabs(R[6])))) {
        values[0] = (float)atan2f(-R[3], R[0]);
    } else {
        values[0] = (float)atan2f(R[1], R[4]);
    }
    values[0] *= 57.295779513082320876798154814105f;
    if (values[0]<0) {
        values[0] += 360.0f;
    }
    //Pitch
    tmp = R[7];
    if (tmp>1.0f) tmp = 1.0f;
    if (tmp<-1.0f) tmp = -1.0f;
    values[1] = -asinf(tmp)*57.295779513082320876798154814105f;
    if (R[8] < 0) {
        values[1] = 180.0f-values[1];
    }
    if (values[1] > 180.0f) {
        values[1] -= 360.0f;
    }
    //Roll
    if ((R[7]>0.707f) || ((R[8]<0) && (fabs(R[7])>fabs(R[6])))) {
        values[2] = (float)atan2f(R[6], R[7]);
    } else {
        values[2] = (float)atan2f(R[6], R[8]);
    }
    values[2] *= 57.295779513082320876798154814105f;
    if (values[2]>90.0f) {
        values[2] = 180.0f-values[2];
    }
    if (values[2]<-90.0f) {
        values[2] = -180.0f-values[2];
    }
}

void orien_handler(sensors_event_t* s, uint32_t* pending_mask, int index) //note that this is the handler for the android 'orientation' sensor, not the mpl orientation output
{
    VFUNC_LOG;
    tMLError r1, r2;
    float euler[3];
    float heading[1];
    float rot_mat[9];

    r1 = MLGetFloatArray(ML_EULER_ANGLES, euler);
    r2 = MLGetFloatArray(ML_HEADING, heading);
    r2 = MLGetFloatArray(ML_ROTATION_MATRIX, rot_mat);

    //ComputeAndOrientation(heading[0], euler, s->orientation.v);
    calcOrientationSensor(rot_mat, s->orientation.v);

    s->orientation.status = ((mpu_accuracy < estimate_compass_accuracy()) ? mpu_accuracy : estimate_compass_accuracy());

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
    int mpu_int_fd, i;
    char *port = NULL;
	
    LOGV_IF(EXTRA_VERBOSE, "MPLSensor constructor: numSensors = %d", numSensors);
    log_sys_api_addr();

    mForceSleep = false;
    
    s_step_params.threshold = 25000000L;
    s_step_params.minUpTime = 16*20;
    s_step_params.maxUpTime = 60*20;
    s_step_params.minSteps = 5;
    s_step_params.minEnergy = 0x0d000000L;
    s_step_params.maxStepBufferTime = 125*20;
    s_step_params.clipThreshold = 0x06000000L;
	
    for(i=0; i<ARRAY_SIZE(s_poll_fds);i++){
        s_poll_fds[i].fd = -1;
        s_poll_fds[i].events = 0;
    }

    pthread_mutex_lock(&mpld_mutex);

    mpu_int_fd = open("/dev/mpuirq", O_RDWR);
    if (mpu_int_fd == -1) {
        LOGE("could not open the mpu irq device node");
    } else {
        fcntl(mpu_int_fd, F_SETFL, O_NONBLOCK);
        //ioctl(mpu_int_fd, MPUIRQ_SET_TIMEOUT, 0);
        irq_fds.add(MPUIRQ_FD, mpu_int_fd);
        s_poll_fds[MPUIRQ_FD].fd = mpu_int_fd;
        s_poll_fds[MPUIRQ_FD].events = POLLIN;
    }

    accel_fd = open("/dev/accelirq", O_RDWR);
    if(accel_fd == -1) {
        LOGE("could not open the accel irq device node");
    } else {
        fcntl(accel_fd, F_SETFL, O_NONBLOCK);
        //ioctl(accel_fd, SLAVEIRQ_SET_TIMEOUT, 0);
        irq_fds.add(ACCELIRQ_FD, accel_fd);
        s_poll_fds[ACCELIRQ_FD].fd = accel_fd;
        s_poll_fds[ACCELIRQ_FD].events = POLLIN;
    }

    timer_fd = open("/dev/timerirq", O_RDWR);
    if(timer_fd == -1) {
        LOGE("could not open the timer irq device node");
    } else {
        fcntl(timer_fd, F_SETFL, O_NONBLOCK);
        //ioctl(timer_fd, TIMERIRQ_SET_TIMEOUT, 0);
        irq_fds.add(TIMERIRQ_FD, timer_fd);
        s_poll_fds[TIMERIRQ_FD].fd = timer_fd;
        s_poll_fds[TIMERIRQ_FD].events = POLLIN;
    }

    data_fd = mpu_int_fd;

    if((accel_fd == -1) && (timer_fd != -1)) {
        //no accel irq and timer available
        s_use_timerirq_accel = true;
        LOGD("MPLSensor falling back to timerirq for accel data");
    }

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

#ifdef ENABLE_GESTURE_MANAGER
    mPendingEvents[Tap].version = sizeof(sensors_event_t);
    mPendingEvents[Tap].sensor = Tap;
    mPendingEvents[Tap].type = 0;

    mPendingEvents[Shake].version = sizeof(sensors_event_t);
    mPendingEvents[Shake].sensor = Shake;
    mPendingEvents[Shake].type = 0;

    mPendingEvents[YawIR].version = sizeof(sensors_event_t);
    mPendingEvents[YawIR].sensor = YawIR;
    mPendingEvents[YawIR].type = 0;

    mPendingEvents[Orient6].version = sizeof(sensors_event_t);
    mPendingEvents[Orient6].sensor = Orient6;
    mPendingEvents[Orient6].type = 0;

    mPendingEvents[GridNum].version = sizeof(sensors_event_t);
    mPendingEvents[GridNum].sensor = GridNum;
    mPendingEvents[GridNum].type = 0;

    mPendingEvents[GridDelta].version = sizeof(sensors_event_t);
    mPendingEvents[GridDelta].sensor = GridDelta;
    mPendingEvents[GridDelta].type = 0;

    mPendingEvents[CtrlSig].version = sizeof(sensors_event_t);
    mPendingEvents[CtrlSig].sensor = CtrlSig;
    mPendingEvents[CtrlSig].type = 0;

    mPendingEvents[Motion].version = sizeof(sensors_event_t);
    mPendingEvents[Motion].sensor = Motion;
    mPendingEvents[Motion].type = 0;

    mPendingEvents[Step].version = sizeof(sensors_event_t);
    mPendingEvents[Step].sensor = Step;
    mPendingEvents[Step].type = 0;

    mPendingEvents[Snap].version = sizeof(sensors_event_t);
    mPendingEvents[Snap].sensor = Snap;
    mPendingEvents[Snap].type = 0;
#endif

    mHandlers[RotationVector] = rv_handler;
    mHandlers[LinearAccel] = la_handler;
    mHandlers[Gravity] = grav_handler;
    mHandlers[Gyro] = gyro_handler;
    mHandlers[Accelerometer] = accel_handler;
    mHandlers[MagneticField] = compass_handler;
    mHandlers[Orientation] = orien_handler;

#ifdef ENABLE_GESTURE_MANAGER
    mHandlers[Tap]       = noop_handler;  //gestures don't get handlers as they are done via callbacks
    mHandlers[Shake]     = noop_handler;
    mHandlers[YawIR]     = noop_handler;
    mHandlers[Orient6]   = noop_handler;
    mHandlers[GridNum]   = noop_handler;
    mHandlers[GridDelta] = noop_handler;
    mHandlers[CtrlSig]   = noop_handler;
    mHandlers[Motion]    = noop_handler;
    mHandlers[Step]      = noop_handler;
    mHandlers[Snap]      = noop_handler;

    output_gesture_list = &(mPendingEvents[0]);
#endif

    for (int i = 0; i < numSensors; i++)
        mDelays[i] = 30000000LLU; // 30 ms by default

    s_enabledMask = &mEnabled;
    s_pendingMask = &mPendingMask;

    if (MLSerialOpen(port) != ML_SUCCESS) {
        LOGE("Fatal Error : could not open MPL serial interface");
    }
    
    //initialize library parameters
    initMPL();

    //setup the FIFO contents
    setupFIFO();

    //we start the motion processing only when a sensor is enabled...
    //rv = MLDmpStart();
    //LOGE_IF(rv != ML_SUCCESS, "Fatal error: could not start the DMP correctly. (code = %d)\n", rv);
    //dmp_started = true;

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
    default:  //this takes care of all the gestures
        what = handle;
        break;
    }
    if (uint32_t(what) >= numSensors)
        return -EINVAL;

    int newState = en ? 1 : 0;
    int err = 0;
    LOGV_IF((uint32_t(newState) << what) != (mEnabled & (1 << what)), "sensor state change what=%d",what);


    pthread_mutex_lock(&mpld_mutex);
    if ( (uint32_t(newState) << what) != (mEnabled & (1 << what)) ) {
        uint32_t sensor_type;
        short flags = newState;
        mEnabled &= ~(1 << what);
        mEnabled |= (uint32_t(flags) << what);
        LOGV_IF(EXTRA_VERBOSE, "mEnabled = %x", mEnabled);
        set_power_states(mEnabled, false);
        pthread_mutex_unlock(&mpld_mutex);
        update_delay();
        return err;
    }
    pthread_mutex_unlock(&mpld_mutex);
    return err;
}

int MPLSensor::setDelay(int32_t handle, int64_t ns)
{
    FUNC_LOG;
    LOGV_IF(EXTRA_VERBOSE, " setDelay handle: %d rate %d", handle, (int)(ns/1000000LL));
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

    if (ns < 0)
        return -EINVAL;

    pthread_mutex_lock(&mpld_mutex);
    mDelays[what] = ns;
    pthread_mutex_unlock(&mpld_mutex);
    return update_delay();
}

int MPLSensor::update_delay()
{
    FUNC_LOG;
    int rv = 0;
    bool irq_set[5];

    pthread_mutex_lock(&mpld_mutex);

    if (mEnabled) {
        uint64_t wanted = -1LLU;
        for (int i = 0; i < numSensors; i++) {
            if (mEnabled & (1 << i)) {
                uint64_t ns = mDelays[i];
                wanted = wanted < ns ? wanted : ns;
            }
        }

        //Limit all rates to 100Hz max. 100Hz = 10ms = 10000000ns
        if(wanted < 10000000LLU) {
            wanted = 10000000LLU;
        }

        int rate = ((wanted) / 5000000LLU) -  ((wanted%5000000LLU==0)?1:0); //mpu fifo rate is in increments of 5ms
        if (rate == 0) //KLP disallow fifo rate 0
            rate = 1;

        if(s_sys_ped_enabled || s_ped_enabled || s_gest_enabled) {
            if(rate > 9)
                rate = 9;  //slowest we can go with ped or gest
        }

        if(dmp_started && (rate != s_cur_fifo_rate)) {
            LOGV("set fifo rate: %d %llu", rate, wanted);
            tMLError res = MLDmpStop();
            res = MLSetFIFORate(rate);
            LOGE_IF(res != ML_SUCCESS, "error setting FIFO rate");
            if(((MLDLGetCfg()->requested_sensors & ML_DMP_PROCESSOR) == 0) ) {
                SetSampleStepSizeMs(wanted/1000000LLU);
                res = MLSetMPUSensors(MLDLGetCfg()->requested_sensors);
                if(s_use_timerirq) {
                    ioctl(irq_fds.valueFor(TIMERIRQ_FD), TIMERIRQ_STOP, 0);
                    clearIrqData(irq_set);
                    if(MLDLGetCfg()->requested_sensors == ML_THREE_AXIS_COMPASS) {
                        ioctl(irq_fds.valueFor(TIMERIRQ_FD), TIMERIRQ_START, (unsigned long)(wanted/1000000LLU));
                        LOGV_IF(EXTRA_VERBOSE, "updated timerirq period to %d", (int)(wanted/1000000LLU));
                    }else{
                        ioctl(irq_fds.valueFor(TIMERIRQ_FD), TIMERIRQ_START, (unsigned long)GetSampleStepSizeMs());
                        LOGV_IF(EXTRA_VERBOSE, "updated timerirq period to %d", (int)GetSampleStepSizeMs());
                    }
                }
            } else {
                res = MLSetMPUSensors(MLDLGetCfg()->requested_sensors);
            }
            
            res = MLDmpStart();
            LOGE_IF(res != ML_SUCCESS, "error re-starting DMP");

            s_cur_fifo_rate = rate;
            rv = (res == ML_SUCCESS);
        }

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
    //VFUNC_LOG;
    int i;
    bool irq_set[5] = {false, false, false, false, false};
    tMLError rv;
    if (count < 1)
        return -EINVAL;
    int numEventReceived = 0;

    clearIrqData(irq_set);

    pthread_mutex_lock(&mpld_mutex);
    if(dmp_started) {
        //LOGV_IF(EXTRA_VERBOSE, "Update Data");
        rv = MLUpdateData();
        LOGE_IF(rv != ML_SUCCESS, "MLUpdateData error (code %d)", (int)rv);
    } else if(s_ped_state == PED_SLEEP || s_ped_state == PED_STANDALONE){
        LOGV_IF(EXTRA_VERBOSE, "Possible Ped event");
        int idx = irq_fds.indexOfKey(ACCELIRQ_FD);
        LOGE_IF(idx < 1, "ERROR -- accel irq not present.  pedometer will not function");
/* -- KLP removed until accel can wakeup AP
        if(s_ped_state == PED_SLEEP && irq_set[ACCELIRQ_FD] ) {
            //accel int -- turn ped back on
            LOGV_IF(EXTRA_VERBOSE, "motion int -- turn ped back on");
            tMLError e = MLPedometerStandAloneSetNumOfSteps(s_ped_steps);
            LOGE_IF(e != ML_SUCCESS, "failed to reset step count in Standalone Pedometer on motion int");
            MLPedometerStandAloneSetWalkTime(s_ped_wt);
            e = MLDmpPedometerStandAloneStart();
            LOGE_IF(e != ML_SUCCESS, "failed to restart standalone pedometer on motion int");
            s_ped_state = PED_STANDALONE;
        } else if (s_ped_state == PED_STANDALONE && irq_set[ACCELIRQ_FD] ) {
            LOGV_IF(EXTRA_VERBOSE, "no-motion int -- sleep sensors");
            //accel int -- switch to sleep
            tMLError e = MLDmpPedometerStandAloneStop();
            LOGE_IF(e != ML_SUCCESS, "failed to sleep pedometer on no-motion event");
            s_ped_state = PED_SLEEP;
        }
*/
    } else {
        //probably just one extra read after shutting down
        LOGV_IF(EXTRA_VERBOSE, "MPLSensor::readEvents called, but there's nothing to do.");
    }

    pthread_mutex_unlock(&mpld_mutex);

    if (!new_data) {
        LOGV_IF(EXTRA_VERBOSE, "no new data");
        return 0;
    }
    new_data = 0;
    int64_t tt = now_ns();
    pthread_mutex_lock(&mpld_mutex);
    for(int i = 0;i < numSensors;i++){
        if(mEnabled & (1 << i)){
            mHandlers[i](mPendingEvents + i, &mPendingMask, i);
            mPendingEvents[i].timestamp = tt;
        }
    }

    for(int j = 0;count && mPendingMask && j < numSensors;j++){
        if(mPendingMask & (1 << j)){
            mPendingMask &= ~(1 << j);
            if(mEnabled & (1 << j)){
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

int MPLSensor::getTimerFd() const
{
    LOGV("MPLSensor::getTimerFd returning %d", timer_fd);
    return timer_fd;
}

int MPLSensor::getPowerFd() const
{
    int hdl = (int)MLSerialGetHandle();
    LOGV("MPLSensor::getPowerFd returning %d", hdl);
    return hdl;
}
    
int MPLSensor::getPollTime()
{
    return s_poll_time;
}

bool MPLSensor::hasPendingEvents() const {
    //if we are using the polling workaround, force the main loop to check for data every time
    return (s_poll_time != -1);
}

void MPLSensor::handlePowerEvent() {
    VFUNC_LOG;
    mpuirq_data irqd;
    unsigned long extra;
    int fd = (int)MLSerialGetHandle();
    irqd.data_size = sizeof(extra);
    irqd.data = &extra;
    
    read(fd, &irqd, sizeof(irqd) + sizeof(extra));
    
    if(extra == MPU_PM_EVENT_SUSPEND_PREPARE) {
        //going to sleep
        sleepEvent();
    } else if (extra == MPU_PM_EVENT_POST_SUSPEND)  {
        //waking up
        wakeEvent();
    }
    
    ioctl(fd, MPU_PM_EVENT_HANDLED, 0);
}

void MPLSensor::sleepEvent() {
    VFUNC_LOG;
    pthread_mutex_lock(&mpld_mutex);
    if(*s_enabledMask != 0) {
        mForceSleep = true;
        mOldEnabledMask = *s_enabledMask;
        set_power_states(0, false);
    }
    pthread_mutex_unlock(&mpld_mutex);
}

void MPLSensor::wakeEvent() {
    VFUNC_LOG;
    pthread_mutex_lock(&mpld_mutex);
    if(mForceSleep) {
        set_power_states( (mOldEnabledMask | *s_enabledMask) , false);
    }
    mForceSleep = false;
    pthread_mutex_unlock(&mpld_mutex);
}
        
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

//implementation for startPed system api, do not call from MPLSensor
int startPed(void) {
    VFUNC_LOG;
    pthread_mutex_lock(&mpld_mutex);
    if(*s_enabledMask == 0 && s_ped_state == PED_NONE) {
        //all sensors off and no ped running
        LOGV_IF(EXTRA_VERBOSE, "sys starting standalone pedometer");
        MLDmpClose();
        MLDmpPedometerStandAloneOpen();
        MLPedometerStandAloneSetNumOfSteps(0);
        s_step_params.threshold = 25000000L;
        s_step_params.minUpTime = 16*20;
        s_step_params.maxUpTime = 60*20;
        s_step_params.minSteps = 5;
        s_step_params.minEnergy = 0x0d000000L;
        s_step_params.maxStepBufferTime = 125*20;
        s_step_params.clipThreshold = 0x06000000L;
        MLPedometerStandAloneSetParams(&s_step_params);
        MLDmpPedometerStandAloneStart();
        s_ped_state = PED_STANDALONE;
        s_sys_ped_enabled= true;
    } else if (s_ped_state == PED_NONE) {
        LOGV_IF(EXTRA_VERBOSE, "sys starting fullpower pedometer");
        s_sys_ped_enabled = true;
        s_ped_state = PED_FULL;
        set_power_states(*s_enabledMask, true);
    }
    pthread_mutex_unlock(&mpld_mutex);
    return ML_SUCCESS;
}

int stopPed(void) {
    VFUNC_LOG;
    if(!s_sys_ped_enabled)
        return ML_SUCCESS;
    
    pthread_mutex_lock(&mpld_mutex);
    
    if(s_ped_state == PED_STANDALONE) {
        LOGV_IF(EXTRA_VERBOSE, "sys stopping standalone pedometer");
        MLDmpPedometerStandAloneStop();
    }

    if(s_ped_state == PED_SLEEP || s_ped_state == PED_STANDALONE) {
        LOGV_IF(EXTRA_VERBOSE && s_ped_state == PED_SLEEP, "sys ped was asleep");
        MLDmpPedometerStandAloneClose();
        s_ped_state = PED_NONE;
        s_ped_steps = 0;
        s_ped_wt = 0;
        initMPL();
        setupFIFO();
    }
    
    if(s_ped_state == PED_FULL && !s_ped_enabled) {
        LOGV_IF(EXTRA_VERBOSE, "sys stopping full power pedometer");
        MLDisablePedometerFullPower();
        s_ped_state = PED_NONE;
        s_ped_steps = 0;
        s_ped_wt = 0;
    }
 
    s_sys_ped_enabled = false;

    set_power_states(*s_enabledMask, false);
    /* KLP -- we should call update_rates here, but cannot as the context object is unavailable */
    
    pthread_mutex_unlock(&mpld_mutex);
    return ML_SUCCESS;
}

int getSteps(void) {
    VFUNC_LOG;
    unsigned long steps = 0;
	
    pthread_mutex_lock(&mpld_mutex);
    if(s_ped_state == PED_FULL) {
        steps = s_ped_steps;
    } else if(s_ped_state == PED_STANDALONE) {
        if(MLPedometerStandAloneGetNumOfSteps(&steps) == ML_SUCCESS) {
            s_ped_steps = steps;
        }
    } else if(s_ped_state == PED_SLEEP) {
        steps = s_ped_steps; //if state is PED_SLEEP, just return the current count
    }

    pthread_mutex_unlock(&mpld_mutex);
    LOGV_IF(EXTRA_VERBOSE, "getSteps returning %d", (int)steps);
    return (int)steps;
}

int getWalkTime(void) {
    VFUNC_LOG;
    unsigned long timeMs = 0;
    pthread_mutex_lock(&mpld_mutex);
    if(s_ped_state == PED_FULL) {
        if(MLGetPedometerFullPowerWalkTime(&timeMs) == ML_SUCCESS)
            s_ped_wt = timeMs;
    } else if(s_ped_state == PED_STANDALONE ) {
        if(MLPedometerStandAloneGetWalkTime(&timeMs) == ML_SUCCESS)
            s_ped_wt = timeMs;
    } else if(s_ped_state == PED_SLEEP){
        timeMs = s_ped_wt; //if state is PED_SLEEP, just return the current wt
    }

    pthread_mutex_unlock(&mpld_mutex);
    return (int)timeMs;
}

int clearPedData(void) {
    VFUNC_LOG;
    pthread_mutex_lock(&mpld_mutex);
    if(s_ped_state == PED_FULL) {
        MLSetPedometerFullPowerStepCount(0);
        MLSetPedometerFullPowerWalkTime(0);
    } else if(s_ped_state == PED_STANDALONE) {
        MLPedometerStandAloneSetNumOfSteps(0);
        MLPedometerStandAloneSetWalkTime(0);
    }
    s_ped_steps = 0;
    s_ped_wt = 0;
    pthread_mutex_unlock(&mpld_mutex);
    return ML_SUCCESS;
}
    
typedef struct {
    int (*fpStartPed)(void);
    int (*fpStopPed)(void);
    int (*fpGetSteps)(void);
    int (*fpGetWalkTime)(void);
    int (*fpClearPedData)(void);
} tMplPedometerApi;

tMplPedometerApi mplPedApi = {startPed, stopPed, getSteps, getWalkTime, clearPedData};
