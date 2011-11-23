/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */

/******************************************************************************
 *
 * $Id: pedometer.c 3863 2010-10-08 22:05:31Z nroyer $
 *
 *****************************************************************************/

#include <string.h>
#include <stdio.h>
#include <time.h>

#include "ml.h"
#include "slave.h"
#include "compass.h"
#include "mlFIFO.h"
#include "mldl.h"
#include "pedometer.h"
#include "orientation.h"
#include "gesture.h"
#include "mldl_cfg.h"

#include "mlsetup.h"
#include "helper.h"
#include "gestureMenu.h"
#include "mlerrorcode.h"

#include "mlpedometer_lowpower.h"
#include "mlpedometer_fullpower.h"

#include "int.h"
#include "mlos.h"

#include "log.h"
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-main"

#define CHECK_WARNING(x) {                                                   \
    int reSuLt = x;                                                          \
    if(ML_SUCCESS != reSuLt) {                                               \
        MPL_LOGI("%s returned %s (#%d)\n", #x, MLErrorCode(reSuLt), reSuLt); \
    }                                                                        \
    else                                                                     \
        MPL_LOGI("%s\n", #x);                                                \
}

unsigned long stepCount;
unsigned long walkTime;
static tGestureMenuParams gestureParams;
static int flag = 0;
static struct stepParams params;
static unsigned int minSteps = 5;
static unsigned int minStepsTime = 3000;

enum ped_power_state {
    PED_POWER_SLEEP,
    PED_POWER_LOW,
    PED_POWER_FULL
};

void PrintPedometerMenu(void) {
    MPL_LOGI("\n\n");
    MPL_LOGI("   g) Increase Step Buffer : %d\n", minSteps);
    MPL_LOGI("   G) Decrease Step Buffer\n");
    MPL_LOGI("   v) Increase Step Buffer Time: %d\n", minStepsTime);
    MPL_LOGI("   V) Decrease Step Buffer Time\n");
}

static int ProcessKbhit(void)
{
    int exit = FALSE;
    tMLError result = ML_ERROR;

    /* Dynamic keyboard processing */
    char ch = ConsoleGetChar();

    result = GestureMenuProcessChar(&gestureParams, ch);
    if (ML_SUCCESS == result || /*ch == ' ' ||*/ ch == '\n' || ch == '\r') {
        exit = FALSE;
    } else if (ch == 'b') {
        flag ^= 0x20;
    } else if (ch == 'h') {
        PrintPedometerMenu();
    } else if (ch == 'g') {
        result = MLSetPedometerFullPowerStepBuffer(++minSteps);
        MPL_LOGI("MLSetPedometerFullPowerStepBuffer(%d)\n", minSteps);
        params.minSteps = minSteps;
    } else if (ch == 'G') {
        --minSteps;
        if (minSteps < 0) 
            minSteps = 0;
        result = MLSetPedometerFullPowerStepBuffer(minSteps);
        MPL_LOGI("MLSetPedometerFullPowerStepBuffer(%d)\n", minSteps);
        params.minSteps = minSteps;
    } else if (ch == 'v') {
        minStepsTime += 200;
        MLSetPedometerFullPowerStepBufferResetTime(minStepsTime);
        MPL_LOGI("MLSetPedometerFullPowerStepBufferResetTime(%d)\n", minStepsTime);
        params.maxStepBufferTime = minStepsTime / 5;
    } else if (ch == 'V') {
        minStepsTime -= 200;
        if (minStepsTime < 0) 
            minStepsTime = 0;
        MLSetPedometerFullPowerStepBufferResetTime(minStepsTime);
        MPL_LOGI("MLSetPedometerFullPowerStepBufferResetTime(%d)\n", minStepsTime);
        params.maxStepBufferTime = minStepsTime / 5;
    } else {
        exit = TRUE;
    }
    return exit;
}

void dumpData(void)
{
    if (flag & 0x20) {
        int ii;
        float data[6];
        float compData[4];
        long fixedData[6];

        FIFOGetAccel(fixedData);
        FIFOGetGyro(&fixedData[3]);
        for (ii = 0; ii < 6; ii++) {
            data[ii] = fixedData[ii] / 65536.0f;
        }
        MLGetFloatArray(ML_MAGNETOMETER,compData);

        MPL_LOGI("A: %12.4f %12.4f %12.4f "
                 "G: %12.4f %12.4f %12.4f "
                 "C: %12.4f %12.4f %12.4f \n",
                 data[0], data[1], data[2],
                 data[3], data[4], data[5],
                 compData[0], compData[1], compData[2]);
    }
}

void DumpDataLowPower(void)
{
    if (flag & 0x20) {
        int ii;
        float data[6];

        long fixedData[6];

        FIFOGetAccel(fixedData);
        for (ii = 0; ii < 6; ii++) {
            data[ii] = fixedData[ii] / 65536.0f;
        }

        MPL_LOGI("A: %12.4f %12.4f %12.4f\n",
                 data[0], data[1], data[2]);
    }
}

static void OnStep(unsigned long step, unsigned long time)
{
    MPL_LOGI("Step %6lu, Time %8.3f\n",
             step, (float) time / 1000.);
    stepCount = step;
    walkTime = time;
}

void InitPedLowPower() 
{
    CHECK_WARNING(MLDmpPedometerStandAloneOpen());
    //CHECK_WARNING(MLSetMotionInterrupt(TRUE));
}

void InitPedFullPower()
{
    CHECK_WARNING(MLDmpOpen());
    CHECK_WARNING(MLEnablePedometerFullPower());
    CHECK_WARNING(MLSetFIFORate(6));
    CHECK_WARNING(MLSetFifoInterrupt(TRUE));
    CHECK_WARNING(MLSetPedometerFullPowerStepCallback(OnStep));
    CHECK_WARNING(MLSetBiasUpdateFunc(ML_ALL));
    CHECK_WARNING(MLEnableOrientation());
    CHECK_WARNING(MLSetOrientations(ML_ORIENTATION_ALL));
    CHECK_WARNING(MLSetOrientationCallback(PrintOrientation));
    PrintOrientation(0); /* reset for flip */
    //Register gestures to be detected
    CHECK_WARNING(MLEnableGesture());
    CHECK_WARNING(MLSetGestures(ML_GESTURE_ALL));
    CHECK_WARNING(MLSetGestureCallback(PrintGesture));

    // Set up default parameters and gesture menu
    GestureMenuSetDefaults(&gestureParams);
    CHECK_WARNING(GestureMenuSetMpl(&gestureParams));
}

void TransitionToLowPower(int *handles,
                          int numHandles)
{
    struct mpuirq_data **data;

    CHECK_WARNING(MLDmpClose());
    InitPedLowPower();
    CHECK_WARNING(MLPedometerStandAloneSetParams(&params));
    CHECK_WARNING(MLDmpPedometerStandAloneStart());
    CHECK_WARNING(MLPedometerStandAloneSetNumOfSteps(stepCount));
    CHECK_WARNING(MLPedometerStandAloneSetWalkTime(walkTime));
    data = InterruptPoll(handles, numHandles, 0, 0);
#ifdef LINUX
    if (data[4]->interruptcount) {
        ioctl(handles[4], MPU_PM_EVENT_HANDLED, 0);
    }
#endif
    InterruptPollDone(data);
    MLDLClearIntTrigger(INTSRC_AUX1);
    MLDLClearIntTrigger(INTSRC_MPU);
    MPL_LOGI("\n");
    MPL_LOGI("**** MPU LOW POWER ****\n");
}
void TransitionToFullPower(int *handles, int numHandles)
{
    CHECK_WARNING(MLDmpPedometerStandAloneClose());
    InitPedFullPower();
    CHECK_WARNING(MLSetPedometerFullPowerStepCount(stepCount));
    CHECK_WARNING(MLSetPedometerFullPowerWalkTime(walkTime));
    CHECK_WARNING(MLSetPedometerFullPowerParams(&params));
    CHECK_WARNING(MLDmpStart());
    MPL_LOGI("\n");
    MPL_LOGI("**** MPU FULL POWER ****\n");
}

/**
 * Runs the pedometer by polling the step counter.
 */
void PedometerLpExample(unsigned short platform,
                        unsigned short accelid,
                        unsigned short compassid,
                        int *handles,
                        int numHandles)
{
    unsigned long lastStep = 0;
    unsigned long lastWalkTime = 0;
    unsigned int state = PED_POWER_LOW;
    unsigned int pre_freeze_state = state;
    int mode;
    int exit = FALSE;
    tMLError result;
    struct mpuirq_data **data;

    lastStep  = 0;
    stepCount = 0;
    walkTime  = 0;

    MPL_LOGI("\n\nPerforming Pedometer\n\n");
    MPL_LOGI("You should see:\n"
             "\t"
             "pedometer step count incrementing as steps are taken\n"
             "\n");
    MPL_LOGI("Loading example...\n");
    MPL_LOGI("Select sensitivity:\n");
    MPL_LOGI("   0) Low    (less steps)\n");
    MPL_LOGI("   1) Normal (more steps)\n");
    MPL_LOGI("   2) High   (most steps)\n");
    scanf("%d", &mode);

    PrintPedometerMenu();

    InitPedLowPower();

    if (mode == 0) {
        params.threshold     = 30000000L;
        params.minUpTime     = 320; //  3.125 Hz maximum
        params.maxUpTime     = 1200; // 0.833 Hz minimum 
        params.minSteps      = 5;
        params.minEnergy     = 0x10000000L;
        params.maxStepBufferTime = 2000;
        params.clipThreshold = 0x06000000L;
    } else if (mode == 1) {
        params.threshold     = 25000000L;
        params.minUpTime     = 320;  // 3.125 Hz maximum
        params.maxUpTime     = 1200; // 0.833 Hz minimum 
        params.minSteps      = 5;
        params.minEnergy     = 0x0d000000L;
        params.maxStepBufferTime = 2500;
        params.clipThreshold = 0x06000000L;
    } else {
        params.threshold     = 20000000L;
        params.minUpTime     = 200;  // 5.00 Hz maximum
        params.maxUpTime     = 1500; // 0.66 Hz minimum 
        params.minSteps      = 5;
        params.minEnergy     = 0x0a000000L;
        params.maxStepBufferTime = 3000;
        params.clipThreshold = 0x06000000L;
    }

    CHECK_WARNING(MLPedometerStandAloneSetParams(&params));

    MPL_LOGI("Example Loaded\n");
    MPL_LOGI("\n");

    CHECK_WARNING(MLPedometerStandAloneSetNumOfSteps(0));
    CHECK_WARNING(MLDmpPedometerStandAloneStart());
    MPL_LOGI("\n");
    MPL_LOGI("**** MPU LOW POWER ****\n");
    state = PED_POWER_LOW;
    pre_freeze_state = state;
    //Loop until a key is hit
    while (!exit) {
        
        if (state == PED_POWER_LOW) {
            CHECK_WARNING(MLPedometerStandAloneGetWalkTime(&walkTime));
            result = MLPedometerStandAloneGetNumOfSteps(&stepCount);
            if (result == ML_SUCCESS) {
                if ((walkTime != lastWalkTime) || (stepCount != lastStep)) {
                    lastStep = stepCount;
                    lastWalkTime = walkTime;
                    MPL_LOGI("Step %6lu, Time %8.3f s\n",
                             lastStep,
                             (float)walkTime / 1000.);
                }
            }

            // No Motion tracking
            if (0) {
                unsigned char data[16];
                CHECK_WARNING(MLSLSerialRead(MLSerialGetHandle(), 
                                             0x68, MPUREG_23_RSVD, 6, data));
                MPL_LOGI("%02x%02x, %02x%02x, %02x%02x\n",
                         data[0], data[1],
                         data[2], data[3],
                         data[4], data[5]);

            }
        }
        
        data = InterruptPoll(handles, numHandles, 2, 50000);

        /* handle system suspend/resume */
#ifdef LINUX
        if (data[4]->interruptcount) {
            unsigned long power_state = *((unsigned long *)data[4]->data);
            if (power_state == MPU_PM_EVENT_SUSPEND_PREPARE &&
                state == PED_POWER_FULL) {
                pre_freeze_state = PED_POWER_FULL;
                TransitionToLowPower(handles, numHandles);
            } else if (power_state == MPU_PM_EVENT_POST_SUSPEND &&
 pre_freeze_state == PED_POWER_FULL) {
                TransitionToFullPower(handles, numHandles);
            }
            ioctl(handles[4], MPU_PM_EVENT_HANDLED, 0);
        }
#endif
        InterruptPollDone(data);

        if (state == PED_POWER_SLEEP && MLDLGetIntTrigger(INTSRC_AUX1)) {
            MPL_LOGI("\n");
            MPL_LOGI("**** MPU LOW POWER ****\n");

            CHECK_WARNING(MLPedometerStandAloneSetNumOfSteps(stepCount));
            CHECK_WARNING(MLPedometerStandAloneSetWalkTime(walkTime));
            CHECK_WARNING(MLDmpPedometerStandAloneStart());
            MLDLClearIntTrigger(INTSRC_AUX1);
            MLDLClearIntTrigger(INTSRC_MPU);
            state = PED_POWER_LOW;
            pre_freeze_state = state;
        }

        if (state == PED_POWER_LOW && MLDLGetIntTrigger(INTSRC_AUX1)) {
            CHECK_WARNING(MLPedometerStandAloneGetWalkTime(&walkTime));
            CHECK_WARNING(MLPedometerStandAloneGetNumOfSteps(&stepCount));
            MPL_LOGI("\n");
            MPL_LOGI("**** MPU SLEEP POWER ****\n");
            CHECK_WARNING(MLDmpPedometerStandAloneStop());
            MLDLClearIntTrigger(INTSRC_AUX1);
            MLDLClearIntTrigger(INTSRC_MPU);
            state = PED_POWER_SLEEP;
            pre_freeze_state = state;
        }
        
        if (state == PED_POWER_LOW && MLDLGetIntTrigger(INTSRC_MPU)) {
            CHECK_WARNING(MLUpdateData());
            DumpDataLowPower();
            MLDLClearIntTrigger(INTSRC_MPU);
        }

        if (state == PED_POWER_FULL && 
            (MLDLGetIntTrigger(INTSRC_MPU) || MLDLGetIntTrigger(INTSRC_AUX1))) {
            CHECK_WARNING(MLUpdateData());
            dumpData();
            MLDLClearIntTrigger(INTSRC_AUX1);
            MLDLClearIntTrigger(INTSRC_MPU);
        }

        if(ConsoleKbhit()) {
            char ch;
            switch (state) {
            case PED_POWER_SLEEP:
            case PED_POWER_LOW:
                /* Transition to full power */
                ch = ConsoleGetChar();
                switch (ch) {
                case 'q':
                    exit = TRUE;
                    break;
                case 'h':
                    PrintPedometerMenu();
                    break;
                case 'g':
                    minSteps += 2;
                case 'G':
                    --minSteps;
                    if (minSteps < 0) 
                        minSteps = 0;
                    result = MLPedometerStandAloneSetStepBuffer(minSteps);
                    MPL_LOGI("MLPedometerStandAloneSetStepBuffer(%d)\n",
                             minSteps);
                    break;
                case 'v':
                    minStepsTime += 400;
                case 'V':
                    minStepsTime -= 200;
                    if (minStepsTime < 0) 
                        minStepsTime = 0;
                    MLPedometerStandAloneSetStepBufferResetTime(minStepsTime);
                    MPL_LOGI(
                        "MLPedometerStandAloneSetStepBufferResetTime(%d)\n",
                        minStepsTime);
                    break;
                case 'b':
                    flag ^= 0x20;
                    if (flag & 0x20) {
                        FIFOSendAccel(ML_ELEMENT_1 |
                                      ML_ELEMENT_2 |
                                      ML_ELEMENT_3,
                                      ML_32_BIT);
                        MLSetFifoInterrupt(TRUE);
                    } else {
                        FIFOSendAccel(ML_ELEMENT_1 |
                                      ML_ELEMENT_2 |
                                      ML_ELEMENT_3,
                                      0);
                        MLSetFifoInterrupt(FALSE);
                    }
                    break;
                case '\n':
                case '\r':
                    /* Ignore carrage return and line feeds */
                    break;
                default:
                    TransitionToFullPower(handles,numHandles);
                    state = PED_POWER_FULL;
                    pre_freeze_state = state;
                    break;
                };
                break;
            case PED_POWER_FULL:
            default:
                /* Transition to low power */
                exit = ProcessKbhit();
                if (exit) {
                    TransitionToLowPower(handles, numHandles);
                    state = PED_POWER_LOW;
                    pre_freeze_state = state;
                    exit = FALSE;
                }
                break;
            };
        }
    }

    if (state == PED_POWER_LOW) {
        CHECK_WARNING(MLDmpPedometerStandAloneClose());
    } else if (PED_POWER_FULL == state) {
        CHECK_WARNING(MLDmpClose());
    }
}


