/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */

/******************************************************************************
 *
 * $Id: consoledmp.c 5144 2011-04-05 18:09:41Z mcaramello $
 *
 *****************************************************************************/

#include <stdio.h>
#include <time.h>
#include <sys/select.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include "mltypes.h"
#include "ml.h"
#include "mldl.h"
#include "slave.h"
#include "compass.h"
#include "mlFIFO.h"
#include "int.h"
#include "mpu.h"
#include "mldl_cfg.h"
#include "mlsupervisor_9axis.h"
#include "ml_mputest.h"
#include "mlstates.h"
#include "ml_stored_data.h"

#include "gesture.h"
#include "orientation.h"
#include "mlMathFunc.h"

#include "helper.h"
#include "mlsetup.h"
#include "mputest.h"

#include "gestureMenu.h"
#include "timerirq.h"

#include "mlos.h"
#include "mlmath.h"
#include "log.h"
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "main:"

#define TIMERIRQ_NAME "/dev/timerirq"

/* 
    Globals 
*/
static tGestureMenuParams gestureMenuParams;
unsigned int flag = 0x2;

/* Motion/no motion callback function */
void onMotion(unsigned short motionType)
{
    switch(motionType) {
        case ML_MOTION:
            MPL_LOGI("Motion\n");
            break;
        case ML_NO_MOTION:
            MPL_LOGI("No Motion\n");
            break;
        default:
            break;
    }
}

/* Heading Information for Compass */
void headingInfo(void)
{ 
    float hInfo[4];
    float eulerAngle[3];

    if (flag & 0x40) {
        CALL_N_CHECK(MLGetFloatArray(ML_EULER_ANGLES, eulerAngle));
        CALL_N_CHECK(MLGetFloatArray(ML_HEADING, hInfo));
        MPL_LOGI(
            "Heading : %+12.3f    Euler Angle : %12.3f    %12.3f    %12.3f \n",
            hInfo[0], eulerAngle[0], eulerAngle[1], eulerAngle[2]);
    }
}

void dumpData(void)
{
    if (flag & 0x20) {
        float data[6];
        float compData[3];

        memset(data, 0, sizeof(data));
        memset(compData, 0, sizeof(compData));

        if (MLDLGetCfg()->requested_sensors & ML_THREE_AXIS_ACCEL) {
            CALL_N_CHECK(MLGetFloatArray(ML_ACCELS, data));
        }
        
        if (MLDLGetCfg()->requested_sensors & ML_THREE_AXIS_GYRO) {
            CALL_N_CHECK(MLGetFloatArray(ML_GYROS, &data[3]));
        }
        if (MLDLGetCfg()->requested_sensors & ML_THREE_AXIS_COMPASS) {
            CALL_N_CHECK(MLGetFloatArray(ML_MAGNETOMETER, compData));
        }

        MPL_LOGI(("A: %12.4f %12.4f %12.4f "
                  "G: %12.4f %12.4f %12.4f "
                  "C: %12.4f %12.4f %12.4f \n"),
                 data[0], data[1], data[2],
                 data[3], data[4], data[5],
                 compData[0], compData[1], compData[2]);
    }
}

// Processed data callback function
void processedData(void)
{
    if (flag & 0x80) {
        float checksum = 0.0;
        float quat[4];
        int i;

        CALL_N_CHECK(FIFOGetQuaternionFloat(quat));
        for(i = 0; i < 4; i++)
            checksum += (quat[i] * quat[i]);
        MPL_LOGI("%12.4f %12.4f %12.4f %12.4f    -(%12.4f)\n",
                 quat[0], quat[1], quat[2], quat[3], sqrtf(checksum));
    }
}

//Orientation callback function
void onOrientation(unsigned short orientation)
{
    /* Determine if it was a flip */
    static int sLastOrientation = 0;
    int flip = orientation | sLastOrientation;

    if ((ML_X_UP | ML_X_DOWN) == flip) {
        MPL_LOGI("Flip about the X Axis: \n");
    } else if ((ML_Y_UP | ML_Y_DOWN) == flip) {
        MPL_LOGI("Flip about the Y axis: \n");
    } else if ((ML_Z_UP | ML_Z_DOWN) == flip) {
        MPL_LOGI("Flip about the Z axis: \n");
    }

    sLastOrientation = orientation;

    switch (orientation) {
        case ML_X_UP:
            MPL_LOGI("X Axis is up\n");
            break;
        case ML_X_DOWN:
            MPL_LOGI("X Axis is down\n");
            break;
        case ML_Y_UP:
            MPL_LOGI("Y Axis is up\n");
            break;
        case ML_Y_DOWN:
            MPL_LOGI("Y Axis is down\n");
            break;
        case ML_Z_UP:
            MPL_LOGI("Z Axis is up\n");
            break;
        case ML_Z_DOWN:
            MPL_LOGI("Z Axis is down\n");
            break;
        default:
            break;
    }
}

int test_register_dump(void) 
{
    unsigned int error = FALSE;
    unsigned char data;
    unsigned short res;
    static char buf[256];
    static char tmp[16];
    int ii;
    
    /* Successful whoami read.  Now read all registers */
    MPL_LOGD("Register Dump:\n    ");
    MPL_LOGD("        00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f\n");
    MPL_LOGD("         |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |\n");
    snprintf(buf, sizeof(buf), "    %02d: ", 0);
    for (ii = 0; ii < NUM_OF_MPU_REGISTERS; ii++) {
        res = MLSLSerialRead(
                MLSerialGetHandle(), MLDLGetMPUSlaveAddr(), ii, 1, &data);
        if (ML_SUCCESS != res) { 
            snprintf(tmp, sizeof(buf), "-- ");
            strcat(buf, tmp);
            error = TRUE;
        } else {
            snprintf(tmp,sizeof(buf), "%02x ", data);
            strcat(buf, tmp);
        }
        if ((ii + 1) % 16 == 0) {
            MPL_LOGD("%s\n", buf);
            snprintf(buf, sizeof(buf), "    %02d: ", 1 + (ii / 16));
        }
    }
    MPL_LOGD("%s\n", buf);

    return error;
}

int start_timerirqs(const char **ints, int *handles, int num_handles)
{
    int result;
    int ii;
    for (ii = 0; ii < num_handles; ii++) {
        if (0 == strcmp(ints[ii], TIMERIRQ_NAME)) {
            /* Check to see if the curren sensors are enabled */
            if ((MLDLGetCfg()->requested_sensors & ML_DMP_PROCESSOR &&
                 ii == INTSRC_MPU)
                ||
                (!(MLDLGetCfg()->requested_sensors & ML_DMP_PROCESSOR) &&
                 MLDLGetCfg()->requested_sensors & ML_THREE_AXIS_ACCEL
                 && ii == INTSRC_AUX1)
                ||
                (!(MLDLGetCfg()->requested_sensors & ML_SIX_AXIS_GYRO_ACCEL) &&
                 MLDLGetCfg()->requested_sensors & ML_THREE_AXIS_COMPASS
                 && ii == INTSRC_AUX2)
                ||
                (!(MLDLGetCfg()->requested_sensors & ML_NINE_AXIS) &&
                 MLDLGetCfg()->requested_sensors & ML_THREE_AXIS_PRESSURE
                 && ii == INTSRC_AUX3)) {
                    result = ioctl(handles[ii], TIMERIRQ_START,
                                   GetSampleStepSizeMs());
                    if (result) {
                        MPL_LOGE("TIMERIRQ_START returned %d\n", result);
                    } else {
                        MPL_LOGI("TIMERIRQ_STARTed with period %d, %d\n, %lx",
                                 GetSampleStepSizeMs(), ii, 
                                 MLDLGetCfg()->requested_sensors);
                        break;
                    }
            }
        }
    }
    return result;
}

int stop_timerirqs(const char **ints, int *handles, int num_handles)
{
    int ii;
    int result;
    for (ii = 0; ii < num_handles; ii++) {
        if (0 == strcmp(ints[ii], TIMERIRQ_NAME)) {
            result = ioctl(handles[ii], TIMERIRQ_STOP, 0);
            if (result) {
                MPL_LOGE("TIMERIRQ_STOP returned %d\n", result);
            }
        }
    }
    return result;
}

/* Main function */
int main(int argc, char *argv[])
{
    unsigned short accelid = ID_INVALID;
    unsigned short compassid = ID_INVALID;
    unsigned char reg[32];
    unsigned char *verStr;
    int key = 0, oldKey;
    int result;
    int ii;
    int intstatus;
    const char *ints[5];
    int handles[DIM(ints)];

    CALL_N_CHECK(MLVersion(&verStr));
    MPL_LOGI("%s\n", verStr);

    if(ML_SUCCESS == MenuHwChoice(&accelid, &compassid)) {
#ifdef M_HW
        CALL_CHECK_N_RETURN_ERROR(SetupPlatform(PLATFORM_ID_MANTIS_MSB,
                                                accelid, compassid));
#else
        CALL_CHECK_N_RETURN_ERROR(SetupPlatform(PLATFORM_ID_MSB,
                                                accelid, compassid));
#endif
    }

    CALL_CHECK_N_RETURN_ERROR(MLSerialOpen("/dev/mpu"));

    ints[0]  = "/dev/mpuirq";
    ints[1] = "/dev/accelirq";
    ints[2] = TIMERIRQ_NAME; // "/dev/compassirq";
    ints[3] = "/dev/pressureirq";
    ints[4] = "/dev/mpu";

    intstatus = ML_SUCCESS;
    IntOpen(ints, handles, DIM(ints) - 1);
    handles[4] = (int)MLSerialGetHandle();
    MPL_LOGI("/dev/mpu: %d\n", handles[4]);

    for (ii = 0; ii < DIM(handles); ii++) {
        if (handles[ii] < 0) {
            MPL_LOGE("IntOpen failed for %d, switching to timerirq\n", ii);
            handles[ii] = open(TIMERIRQ_NAME, O_RDWR);
            if (handles[ii] < 0) {
                MPL_LOGE("%s open failed, polling\n", TIMERIRQ_NAME);
                intstatus = ML_ERROR;
            } else {
                ints[ii] = TIMERIRQ_NAME;
            }
        } else {
            int flags;
            fcntl(handles[ii], F_GETFL, &flags);
            fcntl(handles[ii], F_SETFL, flags | O_NONBLOCK);
        }
    }

    CALL_CHECK_N_RETURN_ERROR(MLDmpOpen());

    CALL_CHECK_N_RETURN_ERROR(MLSetBiasUpdateFunc(ML_NONE));
    CALL_CHECK_N_RETURN_ERROR(MLEnable9axisFusion());
    CALL_CHECK_N_RETURN_ERROR(MLEnableTempComp());

    /* Register callback function to detect "motion" or "no motion" */
    CALL_CHECK_N_RETURN_ERROR(MLSetMotionCallback(onMotion));

    /* Set up orientation */
    CALL_CHECK_N_RETURN_ERROR(MLEnableOrientation());
    CALL_CHECK_N_RETURN_ERROR(MLSetOrientations(ML_ORIENTATION_ALL));
    CALL_CHECK_N_RETURN_ERROR(MLSetOrientationCallback(onOrientation));

    /* Register gestures to be detected */
    CALL_CHECK_N_RETURN_ERROR(MLEnableGesture());
    CALL_CHECK_N_RETURN_ERROR(MLSetGestureCallback(PrintGesture));
    CALL_CHECK_N_RETURN_ERROR(MLSetGestures(ML_GESTURE_ALL));

    /* Set up default parameters and gesture menu */
    GestureMenuSetDefaults(&gestureMenuParams);
    CALL_CHECK_N_RETURN_ERROR(GestureMenuSetMpl(&gestureMenuParams));
    
    /* Set up FIFO */
    CALL_CHECK_N_RETURN_ERROR(FIFOSendQuaternion(ML_32_BIT));
    CALL_CHECK_N_RETURN_ERROR(FIFOSendGyro(ML_ALL, ML_32_BIT));        
    CALL_CHECK_N_RETURN_ERROR(FIFOSendAccel(ML_ALL, ML_32_BIT));
    CALL_CHECK_N_RETURN_ERROR(MLSetFIFORate(6));

    /* Check to see if interrupts are available.  If so use them */
    if (ML_SUCCESS == intstatus) {
        CALL_CHECK_N_RETURN_ERROR(MLSetFifoInterrupt(TRUE));
        CALL_CHECK_N_RETURN_ERROR(MLSetMotionInterrupt(TRUE));
        MPL_LOGI("Interrupts Configured\n");
        flag |= 0x04;
        start_timerirqs(ints, handles, DIM(handles));
    } else {
        MPL_LOGI("Interrupts unavailable on this platform\n");
        flag &= ~0x04;
    }
           
    CALL_CHECK_N_RETURN_ERROR(MLDmpStart());
    
    PrintGestureMenu(&gestureMenuParams);
    test_register_dump();

    MPL_LOGI("Starting using flag %d\n", flag);
    
    while (1) {
        result = ConsoleKbhit();
        if (result) {
            oldKey = key;
            key = ConsoleGetChar();
        } else {
            oldKey = 0;
            key = 0;
        }

        if (key == 'q') {
            MPL_LOGI("quit...\n");
            break;
        } else if (key == '0') {
            MPL_LOGI("flag = 0\n");
            flag  = 0;
        } else if (key == '1') {
            if (flag & 1) {
                MPL_LOGI("flag &= ~1 - who am i\n");
                flag &= ~1;
            } else {
                MPL_LOGI("flag |=  1 - who am i\n");
                flag |= 1;
            }
        } else if (key == '2') {
            if (flag & 2) {
                MPL_LOGI("flag &= ~2 - MLUpdateData()\n");
                flag &= ~2;
            } else {
                MPL_LOGI("flag |=  2 - MLUpdateData()\n");
                flag |= 2;
            }
        } else if (key == '4') {
            if (flag & 4) {
                MPL_LOGI("flag &= ~4 - IntProcess()\n");
                flag &= ~4;
            } else {
                MPL_LOGI("flag |=  4 - IntProcess()\n");
                flag |= 4;
            }
        } else if (key == 'v') {
            if (flag & 0x80) {
                MPL_LOGI("flag &= ~0x80 - Quaternion\n");
                flag &= ~0x80;
                if (ML_SUCCESS != MLSetProcessedDataCallback(NULL))
                    MPL_LOGI("could not set the callbacki\n");
            } else {
                MPL_LOGI("flag |=  0x80  - Quaternion\n");
                flag |= 0x80;
                if (ML_SUCCESS != MLSetProcessedDataCallback(processedData))
                    MPL_LOGI("could not set the callbacki\n");
            }
        } else if (key == 'b') {
            if (flag & 0x20) {
                MPL_LOGI("flag &= ~0x20 - dumpData()\n");
                flag &= ~0x20;
            } else {
                MPL_LOGI("flag |=  0x20 - dumpData()\n");
                flag |= 0x20;
            }                      
        } else if (key == 'c') {
            if (flag & 0x40) {
                MPL_LOGI("flag &= ~0x40 - Heading & Euler Angle\n");
                flag &= ~0x40;
            } else {
                MPL_LOGI("flag |=  0x40 - Heading & Euler Angle\n");
                flag |= 0x40;
            }
        } else if (key  ==  't') {
            test_register_dump();
#ifndef M_HW
        } else if (key == 'V') {
            key = oldKey;
            MPL_LOGI("run MPU Self-Test...\n");
            CALL_CHECK_N_RETURN_ERROR(MLSelfTestRun());
            MLOSSleep(5);
            continue;
#endif
        } else if (key == 'l') {
            key = oldKey;
            MPL_LOGI("load calibration...\n");
            MPL_LOGI("\tstopping MPL\n");
            CALL_CHECK_N_RETURN_ERROR(MLDmpStop());
            MLOSSleep(5);
            MPL_LOGI("\tloading calibration\n");
            CALL_CHECK_N_RETURN_ERROR(MLLoadCalibration());
            MLOSSleep(5);
            MPL_LOGI("\trestarting MPL\n");
            CALL_CHECK_N_RETURN_ERROR(MLDmpStart());
            continue;
        } else if (key == 's') {
            static int toggle = 1;
            if (toggle) {
                MPL_LOGI("\tMLDmpStop\n");
                CALL_CHECK_N_RETURN_ERROR(MLDmpStop());
            } else {
                MPL_LOGI("\tMLDmpStart\n");
                CALL_CHECK_N_RETURN_ERROR(MLDmpStart());
            }
            toggle = toggle? 0:1;
            continue;
        } else if (key == 'h') {
            printf(
                "\n\n"
                "0   -   turn all the features OFF\n"
                "1   -   read WHO_AM_I\n"
                "2   -   call MLUpdateData()\n"
                "4   -   call IntProcess()\n"
                "v   -   print Quaternion data\n"
                "b   -   Print raw accel and gyro data\n"
                "c   -   Heading & Euler Angle information\n"
                "t   -   dump register information\n"
                "(Sh-V) -interrupt execution and run functional test\n"
                "l   -   load calibration from file\n"
                "h   -   show this help\n"
                "\n\n"
               );
            PrintGestureMenu(&gestureMenuParams);
        } else {
            (void)GestureMenuProcessChar(&gestureMenuParams, key);
            if (key == 'f' || key == 'F' || key == 'S') {
                stop_timerirqs(ints, handles, DIM(handles));
                start_timerirqs(ints, handles, DIM(handles));
            }
        }

        if (flag & 1) {
            CALL_CHECK_N_RETURN_ERROR(MLSLSerialRead(MLSerialGetHandle(), 
                                                      MLDLGetMPUSlaveAddr(), 0, 1, reg));
            MPL_LOGI("\nreg[0]=%02x\n", reg[0]);
            flag &= ~1;
        }
        if (flag & 2) {
            if (MLGetState() == ML_STATE_DMP_STARTED)
                CALL_N_CHECK(MLUpdateData());
        }
        if (flag & 4) {
            /* Long sleep, long enough to overflow the fifo so as to make it
             * obvious when it doesn't work */
            struct mpuirq_data **data;
            data = InterruptPoll(handles, DIM(handles), 2, 500000);
            if (data[4]->interruptcount) {
                ioctl(handles[4], MPU_PM_EVENT_HANDLED, 0);
            }
            InterruptPollDone(data);
        } else {
            MLOSSleep(GetSampleStepSizeMs());
        }
        if (flag & 0x20) {
            dumpData();
        }
        if (flag & 0x40) {
            headingInfo();
        }
    }    /* end of while(1) */

    stop_timerirqs(ints, handles, DIM(handles));
    CALL_CHECK_N_RETURN_ERROR(MLDisableOrientation());

    /* Close Motion Library */
    CALL_CHECK_N_RETURN_ERROR(MLDmpStop());

    printf("MLStoreCalibration\n");
    CALL_CHECK_N_RETURN_ERROR(MLStoreCalibration());

    CALL_CHECK_N_RETURN_ERROR(MLDmpClose());
    CALL_CHECK_N_RETURN_ERROR(MLSerialClose());

    IntClose(handles, DIM(handles));

    return ML_SUCCESS;
}
