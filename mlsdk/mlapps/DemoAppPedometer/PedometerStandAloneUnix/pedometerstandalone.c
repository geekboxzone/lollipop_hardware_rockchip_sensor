/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */

/*******************************************************************************
 *
 * $Id: pedometer.c 3863 2010-10-08 22:05:31Z nroyer $
 *
 *******************************************************************************/

#include <string.h>
#include <stdio.h>
#include <time.h>
#include <sys/select.h>
#include <unistd.h>

#include "mltypes.h"
#include "ml.h"
#include "slave.h"
#include "compass.h"
#include "mlFIFO.h"
#include "mldl.h"
#include "pedometer.h"

#include "mlsetup.h"
#include "helper.h"
#include "int.h"

#include "orientation.h"
#include "gesture.h"
#include "mlpedometer_lowpower.h"
#include "mlpedometer_fullpower.h"
#include "helper.h"
#include "gestureMenu.h"
#include "mlerrorcode.h"

#include "log.h"
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-main"
#include "pedometer_lp_example.h"

int main(int argc, char *argv[])
{
    unsigned short accelid = ID_INVALID;
    unsigned short compassid = ID_INVALID;
    unsigned char *verStr;
    const char *ints[5];
    int handles[DIM(ints)];

    ints[0] = "/dev/mpuirq";
    ints[1] = "/dev/accelirq";
    ints[2] = "/dev/timerirq"; // "/dev/compassirq";
    ints[3] = "/dev/pressureirq";
    ints[4] = "/dev/mpu";

    CALL_N_CHECK( MLVersion(&verStr) );
    MPL_LOGI("%s\n", verStr);

    if (MenuHwChoice(&accelid, &compassid) == ML_SUCCESS) {
        CALL_CHECK_N_RETURN_ERROR( 
            SetupPlatform(PLATFORM_ID_MSB, accelid, compassid) );
    }

    CALL_CHECK_N_RETURN_ERROR( MLSerialOpen("/dev/mpu") );
    handles[4] = (int) MLSerialGetHandle();
    IntOpen(ints, handles, DIM(ints) - 1);
    PedometerLpExample(
        PLATFORM_ID_MSB, accelid, compassid, handles, DIM(ints));
    CALL_N_CHECK( IntClose(handles, DIM(handles) - 1) );
    CALL_CHECK_N_RETURN_ERROR( MLSerialClose() );

    return ML_SUCCESS;
}

