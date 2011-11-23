/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/******************************************************************************
 *
 * $Id: mlsetup.c 5124 2011-04-01 21:58:56Z dsrivastava $
 *
 *****************************************************************************/
#undef MPL_LOG_NDEBUG
#define MPL_LOG_NDEBUG 1
/**
 *  @defgroup MLSETUP
 *  @brief  The Motion Library external slaves setup override suite.
 *
 *          Use these APIs to override the kernel/default settings in the
 *          corresponding data structures for gyros, accel, and compass.
 *
 *  @{
 *      @file mlsetup.c
 *      @brief The Motion Library external slaves setup override suite.
 */

/* ------------------ */
/* - Include Files. - */
/* ------------------ */

/*
    Defines
*/
// these have to appear before inclusion of mpu.h
#define CONFIG_MPU_SENSORS_MPU3050       y   // MPU Gyroscopes

#define CONFIG_MPU_SENSORS_KXSD9         y   // Kionix accel
#define CONFIG_MPU_SENSORS_KXTF9         y   // Kionix accel
#define CONFIG_MPU_SENSORS_LIS331DLH     y   // ST accelerometer
#define CONFIG_MPU_SENSORS_LSM303DLHA    y   // ST accelerometer
#define CONFIG_MPU_SENSORS_LIS3DH        y     // ST accelerometer
#define CONFIG_MPU_SENSORS_BMA150        y   // Bosch 150 accelerometer
#define CONFIG_MPU_SENSORS_BMA222        y   // Bosch 222 accelerometer
#define CONFIG_MPU_SENSORS_ADXL346       y   // ADI accelerometer
#define CONFIG_MPU_SENSORS_MMA8450       y   // Freescale MMA8450 accelerometer
#define CONFIG_MPU_SENSORS_MMA845X       y   // Freescale MMA845X accelerometer
#define CONFIG_MPU_SENSORS_MPU6000       y   // Mantis Accel

#define CONFIG_MPU_SENSORS_AK8975        y   // AKM compass
#define CONFIG_MPU_SENSORS_AMI30X        y   // AICHI AMI304/305 compass
#define CONFIG_MPU_SENSORS_AMI306        y   // AICHI AMI306 compass
#define CONFIG_MPU_SENSORS_HMC5883       y   // Honeywell compass
#define CONFIG_MPU_SENSORS_LSM303DLHM    y   // ST compass
#define CONFIG_MPU_SENSORS_YAS529        y   // Yamaha compass
#define CONFIG_MPU_SENSORS_YAS530        y   // Yamaha compass
#define CONFIG_MPU_SENSORS_MMC314X       y   // MEMSIC compass
#define CONFIG_MPU_SENSORS_HSCDTD002B    y   // ALPS compass
#define CONFIG_MPU_SENSORS_HSCDTD004A    y   // ALPS HSCDTD004A compass

#define CONFIG_MPU_SENSORS_BMA085        y   // Bosch 085 pressure

#include <stdio.h>
#include <string.h>

#include "mlsetup.h"

#include "slave.h"
#include "compass.h"
#include "pressure.h"
#include "ml.h"
#include "mldl.h"
#include "mlstates.h"

#include "log.h"
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-mlsetup"

#include "mpu.h"
#include "mldl_cfg.h"

/* Override these structures from mldl.c */
extern struct ext_slave_descr gAccel;
extern struct ext_slave_descr gCompass;
extern struct ext_slave_descr gPressure;
extern struct mpu3050_platform_data gPdata;

/*
    Typedefs
*/
typedef void tSetupFuncGyro     (void);
typedef void tSetupFuncAccel    (void);
typedef void tSetupFuncCompass  (void);
typedef void tSetupFuncPressure (void);

/*
    Prototypes
*/
void Rotate180DegAroundZAxis (signed char matrix[]);

#ifdef LINUX
#include <sys/ioctl.h>
#endif

/*********************************************************************
              Multi sensor board - PLATFORM_ID_MSB
*********************************************************************/

/* selectors */
tMLError SetupGyroCalibration_MSB(void);
tMLError SetupAccelCalibration_MSB(unsigned short accelSelection);
tMLError SetupCompassCalibration_MSB(unsigned short compassSelection);
tMLError SetupPressureCalibration_MSB(unsigned short pressureSelection);

tMLError SetupGyroCalibration_ST_6Axis(void);
tMLError SetupAccelCalibration_ST_6Axis(unsigned short accelId);

/* sensor specific */
static tSetupFuncGyro SetupMPUCalibration_MSB;
// accel
static tSetupFuncAccel SetupAccelKionixKXSD9Calibration_MSB;
static tSetupFuncAccel SetupAccelKionixKXTF9Calibration_MSB;
static tSetupFuncAccel SetupAccelSTLIS331Calibration_MSB;
static tSetupFuncAccel SetupAccelSTLSM303Calibration_MSB;
static tSetupFuncAccel SetupAccelBMA150Calibration_MSB;
static tSetupFuncAccel SetupAccelBMA222Calibration_MSB;
static tSetupFuncAccel SetupAccelADI346Calibration_MSB;
static tSetupFuncAccel SetupAccelMMA8450Calibration_MSB;
static tSetupFuncAccel SetupAccelMMA845XCalibration_MSB;
static tSetupFuncAccel SetupAccelSTLIS3DHCalibration_MSB;
// compass
static tSetupFuncCompass SetupCompassAKMCalibration_MSB;
static tSetupFuncCompass SetupCompassAICHICalibration_MSB;
static tSetupFuncCompass SetupCompassAMI306Calibration_MSB;
static tSetupFuncCompass SetupCompassHMCCalibration_MSB;
static tSetupFuncCompass SetupCompassLSM303Calibration_MSB;
static tSetupFuncCompass SetupCompassYAS529Calibration_MSB;
static tSetupFuncCompass SetupCompassYAS530Calibration_MSB;
static tSetupFuncCompass SetupCompassMMCCalibration_MSB;
static tSetupFuncCompass SetupCompassHSCDTD002BCalibration_MSB;
static tSetupFuncCompass SetupCompassHSCDTD004ACalibration_MSB;
// pressure
static tSetupFuncPressure SetupPressureBMA085Calibration_MSB;

/******************************************************************************
              USB dongle with - PLATFORM_ID_DONGLE/PLATFORM_ID_MANTIS_DONGLE
******************************************************************************/
tMLError SetupMantisCalibration(int side);
static tSetupFuncGyro SetupMPUCalibration_Dongle;
static tSetupFuncAccel SetupAccelKionixKXTF9Calibration_Dongle;
static tSetupFuncCompass SetupCompassAKMCalibration_Dongle;

/*********************************************************************
              Dragon - PLATFORM_ID_DRAGON_PROTOTYPE
*********************************************************************/
#ifdef M_HW
static tSetupFuncCompass SetupCompassAKMCalibration_Dragon;
#endif


/************************
        User API
*************************/

/**
 *  @brief  Setup the Hw orientation and full scale.
 *  @param  platfromId
 *              an user defined Id to distinguish the Hw platform in
 *              use from others.
 *  @param  accelId
 *              the accelerometer specific id, as specified in the MPL.
 *  @param  compassId
 *              the compass specific id, as specified in the MPL.
 *  @return ML_SUCCESS or a non-zero error code.
 */
tMLError SetupPlatform(
                unsigned short platformId,
                unsigned short accelId,
                unsigned short compassId)
{
    tMLError result;

    if (MLGetState() > ML_STATE_SERIAL_OPENED) 
        return ML_ERROR_SM_IMPROPER_STATE;

    memset(&gAccel, 0, sizeof(gAccel));
    memset(&gCompass, 0, sizeof(gCompass));
    memset(&gPressure, 0, sizeof(gPressure));
    memset(&gPdata, 0, sizeof(gPdata));

#ifdef LINUX
    /* On Linux initialize the global platform data with the driver defaults */
    {
        void *mpu_handle;
        static struct mldl_cfg mldl_cfg;
        struct ext_slave_descr accel;
        struct ext_slave_descr compass;
        struct ext_slave_descr pressure;
        struct mpu3050_platform_data pdata;
        mldl_cfg.accel   = &accel;
        mldl_cfg.compass = &compass;
        mldl_cfg.pressure = &pressure;
        mldl_cfg.pdata   = &pdata;

        MPL_LOGI("Setting the MPU_SET_PLATFORM_DATA\n");
        result = MLSLSerialOpen("/dev/mpu",&mpu_handle);
        if (result) {
            MPL_LOGE("MPU_SET_PLATFORM_DATA failed %d\n", result);
        }
        result = ioctl((int)mpu_handle,
                       MPU_GET_MPU_CONFIG,
                       &mldl_cfg);
        if (result) {
            MPL_LOGE("MPU_SET_PLATFORM_DATA failed %d\n", result);
        }
        MLSLSerialClose(mpu_handle);
        gPdata = pdata; /* Structure copy */
    }
#endif

    /*----  setup the gyros ----*/
    switch(platformId) {
    case PLATFORM_ID_MSB:
        result = SetupGyroCalibration_MSB();
        ERROR_CHECK(result);
        result = SetupAccelCalibration_MSB(accelId);
        ERROR_CHECK(result);
        result = SetupCompassCalibration_MSB(compassId);
        ERROR_CHECK(result);
        break;
    case PLATFORM_ID_ST_6AXIS:
        result = SetupAccelCalibration_ST_6Axis(accelId);
        ERROR_CHECK(result);
        result = SetupGyroCalibration_ST_6Axis();
        ERROR_CHECK(result);
        break;
    case PLATFORM_ID_DONGLE:
        SetupMPUCalibration_Dongle();
        SetupAccelKionixKXTF9Calibration_Dongle();
        SetupCompassAKMCalibration_Dongle();
        break;
#ifdef M_HW
    case PLATFORM_ID_MANTIS_PROTOTYPE:
        result = SetupMantisCalibration(1);
        ERROR_CHECK(result);
        break;
    case PLATFORM_ID_MANTIS_MSB:
        result = SetupMantisCalibration(0);
        ERROR_CHECK(result);
        result = SetupCompassCalibration_MSB(compassId);
        ERROR_CHECK(result);
        break;
    case PLATFORM_ID_MANTIS_USB_DONGLE:
        result = SetupMantisCalibration(2);
        ERROR_CHECK(result);
        SetupCompassAKMCalibration_Dongle();
        ERROR_CHECK(result);
        break;
    case PLATFORM_ID_DRAGON_PROTOTYPE:
        result = SetupMantisCalibration(1);
        ERROR_CHECK(result);
        SetupCompassAKMCalibration_Dragon();
        break;
#endif
    case PLATFORM_ID_MSB_10AXIS:
        result = SetupGyroCalibration_MSB();
        ERROR_CHECK(result);
        result = SetupAccelCalibration_MSB(accelId);
        ERROR_CHECK(result);
        result = SetupCompassCalibration_MSB(compassId);
        ERROR_CHECK(result);
        result = SetupPressureCalibration_MSB(0);
        ERROR_CHECK(result);
        break;
    default:
        MPL_LOGE("Unrecognized platform ID %d\n", platformId);
        return ML_ERROR_INVALID_PARAMETER;
        break;
    }
#ifdef LINUX
    /* On Linux override the orientation, level shifter etc */
    {
        void *mpu_handle;
        MPL_LOGI("Setting the MPU_SET_PLATFORM_DATA\n");
        result = MLSLSerialOpen("/dev/mpu",&mpu_handle);
        if (result) {
            MPL_LOGE("MPU_SET_PLATFORM_DATA failed %d\n", result);
        }
        result = ioctl((int)mpu_handle,
                        MPU_SET_PLATFORM_DATA,
                        &gPdata);
        if (result) {
            MPL_LOGE("MPU_SET_PLATFORM_DATA failed %d\n", result);
        }
        MLSLSerialClose(mpu_handle);
    }
#endif
    return ML_SUCCESS;
}

/**
 *  @brief  performs a 180' rotation around Z axis to reflect
 *          usage of the multi sensor board (MSB) with the
 *          beagleboard
 *  @note   assumes well formed mounting matrix, with only
 *          one 1 for each row.
 */
void Rotate180DegAroundZAxis(signed char matrix[])
{
    int i;
    for(i=0; i<6; i++) {
        matrix[i] = -matrix[i];
    }
}

/*******************************************************************************
        MULTI SENSOR BOARD - Selection and Setup APIs
*******************************************************************************/

tMLError SetupGyroCalibration_MSB(void)
{
    SetupMPUCalibration_MSB();

#ifndef WIN32
    Rotate180DegAroundZAxis(gPdata.orientation);
#endif

    return ML_SUCCESS;
}


tMLError SetupAccelCalibration_MSB(unsigned short accelId)
{
     /*----  setup the accels ----*/
     switch(accelId) {
         case ACCEL_ID_LSM303:
             SetupAccelSTLSM303Calibration_MSB();
             break;
         case ACCEL_ID_LIS331:
             SetupAccelSTLIS331Calibration_MSB();
             break;
         case ACCEL_ID_KXSD9:
             SetupAccelKionixKXSD9Calibration_MSB();
             break;
         case ACCEL_ID_KXTF9:
             SetupAccelKionixKXTF9Calibration_MSB();
             break;
         case ACCEL_ID_BMA150:
             SetupAccelBMA150Calibration_MSB();
             break;
         case ACCEL_ID_BMA222:
             SetupAccelBMA222Calibration_MSB();
             break;
         case ACCEL_ID_ADI346:
             SetupAccelADI346Calibration_MSB();
             break;
         case ACCEL_ID_MMA8450:
             SetupAccelMMA8450Calibration_MSB();
             break;
         case ACCEL_ID_MMA845X:
             SetupAccelMMA845XCalibration_MSB();
             break;
         case ACCEL_ID_LIS3DH:
             SetupAccelSTLIS3DHCalibration_MSB();
             break;
         default:
             break;
    }

#ifndef WIN32
    if (accelId != ID_INVALID)
        Rotate180DegAroundZAxis(gPdata.accel.orientation);
#endif

    return ML_SUCCESS;
}

tMLError SetupAccelCalibration_ST_6Axis(unsigned short accelId)
{
    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.accel.orientation, 0, sizeof(gPdata.accel.orientation));
    gPdata.accel.orientation[0] = 1;
    gPdata.accel.orientation[4] = 1;
    gPdata.accel.orientation[8] = 1;

    gPdata.accel.adapt_num = 0;
    gPdata.accel.bus       = EXT_SLAVE_BUS_SECONDARY;
    gPdata.accel.address         = ACCEL_SLAVEADDR_LIS331;
    if (accelId == ACCEL_ID_LIS331) {
#ifndef LINUX
        gPdata.accel.get_slave_descr = lis331dlh_get_slave_descr;
        gAccel = *lis331dlh_get_slave_descr();
#endif
    } else {
        return ML_ERROR_INVALID_PARAMETER;
    }
    return ML_SUCCESS;
}

tMLError SetupCompassCalibration_MSB(unsigned short compassId)
{
    /*----  setup the compass ----*/
     switch(compassId) {
         case COMPASS_ID_AKM:
             SetupCompassAKMCalibration_MSB();
             break;
         case COMPASS_ID_AMI30X:
             SetupCompassAICHICalibration_MSB();
             break;
         case COMPASS_ID_AMI306:
             SetupCompassAMI306Calibration_MSB();
             break;         
         case COMPASS_ID_LSM303:
             SetupCompassLSM303Calibration_MSB();
             break;
         case COMPASS_ID_HMC5883:
             SetupCompassHMCCalibration_MSB();
             break;
         case COMPASS_ID_YAS529:
             SetupCompassYAS529Calibration_MSB();
             break;
         case COMPASS_ID_YAS530:
             SetupCompassYAS530Calibration_MSB();
             break;
         case COMPASS_ID_MMC314X:
             SetupCompassMMCCalibration_MSB();
             break;
         case COMPASS_ID_HSCDTD002B:
             SetupCompassHSCDTD002BCalibration_MSB();
             break;
         case COMPASS_ID_HSCDTD004A:
             SetupCompassHSCDTD004ACalibration_MSB();
             break;
         default:
             break;
    }

#ifndef WIN32
    if (compassId != ID_INVALID)
        Rotate180DegAroundZAxis(gPdata.compass.orientation);
#endif

    return ML_SUCCESS;
}

tMLError SetupPressureCalibration_MSB(unsigned short pressureId)
{
    /*----  setup the compass ----*/
    switch(pressureId) {
        case PRESSURE_ID_BMA085:
        default:
            SetupPressureBMA085Calibration_MSB();
            break;
    }

    return ML_SUCCESS;
}


/*******************************************************************************
 *        WARNING :
 *        all the mounting matrices for the Multi sensor board
 *        default on Windows.  A 180' rotation around Z has to be
 *        applyed to correctly work with the Beagleboard mounting.
 ******************************************************************************/


/**************************
    Gyro Setup Functions
***************************/

void SetupMPUCalibration_MSB (void)
{
    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.orientation, 0, sizeof(gPdata.orientation));
    gPdata.orientation[0] =  1;
    gPdata.orientation[4] = -1;
    gPdata.orientation[8] = -1;

    /* Interrupt */
    gPdata.int_config = 0x10;
}

/****************************
    Accel Setup Functions
*****************************/

static
void SetupAccelKionixKXSD9Calibration_MSB (void)
{
    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.accel.orientation, 0, sizeof(gPdata.accel.orientation));
    gPdata.accel.orientation[1] = +1;
    gPdata.accel.orientation[3] = +1;
    gPdata.accel.orientation[8] = -1;

    gPdata.accel.adapt_num       = 0;
    gPdata.accel.bus             = EXT_SLAVE_BUS_SECONDARY;
#ifndef LINUX
    gAccel = *kxsd9_get_slave_descr();
    gPdata.accel.get_slave_descr = kxsd9_get_slave_descr;
#endif
    gPdata.accel.address         = ACCEL_SLAVEADDR_KXSD9;
}

static
void SetupAccelKionixKXTF9Calibration_MSB(void)
{
    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.accel.orientation, 0, sizeof(gPdata.accel.orientation));
    gPdata.accel.orientation[1] = +1;   // X is  Y
    gPdata.accel.orientation[3] = +1;   // Y is  X
    gPdata.accel.orientation[8] = -1;   // Z is -Z

    gPdata.accel.adapt_num       = 0;
    gPdata.accel.bus             = EXT_SLAVE_BUS_SECONDARY;
#ifndef LINUX
    gAccel = *kxtf9_get_slave_descr();
    gPdata.accel.get_slave_descr = kxtf9_get_slave_descr;
#endif
    gPdata.accel.address         = ACCEL_SLAVEADDR_KXTF9;
}

static
void SetupAccelSTLIS331Calibration_MSB (void)
{
    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.accel.orientation, 0, sizeof(gPdata.accel.orientation));
    gPdata.accel.orientation[1] = -1;   // X is -Y
    gPdata.accel.orientation[3] = -1;   // Y is -X
    gPdata.accel.orientation[8] = -1;   // Z is -Z

    gPdata.accel.adapt_num       = 0;
    gPdata.accel.bus             = EXT_SLAVE_BUS_SECONDARY;
#ifndef LINUX
    gAccel = *lis331dlh_get_slave_descr();
    gPdata.accel.get_slave_descr = lis331dlh_get_slave_descr;
#endif
    gPdata.accel.address         = ACCEL_SLAVEADDR_LIS331;
}

static
void SetupAccelSTLIS3DHCalibration_MSB (void)
{

    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.accel.orientation, 0, sizeof(gPdata.accel.orientation));
    gPdata.accel.orientation[1] = 1;   // X is -Y
    gPdata.accel.orientation[3] = -1;   // Y is -X
    gPdata.accel.orientation[8] = 1;   // Z is -Z

    gPdata.accel.adapt_num       = 0;
    gPdata.accel.bus             = EXT_SLAVE_BUS_SECONDARY;
#ifndef LINUX
    gAccel = *lis3dh_get_slave_descr();
    gPdata.accel.get_slave_descr = lis3dh_get_slave_descr;
#endif
    gPdata.accel.address         = ACCEL_SLAVEADDR_LIS3DH;
}

static
void SetupAccelSTLSM303Calibration_MSB (void)
{
    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.accel.orientation, 0, sizeof(gPdata.accel.orientation));
    gPdata.accel.orientation[1] = -1;   // X is -Y
    gPdata.accel.orientation[3] =  1;   // Y is X
    gPdata.accel.orientation[8] =  1;   // Z is Z

    gPdata.accel.adapt_num       = 0;
    gPdata.accel.bus             = EXT_SLAVE_BUS_SECONDARY;
#ifndef LINUX
    gAccel = *lsm303dlha_get_slave_descr();
    gPdata.accel.get_slave_descr = lsm303dlha_get_slave_descr;
#endif
    gPdata.accel.address         = ACCEL_SLAVEADDR_LSM303;
}

static
void SetupAccelBMA150Calibration_MSB (void)
{
    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.accel.orientation, 0, sizeof(gPdata.accel.orientation));
    gPdata.accel.orientation[0] =  1;   // X is  X
    gPdata.accel.orientation[4] = -1;   // Y is -Y
    gPdata.accel.orientation[8] = -1;   // Z is -Z

    gPdata.accel.adapt_num       = 0;
    gPdata.accel.bus             = EXT_SLAVE_BUS_SECONDARY;
#ifndef LINUX
    gAccel = *bma150_get_slave_descr();
    gPdata.accel.get_slave_descr = bma150_get_slave_descr;
#endif
    gPdata.accel.address         = ACCEL_SLAVEADDR_BMA150;
}

static
void SetupAccelBMA222Calibration_MSB (void)
{

    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.accel.orientation, 0, sizeof(gPdata.accel.orientation));
    gPdata.accel.orientation[0] =  1;   // X is X
    gPdata.accel.orientation[4] =  1;   // Y is Y
    gPdata.accel.orientation[8] =  1;   // Z is Z

    gPdata.accel.adapt_num       = 0;
    gPdata.accel.bus             = EXT_SLAVE_BUS_SECONDARY;
#ifndef LINUX
    gAccel = *bma222_get_slave_descr();
    gPdata.accel.get_slave_descr = bma222_get_slave_descr;
#endif
    gPdata.accel.address         = ACCEL_SLAVEADDR_BMA222;
}

static
void SetupAccelADI346Calibration_MSB (void)
{
    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.accel.orientation, 0, sizeof(gPdata.accel.orientation));
    gPdata.accel.orientation[0] =  1;   // X is  X
    gPdata.accel.orientation[4] = -1;   // Y is -Y
    gPdata.accel.orientation[8] = -1;   // Z is -Z

    gPdata.accel.adapt_num       = 0;
    gPdata.accel.bus             = EXT_SLAVE_BUS_SECONDARY;
#ifndef LINUX
    gAccel = *adxl346_get_slave_descr();
    gPdata.accel.get_slave_descr = adxl346_get_slave_descr;
#endif
    gPdata.accel.address         = ACCEL_SLAVEADDR_ADI346;
}


static
void SetupAccelMMA8450Calibration_MSB (void)
{

    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.accel.orientation, 0, sizeof(gPdata.accel.orientation));
    gPdata.accel.orientation[1] = -1;   // X is -Y
    gPdata.accel.orientation[3] = -1;   // Y is -X
    gPdata.accel.orientation[8] = -1;   // Z is -Z

    gPdata.accel.adapt_num       = 0;
    gPdata.accel.bus             = EXT_SLAVE_BUS_SECONDARY;
#ifndef LINUX
    gAccel = *mma8450_get_slave_descr();
    gPdata.accel.get_slave_descr = mma8450_get_slave_descr;
#endif
    gPdata.accel.address         = ACCEL_SLAVEADDR_MMA8450;
}


static
void SetupAccelMMA845XCalibration_MSB (void)
{

    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.accel.orientation, 0, sizeof(gPdata.accel.orientation));
#if 1
    gPdata.accel.orientation[1] = -1;   // X is -Y
    gPdata.accel.orientation[3] = -1;   // Y is -X
    gPdata.accel.orientation[8] = -1;   // Z is -Z
#else /*MMA8452*/
    gPdata.accel.orientation[0] = 1;   // X is X
    gPdata.accel.orientation[4] = 1;   // Y is Y
    gPdata.accel.orientation[8] = 1;   // Z is Z
#endif

    gPdata.accel.adapt_num       = 0;
    gPdata.accel.bus             = EXT_SLAVE_BUS_SECONDARY;
#ifndef LINUX
    gAccel = *mma845x_get_slave_descr();
    gPdata.accel.get_slave_descr = mma845x_get_slave_descr;
#endif
    gPdata.accel.address         = ACCEL_SLAVEADDR_MMA845X;
}


/*****************************
    Compass Setup Functions
******************************/
static
void SetupCompassAKMCalibration_MSB(void)
{

    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.compass.orientation, 0, sizeof(gPdata.compass.orientation));

#if 1 /* AKM8975 */
    gPdata.compass.orientation[0] = -1;   // X is -X
    gPdata.compass.orientation[4] = -1;   // Y is -Y
    gPdata.compass.orientation[8] =  1;   // Z is  Z
#else /* AKM8975C */
    gPdata.compass.orientation[0] =  1;
    gPdata.compass.orientation[4] = -1;
    gPdata.compass.orientation[8] = -1;
#endif

    gPdata.compass.adapt_num       = 0;
#ifdef M_HW
    gPdata.compass.bus             = EXT_SLAVE_BUS_SECONDARY;
#else
    gPdata.compass.bus             = EXT_SLAVE_BUS_PRIMARY;
#endif
#ifndef LINUX
    gCompass = *ak8975_get_slave_descr();
    gPdata.compass.get_slave_descr = ak8975_get_slave_descr;
#endif
    gPdata.compass.address         = COMPASS_SLAVEADDR_AKM;
}

static
void SetupCompassMMCCalibration_MSB (void)
{
    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.compass.orientation, 0, sizeof(gPdata.compass.orientation));
    gPdata.compass.orientation[1] = 1;   // X is Y
    gPdata.compass.orientation[3] = 1;   // Y is X
    gPdata.compass.orientation[8] = -1;   // Z is  Z

    gPdata.compass.adapt_num       = 0;
    gPdata.compass.bus             = EXT_SLAVE_BUS_PRIMARY;
#ifndef LINUX
    gCompass = *mmc314x_get_slave_descr();
    gPdata.compass.get_slave_descr = mmc314x_get_slave_descr;
#endif
    gPdata.compass.address         = COMPASS_SLAVEADDR_MMC314X;
}

static
void SetupCompassAICHICalibration_MSB (void)
{
    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.compass.orientation, 0, sizeof(gPdata.compass.orientation));

#if 1 /*AMI304*/
    gPdata.compass.orientation[0] = -1;   // X is -X
    gPdata.compass.orientation[4] =  1;   // Y is  Y
    gPdata.compass.orientation[8] = -1;   // Z is -Z
    
    gPdata.compass.adapt_num       = 0;
    gPdata.compass.bus             = EXT_SLAVE_BUS_PRIMARY;
    gPdata.compass.address         = COMPASS_SLAVEADDR_AMI304;
#else /*AMI305*/
    gPdata.compass.orientation[1] =  1;   // X is  Y
    gPdata.compass.orientation[3] = -1;   // Y is -X
    gPdata.compass.orientation[8] =  1;   // Z is  Z
    
    gPdata.compass.adapt_num       = 0;
    gPdata.compass.bus             = EXT_SLAVE_BUS_PRIMARY;
    gPdata.compass.address         = COMPASS_SLAVEADDR_AMI30X;

#endif
#ifndef LINUX
    gCompass = *ami30x_get_slave_descr();
    gPdata.compass.get_slave_descr = ami30x_get_slave_descr;
#endif
}

static
void SetupCompassAMI306Calibration_MSB (void)
{
    MPL_LOGV("Calibrating '%s'\n", __func__);

    gPdata.compass.orientation[1] =  -1;   // X is  Y
    gPdata.compass.orientation[3] =  +1;   // Y is -X
    gPdata.compass.orientation[8] =  +1;   // Z is  Z
    
    gPdata.compass.adapt_num       = 0;
    gPdata.compass.bus             = EXT_SLAVE_BUS_PRIMARY;
#ifndef LINUX
    gCompass = *ami306_get_slave_descr();
    gPdata.compass.get_slave_descr = ami306_get_slave_descr;
#endif
    gPdata.compass.address         = COMPASS_SLAVEADDR_AMI30X;
}

static
void SetupCompassHMCCalibration_MSB (void)
{
    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.compass.orientation, 0, sizeof(gPdata.compass.orientation));
    gPdata.compass.orientation[0] =  1;   // X is -Y
    gPdata.compass.orientation[4] = -1;   // Y is -X
    gPdata.compass.orientation[8] = -1;   // Z is -Z

    gPdata.compass.adapt_num       = 0;
    gPdata.compass.bus             = EXT_SLAVE_BUS_PRIMARY;
#ifndef LINUX
    gCompass = *hmc5883_get_slave_descr();
    gPdata.compass.get_slave_descr = hmc5883_get_slave_descr;
#endif
    gPdata.compass.address         = COMPASS_SLAVEADDR_HMC5883;
}


static
void SetupCompassLSM303Calibration_MSB (void)
{
    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.compass.orientation, 0, sizeof(gPdata.compass.orientation));
    gPdata.compass.orientation[1] = -1;   // X is -Y
    gPdata.compass.orientation[3] =  1;   // Y is X
    gPdata.compass.orientation[8] =  1;   // Z is Z

    gPdata.compass.adapt_num       = 0;
    gPdata.compass.bus             = EXT_SLAVE_BUS_PRIMARY;
#ifndef LINUX
    gCompass = *lsm303dlhm_get_slave_descr();
    gPdata.compass.get_slave_descr = lsm303dlhm_get_slave_descr;
#endif
    gPdata.compass.address         = COMPASS_SLAVEADDR_HMC5883;
}


static
void SetupCompassYAS530Calibration_MSB (void)
{
    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Interrupt configuration */
    gPdata.int_config = 0x10;

    /* Orientation */
    memset(gPdata.compass.orientation, 0, sizeof(gPdata.compass.orientation));
    gPdata.compass.orientation[1] =  1;   // X is  Y
    gPdata.compass.orientation[3] = -1;   // Y is -X
    gPdata.compass.orientation[8] = 1;   // Z is Z

    gPdata.compass.adapt_num       = 0;
    gPdata.compass.bus             = EXT_SLAVE_BUS_PRIMARY;
#ifndef LINUX
    gCompass = *yas530_get_slave_descr();
    gPdata.compass.get_slave_descr = yas530_get_slave_descr;
#endif
    gPdata.compass.address         = COMPASS_SLAVEADDR_YAS530;
}

static
void SetupCompassYAS529Calibration_MSB (void)
{
    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Interrupt configuration */
    gPdata.int_config = 0x10;

    /* Orientation */
    memset(gPdata.compass.orientation, 0, sizeof(gPdata.compass.orientation));
    gPdata.compass.orientation[0] =  1;   // X is  X
    gPdata.compass.orientation[4] = -1;   // Y is -Y
    gPdata.compass.orientation[8] = -1;   // Z is -Z

    gPdata.compass.adapt_num       = 0;
    gPdata.compass.bus             = EXT_SLAVE_BUS_PRIMARY;
#ifndef LINUX
    gCompass = *yas529_get_slave_descr();
    gPdata.compass.get_slave_descr = yas529_get_slave_descr;
#endif
    gPdata.compass.address         = COMPASS_SLAVEADDR_YAS529;
}


static
void SetupCompassHSCDTD002BCalibration_MSB (void)
{
    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.compass.orientation, 0, sizeof(gPdata.compass.orientation));
    gPdata.compass.orientation[0] = -1;   // X is -X
    gPdata.compass.orientation[4] = -1;   // Y is -Y
    gPdata.compass.orientation[8] =  1;   // Z is Z

    gPdata.compass.adapt_num       = 0;
    gPdata.compass.bus             = EXT_SLAVE_BUS_PRIMARY;
#ifndef LINUX
    gCompass = *hscdtd002b_get_slave_descr();
    gPdata.compass.get_slave_descr = hscdtd002b_get_slave_descr;
#endif
    gPdata.compass.address         = COMPASS_SLAVEADDR_HSCDTD00XX;
}

static
void SetupCompassHSCDTD004ACalibration_MSB (void)
{
    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.compass.orientation, 0, sizeof(gPdata.compass.orientation));
    gPdata.compass.orientation[1] = +1;
    gPdata.compass.orientation[3] = -1;
    gPdata.compass.orientation[8] = +1;

    gPdata.compass.adapt_num       = 0;
    gPdata.compass.bus             = EXT_SLAVE_BUS_PRIMARY;
#ifndef LINUX
    gCompass = *hscdtd004a_get_slave_descr();
    gPdata.compass.get_slave_descr = hscdtd004a_get_slave_descr;
#endif
    gPdata.compass.address         = COMPASS_SLAVEADDR_HSCDTD00XX;
}


/*****************************
    Pressure Setup Functions
******************************/
static
void SetupPressureBMA085Calibration_MSB (void)
{
    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.pressure.orientation, 0, sizeof(gPdata.pressure.orientation));

    gPdata.pressure.adapt_num       = 0;
    gPdata.pressure.bus             = EXT_SLAVE_BUS_PRIMARY;
#ifndef LINUX
    gPressure = *bma085_get_slave_descr();
    gPdata.pressure.get_slave_descr = bma085_get_slave_descr;
#endif
    gPdata.pressure.address         = PRESSURE_SLAVEADDR_BMA085;
}


/*******************************************************************************
        6 axis EVB - Selection and Setup APIs
*******************************************************************************/

tMLError SetupGyroCalibration_ST_6Axis(void)
{
    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.orientation, 0, sizeof(gPdata.orientation));
    gPdata.orientation[0] =  1;
    gPdata.orientation[4] =  1;
    gPdata.orientation[8] =  1;

    return ML_SUCCESS;
}

/*******************************************************************************
        Mantis Generic - Selection and Setup APIs
*******************************************************************************/

#ifdef M_HW
/**
 * @param side
 *          Selects the orientation for:
 *              0 = MSB,
 *              1 = Mounted top of board, Y chip pointing along board (Prototype board)
 *              2 = on top of the USB dongle (demo board with AKM compass)
 */
tMLError SetupMantisCalibration(int side)
{
    MPL_LOGV("Calibrating '%s'\n", __func__);

    /*
        Gyroscope
    */

    /* Orientation */
    memset(gPdata.orientation, 0, sizeof(gPdata.orientation));
    if (side == 0) {
        gPdata.orientation[0] = +1;
        gPdata.orientation[4] = -1;
        gPdata.orientation[8] = -1;
    } else if (side == 1) {
        gPdata.orientation[0] = +1;
        gPdata.orientation[4] = +1;
        gPdata.orientation[8] = +1;
    } else if (side == 2) {
        gPdata.orientation[1] = +1;
        gPdata.orientation[3] = -1;
        gPdata.orientation[8] = +1;
    } else {
        MPL_LOGE("Invalid side parameter %d passed to function '%s'\n", side, __func__);
        return ML_ERROR_INVALID_PARAMETER;
    }

    /* Interrupt */
    gPdata.int_config = BIT_INT_ANYRD_2CLEAR|BIT_BYPASS_EN;

    /*
     *  Accelerometer
     */

    /* Orientation */
    memset(gPdata.accel.orientation, 0, sizeof(gPdata.accel.orientation));
    if (side == 0) {
        gPdata.accel.orientation[0] =   1;
        gPdata.accel.orientation[4] =  -1;
        gPdata.accel.orientation[8] =  -1;
#ifndef WIN32
        Rotate180DegAroundZAxis(gPdata.accel.orientation);
#endif
    } else if (side == 1) {
        gPdata.accel.orientation[0] = +1;
        gPdata.accel.orientation[4] = +1;
        gPdata.accel.orientation[8] = +1;
    } else if (side == 2) {
        gPdata.accel.orientation[1] = +1;
        gPdata.accel.orientation[3] = -1;
        gPdata.accel.orientation[8] = +1;
    }

    gPdata.accel.adapt_num       = 0;
    gPdata.accel.bus             = EXT_SLAVE_BUS_INVALID;
#ifndef LINUX
    gAccel = *mantis_get_slave_descr();
    gPdata.accel.get_slave_descr = mantis_get_slave_descr;
#endif
    gPdata.accel.address         = 0x68;

    return ML_SUCCESS;
}
#endif


/******************************************************************************
        USB dongle - Selection and Setup APIs
******************************************************************************/
static 
void SetupMPUCalibration_Dongle(void)
{
    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.orientation, 0, sizeof(gPdata.orientation));
    gPdata.orientation[1] = +1;
    gPdata.orientation[3] = -1;
    gPdata.orientation[8] = +1;

    /* Interrupt */
    gPdata.int_config = 0x10;
}

static
void SetupAccelKionixKXTF9Calibration_Dongle(void)
{
    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.accel.orientation, 0, sizeof(gPdata.accel.orientation));
    gPdata.accel.orientation[1] = +1;
    gPdata.accel.orientation[3] = -1;
    gPdata.accel.orientation[8] = +1;

    gPdata.accel.adapt_num       = 0;
    gPdata.accel.bus             = EXT_SLAVE_BUS_SECONDARY;
#ifndef LINUX
    gAccel = *kxtf9_get_slave_descr();
    gPdata.accel.get_slave_descr = kxtf9_get_slave_descr;
#endif
    gPdata.accel.address         = ACCEL_SLAVEADDR_KXTF9;
}

static
void SetupCompassAKMCalibration_Dongle(void)
{
    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.compass.orientation, 0, sizeof(gPdata.compass.orientation));
    gPdata.compass.orientation[0] = -1;
    gPdata.compass.orientation[4] = +1;
    gPdata.compass.orientation[8] = -1;

    gPdata.compass.adapt_num       = 0;
    gPdata.compass.bus             = EXT_SLAVE_BUS_PRIMARY;
#ifndef LINUX
    gCompass = *ak8975_get_slave_descr();
    gPdata.compass.get_slave_descr = ak8975_get_slave_descr;
#endif
    gPdata.compass.address         = COMPASS_SLAVEADDR_AKM;
}

#ifdef M_HW
static
void SetupCompassAKMCalibration_Dragon(void)
{
    MPL_LOGV("Calibrating '%s'\n", __func__);

    /* Orientation */
    memset(gPdata.compass.orientation, 0, sizeof(gPdata.compass.orientation));

    gPdata.compass.orientation[1] = +1;   // X is -X
    gPdata.compass.orientation[3] = +1;   // Y is -Y
    gPdata.compass.orientation[8] = -1;   // Z is  Z

    gPdata.compass.adapt_num       = 0;
    gPdata.compass.bus             = EXT_SLAVE_BUS_SECONDARY;
#ifndef LINUX
    gCompass = *ak8975_get_slave_descr();
    gPdata.compass.get_slave_descr = ak8975_get_slave_descr;
#endif
    gPdata.compass.address         = COMPASS_SLAVEADDR_AKM_BASE;
}
#endif

/**
 * @}
 */


