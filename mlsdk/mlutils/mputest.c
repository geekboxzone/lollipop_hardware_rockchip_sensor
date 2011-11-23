/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */

/******************************************************************************
 *
 * $Id: mputest.c 5154 2011-04-07 02:25:46Z mcaramello $
 *
 *****************************************************************************/

/*
 *  MPU Self Test functions
 *  Version 2.3
 *  February 22nd, 2011
 */

/**
 *  @defgroup MPU_SELF_TEST
 *  @brief  MPU Self Test functions
 *
 *  These functions provide an in-site test of the MPU 3xxx chips. The main 
 *      entry point is the MPUTest function.  
 *  This runs the tests (as described in the accompanying documentation) and 
 *      writes a configuration file containing initial calibration data.  
 *  MPUTest returns ML_SUCCESS if the chip passes the tests.  
 *  Otherwise, an error code is returned.  
 *  The functions in this file rely on MLSL and MLOS: refer to the MPL 
 *      documentation for more information regarding the system interface 
 *      files.
 *
 *  @{
 *      @file   mputest.c
 *      @brief  MPU Self Test routines for assessing gyro sensor status 
 *              after surface mount has happened on the target host platform.
 */

#include <stdio.h>
#include <time.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#ifdef LINUX
#include <unistd.h>
#endif

/* not exposing internal functions when part of MPL.
   APIs are described in ml_mputest.h in the mllite folder */
#ifdef MPUSELFTEST_STANDALONE
#include "mputest.h"
#endif

#include "mpu.h"
#include "mldl.h"
#include "mldl_cfg.h"
#include "accel.h"
#include "mlFIFO.h"
#include "slave.h"
#include "ml.h"
#include "ml_stored_data.h"
#include "checksum.h"

#include "mlsl.h"
#include "mlos.h"

#include "log.h"
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-mpust"

#ifdef __cplusplus
extern "C" {
#endif

/*
    Defines
*/

#define VERBOSE_OUT 0

/*--- Set to 0 to suprress the Sense Path self-test portion of the gyro test.
      Set to 1 to enable it. ---*/
#define USE_SENSE_PATH_TEST 0

/*--- Test parameters defaults. See SetTestParameters for more details ---*/

#define DEF_MPU_ADDR             (0x68)        /* I2C address of the mpu     */

#if (USE_SENSE_PATH_TEST == 1)                 /* gyro full scale dps        */
#define DEF_GYRO_FULLSCALE       (2000)
#else
#define DEF_GYRO_FULLSCALE       (250) 
#endif

#define DEF_GYRO_SENS            (32768.f / DEF_GYRO_FULLSCALE)
                                               /* gyro sensitivity LSB/dps   */
#define DEF_PACKET_THRESH        (75)          /* 600 ms / 8ms / sample      */
#define DEF_TOTAL_TIMING_TOL     (.03f)        /* 3% = 2 pkts + 1% proc tol. */
#define DEF_BIAS_THRESH          (40 * DEF_GYRO_SENS)
                                               /* 40 dps in LSBs             */
#define DEF_RMS_THRESH           (0.4f * DEF_GYRO_SENS)
                                               /* 0.4 dps-rms in LSB-rms     */
#define DEF_SP_SHIFT_THRESH_CUST (.05f)        /* 5%                         */
#define DEF_TEST_TIME_PER_AXIS   (600)         /* ms of time spent collecting
                                                  data for each axis,
                                                  multiple of 600ms          */

#define ML_INIT_CAL_LEN          (36)          /* length in bytes of 
                                                  calibration data file      */

/*
    Macros
*/

#define CHECK_TEST_ERROR(x)                                                 \
    if (x) {                                                                \
        MPL_LOGI("error %d @ %s|%d\n", x, __func__, __LINE__);              \
        return (-1);                                                        \
    }

#define SHORT_TO_TEMP_C(shrt)         (((shrt+13200.f)/280.f)+35.f)

#define USHORT_TO_CHARS(chr,shrt)     (chr)[0]=(unsigned char)(shrt>>8);    \
                                      (chr)[1]=(unsigned char)(shrt);

#define UINT_TO_CHARS(chr,ui)         (chr)[0]=(unsigned char)(ui>>24);     \
                                      (chr)[1]=(unsigned char)(ui>>16);     \
                                      (chr)[2]=(unsigned char)(ui>>8);      \
                                      (chr)[3]=(unsigned char)(ui);

#define FLOAT_TO_SHORT(f)             (                                     \
                                        (fabs(f-(short)f)>=0.5) ? (         \
                                            ((short)f)+(f<0?(-1):(+1))) :   \
                                            ((short)f)                      \
                                      )

#define CHARS_TO_SHORT(d)             ((((short)(d)[0])<<8)+(d)[1])
#define CHARS_TO_SHORT_SWAPPED(d)     ((((short)(d)[1])<<8)+(d)[0])

#define ACCEL_UNPACK(d) d[0], d[1], d[2], d[3], d[4], d[5]

#define CHECK_NACKS(d)  (                               \
                         d[0]==0xff && d[1]==0xff &&    \
                         d[2]==0xff && d[3]==0xff &&    \
                         d[4]==0xff && d[5]==0xff       \
                        )

/*
    Prototypes
*/

static int SetSensePathTest(void *mlsl_handle, int enable);
static tMLError TestGetData(
                    void *mlsl_handle, 
                    struct mldl_cfg *mputestCfgPtr,
                    short *vals);

struct ext_slave_descr *adxl346_get_slave_descr(void);
struct ext_slave_descr *bma150_get_slave_descr(void);
struct ext_slave_descr *bma222_get_slave_descr(void);
struct ext_slave_descr *kxsd9_get_slave_descr(void);
struct ext_slave_descr *kxtf9_get_slave_descr(void);
struct ext_slave_descr *lis331dlh_get_slave_descr(void);
struct ext_slave_descr *lsm303dlha_get_slave_descr(void);
struct ext_slave_descr *mma8450_get_slave_descr(void);
struct ext_slave_descr *mma845x_get_slave_descr(void);

/*
    Types
*/
typedef struct {
    float gyroSens;
    int gyroFS;
    int packetThresh;
    float totalTimingTol;
    int biasThresh;
    float rmsThreshSq;
    float SPShiftThresh;
    unsigned int testTimePerAxis;
} tTestSetup;

/*
    Global variables
*/
static unsigned char dataout[20];
static unsigned char dataStore[ML_INIT_CAL_LEN];

static tTestSetup test_setup = {
    DEF_GYRO_SENS,
    DEF_GYRO_FULLSCALE,
    DEF_PACKET_THRESH,
    DEF_TOTAL_TIMING_TOL,
    (int)DEF_BIAS_THRESH,
    DEF_RMS_THRESH * DEF_RMS_THRESH,
    DEF_SP_SHIFT_THRESH_CUST,
    DEF_TEST_TIME_PER_AXIS
};

static float adjGyroSens;
static char a_name[3][2] = {"X", "Y", "Z"};

/*
    NOTE :  modify get_slave_descr parameter below to reflect
                the DEFAULT accelerometer in use. The accelerometer in use 
                can be modified at run-time using the TestSetupAccel API.
    NOTE :  modify the expected z axis orientation (Z axis pointing
                upward or downward)
*/

signed char g_zSign = +1;

#ifdef MPUSELFTEST_STANDALONE
struct mpu3050_platform_data mputestPlatformData = {
    /* int_config */ 0,
    /* orientation[MPU_NUM_AXES * MPU_NUM_AXES] */ {1, 0, 0, 0, 1, 0, 0, 0, 1},
    /* level_shifter */ 0,
    /* ext_slave_platform_data accel */ {
        /* get_slave_descr */ NULL,
        /* irq */ 0,
        /* adapt_num */ 0,
        /* bus */ 0,
        /* address */ 0,
        /* orientation[9] */ {1, 0, 0, 0, 1, 0, 0, 0, 1},
        /* private_data */ NULL
    },
    /* ext_slave_platform_data compass */ {NULL}
};

struct mldl_cfg mputestCfg = {
    /* requested_sensors */ 0,
    /* ignore_system_suspend */ 0,
    /* addr */ DEF_MPU_ADDR,
    /* int_config */ 0, 
    /* ext_sync */ 0,
    /* full_scale */ MPU_FS_2000DPS,
    /* lpf */ MPU_FILTER_42HZ,
    /* clk_src */ MPU_CLK_SEL_INTERNAL,
    /* divider */ 7,
    /* dmp_enable */ 0,
    /* fifo_enable */ 0,
    /* dmp_cfg1 */ 0,
    /* dmp_cfg2 */ 0,
    /* gyro_power */ 0,
    /* offset_tc[MPU_NUM_AXES] */ {0, 0, 0},
    /* offset[MPU_NUM_AXES] */ {0, 0, 0},
    /* ram[MPU_MEM_NUM_RAM_BANKS][MPU_MEM_BANK_SIZE] */ {{0}},
    
    /* silicon_revision */ 0,
    /* product_id */ 0,
    /* trim */ 131,
    
    /* gyro_is_bypassed */ 0,
    /* dmp_is_running */ 0,
    /* gyro_is_suspended */ 0,
    /* accel_is_suspended */ 0,
    /* compass_is_suspended */ 0,
    /* pressure_is_suspended */ 0,
    /* gyro_needs_reset */ 0,
    
    /* ext_slave_descr *accel */ NULL,
    /* ext_slave_descr *compass */ NULL,
    /* ext_slave_descr *pressure */ NULL,
    
    /* mpu3050_platform_data *pdata */ &mputestPlatformData
};
struct mldl_cfg *mputestCfgPtr = &mputestCfg;
#else
struct mldl_cfg *mputestCfgPtr = NULL;
#endif

#ifndef LINUX
/**
 *  @internal
 *  @brief  usec precision sleep function.
 *  @param  number of micro seconds (us) to sleep for.
 */
static void usleep(unsigned long t)
{
    unsigned long start = MLOSGetTickCount();
    while (MLOSGetTickCount()-start < t / 1000);
}
#endif

/**
 *  @brief  Modify the self test limits from their default values.
 *
 *  @param  slaveAddr
 *              the slave address the MPU device is setup to respond at. 
 *              The default is DEF_MPU_ADDR = 0x68.
 *  @param  sensitivity
 *              the read sensitivity of the device in LSB/dps as it is trimmed.
 *              NOTE :  if using the self test as part of the MPL, the 
 *                      sensitivity the different sensitivity trims are already
 *                      taken care of.
 *  @param  pThresh
 *              number of packets expected to be received in a 600 ms period. 
 *              Depends on the sampling frequency of choice (set by default to 
 *              125 Hz) and low pass filter cut-off frequency selection (set 
 *              to 42 Hz).
 *              The default is DEF_PACKET_THRESH = 75 packets.
 *  @param  totalTimeTol
 *              time skew tolerance, taking into account imprecision in turning
 *              the FIFO on and off and the processor time imprecision (for 
 *              1 GHz processor).
 *              The default is DEF_TOTAL_TIMING_TOL = 3 %, about 2 packets.
 *  @param  biasThresh
 *              bias level threshold, the maximun acceptable no motion bias 
 *              for a production quality part.
 *              The default is DEF_BIAS_THRESH = 40 dps.
 *  @param  rmsThresh
 *              the limit standard deviation (=~ RMS) set to assess whether 
 *              the noise level on the part is acceptable.
 *              The default is DEF_RMS_THRESH = 0.2 dps-rms.
 *  @param  SPShiftThresh
 *              the limit shift applicable to the Sense Path self test 
 *              calculation.
 */
void SetTestParameters(unsigned int slaveAddr,
                       float sensitivity,
                       int pThresh,
                       float totalTimeTol,
                       int biasThresh,
                       float rmsThresh,
                       float SPShiftThresh)
{
    mputestCfgPtr->addr = slaveAddr;
    test_setup.gyroSens = sensitivity; 
    test_setup.gyroFS = (int)(32768.f / sensitivity);
    test_setup.packetThresh = pThresh;
    test_setup.totalTimingTol = totalTimeTol;
    test_setup.biasThresh = biasThresh;
    test_setup.rmsThreshSq = rmsThresh * rmsThresh;
    test_setup.SPShiftThresh = SPShiftThresh;
}

#ifdef MPUSELFTEST_STANDALONE
/**
 *  @brief  Setup the accelerometer device parameter to be used to capture
 *          and calculate the accelerometer biases.
 *          Use ID_INVALID or 0 to disable bias calculation for the 
 *          accelerometer.
 *
 *  @note   if used stand-alone (not integrated with the MPL), to completely 
 *          remove accelerometer support from the MPU Self Test remove the 
 *          prototypes' declaration at the beginning of this file, then
 *          remove/comment out this function.
 *
 *  @note   when using the MPU Self Test as part of the MPL, the accelerometer
 *          in use will be picked up from the kernel driver configuration (on 
 *          Linux/Android) or from the SetupPlatform() selection (on Windows). 
 *
 *  @param  accelId
 *              ID associated to the accelerometer device to be used. See
 *              platform/include/mpu.h, enum ext_slave_id for the list of 
 *              device IDs.
 *              Using ID_INVALID or 0 will suppress testing of the 
 *              accelerometer.
 *  @param  accelAddr
 *              specify the slave address associated with the accelerometer 
 *              device specified by the accelId parameter, if different from 
 *              the default. The specified slave address must be supported by 
 *              the accelerometer device or otherwise it will be ignored.
 *              Some devices do not support alternative slave address 
 *              selection.
 *              Using ACCEL_SLAVEADDR_INVALID or 0 will select the default 
 *              address.
 *              See mlapps/common/slave.h for a list of the default slave 
 *              addresses.
 *  @param  accelZSign
 *              the expected orientation of the Z axis during the test of the 
 *              accelerometer sensor.  In can be either +1 or -1, indicating
 *              positive and negative Z axis orientation.
 */
void TestSetupAccel(unsigned char accelId, unsigned char accelAddr,
                    char accelZSign)
{
    struct mpu3050_platform_data *mputestPData = mputestCfgPtr->pdata;

    switch(accelId) {
        case ACCEL_ID_ADI346:
            mputestPData->accel.get_slave_descr = adxl346_get_slave_descr;
            mputestCfgPtr->accel = mputestPData->accel.get_slave_descr();
            if (accelAddr == ACCEL_SLAVEADDR_ADI346 ||
                accelAddr == ACCEL_SLAVEADDR_ADI346_ALT) {
                mputestPData->accel.address = accelAddr;
            } else {
                mputestPData->accel.address = ACCEL_SLAVEADDR_ADI346;
            }
            break;
        case ACCEL_ID_BMA150:
            mputestPData->accel.get_slave_descr = bma150_get_slave_descr;
            mputestCfgPtr->accel = mputestPData->accel.get_slave_descr();
            if ((accelAddr & 0x7e) != ACCEL_SLAVEADDR_BMA150) {
                mputestPData->accel.address = accelAddr;
            } else {
                mputestPData->accel.address = ACCEL_SLAVEADDR_BMA150;
            }
            break;
        case ACCEL_ID_BMA222:
            mputestPData->accel.get_slave_descr = bma222_get_slave_descr;
            mputestCfgPtr->accel = mputestPData->accel.get_slave_descr();
            if ((accelAddr & 0x7e) != ACCEL_SLAVEADDR_BMA222) {
                mputestPData->accel.address = accelAddr;
            } else {
                mputestPData->accel.address = ACCEL_SLAVEADDR_BMA222;
            }
            break;
        case ACCEL_ID_KXSD9:
            mputestPData->accel.get_slave_descr = kxsd9_get_slave_descr;
            mputestCfgPtr->accel = mputestPData->accel.get_slave_descr();
            mputestPData->accel.address = ACCEL_SLAVEADDR_KXSD9;
            break;
        case ACCEL_ID_KXTF9:
            mputestPData->accel.get_slave_descr = kxtf9_get_slave_descr;
            mputestCfgPtr->accel = mputestPData->accel.get_slave_descr();
            mputestPData->accel.address = ACCEL_SLAVEADDR_KXTF9;
            break;
        case ACCEL_ID_LIS331:
            mputestPData->accel.get_slave_descr = lis331dlh_get_slave_descr;
            mputestCfgPtr->accel = mputestPData->accel.get_slave_descr();
            if ((accelAddr & 0x7e) != ACCEL_SLAVEADDR_LIS331) {
                mputestPData->accel.address = accelAddr;
            } else {
                mputestPData->accel.address = ACCEL_SLAVEADDR_LIS331;
            }
            break;
        case ACCEL_ID_LSM303:
            mputestPData->accel.get_slave_descr = lsm303dlha_get_slave_descr;
            mputestCfgPtr->accel = mputestPData->accel.get_slave_descr();
            if ((accelAddr & 0x7e) != ACCEL_SLAVEADDR_LSM303) {
                mputestPData->accel.address = accelAddr;
            } else {
                mputestPData->accel.address = ACCEL_SLAVEADDR_LSM303;
            }
            break;
        case ACCEL_ID_MMA8450:
            mputestPData->accel.get_slave_descr = mma8450_get_slave_descr;
            mputestCfgPtr->accel = mputestPData->accel.get_slave_descr();
            mputestPData->accel.address = ACCEL_SLAVEADDR_MMA8450;
            break;
        case ACCEL_ID_MMA845X:
            mputestPData->accel.get_slave_descr = mma845x_get_slave_descr;
            mputestCfgPtr->accel = mputestPData->accel.get_slave_descr();
            mputestPData->accel.address = ACCEL_SLAVEADDR_MMA845X;
            break;
        case ID_INVALID:
        default:
            mputestPData->accel.get_slave_descr = NULL;
            mputestCfgPtr->accel = NULL;
            mputestPData->accel.address = ACCEL_SLAVEADDR_INVALID;
            break;
    }

    g_zSign = accelZSign;
}
#endif

/**
 *  @internal
 *  @brief  Enables/Disables the Sense Path self-test portion of the gyro MPU 
 *          Self Test.
 *  @note   To properly ensure the Sense Path test is run appropriately, the 
 *          <b>INT/DP3</b> pin of the MPU device must be hold low during the test.
 *  @param  mlsl_handle
 *              serial interface handle to allow serial communication with the 
 *              device, both gyro and accelerometer.
 *  @param  enable
 *              1 enables the Sense Path self-test internal logic, 
 *              0 disables it.
 *  @return 0 on success, a non-zero error code on error.
 */
static int SetSensePathTest(void *mlsl_handle, int enable)
{
    int result;

    enable = (enable>0);

    /* enable CFG mode */
    result = MLSLSerialWriteSingle(mlsl_handle, mputestCfgPtr->addr, 
                                   MPUREG_BANK_SEL, 0x10);
    CHECK_TEST_ERROR(result);

    /* set SP mode to differential */
    result = MLSLSerialWriteSingle(mlsl_handle, mputestCfgPtr->addr, 
                                   0x29, 0x04 * enable);
    CHECK_TEST_ERROR(result);
    /* change functionality of INT pin from output to Hi-Z input
        Note: make sure the INT pin is either pulled low or high */
    result = MLSLSerialWriteSingle(mlsl_handle, mputestCfgPtr->addr, 
                                   0x2B, 0x28 * enable);
    /* enable USER mode */
    result = MLSLSerialWriteSingle(mlsl_handle, mputestCfgPtr->addr, 
                                   MPUREG_BANK_SEL, 0x00);
    CHECK_TEST_ERROR(result);

    return 0;
}


#define X   (0)
#define Y   (1)
#define Z   (2)
/**
 *  @brief  Test the gyroscope sensor.
 *          Implements the core logic of the MPU Self Test.
 *          Produces the PASS/FAIL result. Loads the calculated gyro biases 
 *          and temperature datum into the corresponding pointers.
 *  @param  mlsl_handle
 *              serial interface handle to allow serial communication with the 
 *              device, both gyro and accelerometer.
 *  @param  gyro_biases
 *              output pointer to store the initial bias calculation provided 
 *              by the MPU Self Test.  Requires 3 elements for gyro X, Y, 
 *              and Z.
 *  @param  temp_avg
 *              output pointer to store the initial average temperature as
 *              provided by the MPU Self Test.
 *  @return 0 on success.
 *          On error, the return value is a bitmask representing:
 *          0, 1, 2     Failures with PLLs on X, Y, Z gyros respectively 
 *                      (decimal values will be 1, 2, 4 respectively).
 *          3, 4, 5     Excessive offset with X, Y, Z gyros respectively 
 *                      (decimal values will be 8, 16, 32 respectively).
 *          6, 7, 8     Excessive noise with X, Y, Z gyros respectively 
 *                      (decimal values will be 64, 128, 256 respectively).
 *          9           If any of the RMS noise values is zero, it could be 
 *                      due to a non-functional gyro or FIFO/register failure.
 *                      (decimal value will be 512).
 *          10, 11, 12  Excessive bias shift in Sense Path MPU Self Test for 
 *                      X, Y, Z gyros respectively 
 *                      (decimal values will be 1024, 2048, 4096 respectively).
 *
 *
 */
int TestGyro(void *mlsl_handle, short gyro_biases[3], short *temp_avg)
{
#ifdef M_HW
    return ML_SUCCESS;
#else
    int retVal = 0;
    tMLError result;

    int total_count = 0;
    int total_count_axis[3] = {0, 0, 0};
    int packet_count;
    short x[DEF_TEST_TIME_PER_AXIS / 8 * 4] = {0};
    short y[DEF_TEST_TIME_PER_AXIS / 8 * 4] = {0};
    short z[DEF_TEST_TIME_PER_AXIS / 8 * 4] = {0};
    unsigned char regs[7];

    int temperature;
    float Avg[3];
    float RMS[3];
    const unsigned char SPcode_min_limit[3] = {
        128,    /* (94dps /1500lsb) */
        156,    /* (188dps/3000lsb) */
        168     /* (250dps/4000lsb) */
    };
    float oldDelta[3] = {0.f, 0.f, 0.f},
          newDelta[3];
    int i, j, tmp;
    char tmpStr[200];

    temperature = 0;

    /* sample rate = 8ms */
    result = MLSLSerialWriteSingle(
                mlsl_handle, mputestCfgPtr->addr, 
                MPUREG_SMPLRT_DIV, 0x07); 
    CHECK_TEST_ERROR(result);

    regs[0] = 0x03; /* filter = 42Hz, analog_sample rate = 1 KHz */
    switch (DEF_GYRO_FULLSCALE) {
        case 2000:
            regs[0] |= 0x18;
            break;
        case 1000:
            regs[0] |= 0x10;
            break;
        case 500:
            regs[0] |= 0x08;
            break;
        case 250:
        default:
            regs[0] |= 0x00;
            break;
    }
    result = MLSLSerialWriteSingle(
                mlsl_handle, mputestCfgPtr->addr, 
                MPUREG_DLPF_FS_SYNC, regs[0]); 
    CHECK_TEST_ERROR(result);
    result = MLSLSerialWriteSingle(
                mlsl_handle, mputestCfgPtr->addr, 
                MPUREG_INT_CFG, 0x00); 

    if (USE_SENSE_PATH_TEST) {
        /* calculate abs(oldDelta) */
        result = MLSLSerialRead(    /* interested in 0x04, 0x07, 0x0a */
                    mlsl_handle, mputestCfgPtr->addr, 
                    MPUREG_04_RSVD, 7, regs); 
        CHECK_TEST_ERROR(result);
        if (regs[0] == 0  &&  regs[3] == 0  &&  regs[6] == 0) {
            MPL_LOGI("Skip the Sense Path test - no OTP test data\n");
        } else {
            if (regs[0] < SPcode_min_limit[X]) {
                MPL_LOGI("Skip the Sense Path test for %s Gyro - "
                         "OTP data (%d) less than required minimum (%d)\n", 
                         a_name[X], regs[0], SPcode_min_limit[X]);
            } else { 
                oldDelta[X] = 65.5f * powf(1.025f, regs[0] - 1.f);
            }
            if (regs[3] < SPcode_min_limit[Y]) {
                MPL_LOGI("Skip the Sense Path test for %s Gyro - "
                         "OTP data (%d) less than required minimum (%d)\n", 
                         a_name[Y], regs[3], SPcode_min_limit[Y]);
            } else {
                oldDelta[Y] = 65.5f * powf(1.025f, regs[3] - 1.f);
            }
            if (regs[6] < SPcode_min_limit[Z]) {
                MPL_LOGI("Skip the Sense Path test for %s Gyro - "
                         "OTP data (%d) less than required minimum (%d)\n", 
                         a_name[Z], regs[6], SPcode_min_limit[Z]);
            } else {
                oldDelta[Z] = 65.5f * powf(1.025f, regs[6] - 1.f);
            }
        }
    }

    /* read Temperature */ 
    /*
    tmp = 1000;
    while(tmp--) {
    result = MLSLSerialRead(
                mlsl_handle, mputestCfgPtr->addr, 
                MPUREG_TEMP_OUT_H, 2, dataout);
    CHECK_TEST_ERROR(result);
    temperature += (short)CHARS_TO_SHORT(dataout);
    MPL_LOGI("%d / %hd\n", temperature, CHARS_TO_SHORT(dataout));
    }
    */

    /* 1st, timing test */
    for (j = 0; j < 3; j++) {

        if (USE_SENSE_PATH_TEST  && j == Z) { 
            MPL_LOGI("Switching Sense Path self test ON\n");
            result = SetSensePathTest(mlsl_handle, 1);
            CHECK_TEST_ERROR(result);
        }
        MPL_LOGI("Collecting gyro data from %s gyro PLL\n", a_name[j]);

        /* turn on all gyros, use gyro X for clocking
           Set to Y and Z for 2nd and 3rd iteration */
        result = MLSLSerialWriteSingle(
                    mlsl_handle, mputestCfgPtr->addr, 
                    MPUREG_PWR_MGM, j+1);
        CHECK_TEST_ERROR(result);

        /* wait for 2 ms after switching clock source */
        usleep(2000);

        /* we will enable XYZ gyro in FIFO and nothing else */
        result = MLSLSerialWriteSingle(
                    mlsl_handle, mputestCfgPtr->addr, 
                    MPUREG_FIFO_EN2, 0x00);
        CHECK_TEST_ERROR(result);
        /* enable/reset FIFO */
        result = MLSLSerialWriteSingle(
                    mlsl_handle, mputestCfgPtr->addr, 
                    MPUREG_USER_CTRL, 0x42);

        tmp = (int)(test_setup.testTimePerAxis / 600);
        while (tmp-->0) {
            /* enable XYZ gyro in FIFO and nothing else */
            result = MLSLSerialWriteSingle(
                        mlsl_handle, mputestCfgPtr->addr, 
                        MPUREG_FIFO_EN1, 0x70);
            CHECK_TEST_ERROR(result);

            /* wait for 600 ms for data */
            usleep(600000);

            /* stop storing gyro in the FIFO */
            result = MLSLSerialWriteSingle(
                        mlsl_handle, mputestCfgPtr->addr, 
                        MPUREG_FIFO_EN1, 0x00);
            CHECK_TEST_ERROR(result);

            /* Getting number of bytes in FIFO */
            result = MLSLSerialRead(
                           mlsl_handle, mputestCfgPtr->addr, 
                           MPUREG_FIFO_COUNTH, 2, dataout); 
            CHECK_TEST_ERROR(result);
            /* number of 6 B packets in the FIFO */
            packet_count = CHARS_TO_SHORT(dataout) / 6; 
            sprintf(tmpStr, "Packet Count: %d - ", packet_count);

            if ( abs(packet_count - test_setup.packetThresh) 
                        <=      /* Within +/- totalTimingTol % range */
                     test_setup.totalTimingTol * test_setup.packetThresh) {
                for (i=0; i<packet_count; i++) {
                    /* getting FIFO data */
                    result = MLSLSerialReadFifo(
                                mlsl_handle, mputestCfgPtr->addr, 
                                6, dataout);
                    CHECK_TEST_ERROR(result);
                    x[total_count+i] = CHARS_TO_SHORT(&dataout[0]);
                    y[total_count+i] = CHARS_TO_SHORT(&dataout[2]);
                    z[total_count+i] = CHARS_TO_SHORT(&dataout[4]);
                    if (VERBOSE_OUT) {
                        MPL_LOGI("Gyros %-4d    : %+13d %+13d %+13d\n",
                                    total_count + i, x[total_count+i], 
                                    y[total_count+i], z[total_count+i]
                        );
                    }
                }
                total_count += packet_count;
                total_count_axis[j] += packet_count;
                sprintf(tmpStr, "%sOK", tmpStr);
            } else {
                retVal |= 1 << j;
                sprintf(tmpStr, "%sNOK - samples ignored", tmpStr);
            }
            MPL_LOGI("%s\n", tmpStr);
        }

        /* remove gyros from FIFO */
        result = MLSLSerialWriteSingle(
                    mlsl_handle, mputestCfgPtr->addr, 
                    MPUREG_FIFO_EN1, 0x00);
        CHECK_TEST_ERROR(result);

        if (USE_SENSE_PATH_TEST && j == Z) {
            MPL_LOGI("Switching Sense Path self test OFF\n");
            result = SetSensePathTest(mlsl_handle, 0);
            CHECK_TEST_ERROR(result);
        }

        /* Read Temperature */
        result = MLSLSerialRead(mlsl_handle, mputestCfgPtr->addr,
                    MPUREG_TEMP_OUT_H, 2, dataout);
        temperature += (short)CHARS_TO_SHORT(dataout);
    }

    MPL_LOGI("\n");
    MPL_LOGI("Total %d samples\n", total_count);
    MPL_LOGI("\n");

    if (USE_SENSE_PATH_TEST) {
        total_count = total_count_axis[X] + total_count_axis[Y];
    }

    /* 2nd, check bias from X and Y PLL clock source */
    tmp = total_count != 0 ? total_count : 1;
    for (i = 0, 
            Avg[X] = .0f, Avg[Y] = .0f, Avg[Z] = .0f;
         i < total_count; i++) {
        Avg[X] += 1.f * x[i] / tmp; 
        Avg[Y] += 1.f * y[i] / tmp; 
        Avg[Z] += 1.f * z[i] / tmp;
    }
    MPL_LOGI("bias          : %+13.3f %+13.3f %+13.3f (LSB)\n", 
             Avg[X], Avg[Y], Avg[Z]); 
    if (VERBOSE_OUT) {
        MPL_LOGI("              : %+13.3f %+13.3f %+13.3f (dps)\n", 
                 Avg[X] / adjGyroSens,
                 Avg[Y] / adjGyroSens, 
                 Avg[Z] / adjGyroSens); 
    }
    for (j = 0; j < 3; j++) {
        if (fabs(Avg[j]) > test_setup.biasThresh) {
            MPL_LOGI("%s-Gyro bias (%.0f) exceeded threshold "
                     "(threshold = %d)\n", 
                     a_name[j], Avg[j], test_setup.biasThresh);
            retVal |= 1 << (3+j);
        }
    }

    /* 3rd, check RMS */
    for (i = 0,
            RMS[X] = 0.f, RMS[Y] = 0.f, RMS[Z] = 0.f;
         i < total_count; i++) {
        RMS[X] += (x[i] - Avg[X]) * (x[i] - Avg[X]);
        RMS[Y] += (y[i] - Avg[Y]) * (y[i] - Avg[Y]);
        RMS[Z] += (z[i] - Avg[Z]) * (z[i] - Avg[Z]);
    }
    for (j = 0; j < 3; j++) {
        if (RMS[j] > test_setup.rmsThreshSq * total_count) {
            MPL_LOGI("%s-Gyro RMS (%.2f) exceeded threshold "
                     "(threshold = %.2f)\n", 
                     a_name[j], sqrt(RMS[j] / total_count), 
                     sqrt(test_setup.rmsThreshSq));
            retVal |= 1 << (6+j);
        }
    }

    MPL_LOGI("RMS           : %+13.3f %+13.3f %+13.3f (LSB-rms)\n", 
                sqrt(RMS[X] / total_count),
                sqrt(RMS[Y] / total_count),
                sqrt(RMS[Z] / total_count));
    if (VERBOSE_OUT) {
        MPL_LOGI("RMS ^ 2       : %+13.3f %+13.3f %+13.3f\n", 
                    RMS[X] / total_count, 
                    RMS[Y] / total_count, 
                    RMS[Z] / total_count); 
    }

    if (RMS[X] == 0 || RMS[Y] == 0 || RMS[Y] == 0) {
        /*  If any of the RMS noise value returns zero, 
            then we might have dead gyro or FIFO/register failure, 
            the part is sleeping, or the part is not responsive */
        retVal |= 1 << 9;
    }

    temperature /= 3;
    if (VERBOSE_OUT)
        MPL_LOGI("Temperature   : %+13.3f %13s %13s (deg. C)\n", 
                 SHORT_TO_TEMP_C(temperature), "", "");
    
    /* 4th, Sense path test */
    if (USE_SENSE_PATH_TEST) {
        float AvgFromZ[3] = {.0f, .0f, .0f};

        total_count = total_count_axis[Z];
        tmp = total_count != 0 ? total_count : 1;
        for (i = total_count_axis[X] + total_count_axis[Y];
             i < total_count_axis[X] + total_count_axis[Y] + total_count;
             i++) {
            AvgFromZ[X] += 1.f * x[i] / tmp; 
            AvgFromZ[Y] += 1.f * y[i] / tmp; 
            AvgFromZ[Z] += 1.f * z[i] / tmp;
        }
        for(j = 0; j < 3; j++) {
            newDelta[j] = (float)fabs(Avg[j] - AvgFromZ[j]) ;
        }
        MPL_LOGI("SP Z Gyro bias: %+13.3f %+13.3f %+13.3f (LSB)\n", 
                    AvgFromZ[X], AvgFromZ[Y], AvgFromZ[Z]); 
        if (VERBOSE_OUT && 
            (oldDelta[X] != 0.f || oldDelta[Y] != 0.f || oldDelta[Z] != 0.f)) {
                MPL_LOGI("absOldDelta   : %+13.3f %+13.3f %+13.3f\n", 
                            oldDelta[X], oldDelta[Y], oldDelta[Z]); 
                MPL_LOGI("absNewDelta   : %+13.3f %+13.3f %+13.3f\n", 
                            newDelta[X], newDelta[Y], newDelta[Z]); 
        }
        for (j = 0; j < 3; j++) {
            if (oldDelta[j] <= 0.f) {
                /* OTP stored SPcode was less that designed SPcode_min_limit
                   ==> ignore Sense Path test for this axis */
                continue;   
            }
            if (fabs(newDelta[j] - oldDelta[j]) > 
                     oldDelta[j] * test_setup.SPShiftThresh) {
                MPL_LOGI("%s-Gyro Sense Path test exceeded threshold "
                         "(threshold = %.3f) : %.3f\n", 
                         a_name[j], test_setup.SPShiftThresh * oldDelta[j], 
                         fabs(newDelta[j] - oldDelta[j]));
                retVal |= 1 << (10+j);
            }
        }
        sprintf(tmpStr, "SP delta      : ");
        for (j = 0; j < 3; j++) {
            if (oldDelta[j] > 0.f) {
                sprintf(tmpStr, "%s%+13.3f ",
                        tmpStr, fabs(newDelta[j] - oldDelta[j]));
            }
            else {
                sprintf(tmpStr, "%s%13s ", tmpStr, "N/A");
            }
        }
        MPL_LOGI("%s\n", tmpStr);
        if (VERBOSE_OUT && 
            (oldDelta[X] != 0.f || oldDelta[Y] != 0.f || oldDelta[Z] != 0.f)) {
                float deltaPerc[3] = {0.f, 0.f, 0.f};
                char percs[100] = "Perc Var Delta: ";
                for(j = 0; j < 3; j++) {
                    if (oldDelta[j] != 0.f) {
                        deltaPerc[j] = (newDelta[j] - oldDelta[j]) / 
                                            oldDelta[j] * 100.f;
                        sprintf(percs, "%s%+13.3f ", percs, deltaPerc[j]);
                    } else {
                        sprintf(percs, "%s%13s ", percs, "N/A");
                    }
                }
                MPL_LOGI("%s(%%)\n", percs);
        }

    }

    /* load into final storage */
    *temp_avg = (short)temperature;
    gyro_biases[X] = FLOAT_TO_SHORT(Avg[X]);
    gyro_biases[Y] = FLOAT_TO_SHORT(Avg[Y]);
    gyro_biases[Z] = FLOAT_TO_SHORT(Avg[Z]);
    return retVal;
#endif
}

/* ****************************************************************************
 * Leave code in file but [#if 0] it out to prevent 'unused' warning.
 * ****************************************************************************/
#if 0
/**
 *  @internal
 *  @brief  Retrieve the unique MPU device identifier from the internal OTP
 *          bank 0 memory.
 *  @param  mlsl_handle
 *              serial interface handle to allow serial communication with the 
 *              device, both gyro and accelerometer.
 *  @return 0 on success, a non-zero error code from the serial layer on error.
 */
static tMLError TestGetMPUId(void *mlsl_handle)
{
    tMLError result;
    unsigned char otp0[8];

    
    result = 
        MLSLSerialReadMem(mlsl_handle, mputestCfgPtr->addr,
            (BIT_PRFTCH_EN | BIT_CFG_USER_BANK | MPU_MEM_OTP_BANK_0) << 8 | 
            0x00, 6, otp0);
    if (result)
        goto close;

    MPL_LOGI("\n");
    MPL_LOGI("DIE_ID   : %06X\n", 
                ((int)otp0[1] << 8 | otp0[0]) & 0x1fff);
    MPL_LOGI("WAFER_ID : %06X\n", 
                (((int)otp0[2] << 8 | otp0[1]) & 0x03ff ) >> 5);
    MPL_LOGI("A_LOT_ID : %06X\n", 
                ( ((int)otp0[4] << 16 | (int)otp0[3] << 8 | 
                otp0[2]) & 0x3ffff) >> 2);
    MPL_LOGI("W_LOT_ID : %06X\n", 
                ( ((int)otp0[5] << 8 | otp0[4]) & 0x3fff) >> 2);
    MPL_LOGI("WP_ID    : %06X\n", 
                ( ((int)otp0[6] << 8 | otp0[5]) & 0x03ff) >> 7);
    MPL_LOGI("REV_ID   : %06X\n", otp0[6] >> 2);
    MPL_LOGI("\n");

close:
    result = 
        MLSLSerialWriteSingle(mlsl_handle, mputestCfgPtr->addr, MPUREG_BANK_SEL, 0x00);
    return result;
}
#endif


#define N_ACCEL_SAMPLES 20
/**
 *  @brief  If requested via TestSetupAccel(), test the accelerometer biases 
 *          and calculate the necessary bias correction.
 *  @param  mlsl_handle
 *              serial interface handle to allow serial communication with the 
 *              device, both gyro and accelerometer.
 *  @param  bias
 *              output pointer to store the initial bias calculation provided 
 *              by the MPU Self Test.  Requires 3 elements to store accel X, Y, 
 *              and Z axis bias.
 *  @return 0 on success. A non-zero error code on error.
 */
int TestAccel(void *mlsl_handle, short *bias)
{
    int i;
    short vals[N_ACCEL_SAMPLES][3];
    float x = 0.f, y = 0.f, z = 0.f, zg = 0.f;
    float RMS[3];
    float accelRmsThresh = 1000000.f;
    float range;
    float gravity;
    unsigned short res;
    struct mpu3050_platform_data *mputestPData = mputestCfgPtr->pdata;
#if !defined(MPUSELFTEST_STANDALONE)
    static struct mldl_cfg mldl_cfg;

    memcpy(&mldl_cfg,MLDLGetCfg(), sizeof(mldl_cfg));
#endif
    /* load the slave descr from the getter */
    if (mputestPData->accel.get_slave_descr == NULL) {
        MPL_LOGI("\n");
        MPL_LOGI("No accelerometer configured\n");
        return 0;
    }
    if (mputestCfgPtr->accel == NULL) {
        MPL_LOGI("\n");
        MPL_LOGI("No accelerometer configured\n");
        return 0;
    }

    RANGE_FIXEDPOINT_TO_FLOAT(mputestCfgPtr->accel->range, range);
    gravity = 32768.f / range;

    /* resume the accel */
#if !defined(MPUSELFTEST_STANDALONE)
    mldl_cfg.requested_sensors = ML_THREE_AXIS_GYRO | ML_THREE_AXIS_ACCEL;
    res = mpu3050_resume(&mldl_cfg, mlsl_handle, NULL, NULL, NULL, 1, 1, 0, 0);
    if(res != ML_SUCCESS) {
        goto accel_error;
    }
    /* wait at least a sample cycle for the
       accel data to be retrieved by MPU */
    MLOSSleep(5); 
#else
    if (mputestCfgPtr->accel->resume == NULL) {
        MPL_LOGI("\n");
        MPL_LOGI("Accelerometer not configured properly - could not init\n");
        return 1;
    }
    res = mputestCfgPtr->accel->resume(mlsl_handle, mputestCfgPtr->accel, &mputestPData->accel);
    if(res != ML_SUCCESS) {
        goto accel_error;
    }
    /* go in bypass mode */
    res = MLSLSerialWriteSingle(mlsl_handle, mputestCfgPtr->addr, 
				MPUREG_USER_CTRL, 0x00);
    if(res != ML_SUCCESS) {
        goto accel_error;
    }
#endif

    /* collect the samples  */ 
    for(i = 0; i < N_ACCEL_SAMPLES; i++) {
        if (TestGetData(mlsl_handle, mputestCfgPtr, vals[i])) {
            goto accel_error;
        }
        x += 1.f * vals[i][X] / N_ACCEL_SAMPLES;
        y += 1.f * vals[i][Y] / N_ACCEL_SAMPLES;
        z += 1.f * vals[i][Z] / N_ACCEL_SAMPLES;
    }

      
#if !defined(MPUSELFTEST_STANDALONE)
    res = mpu3050_suspend(&mldl_cfg, mlsl_handle, NULL, NULL, NULL, 1, 1, 1, 1);
    if (res != ML_SUCCESS) {
        goto accel_error;
    }
#endif

    MPL_LOGI("Accel biases  : %+13.3f %+13.3f %+13.3f (LSB)\n", x, y, z);
    if (VERBOSE_OUT) {
        MPL_LOGI("Accel biases  : %+13.3f %+13.3f %+13.3f (gee)\n",
                    x / gravity, y / gravity, z / gravity);
    }
    
    bias[0] = FLOAT_TO_SHORT(x);
    bias[1] = FLOAT_TO_SHORT(y);
    zg = z - g_zSign * gravity;
    bias[2] = FLOAT_TO_SHORT(zg);

    MPL_LOGI("Accel correct.: %+13d %+13d %+13d (LSB)\n", 
             bias[0], bias[1], bias[2]);
    if (VERBOSE_OUT) {
        float fs;
        RANGE_FIXEDPOINT_TO_FLOAT(mputestCfgPtr->accel->range, fs);
        fs = 32768.f / fs;
        MPL_LOGI("Accel correct.: "
               "%+13.3f %+13.3f %+13.3f (gee)\n",
                    1.f * bias[0] / gravity, 
                    1.f * bias[1] / gravity, 
                    1.f * bias[2] / gravity);
    }

    /* accel RMS - for now the threshold is only indicative */
    for (i = 0,
             RMS[X] = 0.f, RMS[Y] = 0.f, RMS[Z] = 0.f;
         i <  N_ACCEL_SAMPLES; i++) {
        RMS[X] += (vals[i][X] - x) * (vals[i][X] - x);
        RMS[Y] += (vals[i][Y] - y) * (vals[i][Y] - y);
        RMS[Z] += (vals[i][Z] - z) * (vals[i][Z] - z);
    }
    for (i = 0; i < 3; i++) {
        if (RMS[i] >  accelRmsThresh * accelRmsThresh * N_ACCEL_SAMPLES) {
            MPL_LOGI("%s-Accel RMS (%.2f) exceeded threshold "
                     "(threshold = %.2f)\n", 
                     a_name[i], sqrt(RMS[i] / N_ACCEL_SAMPLES), 
                     accelRmsThresh);
            goto accel_error;
        }
    }
    MPL_LOGI("RMS           : %+13.3f %+13.3f %+13.3f (LSB-rms)\n", 
             sqrt(RMS[X] / N_ACCEL_SAMPLES),
             sqrt(RMS[Y] / N_ACCEL_SAMPLES),
             sqrt(RMS[Z] / N_ACCEL_SAMPLES));
    
    return 0; /* success */
   
accel_error:  /* error */
    bias[0] = bias[1] = bias[2] = 0;
    return 1;
}

/**
 *  @brief  The main entry point of the MPU Self Test, triggering the run of 
 *          the single tests, for gyros and accelerometers.
 *          Prepares the MPU for the test, taking the device out of low power
 *          state if necessary, switching the MPU secondary I2C interface into
 *          bypass mode and restoring the original power state at the end of 
 *          the test.
 *          This function is also responsible for encoding the output of each 
 *          test in the correct format as it is stored on the file/medium of 
 *          choice (according to MLSLWriteCal() function).  The format needs to 
 *          stay perfectly consistent with the one expected by the 
 *          corresponding loader in ml_stored_data.c; currectly the loaded in 
 *          use is MLLoadCal_V1 (record type 1 - initial calibration).
 *  @param  mlsl_handle
 *              serial interface handle to allow serial communication with the 
 *              device, both gyro and accelerometer.
 *  @return 0 on success.  A non-zero error code on error. Propagates the 
 *          errors from the tests up to the caller.
 */
int MPUTest(void *mlsl_handle)
{
    int result = 0;

    short temp_avg;
    short gyro_biases[3] = {0, 0, 0};
    short accel_biases[3] = {0, 0, 0};

    unsigned long testStart = MLOSGetTickCount();
    long accelSens;
    unsigned char pwr_mgm;
    int ptr;
    int tmp;
    long long lltmp;
    uint32_t chk;

    MPL_LOGI("Collecting %d groups of 600 ms samples for each axis\n",
                DEF_TEST_TIME_PER_AXIS / 600);
    MPL_LOGI("\n");

    result = MLSLSerialRead(
                mlsl_handle, mputestCfgPtr->addr, 
                MPUREG_PWR_MGM, 1, &pwr_mgm);
    CHECK_TEST_ERROR(result);
    /* reset */
    result = MLSLSerialWriteSingle(
                mlsl_handle, mputestCfgPtr->addr, 
                MPUREG_PWR_MGM, pwr_mgm | 0x80);
    CHECK_TEST_ERROR(result);
    MLOSSleep(5);
    /* wake up */
    if (pwr_mgm & 0x40) {
        result = MLSLSerialWriteSingle(
                    mlsl_handle, mputestCfgPtr->addr, 
                    MPUREG_PWR_MGM, 0x00); 
        CHECK_TEST_ERROR(result);
    }
    MLOSSleep(60);

    /*
    result = TestGetMPUId(mlsl_handle);
    if (result != ML_SUCCESS) {
        MPL_LOGI("Could not read the device's unique ID\n");
        MPL_LOGI("\n");
        return result;
    }
    */

    /* adjust the gyro sensitivity according to the trim value
       of the gyro sensitivity */
    adjGyroSens = test_setup.gyroSens * mputestCfgPtr->trim / 131.f;
    test_setup.gyroFS = (int)(32768.f / adjGyroSens);

    /* collect gyro and temperature data */
    result = TestGyro(mlsl_handle, gyro_biases, &temp_avg);
    MPL_LOGI("\n");
    MPL_LOGI("Test time : %ld ms\n", MLOSGetTickCount() - testStart);
    if (result)
        return result;

    /* collect accel data.  if this step is skipped, 
       ensure the array still contains zeros. */
    if (TestAccel(mlsl_handle, accel_biases))
        return result;

    /* restore the power state the part was found in */
    if (pwr_mgm & 0x40) {
        result = MLSLSerialWriteSingle(mlsl_handle, mputestCfgPtr->addr, 
                                       MPUREG_PWR_MGM, pwr_mgm);
        CHECK_TEST_ERROR(result);
    }

    ptr = 0;
    dataStore[ptr++] = 0;       /* total len of factory cal */
    dataStore[ptr++] = 0;
    dataStore[ptr++] = 0;
    dataStore[ptr++] = ML_INIT_CAL_LEN;
    dataStore[ptr++] = 0;
    dataStore[ptr++] = 5;       /* record type 5 - initial calibration */

    tmp = temp_avg;             /* temperature */
    if (tmp < 0) tmp += 2 << 16;
    USHORT_TO_CHARS(&dataStore[ptr], tmp);
    ptr += 2;

    /* NOTE : 2 * test_setup.gyroFS == 65536 / (32768 / test_setup.gyroFS) */
    lltmp = (long)gyro_biases[0] * 2 * test_setup.gyroFS; /* x gyro avg */
    if (lltmp < 0) lltmp += 1LL << 32;
    UINT_TO_CHARS(&dataStore[ptr], (uint32_t)lltmp);
    ptr += 4;
    lltmp = (long)gyro_biases[1] * 2 * test_setup.gyroFS; /* y gyro avg */
    if (lltmp < 0) lltmp += 1LL << 32;
    UINT_TO_CHARS(&dataStore[ptr], (uint32_t)lltmp);
    ptr += 4;
    lltmp = (long)gyro_biases[2] * 2 * test_setup.gyroFS; /* z gyro avg */
    if (lltmp < 0) lltmp += 1LL << 32;
    UINT_TO_CHARS(&dataStore[ptr], (uint32_t)lltmp);
    ptr += 4;

    if (mputestCfgPtr->accel != NULL) {
        float fs;
        RANGE_FIXEDPOINT_TO_FLOAT(mputestCfgPtr->accel->range, fs);
        accelSens = (long)(32768 / fs);
    } else {
        accelSens = 1; /* would be 0, but 1 to avoid divide-by-0 below */
    }
    lltmp = (long)accel_biases[0] * 65536 / accelSens; /* x accel avg */
    if (lltmp < 0) lltmp += 1LL << 32;
    UINT_TO_CHARS(&dataStore[ptr], (uint32_t)lltmp);
    ptr += 4;
    lltmp = (long)accel_biases[1] * 65536 / accelSens; /* y accel avg */
    if (lltmp < 0) lltmp += 1LL << 32;
    UINT_TO_CHARS(&dataStore[ptr], (uint32_t)lltmp);
    ptr += 4;
    lltmp = (long)accel_biases[2] * 65536 / accelSens; /* z accel avg */
    if (lltmp < 0) lltmp += 1LL << 32;
    UINT_TO_CHARS(&dataStore[ptr], (uint32_t)lltmp);
    ptr += 4;
    
    /* add a checksum for data */
    chk = ml_checksum(
        dataStore + ML_CAL_HDR_LEN, 
        ML_INIT_CAL_LEN - ML_CAL_HDR_LEN - ML_CAL_CHK_LEN);
    UINT_TO_CHARS(&dataStore[ptr], chk);
    ptr += 4;

    if (ptr != ML_INIT_CAL_LEN) {
        MPL_LOGI("Invalid calibration data length: exp %d, got %d\n",
                    ML_INIT_CAL_LEN, ptr);
        return -1;
    }

    return result;
}

/**
 *  @brief  The main test API. Runs the MPU Self Test and, if successful, 
 *          stores the encoded initial calibration data on the final storage 
 *          medium of choice (cfr. MLSLWriteCal() and the MLCAL_FILE define in 
 *          your mlsl implementation).
 *  @param  mlsl_handle
 *              serial interface handle to allow serial communication with the 
 *              device, both gyro and accelerometer.
 *  @return 0 on success or a non-zero error code from the callees on error.
 */
tMLError FactoryCalibrate(void *mlsl_handle) 
{    
    int result;

    result = MPUTest(mlsl_handle);
    MPL_LOGI("\n");
    if (result == 0) {
        MPL_LOGI("Test : PASSED\n");
    } else {
        MPL_LOGI("Test : FAILED %d/%04X - Biases NOT stored\n", result, result);
        return result; /* abort writing the calibration if the 
                          test is not successful */
    }
    MPL_LOGI("\n");

    result = MLSLWriteCal(dataStore, ML_INIT_CAL_LEN);
    if (result) {
        MPL_LOGI("Error : cannot write calibration on file - error %d\n", 
            result);
        return result;
    }

    return ML_SUCCESS;
}



/* -----------------------------------------------------------------------
    accel interface functions
 -----------------------------------------------------------------------*/

/**
 *  @internal
 *  @brief  Reads data for X, Y, and Z axis from the accelerometer device. 
 *          Used only if an accelerometer has been setup using the
 *          TestSetupAccel() API.
 *          Takes care of the accelerometer endianess according to how the 
 *          device has been described in the corresponding accelerometer driver
 *          file.
 *  @param  mlsl_handle
 *              serial interface handle to allow serial communication with the 
 *              device, both gyro and accelerometer.
 *  @param  slave
 *              a pointer to the descriptor of the slave accelerometer device
 *              in use. Contains the necessary information to operate, read, 
 *              and communicate with the accelerometer device of choice.
 *              See the declaration of struct ext_slave_descr in mpu.h.
 *  @param  pdata
 *              a pointer to the platform info of the slave accelerometer 
 *              device in use. Describes how the device is oriented and 
 *              mounted on host platform's PCB.
 *  @param  vals
 *              output pointer to return the accelerometer's X, Y, and Z axis
 *              sensor data collected.
 *  @return 0 on success or a non-zero error code on error.
 */
static tMLError TestGetData(
                    void *mlsl_handle,
                    struct mldl_cfg *mputestCfgPtr, 
                    short *vals)
{
    tMLError result;
    unsigned char data[20];
    struct ext_slave_descr *slave = mputestCfgPtr->accel;
    result = MLSLSerialRead(mlsl_handle, mputestCfgPtr->addr, 0x23, 
                            mputestCfgPtr->accel->len, data);
    ERROR_CHECK(result);

    if (VERBOSE_OUT) {
        MPL_LOGI("Accel         :        0x%02X%02X        0x%02X%02X        0x%02X%02X (raw)\n",
            ACCEL_UNPACK(data));
    }
    
    if ( CHECK_NACKS(data) ) {
        MPL_LOGI("Error fetching data from the accelerometer : "
                 "all bytes read 0xff\n");
        return ML_ERROR_SERIAL_READ;
    }

    if (slave->endian == EXT_SLAVE_BIG_ENDIAN) {
        vals[0] = CHARS_TO_SHORT(&data[0]);
        vals[1] = CHARS_TO_SHORT(&data[2]);
        vals[2] = CHARS_TO_SHORT(&data[4]);
    } else {
        vals[0] = CHARS_TO_SHORT_SWAPPED(&data[0]);
        vals[1] = CHARS_TO_SHORT_SWAPPED(&data[2]);
        vals[2] = CHARS_TO_SHORT_SWAPPED(&data[4]);
    }

    if (VERBOSE_OUT) {
        MPL_LOGI("Accel         : %+13d %+13d %+13d (LSB)\n",
                 vals[0], vals[1], vals[2]);
    }
    return ML_SUCCESS;
}

#ifdef __cplusplus
}
#endif

/**
 *  @}
 */

