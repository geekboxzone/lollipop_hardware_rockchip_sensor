/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */

/******************************************************************************
 *
 * $Id: ml_stored_data.c 5139 2011-04-02 05:40:55Z mcaramello $
 *
 *****************************************************************************/

/**
 * @defgroup ML_STORED_DATA
 *
 * @{
 *      @file     ml_stored_data.c
 *      @brief    functions for reading and writing stored data sets.
 *                Typically, these functions process stored calibration data.
 */

#include "ml_stored_data.h"
#include "tempComp.h"
#include "ml.h"
#include "mltypes.h"
#include "mlinclude.h"
#include "compass.h"
#include "dmpKey.h"
#include "dmpDefault.h"
#include "mlstates.h"
#include "checksum.h"
#include "mlsupervisor.h"

#include "mlsl.h"
#include "mlos.h"

#include "log.h"
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-storeload"

/*
    Typedefs
*/
typedef tMLError(*tMLLoadFunc)(unsigned char*, unsigned short);

/*
    Globals
*/
extern tMLXData mlxData;

/* 
    Debugging Definitions
    set LOADCAL_DEBUG and/or STORECAL_DEBUG to 1 print the fields
    being read or stored in human-readable format.
    When set to 0, the compiler will optimize and remove the printouts 
    from the code.
*/
#define LOADCAL_DEBUG    0
#define STORECAL_DEBUG   0

#define LOADCAL_LOG(...)                        \
    if(LOADCAL_DEBUG)                           \
        MPL_LOGI("LOADCAL: " __VA_ARGS__)
#define STORECAL_LOG(...)                       \
    if(STORECAL_DEBUG)                          \
        MPL_LOGI("STORECAL: " __VA_ARGS__)

/**
 *  @brief  Duplicate of the TempCompFindTempBin function in the libmpl 
 *          advanced algorithms library. To remove cross-dependency, for now,
 *          we reimplement the same function here.
 *  @param  temp
 *              the temperature (1 count == 1 degree C).
 */
int FindTempBin(float temp)
{
    int bin = (int)((temp - MIN_TEMP) / TEMP_PER_BIN);
    if (bin < 0)
        bin = 0;
    if (bin > BINS - 1)
        bin = BINS - 1;
    return bin;
}

/**
 * @brief   Returns the length of the <b>MPL internal calibration data</b>.
 *          Should be called before allocating the memory required to store 
 *          this data to a file.
 *          This function returns the total size required to store the cal 
 *          data including the header (4 bytes) and the checksum (2 bytes).
 *
 *  @pre    Must be in ML_STATE_DMP_OPENED state. 
 *          MLDmpOpen() or MLDmpStop() must have been called.
 *          MLDmpStart() and MLDmpClose() must have <b>NOT</b> 
 *          been called.
 *
 * @param   length
 *              The length of the calibration data.
 *  
 * @return  Zero is returned if the command is successful;
 *          otherwise, an error code is returned.
 */
tMLError MLGetCalLength(unsigned int *length)
{
    INVENSENSE_FUNC_START;

    *length = ML_CAL_HDR_LEN +                              // header
              BINS * PTS_PER_BIN * 4 * 4 + BINS * 4 * 2 +   // gyro cal
              ML_CAL_ACCEL_LEN +                            // accel cal
              ML_CAL_COMPASS_LEN +                          // compass cal
              ML_CAL_CHK_LEN;                               // checksum
    return ML_SUCCESS;
}

/**
 *  @brief  Converts from internal sensor register implementation
 *          to degrees C.
 *  @param  intTemp
 *              internal temperature sensor representation in LSBs.
 *  @return temperature in deg. C
 */
float tempInDegC(float intTemp)
{
    return (35.f + ((intTemp + 13200.f) / 280.f));
}

/**
 *  @brief  Loads a type 0 set of calibration data. 
 *          It parses a binary data set containing calibration data. 
 *          The binary data set is intended to be loaded from a file.
 *          This calibrations data format stores values for (in order of 
 *          appearance) :
 *          - temperature,
 *          - gyro biases for X, Y, Z axes.
 *          This calibration data would normally be produced by the MPU Self 
 *          Test and its size is 18 bytes (header and checksum included).
 *          Calibration format type 0 is currently <b>NOT</b> used and 
 *          is substituted by type 5 : MLLoadCal_V5().
 *
 *  @note   This calibration data format is obsoleted and no longer supported
 *          by the rest of the MPL.
 *
 *  @pre    One of the MLDmpOpen() must be called: MLDmpDefaultOpen() or
 *          MLDmpPedometerStandAloneOpen().
 *
 *  @param  calData
 *              A pointer to an array of bytes to be parsed.
 *  @param  len
 *              the length of the calibration 
 *
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError MLLoadCal_V0(unsigned char *calData, unsigned short len)
{
    INVENSENSE_FUNC_START;
    const short expLen = 18;
    long newGyroData[3] = {0, 0, 0};
    float newTemp = 0;
    int bin;

    LOADCAL_LOG("Entering MLLoadCal_V0\n");

    if (len != expLen) {
        MPL_LOGE("Calibration data type 1 must be %d bytes long\n", expLen);
        return ML_ERROR_FILE_READ;
    }

    newTemp = (float)((long)calData[6]) * 256 + ((long)calData[7]);
    if (newTemp > 32767.f) newTemp -= 65536.f;
    newTemp = tempInDegC(newTemp);
    LOADCAL_LOG("newTemp = %f\n", newTemp);

    newGyroData[0] = ((long)calData[8]) * 256 + ((long)calData[9]);
    if (newGyroData[0] > 32767L) newGyroData[0] -= 65536L;
    LOADCAL_LOG("newGyroData[0] = %ld\n", newGyroData[0]);
    newGyroData[1] = ((long)calData[10]) * 256 + ((long)calData[11]);
    if (newGyroData[1] > 32767L) newGyroData[1] -= 65536L;
    LOADCAL_LOG("newGyroData[2] = %ld\n", newGyroData[2]);
    newGyroData[2] = ((long)calData[12]) * 256 + ((long)calData[13]);
    if (newGyroData[2] > 32767L) newGyroData[2] -= 65536L;
    LOADCAL_LOG("newGyroData[2] = %ld\n", newGyroData[2]);

    bin = FindTempBin(newTemp);
    LOADCAL_LOG("bin = %d", bin);
    
    mlxData.mlTempData[bin][mlxData.mlTempPtrs[bin]] = 
        tempInDegC((float)newTemp / 65536.f);
    LOADCAL_LOG("mlTempData[%d][%d] = %f", 
                    bin, mlxData.mlTempPtrs[bin], 
                    mlxData.mlTempData[bin][mlxData.mlTempPtrs[bin]]);
    mlxData.mlXGyroTempData[bin][mlxData.mlTempPtrs[bin]] =
        ((float)newGyroData[0]) / 65536.f;    
    LOADCAL_LOG("mlXGyroTempData[%d][%d] = %f\n", 
                    bin, mlxData.mlTempPtrs[bin],
                    mlxData.mlXGyroTempData[bin][mlxData.mlTempPtrs[bin]]);
    mlxData.mlYGyroTempData[bin][mlxData.mlTempPtrs[bin]] =
        ((float)newGyroData[0]) / 65536.f;
    LOADCAL_LOG("mlYGyroTempData[%d][%d] = %f\n",
                    bin, mlxData.mlTempPtrs[bin], 
                    mlxData.mlYGyroTempData[bin][mlxData.mlTempPtrs[bin]]);
    mlxData.mlZGyroTempData[bin][mlxData.mlTempPtrs[bin]] =
        ((float)newGyroData[0]) / 65536.f;    
    LOADCAL_LOG("mlZGyroTempData[%d][%d] = %f\n", 
                    bin, mlxData.mlTempPtrs[bin], 
                    mlxData.mlZGyroTempData[bin][mlxData.mlTempPtrs[bin]]);

    mlxData.mlTempPtrs[bin] = (mlxData.mlTempPtrs[bin] + 1) % PTS_PER_BIN;
    LOADCAL_LOG("mlTempPtrs[%d] = %d\n", bin, mlxData.mlTempPtrs[bin]);

    if (mlxData.mlTempPtrs[bin] == 0) 
        mlxData.mlTempValidData[bin] = TRUE;
    LOADCAL_LOG("mlTempValidData[%d] = %ld\n",
                    bin, mlxData.mlTempValidData[bin]);

    mlxData.mlGotNoMotionBias = TRUE;
    LOADCAL_LOG("mlGotNoMotionBias = 1\n");
    mlxData.calLoadedFlag = TRUE;
    LOADCAL_LOG("calLoadedFlag = 1\n");

    LOADCAL_LOG("Exiting MLLoadCal_V0\n");
    return ML_SUCCESS;
}

/**
 *  @brief  Loads a type 1 set of calibration data. 
 *          It parses a binary data set containing calibration data. 
 *          The binary data set is intended to be loaded from a file.
 *          This calibrations data format stores values for (in order of 
 *          appearance) :
 *          - temperature,
 *          - gyro biases for X, Y, Z axes,
 *          - accel biases for X, Y, Z axes.
 *          This calibration data would normally be produced by the MPU Self 
 *          Test and its size is 24 bytes (header and checksum included).
 *          Calibration format type 1 is currently <b>NOT</b> used and 
 *          is substituted by type 5 : MLLoadCal_V5().
 *
 *  @note   In order to successfully work, the gyro bias must be stored 
 *          expressed in 250 dps full scale (131.072 sensitivity). Other full
 *          scale range will produce unpredictable results in the gyro biases.
 *
 *  @pre    One of the MLDmpOpen() must be called: MLDmpDefaultOpen() or
 *          MLDmpPedometerStandAloneOpen().
 *
 *  @param  calData
 *              A pointer to an array of bytes to be parsed.
 *  @param  len
 *              the length of the calibration 
 *
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError MLLoadCal_V1(unsigned char *calData, unsigned short len)
{
    INVENSENSE_FUNC_START;
    const short expLen = 24;
    long newGyroData[3] = {0, 0, 0};
    long accelBias[3] = {0, 0, 0};
    float gyroBias[3] = {0, 0, 0};
    float newTemp = 0;
    int bin;

    LOADCAL_LOG("Entering MLLoadCal_V1\n");

    if (len != expLen) {
        MPL_LOGE("Calibration data type 1 must be %d bytes long\n", expLen);
        return ML_ERROR_FILE_READ;
    }

    newTemp = (float)(((long)calData[6]) * 256 + ((long)calData[7]));
    if (newTemp > 32767.f) newTemp -= 65536.f;
    newTemp = tempInDegC(newTemp);
    LOADCAL_LOG("newTemp = %f\n", newTemp);

    newGyroData[0] = ((long)calData[8]) * 256 + ((long)calData[9]);
    if (newGyroData[0] > 32767L) newGyroData[0] -= 65536L;
    LOADCAL_LOG("newGyroData[0] = %ld\n", newGyroData[0]);
    newGyroData[1] = ((long)calData[10]) * 256 + ((long)calData[11]);
    if (newGyroData[1] > 32767L) newGyroData[1] -= 65536L;
    LOADCAL_LOG("newGyroData[1] = %ld\n", newGyroData[1]);
    newGyroData[2] = ((long)calData[12]) * 256 + ((long)calData[13]);
    if (newGyroData[2] > 32767L) newGyroData[2] -= 65536L;
    LOADCAL_LOG("newGyroData[2] = %ld\n", newGyroData[2]);

    bin = FindTempBin(newTemp);
    LOADCAL_LOG("bin = %d\n", bin);

    gyroBias[0] = ((float)newGyroData[0]) / 131.072f;
    gyroBias[1] = ((float)newGyroData[1]) / 131.072f;
    gyroBias[2] = ((float)newGyroData[2]) / 131.072f;
    LOADCAL_LOG("gyroBias[0] = %f\n", gyroBias[0]);
    LOADCAL_LOG("gyroBias[1] = %f\n", gyroBias[1]);
    LOADCAL_LOG("gyroBias[2] = %f\n", gyroBias[2]);

    mlxData.mlTempData[bin][mlxData.mlTempPtrs[bin]] = newTemp;
    LOADCAL_LOG("mlTempData[%d][%d] = %f", 
                    bin, mlxData.mlTempPtrs[bin], 
                    mlxData.mlTempData[bin][mlxData.mlTempPtrs[bin]]);
    mlxData.mlXGyroTempData[bin][mlxData.mlTempPtrs[bin]] = gyroBias[0];
    LOADCAL_LOG("mlXGyroTempData[%d][%d] = %f\n", 
                    bin, mlxData.mlTempPtrs[bin],
                    mlxData.mlXGyroTempData[bin][mlxData.mlTempPtrs[bin]]);
    mlxData.mlYGyroTempData[bin][mlxData.mlTempPtrs[bin]] = gyroBias[1];
    LOADCAL_LOG("mlYGyroTempData[%d][%d] = %f\n",
                    bin, mlxData.mlTempPtrs[bin], 
                    mlxData.mlYGyroTempData[bin][mlxData.mlTempPtrs[bin]]);
    mlxData.mlZGyroTempData[bin][mlxData.mlTempPtrs[bin]] = gyroBias[2];
    LOADCAL_LOG("mlZGyroTempData[%d][%d] = %f\n", 
                    bin, mlxData.mlTempPtrs[bin], 
                    mlxData.mlZGyroTempData[bin][mlxData.mlTempPtrs[bin]]);

    mlxData.mlTempPtrs[bin] = (mlxData.mlTempPtrs[bin] + 1) % PTS_PER_BIN;
    LOADCAL_LOG("mlTempPtrs[%d] = %d\n", bin, mlxData.mlTempPtrs[bin]);

    if (mlxData.mlTempPtrs[bin] == 0) 
        mlxData.mlTempValidData[bin] = TRUE; 
    LOADCAL_LOG("mlTempValidData[%d] = %ld\n", 
                    bin, mlxData.mlTempValidData[bin]);

    /* load accel biases and apply immediately */
    accelBias[0] = ((long)calData[14]) * 256 + ((long)calData[15]);
    if (accelBias[0] > 32767) accelBias[0] -= 65536L;
    accelBias[0] = (long)(
        (long long)accelBias[0] * 65536L * mlxData.mlAccelSens / 1073741824L);
    LOADCAL_LOG("accelBias[0] = %ld\n", accelBias[0]);
    accelBias[1] = ((long) calData[16]) * 256 + ((long)calData[17]);
    if (accelBias[1] > 32767) accelBias[1] -= 65536L;
    accelBias[1] = (long)(
        (long long)accelBias[1] * 65536L * mlxData.mlAccelSens / 1073741824L);
    LOADCAL_LOG("accelBias[1] = %ld\n", accelBias[1]);
    accelBias[2] = ((long)calData[18]) * 256 + ((int)calData[19]);
    if (accelBias[2] > 32767) accelBias[2] -= 65536L;
    accelBias[2] = (long)(
        (long long)accelBias[2] * 65536L * mlxData.mlAccelSens / 1073741824L);
    LOADCAL_LOG("accelBias[2] = %ld\n", accelBias[2]);
    ERROR_CHECK(MLSetArray(ML_ACCEL_BIAS, accelBias));

    mlxData.mlGotNoMotionBias = TRUE;
    LOADCAL_LOG("mlGotNoMotionBias = 1\n");
    mlxData.calLoadedFlag = TRUE;
    LOADCAL_LOG("calLoadedFlag = 1\n");

    LOADCAL_LOG("Exiting MLLoadCal_V1\n");
    return ML_SUCCESS;
}

/**
 *  @brief  Loads a type 2 set of calibration data. 
 *          It parses a binary data set containing calibration data. 
 *          The binary data set is intended to be loaded from a file.
 *          This calibrations data format stores values for (in order of 
 *          appearance) :
 *          - temperature compensation : temperature data points,
 *          - temperature compensation : gyro biases data points for X, Y, 
 *              and Z axes.
 *          - accel biases for X, Y, Z axes.
 *          This calibration data is produced internally by the MPL and its 
 *          size is 2222 bytes (header and checksum included).
 *          Calibration format type 2 is currently <b>NOT</b> used and 
 *          is substituted by type 4 : MLLoadCal_V4().

 *  @pre    One of the MLDmpOpen() must be called: MLDmpDefaultOpen() or
 *          MLDmpPedometerStandAloneOpen().
 *
 *  @param  calData
 *              A pointer to an array of bytes to be parsed.
 *  @param  len
 *              the length of the calibration 
 *
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError MLLoadCal_V2(unsigned char *calData, unsigned short len)
{
    INVENSENSE_FUNC_START;
    const short expLen = 2222;
    long accel_bias[3] ;
    int ptr = ML_CAL_HDR_LEN;

    int i, j;
    long long tmp;

    LOADCAL_LOG("Entering MLLoadCal_V2\n");

    if (len != expLen) {
        MPL_LOGE("Calibration data type 2 must be %d bytes long (got %d)\n", expLen, len);
        return ML_ERROR_FILE_READ;
    }

    for (i = 0; i < BINS; i++) {
        mlxData.mlTempPtrs[i] = 0;
        mlxData.mlTempPtrs[i] += 16777216L * ((long)calData[ptr++]);
        mlxData.mlTempPtrs[i] += 65536L * ((long)calData[ptr++]);
        mlxData.mlTempPtrs[i] += 256 * ((int)calData[ptr++]);
        mlxData.mlTempPtrs[i] += (int)calData[ptr++];
        LOADCAL_LOG("mlTempPtrs[%d] = %d\n", i, mlxData.mlTempPtrs[i]); 
    }
    for (i = 0; i < BINS; i++) {
        mlxData.mlTempValidData[i] = 0;
        mlxData.mlTempValidData[i] += 16777216L * ((long)calData[ptr++]);
        mlxData.mlTempValidData[i] += 65536L * ((long)calData[ptr++]);
        mlxData.mlTempValidData[i] += 256 * ((int)calData[ptr++]);
        mlxData.mlTempValidData[i] += (int)calData[ptr++];
        LOADCAL_LOG("mlTempValidData[%d] = %ld\n", 
                        i, mlxData.mlTempValidData[i]); 
    }

    for (i = 0; i < BINS; i++) {
        for (j = 0; j < PTS_PER_BIN; j++) {
            tmp = 0;
            tmp += 16777216LL * (long long)calData[ptr++];
            tmp += 65536LL * (long long)calData[ptr++];
            tmp += 256LL * (long long)calData[ptr++];
            tmp += (long long)calData[ptr++];
            if (tmp > 2147483648LL) {
                tmp -= 4294967296LL;
            }
            mlxData.mlTempData[i][j] = ((float)tmp) / 65536.0f;
            LOADCAL_LOG("mlTempData[%d][%d] = %f\n",
                            i, j, mlxData.mlTempData[i][j]);
        }        
        
    }
    for (i = 0; i < BINS; i++) {
        for (j = 0; j < PTS_PER_BIN; j++) {
            tmp = 0;
            tmp += 16777216LL * (long long)calData[ptr++];
            tmp += 65536LL * (long long)calData[ptr++];
            tmp += 256LL * (long long)calData[ptr++];
            tmp += (long long)calData[ptr++];
            if (tmp > 2147483648LL) {
                tmp -= 4294967296LL;
            }
            mlxData.mlXGyroTempData[i][j] = ((float)tmp) / 65536.0f;
            LOADCAL_LOG("mlXGyroTempData[%d][%d] = %f\n", 
                            i, j, mlxData.mlXGyroTempData[i][j]);
        }        
    }
    for (i = 0; i < BINS; i++) {
        for (j = 0; j < PTS_PER_BIN; j++) {
            tmp = 0;
            tmp += 16777216LL * (long long)calData[ptr++];
            tmp += 65536LL * (long long)calData[ptr++];
            tmp += 256LL * (long long)calData[ptr++];
            tmp += (long long)calData[ptr++];
            if (tmp > 2147483648LL) {
                tmp -= 4294967296LL;
            }
            mlxData.mlYGyroTempData[i][j] = ((float)tmp) / 65536.0f;
            LOADCAL_LOG("mlYGyroTempData[%d][%d] = %f\n",
                            i, j, mlxData.mlYGyroTempData[i][j]);
        }        
    }
    for (i = 0; i < BINS; i++) {
        for (j = 0; j < PTS_PER_BIN; j++) {
            tmp = 0;
            tmp += 16777216LL * (long long)calData[ptr++];
            tmp += 65536LL * (long long)calData[ptr++];
            tmp += 256LL * (long long)calData[ptr++];
            tmp += (long long)calData[ptr++];
            if (tmp > 2147483648LL) {
                tmp -= 4294967296LL;
            }
            mlxData.mlZGyroTempData[i][j] = ((float)tmp) / 65536.0f;
            LOADCAL_LOG("mlZGyroTempData[%d][%d] = %f\n",
                            i, j, mlxData.mlZGyroTempData[i][j]);
        }        
    }

    /* read the accel biases */
    for (i = 0; i < 3; i++) {
        uint32_t t = 0;
        t += 16777216UL * ((uint32_t)calData[ptr++]);
        t += 65536UL * ((uint32_t)calData[ptr++]);
        t += 256u * ((uint32_t)calData[ptr++]);
        t += (uint32_t)calData[ptr++];
        accel_bias[i] = (int32_t)t;
        LOADCAL_LOG("accel_bias[%d] = %ld\n", i, accel_bias[i]);
    }

    ERROR_CHECK(MLSetArray(ML_ACCEL_BIAS, accel_bias));

    mlxData.mlGotNoMotionBias = TRUE;
    LOADCAL_LOG("mlGotNoMotionBias = 1\n");
    mlxData.calLoadedFlag = TRUE;
    LOADCAL_LOG("calLoadedFlag = 1\n");

    LOADCAL_LOG("Exiting MLLoadCal_V2\n");
    return ML_SUCCESS;
}

/**
 *  @brief  Loads a type 3 set of calibration data. 
 *          It parses a binary data set containing calibration data. 
 *          The binary data set is intended to be loaded from a file.
 *          This calibrations data format stores values for (in order of 
 *          appearance) :
 *          - temperature compensation : temperature data points,
 *          - temperature compensation : gyro biases data points for X, Y, 
 *              and Z axes.
 *          - accel biases for X, Y, Z axes.
 *          - compass biases for X, Y, Z axes and bias tracking algorithm 
 *              mock-up.
 *          This calibration data is produced internally by the MPL and its 
 *          size is 2429 bytes (header and checksum included).
 *          Calibration format type 3 is currently <b>NOT</b> used and 
 *          is substituted by type 4 : MLLoadCal_V4().

 *  @pre    One of the MLDmpOpen() must be called: MLDmpDefaultOpen() or
 *          MLDmpPedometerStandAloneOpen().
 *
 *  @param  calData
 *              A pointer to an array of bytes to be parsed.
 *  @param  len
 *              the length of the calibration 
 *
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError MLLoadCal_V3(unsigned char *calData, unsigned short len)
{
    INVENSENSE_FUNC_START;
    union doubleToLongLong {
        double db;
        unsigned long long ll;
    } dToLL;  

    const short expLen = 2429;
    long bias[3];
    int i, j;
    int ptr = ML_CAL_HDR_LEN;
    long long tmp;
    
    LOADCAL_LOG("Entering MLLoadCal_V3\n");

    if (len != expLen) {
        MPL_LOGE("Calibration data type 3 must be %d bytes long (got %d)\n", expLen, len);
        return ML_ERROR_FILE_READ;
    }

    for (i = 0; i < BINS; i++) {
        mlxData.mlTempPtrs[i] = 0;
        mlxData.mlTempPtrs[i] += 16777216L * ((long)calData[ptr++]);
        mlxData.mlTempPtrs[i] += 65536L * ((long)calData[ptr++]);
        mlxData.mlTempPtrs[i] += 256 * ((int)calData[ptr++]);
        mlxData.mlTempPtrs[i] += (int)calData[ptr++];
        LOADCAL_LOG("mlTempPtrs[%d] = %d\n", i, mlxData.mlTempPtrs[i]);
    }
    for (i = 0; i < BINS; i++) {
        mlxData.mlTempValidData[i] = 0;
        mlxData.mlTempValidData[i] += 16777216L * ((long)calData[ptr++]);
        mlxData.mlTempValidData[i] += 65536L * ((long)calData[ptr++]);
        mlxData.mlTempValidData[i] += 256 * ((int)calData[ptr++]);
        mlxData.mlTempValidData[i] += (int)calData[ptr++];
        LOADCAL_LOG("mlTempValidData[%d] = %ld\n", 
                        i, mlxData.mlTempValidData[i]);
    }

    for (i = 0; i < BINS; i++) {
        for (j = 0; j < PTS_PER_BIN; j++) {
            tmp = 0;
            tmp += 16777216LL * (long long)calData[ptr++];
            tmp += 65536LL * (long long)calData[ptr++];
            tmp += 256LL * (long long)calData[ptr++];
            tmp += (long long)calData[ptr++];
            if (tmp > 2147483648LL) {
                tmp -= 4294967296LL;
            }
            mlxData.mlTempData[i][j] = ((float)tmp) / 65536.0f;
            LOADCAL_LOG("mlTempData[%d][%d] = %f\n",
                            i, j, mlxData.mlTempData[i][j]);
        }        
    }

    for (i = 0; i < BINS; i++) {
        for (j = 0; j < PTS_PER_BIN; j++) {
            tmp = 0;
            tmp += 16777216LL * (long long)calData[ptr++];
            tmp += 65536LL * (long long)calData[ptr++];
            tmp += 256LL * (long long)calData[ptr++];
            tmp += (long long)calData[ptr++];
            if (tmp > 2147483648LL) {
                tmp -= 4294967296LL;
            }
            mlxData.mlXGyroTempData[i][j] = ((float)tmp) / 65536.0f;
            LOADCAL_LOG("mlXGyroTempData[%d][%d] = %f\n",
                            i, j, mlxData.mlXGyroTempData[i][j]);
        }        
    }
    for (i = 0; i < BINS; i++) {
        for (j = 0; j < PTS_PER_BIN; j++) {
            tmp = 0;
            tmp += 16777216LL * (long long)calData[ptr++];
            tmp += 65536LL * (long long)calData[ptr++];
            tmp += 256LL * (long long)calData[ptr++];
            tmp += (long long)calData[ptr++];
            if (tmp > 2147483648LL) {
                tmp -= 4294967296LL;
            }
            mlxData.mlYGyroTempData[i][j] = ((float)tmp) / 65536.0f;
            LOADCAL_LOG("mlXGyroTempData[%d][%d] = %f\n",
                            i, j, mlxData.mlXGyroTempData[i][j]);
        }        
    }
    for (i = 0; i < BINS; i++) {
        for (j = 0; j < PTS_PER_BIN; j++) {
            tmp = 0;
            tmp += 16777216LL * (long long)calData[ptr++];
            tmp += 65536LL * (long long)calData[ptr++];
            tmp += 256LL * (long long)calData[ptr++];
            tmp += (long long)calData[ptr++];
            if (tmp > 2147483648LL) {
                tmp -= 4294967296LL;
            }
            mlxData.mlZGyroTempData[i][j] = ((float)tmp) / 65536.0f;
            LOADCAL_LOG("mlXGyroTempData[%d][%d] = %f\n",
                            i, j, mlxData.mlXGyroTempData[i][j]);
        }
    }

    /* read the accel biases */
    for (i = 0; i < 3; i++) {
        uint32_t t = 0;
        t += 16777216UL * ((uint32_t)calData[ptr++]);
        t += 65536UL * ((uint32_t)calData[ptr++]);
        t += 256u * ((uint32_t)calData[ptr++]);
        t += (uint32_t)calData[ptr++];
        bias[i] = (int32_t)t;
        LOADCAL_LOG("accel_bias[%d] = %ld\n", i, bias[i]);
    }
    ERROR_CHECK(MLSetArray(ML_ACCEL_BIAS, bias));

    /* read the compass biases */
    mlxData.mlGotCompassBias = (int)calData[ptr++];
    mlxData.mlGotInitCompassBias = (int)calData[ptr++];
    mlxData.mlCompassState = (int)calData[ptr++];
    
    for (i = 0; i < 3; i++) {
        uint32_t t = 0;
        t += 16777216UL * ((uint32_t)calData[ptr++]);
        t += 65536UL * ((uint32_t)calData[ptr++]);
        t += 256u * ((uint32_t)calData[ptr++]);
        t += (uint32_t)calData[ptr++];
        mlxData.mlMagBiasError[i] = (int32_t)t;
        LOADCAL_LOG("mlMagBiasError[%d] = %ld\n", i, mlxData.mlMagBiasError[i]);
    }
    for (i = 0; i < 3; i++) {
        uint32_t t = 0;
        t += 16777216UL * ((uint32_t)calData[ptr++]);
        t += 65536UL * ((uint32_t)calData[ptr++]);
        t += 256u * ((uint32_t)calData[ptr++]);
        t += (uint32_t)calData[ptr++];
        mlxData.mlInitMagBias[i] = (int32_t)t;
        LOADCAL_LOG("mlInitMagBias[%d] = %ld\n", i, mlxData.mlInitMagBias[i]);
    }
    for (i = 0; i < 3; i++) {
        uint32_t t = 0;
        t += 16777216UL * ((uint32_t)calData[ptr++]);
        t += 65536UL * ((uint32_t)calData[ptr++]);
        t += 256u * ((uint32_t)calData[ptr++]);
        t += (uint32_t)calData[ptr++];
        mlxData.mlMagBias[i] = (int32_t)t;
        LOADCAL_LOG("mlMagBias[%d] = %ld\n", i, mlxData.mlMagBias[i]);
    }
    for (i = 0; i < 18; i++) {
        uint32_t t = 0;
        t += 16777216UL * ((uint32_t)calData[ptr++]);
        t += 65536UL * ((uint32_t)calData[ptr++]);
        t += 256u * ((uint32_t)calData[ptr++]);
        t += (uint32_t)calData[ptr++];
        mlxData.mlMagPeaks[i] = (int32_t)t;
        LOADCAL_LOG("mlMagPeaks[%d] = %d\n", i, mlxData.mlMagPeaks[i]);
    }
    for (i = 0; i < 3; i++) {
        dToLL.ll = 0;
        dToLL.ll += 
            72057594037927936ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 281474976710656ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 1099511627776ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 4294967296LL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 16777216ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 65536ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 256LL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += (unsigned long long)calData[ptr++];

        mlxData.mlMagBiasV[i] = dToLL.db;
        LOADCAL_LOG("mlMagBiasV[%d] = %lf\n", i, mlxData.mlMagBiasV[i]);
    }    
    for (i = 0; i < 9; i++) {
        dToLL.ll = 0;
        dToLL.ll += 
            72057594037927936ULL *  ((unsigned long long)calData[ptr++]);
        dToLL.ll += 281474976710656ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 1099511627776ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 4294967296LL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 16777216ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 65536ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 256LL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += (unsigned long long)calData[ptr++];

        mlxData.mlMagBiasP[i] = dToLL.db;
        LOADCAL_LOG("mlMagBiasP[%d] = %lf\n", i, mlxData.mlMagBiasP[i]);
    }        

    mlxData.mlGotNoMotionBias = TRUE;
    LOADCAL_LOG("mlGotNoMotionBias = 1\n");
    mlxData.calLoadedFlag = TRUE;
    LOADCAL_LOG("calLoadedFlag = 1\n");

    LOADCAL_LOG("Exiting MLLoadCal_V3\n");
    return ML_SUCCESS;
}

/**
 *  @brief  Loads a type 4 set of calibration data. 
 *          It parses a binary data set containing calibration data. 
 *          The binary data set is intended to be loaded from a file.
 *          This calibrations data format stores values for (in order of 
 *          appearance) :
 *          - temperature compensation : temperature data points,
 *          - temperature compensation : gyro biases data points for X, Y, 
 *              and Z axes.
 *          - accel biases for X, Y, Z axes.
 *          - compass biases for X, Y, Z axes, compass scale, and bias 
 *              tracking algorithm  mock-up.
 *          This calibration data is produced internally by the MPL and its 
 *          size is 2777 bytes (header and checksum included).
 *          Calibration format type 4 is currently used and 
 *          substitutes type 2 (MLLoadCal_V2()) and 3 (MLLoadCal_V3()).

 *  @pre    One of the MLDmpOpen() must be called: MLDmpDefaultOpen() or
 *          MLDmpPedometerStandAloneOpen().
 *
 *  @param  calData
 *              A pointer to an array of bytes to be parsed.
 *  @param  len
 *              the length of the calibration 
 *
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError MLLoadCal_V4(unsigned char *calData, unsigned short len)
{
    INVENSENSE_FUNC_START;
    union doubleToLongLong {
        double db;
        unsigned long long ll;
    } dToLL;

    const unsigned int expLen = 2777;
    long bias[3];
    int ptr = ML_CAL_HDR_LEN;
    int i, j;
    long long tmp;

    LOADCAL_LOG("Entering MLLoadCal_V4\n");

    if (len != expLen) {
        MPL_LOGE("Calibration data type 4 must be %d bytes long (got %d)\n", expLen, len);
        return ML_ERROR_FILE_READ;
    }

    for (i = 0; i < BINS; i++) {
        mlxData.mlTempPtrs[i] = 0;
        mlxData.mlTempPtrs[i] += 16777216L * ((long)calData[ptr++]);
        mlxData.mlTempPtrs[i] += 65536L * ((long)calData[ptr++]);
        mlxData.mlTempPtrs[i] += 256 * ((int)calData[ptr++]);
        mlxData.mlTempPtrs[i] += (int)calData[ptr++];
        LOADCAL_LOG("mlTempPtrs[%d] = %d\n", i ,mlxData.mlTempPtrs[i]);
    }
    for (i = 0; i < BINS; i++) {
        mlxData.mlTempValidData[i] = 0;
        mlxData.mlTempValidData[i] += 16777216L * ((long)calData[ptr++]);
        mlxData.mlTempValidData[i] += 65536L * ((long)calData[ptr++]);
        mlxData.mlTempValidData[i] += 256 * ((int)calData[ptr++]);
        mlxData.mlTempValidData[i] += (int)calData[ptr++];
        LOADCAL_LOG("mlTempValidData[%d] = %ld\n", 
                        i, mlxData.mlTempValidData[i]);
    }

    for (i = 0; i < BINS; i++) {
        for (j = 0; j < PTS_PER_BIN; j++) {
            tmp = 0;
            tmp += 16777216LL * (long long)calData[ptr++];
            tmp += 65536LL * (long long)calData[ptr++];
            tmp += 256LL * (long long)calData[ptr++];
            tmp += (long long)calData[ptr++];
            if (tmp > 2147483648LL) {
                tmp -= 4294967296LL;
            }
            mlxData.mlTempData[i][j] = ((float)tmp) / 65536.0f;
            LOADCAL_LOG("mlTempData[%d][%d] = %f\n", 
                            i, j, mlxData.mlTempData[i][j]);
        }        
    }

    for (i = 0; i < BINS; i++) {
        for (j = 0; j < PTS_PER_BIN; j++) {
            tmp = 0;
            tmp += 16777216LL * (long long)calData[ptr++];
            tmp += 65536LL * (long long)calData[ptr++];
            tmp += 256LL * (long long)calData[ptr++];
            tmp += (long long)calData[ptr++];
            if (tmp > 2147483648LL) {
                tmp -= 4294967296LL;
            }
            mlxData.mlXGyroTempData[i][j] = ((float)tmp) / 65536.0f;
            LOADCAL_LOG("mlXGyroTempData[%d][%d] = %f\n", 
                            i, j, mlxData.mlXGyroTempData[i][j]);
        }        
    }
    for (i = 0; i < BINS; i++) {
        for (j = 0; j < PTS_PER_BIN; j++) {
            tmp = 0;
            tmp += 16777216LL * (long long)calData[ptr++];
            tmp += 65536LL * (long long)calData[ptr++];
            tmp += 256LL * (long long)calData[ptr++];
            tmp += (long long)calData[ptr++];
            if (tmp > 2147483648LL) {
                tmp -= 4294967296LL;
            }
            mlxData.mlYGyroTempData[i][j] = ((float)tmp) / 65536.0f;
            LOADCAL_LOG("mlXGyroTempData[%d][%d] = %f\n", 
                            i, j, mlxData.mlXGyroTempData[i][j]);
        }        
    }
    for (i = 0; i < BINS; i++) {
        for (j = 0; j < PTS_PER_BIN; j++) {
            tmp = 0;
            tmp += 16777216LL * (long long)calData[ptr++];
            tmp += 65536LL * (long long)calData[ptr++];
            tmp += 256LL * (long long)calData[ptr++];
            tmp += (long long)calData[ptr++];
            if (tmp > 2147483648LL) {
                tmp -= 4294967296LL;
            }
            mlxData.mlZGyroTempData[i][j] = ((float)tmp) / 65536.0f;
            LOADCAL_LOG("mlXGyroTempData[%d][%d] = %f\n",
                            i, j, mlxData.mlXGyroTempData[i][j]);
        }
    }

    /* read the accel biases */
    for (i = 0; i < 3; i++) {
        uint32_t t = 0;
        t += 16777216UL * ((uint32_t)calData[ptr++]);
        t += 65536UL * ((uint32_t)calData[ptr++]);
        t += 256u * ((uint32_t)calData[ptr++]);
        t += (uint32_t)calData[ptr++];
        bias[i] = (int32_t)t;
        LOADCAL_LOG("accel_bias[%d] = %ld\n", 
                        i, bias[i]);
    }
    ERROR_CHECK(MLSetArray(ML_ACCEL_BIAS, bias));

    /* read the compass biases */
    mlxData.mlGotCompassBias = (int)calData[ptr++];
    LOADCAL_LOG("mlGotCompassBias = %ld\n", mlxData.mlGotCompassBias);
    mlxData.mlGotInitCompassBias = (int)calData[ptr++];
    LOADCAL_LOG("mlGotInitCompassBias = %d\n", mlxData.mlGotInitCompassBias);
    mlxData.mlCompassState = (int)calData[ptr++];
    LOADCAL_LOG("mlCompassState = %ld\n", mlxData.mlCompassState);
        
    for (i = 0; i < 3; i++) {
        uint32_t t = 0;
        t += 16777216UL * ((uint32_t)calData[ptr++]);
        t += 65536UL * ((uint32_t)calData[ptr++]);
        t += 256u * ((uint32_t)calData[ptr++]);
        t += (uint32_t)calData[ptr++];
        mlxData.mlMagBiasError[i] = (int32_t)t;
        LOADCAL_LOG("mlMagBiasError[%d] = %ld\n", i, mlxData.mlMagBiasError[i]);
    }
    for (i = 0; i < 3; i++) {
        uint32_t t = 0;
        t += 16777216UL * ((uint32_t)calData[ptr++]);
        t += 65536UL * ((uint32_t)calData[ptr++]);
        t += 256u * ((uint32_t)calData[ptr++]);
        t += (uint32_t)calData[ptr++];
        mlxData.mlInitMagBias[i] = (int32_t)t;
        LOADCAL_LOG("mlInitMagBias[%d] = %ld\n", i, mlxData.mlInitMagBias[i]);
    }
    for (i = 0; i < 3; i++) {
        uint32_t t = 0;
        t += 16777216UL * ((uint32_t)calData[ptr++]);
        t += 65536UL * ((uint32_t)calData[ptr++]);
        t += 256u * ((uint32_t)calData[ptr++]);
        t += (uint32_t)calData[ptr++];
        mlxData.mlMagBias[i] = (int32_t)t;
        LOADCAL_LOG("mlMagBias[%d] = %ld\n", i, mlxData.mlMagBias[i]);
    }
    for (i = 0; i < 18; i++) {
        uint32_t t = 0;
        t += 16777216UL * ((uint32_t)calData[ptr++]);
        t += 65536UL * ((uint32_t)calData[ptr++]);
        t += 256u * ((uint32_t)calData[ptr++]);
        t += (uint32_t)calData[ptr++];
        mlxData.mlMagPeaks[i] = (int32_t)t;
        LOADCAL_LOG("mlMagPeaks[%d] = %d\n", i, mlxData.mlMagPeaks[i]);
    }
    for (i = 0; i < 3; i++) {
        dToLL.ll = 72057594037927936ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 281474976710656ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 1099511627776ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 4294967296LL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 16777216ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 65536ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 256LL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += (unsigned long long)calData[ptr++];

        mlxData.mlMagBiasV[i] = dToLL.db;
        LOADCAL_LOG("mlMagBiasV[%d] = %lf\n", i, mlxData.mlMagBiasV[i]);
    }    
    for (i = 0; i < 9; i++) {
        dToLL.ll = 72057594037927936ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 281474976710656ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 1099511627776ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 4294967296LL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 16777216ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 65536ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 256LL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += (unsigned long long)calData[ptr++];

        mlxData.mlMagBiasP[i] = dToLL.db;
        LOADCAL_LOG("mlMagBiasP[%d] = %lf\n", i, mlxData.mlMagBiasP[i]);
    }        
    for (i = 0; i < 3; i++) {
        uint32_t t = 0;
        t += 16777216UL * ((uint32_t)calData[ptr++]);
        t += 65536UL * ((uint32_t)calData[ptr++]);
        t += 256u * ((uint32_t)calData[ptr++]);
        t += (uint32_t)calData[ptr++];
        mlxData.mlMagScale[i] = (int32_t)t;
        LOADCAL_LOG("mlMagScale[%d] = %d\n", i, (int32_t)t);
    }
    for (i = 0; i < 6; i++) {
        dToLL.ll = 72057594037927936ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 281474976710656ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 1099511627776ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 4294967296LL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 16777216ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 65536ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 256LL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += (unsigned long long)calData[ptr++];

        mlxData.mlMagPrevXTY[i] = dToLL.db;
        LOADCAL_LOG("mlMagPrevXTY[%d] = %f\n", i, dToLL.db);
    }    
    for (i = 0; i < 36; i++) {
        dToLL.ll = 72057594037927936ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 281474976710656ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 1099511627776ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 4294967296LL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 16777216ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 65536ULL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += 256LL * ((unsigned long long)calData[ptr++]);
        dToLL.ll += (unsigned long long)calData[ptr++];

        mlxData.mlMagPrevM[i] = dToLL.db;
        LOADCAL_LOG("mlMagPrevM[%d] = %f\n", i, dToLL.db);
    }

    mlxData.mlGotNoMotionBias = TRUE;
    LOADCAL_LOG("mlGotNoMotionBias = 1\n");
    mlxData.calLoadedFlag = TRUE;
    LOADCAL_LOG("calLoadedFlag = 1\n");

    LOADCAL_LOG("Exiting MLLoadCal_V4\n");
    return ML_SUCCESS;
}

/**
 *  @brief  Loads a type 5 set of calibration data. 
 *          It parses a binary data set containing calibration data. 
 *          The binary data set is intended to be loaded from a file.
 *          This calibrations data format stores values for (in order of 
 *          appearance) :
 *          - temperature,
 *          - gyro biases for X, Y, Z axes,
 *          - accel biases for X, Y, Z axes.
 *          This calibration data would normally be produced by the MPU Self 
 *          Test and its size is 36 bytes (header and checksum included).
 *          Calibration format type 5 is produced by the MPU Self Test and 
 *          substitutes the type 1 : MLLoadCal_V1().
 *
 *  @pre    One of the MLDmpOpen() must be called: MLDmpDefaultOpen() or
 *          MLDmpPedometerStandAloneOpen().
 *
 *  @param  calData
 *              A pointer to an array of bytes to be parsed.
 *  @param  len
 *              the length of the calibration 
 *
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError MLLoadCal_V5(unsigned char *calData, unsigned short len)
{
    INVENSENSE_FUNC_START;
    const short expLen = 36;
    long accelBias[3] = {0, 0, 0};
    float gyroBias[3] = {0, 0, 0};

    int ptr = ML_CAL_HDR_LEN;
    float newTemp;
    int bin;
    int i;

    LOADCAL_LOG("Entering MLLoadCal_V5\n");

    if (len != expLen) {
        MPL_LOGE("Calibration data type 5 must be %d bytes long (got %d)\n", expLen, len);
        return ML_ERROR_FILE_READ;
    }

    /* load the temperature */
    newTemp =  (float)((int32_t)calData[ptr++] * (1L << 8));
    newTemp += (float)calData[ptr++];
    if (newTemp > 32767) newTemp -= 65536L;
    newTemp = tempInDegC(newTemp);
    LOADCAL_LOG("newTemp = %f\n", newTemp);

    /* load the gyro biases (represented in 2 ^ 16 == 1 dps) */
    for (i = 0; i < 3; i++) {
        int32_t tmp = 0;
        tmp += (int32_t)calData[ptr++] * (1L << 24);
        tmp += (int32_t)calData[ptr++] * (1L << 16);
        tmp += (int32_t)calData[ptr++] * (1L << 8);
        tmp += (int32_t)calData[ptr++];
        gyroBias[i] = (float)tmp / 65536.0f;
        LOADCAL_LOG("gyroBias[%d] = %f\n", i, gyroBias[i]);
    }
    /* find the temperature bin */
    bin = FindTempBin(newTemp);
    
    /* populate the temp comp data structure */
    mlxData.mlTempData[bin][mlxData.mlTempPtrs[bin]] = newTemp;
    LOADCAL_LOG("mlTempData[%d][%d] = %f\n", 
                    bin, mlxData.mlTempPtrs[bin], newTemp);

    mlxData.mlXGyroTempData[bin][mlxData.mlTempPtrs[bin]] = gyroBias[0];
    LOADCAL_LOG("mlXGyroTempData[%d][%d] = %f\n",
                    bin, mlxData.mlTempPtrs[bin], gyroBias[0]);
    mlxData.mlYGyroTempData[bin][mlxData.mlTempPtrs[bin]] = gyroBias[1];
    LOADCAL_LOG("mlYGyroTempData[%d][%d] = %f\n", 
                    bin, mlxData.mlTempPtrs[bin], gyroBias[1]);
    mlxData.mlZGyroTempData[bin][mlxData.mlTempPtrs[bin]] = gyroBias[2];
    LOADCAL_LOG("mlXGyroTempData[%d][%d] = %f\n",
                    bin, mlxData.mlTempPtrs[bin], gyroBias[2]);
    mlxData.mlTempPtrs[bin] = (mlxData.mlTempPtrs[bin] + 1) % PTS_PER_BIN;
    LOADCAL_LOG("mlTempPtrs[%d] = %d\n", bin, mlxData.mlTempPtrs[bin]);

    if (mlxData.mlTempPtrs[bin] == 0) 
        mlxData.mlTempValidData[bin] = TRUE; 
    LOADCAL_LOG("mlTempValidData[%d] = %ld\n", 
                    bin, mlxData.mlTempValidData[bin]);

    /* load accel biases (represented in 2 ^ 16 == 1 gee) 
        and apply immediately */
    for (i = 0; i < 3; i++) {
        int32_t tmp = 0;
        tmp += (int32_t)calData[ptr++] * (1L << 24);
        tmp += (int32_t)calData[ptr++] * (1L << 16);
        tmp += (int32_t)calData[ptr++] * (1L << 8);
        tmp += (int32_t)calData[ptr++];
        accelBias[i] = (long)tmp;
        LOADCAL_LOG("accelBias[%d] = %ld\n", i, accelBias[i]);
    }
    ERROR_CHECK(MLSetArray(ML_ACCEL_BIAS, accelBias));

    mlxData.mlGotNoMotionBias = TRUE;
    LOADCAL_LOG("mlGotNoMotionBias = 1\n");
    mlxData.calLoadedFlag = TRUE;
    LOADCAL_LOG("calLoadedFlag = 1\n");

    LOADCAL_LOG("Exiting MLLoadCal_V5\n");
    return ML_SUCCESS;
}

/**
 * @brief   Loads a set of calibration data. 
 *          It parses a binary data set containing calibration data. 
 *          The binary data set is intended to be loaded from a file.
 *
 * @pre     MLDmpOpen() Must be called with MLDmpDefaultOpen() or
 *          MLDmpPedometerStandAloneOpen()
 *
 * @param   calData     
 *              A pointer to an array of bytes to be parsed.
 *
 * @return  ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError MLLoadCal(unsigned char *calData)
{
    INVENSENSE_FUNC_START;
    int calType = 0;
    int len = 0;
    int ptr;
    uint32_t chk = 0;
    uint32_t cmp_chk = 0;

    tMLLoadFunc loaders[] = {
        MLLoadCal_V0,
        MLLoadCal_V1,
        MLLoadCal_V2,
        MLLoadCal_V3,
        MLLoadCal_V4,
        MLLoadCal_V5
    };

    if (MLGetState() < ML_STATE_DMP_OPENED)
        return ML_ERROR_SM_IMPROPER_STATE;

    /* read the header (type and len)
       len is the total record length including header and checksum */
    len = 0;
    len += 16777216L * ((int)calData[0]);
    len += 65536L    * ((int)calData[1]);
    len += 256       * ((int)calData[2]);
    len +=              (int)calData[3];

    calType = ((int)calData[4]) * 256 + ((int)calData[5]);
    if (calType > 5) {
        MPL_LOGE("Unsupported calibration file format %d. "
                 "Valid types 0..5\n", calType);
        return ML_ERROR_INVALID_PARAMETER;
    }

    /* check the checksum */
    chk = 0;
    ptr = len - ML_CAL_CHK_LEN;

    chk += 16777216L * ((uint32_t)calData[ptr++]);
    chk += 65536L    * ((uint32_t)calData[ptr++]);
    chk += 256       * ((uint32_t)calData[ptr++]);
    chk +=              (uint32_t)calData[ptr++];

    cmp_chk = ml_checksum(calData + ML_CAL_HDR_LEN, 
        len - (ML_CAL_HDR_LEN + ML_CAL_CHK_LEN));
         
    if(chk != cmp_chk) {
        return ML_ERROR_CALIBRATION_CHECKSUM;
    }

    /* call the proper method to read in the data */
    return loaders[calType](calData, len);
}

/**
 *  @brief  Stores a set of calibration data. 
 *          It generates a binary data set containing calibration data. 
 *          The binary data set is intended to be stored into a file.
 *
 *  @pre    MLDmpOpen()
 *
 *  @param  calData
 *              A pointer to an array of bytes to be stored.
 *  @param  length
 *              The amount of bytes available in the array.
 *
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError MLStoreCal(unsigned char *calData, int length)
{
    INVENSENSE_FUNC_START;
    int ptr = 0;
    int i = 0;
    int j = 0;
    long long tmp;
    uint32_t chk;
    long bias[3];
    //unsigned char state;
    union doubleToLongLong {
        double db;
        unsigned long long ll;
    } dToLL;    

    if (MLGetState() < ML_STATE_DMP_OPENED)
        return ML_ERROR_SM_IMPROPER_STATE;

    STORECAL_LOG("Entering MLStoreCal\n");

    // length
    calData[0] = (unsigned char)((length>>24) & 0xff);
    calData[1] = (unsigned char)((length>>16) & 0xff);
    calData[2] = (unsigned char)((length>>8)  & 0xff);
    calData[3] = (unsigned char)( length      & 0xff);
    STORECAL_LOG("calLen = %d\n", length); 

    // calibration data format type
    calData[4] = 0;
    calData[5] = 4; 
    STORECAL_LOG("calType = %d\n", (int)calData[4] * 256 + calData[5]); 

    // data
    ptr = 6;    
    for (i = 0; i < BINS; i++) {
        tmp = (int)mlxData.mlTempPtrs[i];
        calData[ptr++] = (unsigned char)((tmp>>24) & 0xff);
        calData[ptr++] = (unsigned char)((tmp>>16) & 0xff);
        calData[ptr++] = (unsigned char)((tmp>>8)  & 0xff);
        calData[ptr++] = (unsigned char)( tmp      & 0xff);
        STORECAL_LOG("mlTempPtrs[%d] = %lld\n", i, tmp);
    }
    
    for (i = 0; i < BINS; i++) {
        tmp = (int)mlxData.mlTempValidData[i];
        calData[ptr++] = (unsigned char)((tmp>>24) & 0xff);
        calData[ptr++] = (unsigned char)((tmp>>16) & 0xff);
        calData[ptr++] = (unsigned char)((tmp>>8)  & 0xff);
        calData[ptr++] = (unsigned char)(tmp       & 0xff);
        STORECAL_LOG("mlTempValid[%d] = %lld\n", i, tmp);
    }
    for (i = 0; i < BINS; i++) {
        for (j = 0; j < PTS_PER_BIN; j++) {
            tmp = (long long)(mlxData.mlTempData[i][j]*65536.0f);
            if (tmp<0) {
                tmp += 4294967296LL;
            }            
            calData[ptr++] = (unsigned char)((tmp>>24) & 0xff);
            calData[ptr++] = (unsigned char)((tmp>>16) & 0xff);
            calData[ptr++] = (unsigned char)((tmp>>8)  & 0xff);
            calData[ptr++] = (unsigned char)( tmp      & 0xff);
            STORECAL_LOG("mlTempData[%d][%d] = %f\n",
                            i, j, mlxData.mlTempData[i][j]);
        }
    }
    
    for (i = 0; i < BINS; i++) {
        for (j = 0; j < PTS_PER_BIN; j++) {
            tmp = (long long)(mlxData.mlXGyroTempData[i][j]*65536.0f);
            if (tmp<0) {
                tmp += 4294967296LL;
            }            
            calData[ptr++] = (unsigned char)((tmp>>24) & 0xff);
            calData[ptr++] = (unsigned char)((tmp>>16) & 0xff);
            calData[ptr++] = (unsigned char)((tmp>>8)  & 0xff);
            calData[ptr++] = (unsigned char)( tmp      & 0xff);
            STORECAL_LOG("mlXGyroTempData[%d][%d] = %f\n",
                            i, j, mlxData.mlXGyroTempData[i][j]);
        }
    }
    for (i = 0; i < BINS; i++) {
        for (j = 0; j < PTS_PER_BIN; j++) {
            tmp = (long long)(mlxData.mlYGyroTempData[i][j]*65536.0f);
            if (tmp<0) {
                tmp += 4294967296LL;
            }            
            calData[ptr++] = (unsigned char)((tmp>>24) & 0xff);
            calData[ptr++] = (unsigned char)((tmp>>16) & 0xff);
            calData[ptr++] = (unsigned char)((tmp>>8)  & 0xff);
            calData[ptr++] = (unsigned char)( tmp      & 0xff);
            STORECAL_LOG("mlYGyroTempData[%d][%d] = %f\n",
                            i, j, mlxData.mlYGyroTempData[i][j]);
        }
    }
    for (i = 0; i < BINS; i++) {
        for (j = 0; j < PTS_PER_BIN; j++) {
            tmp = (long long)(mlxData.mlZGyroTempData[i][j]*65536.0f);
            if (tmp<0) {
                tmp += 4294967296LL;
            }            
            calData[ptr++] = (unsigned char)((tmp>>24) & 0xff);
            calData[ptr++] = (unsigned char)((tmp>>16) & 0xff);
            calData[ptr++] = (unsigned char)((tmp>>8)  & 0xff);
            calData[ptr++] = (unsigned char)( tmp      & 0xff);
            STORECAL_LOG("mlZGyroTempData[%d][%d] = %f\n",
                            i, j, mlxData.mlZGyroTempData[i][j]);
        }
    }

    MLGetArray(ML_ACCEL_BIAS, bias);
 
    /* write the accel biases */
    for (i = 0; i < 3; i++) {
        uint32_t t = (uint32_t)bias[i];
        calData[ptr++] = (unsigned char) ((t>>24) & 0xff);
        calData[ptr++] = (unsigned char) ((t>>16) & 0xff);
        calData[ptr++] = (unsigned char) ((t>>8)  & 0xff);
        calData[ptr++] = (unsigned char) ( t      & 0xff);
        STORECAL_LOG("accel_bias[%d] = %ld\n", i, bias[i]);
    }    

    /* write the compass calibration state */   
    calData[ptr++] = (unsigned char) (mlxData.mlGotCompassBias);
    STORECAL_LOG("mlGotCompassBias = %ld\n", mlxData.mlGotCompassBias);
    calData[ptr++] = (unsigned char) (mlxData.mlGotInitCompassBias);
    STORECAL_LOG("mlGotInitCompassBias = %d\n", mlxData.mlGotInitCompassBias);
    if (mlxData.mlCompassState==SF_UNCALIBRATED) {
        calData[ptr++] = SF_UNCALIBRATED;    
    } else {
        calData[ptr++] = SF_STARTUP_SETTLE;    
    }
    STORECAL_LOG("mlCompassState = %ld\n", mlxData.mlCompassState);

    for (i = 0; i < 3; i++) {
        uint32_t t = (uint32_t)mlxData.mlMagBiasError[i];
        calData[ptr++] = (unsigned char) ((t>>24) & 0xff);
        calData[ptr++] = (unsigned char) ((t>>16) & 0xff);
        calData[ptr++] = (unsigned char) ((t>>8)  & 0xff);
        calData[ptr++] = (unsigned char) (t & 0xff);
        STORECAL_LOG("mlMagBiasError[%d] = %ld\n", 
                        i, mlxData.mlMagBiasError[i]);
    }
    for (i = 0; i < 3; i++) {
        uint32_t t = (uint32_t)mlxData.mlInitMagBias[i];
        calData[ptr++] = (unsigned char) ((t>>24) & 0xff);
        calData[ptr++] = (unsigned char) ((t>>16) & 0xff);
        calData[ptr++] = (unsigned char) ((t>>8)  & 0xff);
        calData[ptr++] = (unsigned char) (t & 0xff);
        STORECAL_LOG("mlInitMagBias[%d] = %ld\n", i, mlxData.mlInitMagBias[i]);
    }
    for (i = 0; i < 3; i++) {
        uint32_t t = (uint32_t)mlxData.mlMagBias[i];
        calData[ptr++] = (unsigned char) ((t>>24) & 0xff);
        calData[ptr++] = (unsigned char) ((t>>16) & 0xff);
        calData[ptr++] = (unsigned char) ((t>>8)  & 0xff);
        calData[ptr++] = (unsigned char) (t & 0xff);
        STORECAL_LOG("mlMagBias[%d] = %ld\n", i, mlxData.mlMagBias[i]);
    }
    for (i = 0; i < 18; i++) {
        uint32_t t = (uint32_t)mlxData.mlMagPeaks[i];
        calData[ptr++] = (unsigned char) ((t>>24) & 0xff);
        calData[ptr++] = (unsigned char) ((t>>16) & 0xff);
        calData[ptr++] = (unsigned char) ((t>>8)  & 0xff);
        calData[ptr++] = (unsigned char) (t & 0xff);
        STORECAL_LOG("mlMagPeaks[%d] = %d\n", i, mlxData.mlMagPeaks[i]);
    }
    for (i = 0; i < 3; i++) {
        dToLL.db = mlxData.mlMagBiasV[i];        
        calData[ptr++] = (unsigned char) ((dToLL.ll>>56) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>48) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>40) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>32)  & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>24) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>16) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>8)  & 0xff);
        calData[ptr++] = (unsigned char) (dToLL.ll & 0xff);
        STORECAL_LOG("mlMagBiasV[%d] = %lf\n", i, mlxData.mlMagBiasV[i]);
    }
    for (i = 0; i < 9; i++) {
        dToLL.db = mlxData.mlMagBiasP[i];
        calData[ptr++] = (unsigned char) ((dToLL.ll>>56) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>48) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>40) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>32)  & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>24) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>16) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>8)  & 0xff);
        calData[ptr++] = (unsigned char) (dToLL.ll & 0xff);
        STORECAL_LOG("mlMagBiasP[%d] = %lf\n", i, mlxData.mlMagBiasP[i]);
    }
    for (i = 0; i < 3; i++) {        
        uint32_t t = (uint32_t)mlxData.mlMagScale[i];
        calData[ptr++] = (unsigned char) ((t>>24) & 0xff);
        calData[ptr++] = (unsigned char) ((t>>16) & 0xff);
        calData[ptr++] = (unsigned char) ((t>>8)  & 0xff);
        calData[ptr++] = (unsigned char) (t & 0xff);
        STORECAL_LOG("mlMagScale[%d] = %ld\n", i, mlxData.mlMagScale[i]);
    }
    for (i = 0; i < 6; i++) {
        dToLL.db = mlxData.mlMagPrevXTY[i];        
        calData[ptr++] = (unsigned char) ((dToLL.ll>>56) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>48) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>40) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>32)  & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>24) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>16) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>8)  & 0xff);
        calData[ptr++] = (unsigned char) (dToLL.ll & 0xff);
        STORECAL_LOG("mlMagPrevXTY[%d] = %lf\n", i, mlxData.mlMagPrevXTY[i]);
    }
    for (i = 0; i < 36; i++) {
        dToLL.db = mlxData.mlMagPrevM[i];
        calData[ptr++] = (unsigned char) ((dToLL.ll>>56) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>48) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>40) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>32)  & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>24) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>16) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>8)  & 0xff);
        calData[ptr++] = (unsigned char) (dToLL.ll & 0xff);
        STORECAL_LOG("mlMagPrevM[%d] = %lf\n", i, mlxData.mlMagPrevM[i]);
    }

    /* add a checksum */
    chk = ml_checksum(calData+ML_CAL_HDR_LEN, 
                      length - (ML_CAL_HDR_LEN+ML_CAL_CHK_LEN));
    calData[ptr++] = (unsigned char)((chk>>24) & 0xff);
    calData[ptr++] = (unsigned char)((chk>>16) & 0xff);
    calData[ptr++] = (unsigned char)((chk>>8)  & 0xff);
    calData[ptr++] = (unsigned char)( chk      & 0xff);

    STORECAL_LOG("Exiting MLStoreCal\n");
    return ML_SUCCESS;
}

/**
 *  @brief  Load a calibration file.
 *
 *  @pre    Must be in ML_STATE_DMP_OPENED state. 
 *          MLDmpOpen() or MLDmpStop() must have been called.
 *          MLDmpStart() and MLDmpClose() must have <b>NOT</b> 
 *          been called.
 *
 *  @return 0 or error code.
 */
tMLError MLLoadCalibration(void)
{
    unsigned char *calData;
    tMLError result;
    unsigned int length;
    
    if (MLGetState() < ML_STATE_DMP_OPENED)
        return ML_ERROR_SM_IMPROPER_STATE;
 
    result = MLSLGetCalLength(&length);
    if (result == ML_ERROR_FILE_OPEN) {
        MPL_LOGI("Calibration data not loaded\n");
        return ML_SUCCESS;
    }
    if (result || length <= 0) {
        MPL_LOGE("Could not get file calibration length - "
               "error %d - aborting\n", result);
        return result;
    }
    calData = (unsigned char*)MLOSMalloc(length);
    if (!calData) {
        MPL_LOGE("Could not allocate buffer of %d bytes - "
               "aborting\n", length);
        return ML_ERROR_MEMORY_EXAUSTED;
    }
    result = MLSLReadCal(calData, length);
    if (result) {
        MPL_LOGE("Could not read the calibration data from file - "
               "error %d - aborting\n", result);
        return result;
    }
    result = MLLoadCal(calData);
    if (result) {
        MPL_LOGE("Could not load the calibration data - "
               "error %d - aborting\n",  result);
        return result;
    }
    MLOSFree(calData);

    return ML_SUCCESS;
}

/**
 *  @brief  Store runtime calibration data to a file
 *
 *  @pre    Must be in ML_STATE_DMP_OPENED state. 
 *          MLDmpOpen() or MLDmpStop() must have been called.
 *          MLDmpStart() and MLDmpClose() must have <b>NOT</b> 
 *          been called.
 *
 *  @return 0 or error code.
 */
tMLError MLStoreCalibration(void)
{
    unsigned char *calData;
    tMLError result;
    unsigned int length;
    
    if (MLGetState() < ML_STATE_DMP_OPENED)
        return ML_ERROR_SM_IMPROPER_STATE;
 
    result = MLGetCalLength(&length);
    if (result || length <= 0) {
        MPL_LOGE("Could not get calibration data length - "
                 "error %d - aborting\n", result);
        return result;
    }
    calData = (unsigned char*)MLOSMalloc(length);
    if (!calData) {
        MPL_LOGE("Could not allocate buffer of %d bytes - "
               "aborting\n", length);
        return ML_ERROR_MEMORY_EXAUSTED;
    }
    result = MLStoreCal(calData, length);
    if (result) {
        MPL_LOGE("Could not store calibrated data on file - "
               "error %d - aborting\n", result);
        return result;
    }
    result = MLSLWriteCal(calData, length);
    if (result) {
        MPL_LOGE("Could not write calibration data - "
               "error %d\n", result);
        return result;
    }
    MLOSFree(calData);

    return ML_SUCCESS;
}

/**
 *  @}
 */

