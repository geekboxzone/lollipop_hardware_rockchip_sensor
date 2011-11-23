/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: mlFIFO.c 5144 2011-04-05 18:09:41Z mcaramello $
 *
 *******************************************************************************/

/**
 *   @defgroup MLFIFO
 *   @brief Motion Library - FIFO Driver.
 *          The FIFO API Interface.
 *
 *   @{
 *       @file mlFIFO.c
 *       @brief FIFO Interface.
**/
 
#include <string.h>
#include "mlFIFO.h"
#include "mlFIFOHW.h"
#include "dmpKey.h"
#include "mlMathFunc.h"
#include "ml.h"
#include "mldl.h"
#include "mldl_cfg.h"
#include "mlstates.h"
#include "mlsupervisor.h"
#include "mlos.h"
#include "mlmath.h"
#include "accel.h"

#include "log.h"
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-fifo"

#define FIFO_DEBUG 0

#define REF_QUATERNION             (0)
#define REF_GYROS                  (REF_QUATERNION + 4)
#define REF_CONTROL                (REF_GYROS + 3)
#define REF_RAW                    (REF_CONTROL + 4)
#define REF_RAW_EXTERNAL           (REF_RAW + 8)
#define REF_ACCEL                  (REF_RAW_EXTERNAL + 3)
#define REF_QUANT_ACCEL            (REF_ACCEL + 3)
#define REF_QUATERNION_6AXIS       (REF_QUANT_ACCEL + ML_MAX_NUM_ACCEL_SAMPLES)
#define REF_EIS                    (REF_QUATERNION_6AXIS + 4)
#define REF_DMP_PACKET             (REF_EIS + 3)
#define REF_GARBAGE                (REF_DMP_PACKET + 1)
#define REF_LAST                   (REF_GARBAGE + 1)

long FIFOScale[REF_LAST] = { 
    (1L<<30),(1L<<30),(1L<<30),(1L<<30), // Quaternion
    // 2^(16+30)/((2^30)*((3.14159265358/180)/200)/2)
    1501974482L, 1501974482L, 1501974482L, // Gyro
    (1L<<30),(1L<<30),(1L<<30),(1L<<30), // Control
    (1L<<14) * 234, // Temperature
    (1L<<14),(1L<<14),(1L<<14), // Raw Gyro
    (1L<<14),(1L<<14),(1L<<14), (0), // Raw Accel, plus padding
    (1L<<14),(1L<<14),(1L<<14), // Raw External
    (1L<<16),(1L<<16),(1L<<16), // Accel
    (1L<<30),(1L<<30),(1L<<30),(1L<<30), // Quant Accel
    (1L<<30),(1L<<30),(1L<<30),(1L<<30),  //Quant Accel
    (1L<<30),(1L<<30),(1L<<30),(1L<<30),  // Quaternion 6 Axis
    (1L<<30),(1L<<30),(1L<<30),  // EIS
    (1L<<30),  // Packet
    (1L<<30),  // Garbage
};
// The scale factors for tap need to match the number in FIFOScale above.
// FIFOOffsetBase below may also need to be changed if this is not 8
#if ML_MAX_NUM_ACCEL_SAMPLES != 8
#error  ML_MAX_NUM_ACCEL_SAMPLES must be defined to 8
#endif

#define CONFIG_QUAT                (0)
#define CONFIG_GYROS               (CONFIG_QUAT + 1)
#define CONFIG_CONTROL_DATA        (CONFIG_GYROS + 1)
#define CONFIG_TEMPERATURE         (CONFIG_CONTROL_DATA + 1)
#define CONFIG_RAW_DATA            (CONFIG_TEMPERATURE + 1)
#define CONFIG_RAW_EXTERNAL        (CONFIG_RAW_DATA + 1)
#define CONFIG_ACCEL               (CONFIG_RAW_EXTERNAL + 1)
#define CONFIG_DMP_QUANT_ACCEL     (CONFIG_ACCEL + 1)
#define CONFIG_EIS                 (CONFIG_DMP_QUANT_ACCEL + 1)
#define CONFIG_DMP_PACKET_NUMBER   (CONFIG_EIS + 1)
#define CONFIG_FOOTER              (CONFIG_DMP_PACKET_NUMBER + 1)
#define NUMFIFOELEMENTS            (CONFIG_FOOTER + 1)


const int FIFOOffsetBase[NUMFIFOELEMENTS]={ 
    REF_QUATERNION * 4,
    REF_GYROS * 4,
    REF_CONTROL * 4,
    REF_RAW * 4,
    REF_RAW * 4 + 4,
    REF_RAW_EXTERNAL * 4,
    REF_ACCEL * 4,
    REF_QUANT_ACCEL * 4,
    REF_EIS * 4,
    REF_DMP_PACKET * 4,
    REF_GARBAGE * 4 };

typedef struct _tFIFOData {
    void (*FIFOProcessCB)(void);
    long decoded[REF_LAST];
    long decodedAccel[ML_MAX_NUM_ACCEL_SAMPLES][ACCEL_NUM_AXES];
    int offsets[REF_LAST*4];
    int cache;
    uint_fast8_t gyroSource;
    unsigned short mlFIFORate;
    unsigned short sampleStepSizeMs;
    uint_fast16_t mlFIFOPacketSize;
    uint_fast16_t mlFIFODataConfig[NUMFIFOELEMENTS];
    unsigned char ReferenceCount[REF_LAST];
    long accBiasFilt[3];
    float accFilterCoef;
    long gravityCache[3];
} tFIFOData;

static tFIFOData FIFOData;

#define FIFO_CACHE_TEMPERATURE 1
#define FIFO_CACHE_GYRO 2
#define FIFO_CACHE_GRAVITY_BODY 4
#define FIFO_CACHE_ACC_BIAS 8

typedef struct {
    // These describe callbacks happening everytime a FIFO block is processed
    int_fast8_t numHighrateProcesses;
    HANDLE mutex;
    tMlxdataFunction highrateProcess[MAX_HIGH_RATE_PROCESSES];
} tMLXCallback;     // MLX_callback_t

tMLXCallback mlxCallback;

/** Sets accuracy to be one of 0, ML_32_BIT, or ML_16_BIT. Looks up old 
 *  accuracy if needed.
 */
static uint_fast16_t FIFOSetAccuracy(uint_fast16_t elements, 
                                    uint_fast16_t accuracy, 
                                    uint_fast8_t configOffset)
{
    if (elements) {
        if (!accuracy)
            accuracy = FIFOData.mlFIFODataConfig[configOffset];
        else if (accuracy & ML_16_BIT) 
            if ((FIFOData.mlFIFODataConfig[configOffset] & ML_32_BIT))
                accuracy = ML_32_BIT; // 32-bits takes priority
            else
                accuracy = ML_16_BIT;
        else
            accuracy = ML_32_BIT;
    } else {
        accuracy = 0;
    }

    return accuracy;
}

/** Adjusts (len) Reference Counts, at offset (refOffset). If increment is 0, 
 * the reference counts are subtracted, otherwise they are incremented for each
 * bit set in element. The value returned are the elements that should be sent
 * out as data through the FIFO.
*/
static uint_fast16_t FIFOSetReference(uint_fast16_t elements,
                                     uint_fast16_t increment,
                                     uint_fast8_t refOffset,
                                     uint_fast8_t len)
{
    uint_fast8_t kk;

    if (increment == 0) {
        for (kk = 0; kk < len; ++kk) {
            if ((elements & 1) && (FIFOData.ReferenceCount[kk+refOffset] > 0)) {
                FIFOData.ReferenceCount[kk+refOffset]--;
            }
            elements >>= 1;
        }
    } else {
        for (kk = 0; kk < len; ++kk) {
            if (elements & 1)
                FIFOData.ReferenceCount[kk+refOffset]++;
            elements >>= 1;
        }
    }
    elements = 0;
    for (kk = 0; kk < len; ++kk) {
        if (FIFOData.ReferenceCount[kk+refOffset] > 0)
            elements |= (1<<kk);
    }
    return elements;
}

/**
 * @param[in] accuracy ML_16_BIT or ML_32_BIT when constructing data to send
 *  out the FIFO, 0 when removing from the FIFO.
 */
static tMLError FIFOConstruct3(unsigned char *regs,
                               uint_fast16_t elements,
                               uint_fast16_t accuracy,
                               uint_fast8_t refOffset,
                               unsigned short key,
                               uint_fast8_t configOffset)
{
    int_fast8_t kk;
    tMLError result;

    elements = FIFOSetReference(elements, accuracy, refOffset, 3);
    accuracy = FIFOSetAccuracy(elements, accuracy, configOffset);

    if (accuracy & ML_16_BIT) {
        regs[0] = DINAF8 + 2;
    }

    FIFOData.mlFIFODataConfig[configOffset] = elements | accuracy;

    for (kk = 0; kk < 3; ++kk) {
        if ((elements & 1) == 0)
            regs[kk + 1] = DINAA0+3;
        elements >>= 1;
    }

    result = MLDLSetMemoryMPU(key, 4, regs);

    return result;
}

/** 
 * @internal
 * Puts footer on FIFO data.
 */
static tMLError SetFIFOFooter()
{
    unsigned char regs = DINA30;
    uint_fast8_t tmpCount;
    int_fast8_t i,j;
    int offset;
    int result;
    int *FIFOOffsetsPtr = FIFOData.offsets;

    FIFOData.mlFIFOPacketSize = 0;
    for (i = 0; i < NUMFIFOELEMENTS; i++) {
        tmpCount = 0;
        offset = FIFOOffsetBase[i];
        for (j=0; j<8; j++) {
            if ((FIFOData.mlFIFODataConfig[i] >> j) & 0x0001) {
#ifndef BIG_ENDIAN
                // Special Case for Byte Ordering on Accel Data
                if ((i == CONFIG_RAW_DATA) && (j > 2)) {
                    tmpCount += 2;
                    switch (MLDLGetCfg()->accel->endian) {
                    case EXT_SLAVE_BIG_ENDIAN:
                        *FIFOOffsetsPtr++ = offset+3;
                        *FIFOOffsetsPtr++ = offset+2;
                        break;
                    case EXT_SLAVE_LITTLE_ENDIAN:
                        *FIFOOffsetsPtr++ = offset+2;
                        *FIFOOffsetsPtr++ = offset+3;
                        break;
                    case EXT_SLAVE_FS8_BIG_ENDIAN:
                        if (j == 3) {
                            // Throw this byte away
                            *FIFOOffsetsPtr++ = FIFOOffsetBase[CONFIG_FOOTER];
                            *FIFOOffsetsPtr++ = offset+3;
                        } else if (j == 4) {
                            *FIFOOffsetsPtr++ = offset+3;
                            *FIFOOffsetsPtr++ = offset+7;
                        } else {
                            // Throw these byte away
                            *FIFOOffsetsPtr++ = FIFOOffsetBase[CONFIG_FOOTER];
                            *FIFOOffsetsPtr++ = FIFOOffsetBase[CONFIG_FOOTER];
                        }
                        break;
                    case EXT_SLAVE_FS16_BIG_ENDIAN:
                        if (j == 3) {
                            // Throw this byte away
                            *FIFOOffsetsPtr++ = FIFOOffsetBase[CONFIG_FOOTER];
                            *FIFOOffsetsPtr++ = offset+3;
                        } else if (j == 4) {
                            *FIFOOffsetsPtr++ = offset-2;
                            *FIFOOffsetsPtr++ = offset+3;
                        } else {
                            *FIFOOffsetsPtr++ = offset-2;
                            *FIFOOffsetsPtr++ = offset+3;
                        }
                        break;
                    default:
                        return ML_ERROR; // Bad value on ordering
                    }
                } else {
                    tmpCount += 2;
                    *FIFOOffsetsPtr++ = offset+3;
                    *FIFOOffsetsPtr++ = offset+2;
                    if (FIFOData.mlFIFODataConfig[i] & ML_32_BIT) {
                        *FIFOOffsetsPtr++ = offset+1;
                        *FIFOOffsetsPtr++ = offset;
                        tmpCount += 2;
                    }
                }
#else
                // Big Endian Platform
                // Special Case for Byte Ordering on Accel Data
                if ((i == CONFIG_RAW_DATA) && (j > 2)) {
                    tmpCount += 2;
                    switch (MLDLGetCfg()->accel->endian) {
                    case EXT_SLAVE_BIG_ENDIAN:
                        *FIFOOffsetsPtr++ = offset+2;
                        *FIFOOffsetsPtr++ = offset+3;
                        break;
                    case EXT_SLAVE_LITTLE_ENDIAN:
                        *FIFOOffsetsPtr++ = offset+3;
                        *FIFOOffsetsPtr++ = offset+2;
                        break;
                    case EXT_SLAVE_FS8_BIG_ENDIAN:
                        if (j == 3) {
                            // Throw this byte away
                            *FIFOOffsetsPtr++ = FIFOOffsetBase[CONFIG_FOOTER];
                            *FIFOOffsetsPtr++ = offset;
                        } else if (j == 4) {
                            *FIFOOffsetsPtr++ = offset;
                            *FIFOOffsetsPtr++ = offset+4;
                        } else {
                            // Throw these bytes away
                            *FIFOOffsetsPtr++ = FIFOOffsetBase[CONFIG_FOOTER];
                            *FIFOOffsetsPtr++ = FIFOOffsetBase[CONFIG_FOOTER];
                        }
                        break;
                    case EXT_SLAVE_FS16_BIG_ENDIAN:
                        if (j == 3) {
                            // Throw this byte away
                            *FIFOOffsetsPtr++ = FIFOOffsetBase[CONFIG_FOOTER];
                            *FIFOOffsetsPtr++ = offset;
                        } else if (j == 4) {
                            *FIFOOffsetsPtr++ = offset-3;
                            *FIFOOffsetsPtr++ = offset;
                        } else {
                            *FIFOOffsetsPtr++ = offset-3;
                            *FIFOOffsetsPtr++ = offset;
                        }
                        break;
                    default:
                        return ML_ERROR; // Bad value on ordering
                    }
                } else {
                    tmpCount += 2;
                    *FIFOOffsetsPtr++ = offset;
                    *FIFOOffsetsPtr++ = offset+1;
                    if (FIFOData.mlFIFODataConfig[i] & ML_32_BIT) {
                        *FIFOOffsetsPtr++ = offset+2;
                        *FIFOOffsetsPtr++ = offset+3;
                        tmpCount += 2;
                    }
                }
                
#endif
            }
            offset += 4;
        }
        FIFOData.mlFIFOPacketSize += tmpCount;
    }
    if (FIFOData.mlFIFODataConfig[CONFIG_FOOTER] == 0 && 
        FIFOData.mlFIFOPacketSize > 0) {
        // Add footer
        result = MLDLSetMemoryMPU(KEY_CFG_16, 1, &regs);
        ERROR_CHECK(result);
        FIFOData.mlFIFODataConfig[CONFIG_FOOTER] = 0x0001 | ML_16_BIT;
        FIFOData.mlFIFOPacketSize += 2;
    } else if (FIFOData.mlFIFODataConfig[CONFIG_FOOTER] && 
               (FIFOData.mlFIFOPacketSize == 2)) {
        // Remove Footer
        regs = DINAA0+3;
        result = MLDLSetMemoryMPU(KEY_CFG_16, 1, &regs);
        ERROR_CHECK(result);
        FIFOData.mlFIFODataConfig[CONFIG_FOOTER] = 0;
        FIFOData.mlFIFOPacketSize = 0;
    }

    return ML_SUCCESS;
}

tMLError FIFODecodeQuantAccel(void)
{
    int kk;
    int fifoRate = MLGetFIFORate();

    if (!FIFOData.mlFIFODataConfig[CONFIG_DMP_QUANT_ACCEL])
        return ML_ERROR_FEATURE_NOT_ENABLED;

    for (kk = (ML_MAX_NUM_ACCEL_SAMPLES - (fifoRate + 1));
         kk < ML_MAX_NUM_ACCEL_SAMPLES;
         kk++) {
        union {
            unsigned int u10 : 10;
            signed   int s10 : 10;
        } temp;
        
        union {
            uint32_t u32;
            int32_t s32;
        } value;

        value.u32 = FIFOData.decoded[REF_QUANT_ACCEL + kk];
        // unquantize this samples.  
        // They are stored as x * 2^20 + y * 2^10 + z
        // Z
        temp.u10 = value.u32 & 0x3ff;
        value.s32 -= temp.s10;
        FIFOData.decodedAccel[kk][2] = temp.s10 * 64;
        // Y
        value.s32 = value.s32 / 1024;
        temp.u10 =value.u32 & 0x3ff;
        value.s32 -= temp.s10;
        FIFOData.decodedAccel[kk][1] = temp.s10 * 64;
        // X
        value.s32 = value.s32 / 1024;
        temp.u10 = value.u32 & 0x3ff;
        FIFOData.decodedAccel[kk][0] = temp.s10 * 64;
    }
    return ML_SUCCESS;
}

static tMLError FIFOStateCallback(unsigned char newState)
{
    tMLError result = ML_SUCCESS;
    unsigned char regs[4];
    struct mldl_cfg *mldl_cfg = MLDLGetCfg();

    /* Don't reset the fifo on a fifo rate change */
    if ((mldl_cfg->requested_sensors & ML_DMP_PROCESSOR) &&
        (newState != MLGetState()) &&
        (DmpFeatureSupported(KEY_D_1_178))) {
        /* Delay output on restart by 50ms due to warm up time of gyros */

        short delay = (short) -((50 / GetSampleStepSizeMs()) + 1);
        FIFOHWInit();
        Short16ToBig8(delay, regs);
        result = MLDLSetMemoryMPU(KEY_D_1_178, 2, regs);
        ERROR_CHECK(result);
    }

    if (ML_STATE_DMP_STARTED == newState) {
        if (DmpFeatureSupported(KEY_D_1_128)) {
            double tmp;
            tmp = (0x20000000L * M_PI) / (FIFOData.mlFIFORate + 1);
            if (tmp > 0x40000000L)
                tmp = 0x40000000L;
            (void)Long32ToBig8((long)tmp, regs);
            result = MLDLSetMemoryMPU(KEY_D_1_128, sizeof(long), regs);
            ERROR_CHECK(result);
            result = FIFOReset();
            ERROR_CHECK(result);
        } 
    }
    return result;
}

/**
 * @internal
 * @brief get the FIFO packet size
 * @return the FIFO packet size
 */
uint_fast16_t FIFOGetPacketSize(void) {
    return FIFOData.mlFIFOPacketSize;
}

/**
 *  @brief  Initializes all the internal static variables for 
 *          the FIFO module.
 *  @note   Should be called by the initialization routine such 
 *          as MLDmpOpen().
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise. 
 */
tMLError FIFOParamInit(void)
{
    tMLError result;
    memset(&FIFOData, 0, sizeof(tFIFOData));
    FIFOData.decoded[REF_QUATERNION] = 1073741824L; // Set to Identity
    FIFOSetLinearAccelFilterCoef( 0.f );
    FIFOData.mlFIFORate = 20;
    FIFOData.sampleStepSizeMs = 100;
    memset(&mlxCallback, 0, sizeof(tMLXCallback));
    result = MLOSCreateMutex(&mlxCallback.mutex);
    ERROR_CHECK(result);
    result = MLStateRegisterCallback(FIFOStateCallback);
    ERROR_CHECK(result);
    return result;
}

/**
 *  @brief  Close the FIFO usage.
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError FIFOClose(void) 
{
    tMLError result;
    tMLError first = ML_SUCCESS;
    result = MLStateUnRegisterCallback(FIFOStateCallback);
    ERROR_CHECK_FIRST(first, result);
    result = MLOSDestroyMutex(mlxCallback.mutex);
    ERROR_CHECK_FIRST(first, result);
    memset(&mlxCallback, 0, sizeof(tMLXCallback));
    return first;
}

/** 
 * Set the gyro source to output to the fifo
 * 
 * @param source The source.  One of 
 * - ML_GYRO_FROM_RAW
 * - ML_GYRO_FROM_QUATERNION
 * 
 * @return ML_SUCCESS or non-zero error code;
 */
tMLError FIFOSetGyroDataSource(uint_fast8_t source)
{
    if (source != ML_GYRO_FROM_QUATERNION && source != ML_GYRO_FROM_RAW) {
        return ML_ERROR_INVALID_PARAMETER;
    }

    FIFOData.gyroSource = source;
    return ML_SUCCESS;
}

/** 
 *  @brief  Reads and processes FIFO data. Also handles callbacks when data is
 *          ready.
 *  @param  numPackets 
 *              Number of FIFO packets to try to read. You should
 *              use a large number here, such as 100, if you want to read all
 *              the full packets in the FIFO, which is typical operation.
 *  @param  processed
 *              The number of FIFO packets processed. This may be incremented
 *              even if high rate processes later fail.
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError readAndProcessFIFO(int_fast8_t numPackets, int_fast8_t *processed)
{
    int_fast8_t packet;
    tMLError result = ML_SUCCESS;
    uint_fast16_t read;
    struct mldl_cfg *mldl_cfg = MLDLGetCfg();
    int kk;

    if (NULL == processed) 
        return ML_ERROR_INVALID_PARAMETER;

    *processed = 0;
    if (FIFOData.mlFIFOPacketSize == 0)
        return result; // Nothing to read

    for (packet = 0; packet < numPackets; ++packet) {
        if (mldl_cfg->requested_sensors & ML_DMP_PROCESSOR) {
            unsigned char footer_n_data[MAX_FIFO_LENGTH + FIFO_FOOTER_SIZE];
            unsigned char *buf = &footer_n_data[FIFO_FOOTER_SIZE];
            read = MPUGetFIFO((uint_fast16_t)FIFOData.mlFIFOPacketSize, 
                              footer_n_data);
            if (0 == read || 
                read != FIFOData.mlFIFOPacketSize-FIFO_FOOTER_SIZE) {
                result = MLDLGetFIFOStatus();
                if (ML_SUCCESS != result) {
                    memset(FIFOData.decoded, 0, sizeof(FIFOData.decoded));
                }
                return result;
            }

            result = MLProcessFIFOData(buf);
            ERROR_CHECK(result);
        } else if (AccelGetPresent()) {
            long data[ACCEL_NUM_AXES];
            result = AccelGetData(data);
            if (result == ML_ERROR_ACCEL_DATA_NOT_READY) {
                return ML_SUCCESS;
            }
            ERROR_CHECK(result);

            memset(FIFOData.decoded, 0, sizeof(FIFOData.decoded));
            FIFOData.cache = 0;
            for (kk = 0; kk < ACCEL_NUM_AXES; ++kk) {
                FIFOData.decoded[REF_RAW + 4 + kk] = 
                    q30_mult((data[kk] << 16), FIFOScale[REF_RAW + 4 + kk]);
                FIFOData.decoded[REF_ACCEL + kk] = 
                    q30_mult((data[kk] << 15), FIFOScale[REF_ACCEL + kk]);
            }
        }

        // The buffer was processed correctly, so increment even if
        // other processes fail later, which will return an error
        *processed = *processed + 1;

        if ((FIFOData.mlFIFORate < ML_MAX_NUM_ACCEL_SAMPLES) &&
            FIFOData.mlFIFODataConfig[CONFIG_DMP_QUANT_ACCEL]) {
            result = FIFODecodeQuantAccel();
            ERROR_CHECK(result);
        }

        if (FIFOData.mlFIFODataConfig[CONFIG_QUAT]) {
            result = MLAccelCompassSupervisor();
            ERROR_CHECK(result);
        }

        result = MLPressureSupervisor();
        ERROR_CHECK(result);

        // Callbacks now that we have a buffer of data ready
        result = RunHighRateProcessFuncs();
        ERROR_CHECK(result);

    }
    return result;
}

/**
 *  @brief  MLSetProcessedDataCallback is used to set a processed data callback
 *          function.  MLSetProcessedDataCallback sets a user defined callback
 *          function that triggers when all the decoding has been finished by
 *          the motion processing engines. It is called before other bigger 
 *          processing engines to allow lower latency for the user.
 *
 *  @pre    MLDmpOpen() with MLDmpDefaultOpen() or 
 *          MLDmpPedometerStandAloneOpen() and MLDmpStart() 
 *          must <b>NOT</b> have been called.
 *
 *  @param  func    A user defined callback function.
 *
 *  @return ML_SUCCESS if successful, or non-zero error code otherwise.
 */
tMLError MLSetProcessedDataCallback(void (*func)(void))
{
    INVENSENSE_FUNC_START;

    if (MLGetState() < ML_STATE_DMP_OPENED)
        return ML_ERROR_SM_IMPROPER_STATE;    

    FIFOData.FIFOProcessCB = func;

    return ML_SUCCESS;
}

/**
 * @internal
 * @brief   Process data from the dmp read via the fifo.  Takes a buffer 
 *          filled with bytes read from the DMP FIFO. 
 *          Currently expects only the 16 bytes of quaternion data. 
 *          Calculates the motion parameters from that data and stores the 
 *          results in an internal data structure.
 * @param[in,out]   dmpData     Pointer to the buffer containing the fifo data.
 * @return  ML_SUCCESS or error code.
**/
tMLError MLProcessFIFOData(const unsigned char* dmpData)
{           
    INVENSENSE_FUNC_START;
    int N,kk;
    unsigned char *p;

    p = (unsigned char *)(&FIFOData.decoded);
    N = FIFOData.mlFIFOPacketSize;
    if (N > sizeof(FIFOData.decoded))
        return ML_ERROR_ASSERTION_FAILURE;

    memset(&FIFOData.decoded,0,sizeof(FIFOData.decoded));

    for (kk = 0; kk < N; ++kk) {
        p[ FIFOData.offsets[kk] ] = *dmpData++;
    }

    // If multiplies are much greater cost than if checks, you could check
    // to see if FIFOScale is non-zero first, or equal to (1L<<30)
    for (kk = 0; kk < REF_LAST; ++kk) {
        FIFOData.decoded[kk] = q30_mult(FIFOData.decoded[kk], FIFOScale[kk]);
    }

    memcpy(&FIFOData.decoded[REF_QUATERNION_6AXIS],
           &FIFOData.decoded[REF_QUATERNION],
           4*sizeof(long));

    mlxData.mlFlags[ML_PROCESSED_DATA_READY] = 1;
    FIFOData.cache = 0;

    return ML_SUCCESS;
}

/** 
 *  @internal
 *  @brief  Turns temperature register values into Temperature and 
 *          sets the temperature element in FIFOData.
 *  @param  reg
 *              the temperature reading.
 */
static void decodeTemperature(const unsigned char *reg)
{
    FIFOData.decoded[REF_RAW] = (((long)reg[0] << 8) + ((long)reg[1]));
    if (FIFOData.decoded[REF_RAW] > 32768L)
        FIFOData.decoded[REF_RAW] -= 65536L;
    
    FIFOData.decoded[REF_RAW] = q30_mult(FIFOData.decoded[REF_RAW] << 16,
                                         FIFOScale[REF_RAW]);
}

/** 
 *  @brief      Returns 1-element vector of temperature. It is read from the hardware if it
 *              doesn't exist in the FIFO.
 *  @param[out] data    1-element vector of temperature
 *  @return     0 on success or an error code.
 */
tMLError FIFOGetTemperature(long *data)
{

    if (data == NULL)
        return ML_ERROR_INVALID_PARAMETER;

    if (!FIFOData.mlFIFODataConfig[CONFIG_TEMPERATURE]) {
        tMLError result;
        unsigned char regs[2];
        if ((FIFOData.cache & FIFO_CACHE_TEMPERATURE) == 0) {
            if (FIFO_DEBUG)
                MPL_LOGI("Fetching the temperature from the registers\n");
            FIFOData.cache |= FIFO_CACHE_TEMPERATURE;
            result = MLSLSerialRead(
                MLSerialGetHandle(), MLDLGetMPUSlaveAddr(),
                MPUREG_TEMP_OUT_H, 2, regs);
            ERROR_CHECK(result);
            decodeTemperature(regs);
        }
    }
    data[0] = FIFOData.decoded[REF_RAW] + 2293760L + 13200L * 234;
    return ML_SUCCESS;
}

/**
 *  @brief  Get the Decoded Accel Data.
 *  @param  data
 *              a buffer to store the quantized data.
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError FIFOGetDecodedAccel(long *data)
{
    int ii, kk;
    if (data == NULL)
        return ML_ERROR_INVALID_PARAMETER;

    if (!FIFOData.mlFIFODataConfig[CONFIG_DMP_QUANT_ACCEL])
        return ML_ERROR_FEATURE_NOT_ENABLED;

    for (ii = 0; ii < ML_MAX_NUM_ACCEL_SAMPLES; ii++){
        for (kk = 0; kk < ACCEL_NUM_AXES; kk++){
            data[ii * ACCEL_NUM_AXES + kk] = FIFOData.decodedAccel[ii][kk];
        }
    }

    return ML_SUCCESS;
}

/**
 *  @brief  Get the Quantized Accel data algorithm output from the FIFO.
 *  @param  data
 *              a buffer to store the quantized data.
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError FIFOGetQuantAccel(long* data)
{
    int ii;
    if (data == NULL)
        return ML_ERROR_INVALID_PARAMETER;

    if (!FIFOData.mlFIFODataConfig[CONFIG_DMP_QUANT_ACCEL])
        return ML_ERROR_FEATURE_NOT_ENABLED;

    for (ii = 0; ii < ML_MAX_NUM_ACCEL_SAMPLES; ii++){
        data[ii] = FIFOData.decoded[REF_QUANT_ACCEL+ii];
    }

    return ML_SUCCESS;
}

/** This gets raw gyro data. The data is taken from the FIFO if it was put in the FIFO
*  and it is read from the registers if it was not put into the FIFO. The data is
*  cached till the next FIFO processing block time.
* @param[out] data Length 3, Gyro data
*/
tMLError FIFOGetSensorGyroData(long *data)
{
    if (data == NULL)
        return ML_ERROR_INVALID_PARAMETER;
    if ((FIFOData.mlFIFODataConfig[CONFIG_RAW_DATA] & 7) != 7) {
        tMLError result;
        unsigned char regs[6];
        if ( (FIFOData.cache & FIFO_CACHE_GYRO) == 0 ) {
            FIFOData.cache |= FIFO_CACHE_GYRO;
            result = MLSLSerialRead(MLSerialGetHandle(), MLDLGetMPUSlaveAddr(),
                                    MPUREG_GYRO_XOUT_H, 6, regs);
            ERROR_CHECK(result);
            FIFOData.decoded[REF_RAW+1] = (((long)regs[0])<<24) | (((long)regs[1])<<16);
            FIFOData.decoded[REF_RAW+2] = (((long)regs[2])<<24) | (((long)regs[3])<<16);
            FIFOData.decoded[REF_RAW+3] = (((long)regs[4])<<24) | (((long)regs[5])<<16);

            // Temperature starts at location 0, Gyro at location 1.
            FIFOData.decoded[REF_RAW+1] = q30_mult( FIFOData.decoded[REF_RAW+1],
                                                    FIFOScale[REF_RAW+1]);
            FIFOData.decoded[REF_RAW+2] = q30_mult( FIFOData.decoded[REF_RAW+2],
                                                    FIFOScale[REF_RAW+2]);
            FIFOData.decoded[REF_RAW+3] = q30_mult( FIFOData.decoded[REF_RAW+3],
                                                    FIFOScale[REF_RAW+3]);
        }
        data[0] = FIFOData.decoded[REF_RAW+1];
        data[1] = FIFOData.decoded[REF_RAW+2];
        data[2] = FIFOData.decoded[REF_RAW+3];
    } else {
        long data2[6];
        FIFOGetSensorData(data2);
        data[0] = data2[0];
        data[1] = data2[1];
        data[2] = data2[2];
    }
    return ML_SUCCESS;
}

/** 
 *  @brief      Returns 6-element vector of gyro and accel data
 *  @param[out] data    6-element vector of gyro and accel data
 *  @return     0 on success or an error code.
 */
tMLError FIFOGetSensorData(long *data)
{
    int ii;
    if (data == NULL)
        return ML_ERROR_INVALID_PARAMETER;

    if (!FIFOData.mlFIFODataConfig[CONFIG_RAW_DATA])
        return ML_ERROR_FEATURE_NOT_ENABLED;

    for (ii = 0; ii < (MPU_NUM_AXES + ACCEL_NUM_AXES); ii++){
        data[ii] = FIFOData.decoded[REF_RAW+1+ii];
    }

    return ML_SUCCESS;
}

/** 
 *  @brief      Returns 3-element vector of external sensor
 *  @param[out] data    3-element vector of external sensor
 *  @return     0 on success or an error code.
 */
tMLError FIFOGetExternalSensorData(long *data)
{
#ifdef M_HW
    int ii;
    if (data == NULL)
        return ML_ERROR_INVALID_PARAMETER;

    if (!FIFOData.mlFIFODataConfig[CONFIG_RAW_EXTERNAL])
        return ML_ERROR_FEATURE_NOT_ENABLED;

    for (ii = 0; ii < COMPASS_NUM_AXES; ii++){
        data[ii] = FIFOData.decoded[REF_RAW_EXTERNAL+ii];
    }

    return ML_SUCCESS;
#else
    memset(data, 0, COMPASS_NUM_AXES * sizeof(long));
    return ML_ERROR_FEATURE_NOT_IMPLEMENTED;
#endif
}

/** 
 *  Sends accelerometer data to the FIFO.
 *
 *  @param[in] elements Which of the 3 elements to send. Use ML_ALL for 3 axis
 *            or ML_ELEMENT_1, ML_ELEMENT_2, ML_ELEMENT_3 or'd together
 *            for a subset.
 *
 * @param[in] accuracy Set to ML_32_BIT for 32-bit data, or ML_16_BIT for 16
 *            bit data. Set to zero to remove it from the FIFO.
 */
tMLError FIFOSendAccel(uint_fast16_t elements, uint_fast16_t accuracy)
{
    INVENSENSE_FUNC_START;
    unsigned char regs[4] = { DINAF8 + 1, DINA28, DINA30, DINA38};
    tMLError result;
    int kk;

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;
 
    result = FIFOConstruct3( regs, elements, accuracy, REF_ACCEL,
                             KEY_CFG_12, CONFIG_ACCEL);
    ERROR_CHECK( result );

    for (kk = 0; kk < ACCEL_NUM_AXES; kk++) {
        FIFOScale[REF_ACCEL + kk] = 2 * mlxData.mlAccelSens;
    }

    return SetFIFOFooter();
}


/** 
 * Sends control data to the FIFO. Control data is a 4 length vector of 32-bits.
 *
 *  @param[in] elements Which of the 4 elements to send. Use ML_ALL for all
 *            or ML_ELEMENT_1, ML_ELEMENT_2, ML_ELEMENT_3, ML_ELEMENT_4 or'd
 *             together for a subset.
 *
 *  @param[in] accuracy Set to ML_32_BIT for 32-bit data, or ML_16_BIT for 16
 *             bit data. Set to zero to remove it from the FIFO.
 */
tMLError FIFOSendControlData(uint_fast16_t elements, uint_fast16_t accuracy)
{
    INVENSENSE_FUNC_START;
    int_fast8_t kk;
    tMLError result;
    unsigned char regs[5] = { DINAF8 + 1, DINA20, DINA28, DINA30, DINA38};

    if (MLGetState() < ML_STATE_DMP_OPENED)
        return ML_ERROR_SM_IMPROPER_STATE;

    elements = FIFOSetReference(elements, accuracy, REF_CONTROL, 4);
    accuracy = FIFOSetAccuracy(elements, accuracy, CONFIG_CONTROL_DATA);

    if (accuracy & ML_16_BIT) {
        regs[0] = DINAF8 + 2;
    }

    FIFOData.mlFIFODataConfig[CONFIG_CONTROL_DATA] = elements | accuracy;

    for (kk = 0; kk < 4; ++kk) {
        if ((elements & 1) == 0)
            regs[kk + 1] = DINAA0+3;
        elements >>= 1;
    }

    result = MLDLSetMemoryMPU(KEY_CFG_1, 5, regs);
    ERROR_CHECK(result);

    return SetFIFOFooter();
}


/** 
 * Adds a rolling counter to the fifo packet.  When used with the footer
 * the data comes out the first time:
 * 
 * @code
 * <data0><data1>...<dataN><PacketNum0><PacketNum1>
 * @endcode
 * for every other packet it is
 *
 * @code
 * <FifoFooter0><FifoFooter1><data0><data1>...<dataN><PacketNum0><PacketNum1>
 * @endcode
 *
 * This allows for scanning of the fifo for packets
 * 
 * @return ML_SUCCESS or error code
 */
tMLError FIFOSendDMPPacketNumber(uint_fast16_t accuracy)
{
    INVENSENSE_FUNC_START;
    tMLError result;
    unsigned char regs;
    uint_fast16_t elements;

    if (MLGetState() < ML_STATE_DMP_OPENED)
        return ML_ERROR_SM_IMPROPER_STATE;

    elements = FIFOSetReference(1, accuracy, REF_DMP_PACKET, 1);
    if (elements & 1) {
        regs = DINA28;
        FIFOData.mlFIFODataConfig[CONFIG_DMP_PACKET_NUMBER] = ML_ELEMENT_1 | ML_16_BIT;
    } else {
        regs = DINAF8+3;
        FIFOData.mlFIFODataConfig[CONFIG_DMP_PACKET_NUMBER] = 0;
    }
    result = MLDLSetMemoryMPU(KEY_CFG_23, 1, &regs);
    ERROR_CHECK(result);

    return SetFIFOFooter();
}


/**
 *  @brief  Send the computed gravity vectors into the FIFO.
 *          The gravity vectors can be retrieved from the FIFO via 
 *          FIFOGetGravBody(), to have the gravitation vector expressed
 *          in coordinates relative to the body.
 *
 *  Gravity is a derived vector derived from the quaternion.
 *  @param  elements
 *              the gravitation vectors components bitmask.
 *              To send all compoents use ML_ALL.
 *  @param  accuracy
 *              The number of bits the gravitation vector is expressed 
 *              into.
 *              Set to ML_32_BIT for 32-bit data, or ML_16_BIT for 16
 *              bit data. 
 *              Set to zero to remove it from the FIFO.
 *
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError FIFOSendGravity(uint_fast16_t elements, uint_fast16_t accuracy)
{
    INVENSENSE_FUNC_START;
    tMLError result;

    result = FIFOSendQuaternion(accuracy);
    ERROR_CHECK(result);

    return SetFIFOFooter();
}


/** Sends gyro data to the FIFO. Gyro data is a 3 length vector
 *  of 32-bits. Should be called once after MLDmpOpen() and before MLDmpStart().
 *  @param[in] elements Which of the 3 elements to send. Use ML_ALL for all of them
 *            or ML_ELEMENT_1, ML_ELEMENT_2, ML_ELEMENT_3 or'd together
 *            for a subset.
 *  @param[in] accuracy Set to ML_32_BIT for 32-bit data, or ML_16_BIT for 16
 *             bit data. Set to zero to remove it from the FIFO.
 */
tMLError FIFOSendGyro(uint_fast16_t elements, uint_fast16_t accuracy)
{
    INVENSENSE_FUNC_START;
    unsigned char regs[4] = { DINAF8 + 1, DINA20, DINA28, DINA30};
    tMLError result;    

    if (MLGetState() < ML_STATE_DMP_OPENED)
        return ML_ERROR_SM_IMPROPER_STATE;

    if (FIFOData.gyroSource == ML_GYRO_FROM_QUATERNION) {
        regs[0] = DINA90 + 5;
        result = MLDLSetMemoryMPU(KEY_CFG_GYRO_SOURCE, 1, regs);
        ERROR_CHECK(result);
        regs[0] = DINAF8 + 1;
        regs[1] = DINA20;
        regs[2] = DINA28;
        regs[3] = DINA30;
    } else {
        regs[0] = DINA90 + 10;
        result = MLDLSetMemoryMPU(KEY_CFG_GYRO_SOURCE, 1, regs);
        ERROR_CHECK(result);
        regs[0] = DINAF8 + 1;
        regs[1] = DINA28;
        regs[2] = DINA30;
        regs[3] = DINA38;
    }
    result = FIFOConstruct3(regs, elements, accuracy, REF_GYROS,
                             KEY_CFG_9, CONFIG_GYROS );

    return SetFIFOFooter();
}

/** Sends linear accelerometer data to the FIFO. 
 *
 *  Linear accelerometer data is a 3 length vector of 32-bits. It is the 
 *  acceleration in the body frame with gravity removed.
 * 
 * 
 *  @param[in] elements Which of the 3 elements to send. Use ML_ALL for all of
 *            them or ML_ELEMENT_1, ML_ELEMENT_2, ML_ELEMENT_3 or'd together
 *            for a subset.
 *
 *  NOTE: Elements is ignored if the fifo rate is < ML_MAX_NUM_ACCEL_SAMPLES
 *  @param[in] accuracy Set to ML_32_BIT for 32-bit data, or ML_16_BIT for 16
 *             bit data. Set to zero to remove it from the FIFO.
 */
tMLError FIFOSendLinearAccel(uint_fast16_t elements, uint_fast16_t accuracy)
{
    INVENSENSE_FUNC_START;
    tMLError result;
    unsigned char state = MLGetState();

    if (state < ML_STATE_DMP_OPENED)
        return ML_ERROR_SM_IMPROPER_STATE;

    result = FIFOSendGravity(elements, accuracy);
    ERROR_CHECK(result);
    result = FIFOSendAccel(elements, accuracy);
    ERROR_CHECK(result);

    return SetFIFOFooter();
}

/** Sends linear world accelerometer data to the FIFO. Linear world
 *  accelerometer data is a 3 length vector of 32-bits. It is the acceleration
 *  in the world frame with gravity removed. Should be called once after
 *  MLDmpOpen() and before MLDmpStart().
 *
 *  @param[in] elements Which of the 3 elements to send. Use ML_ALL for all of 
 *             them or ML_ELEMENT_1, ML_ELEMENT_2, ML_ELEMENT_3 or'd together
 *             for a subset.
 *  @param[in] accuracy Set to ML_32_BIT for 32-bit data, or ML_16_BIT for 16
 *             bit data.
 */
tMLError FIFOSendLinearAccelWorld(uint_fast16_t elements, 
                                  uint_fast16_t accuracy)
{
    INVENSENSE_FUNC_START;
    tMLError result;

    result = FIFOSendLinearAccel(ML_ALL, accuracy);
    ERROR_CHECK(result);
    result = FIFOSendQuaternion(accuracy);

    return SetFIFOFooter();
}

/** Sends quaternion data to the FIFO. Quaternion data is a 4 length vector
 *   of 32-bits. Should be called once after MLDmpOpen() and before MLDmpStart().
 * @param[in] accuracy Set to ML_32_BIT for 32-bit data, or ML_16_BIT for 16
 *            bit data.
 */
tMLError FIFOSendQuaternion(uint_fast16_t accuracy)
{
    INVENSENSE_FUNC_START;
    unsigned char regs[5] = { DINAF8 + 1, DINA20, DINA28,
                              DINA30, DINA38};
    uint_fast16_t elements, kk;
    tMLError result;

    if (MLGetState() < ML_STATE_DMP_OPENED)
        return ML_ERROR_SM_IMPROPER_STATE;

    elements = FIFOSetReference(0xf, accuracy, REF_QUATERNION, 4);
    accuracy = FIFOSetAccuracy(elements, accuracy, CONFIG_QUAT);

    if (accuracy & ML_16_BIT) {
        regs[0] = DINAF8 + 2;
    }

    FIFOData.mlFIFODataConfig[CONFIG_QUAT] = elements | accuracy;

    for (kk = 0; kk < 4; ++kk) {
        if ((elements & 1) == 0)
            regs[kk+1] = DINAA0+3;
        elements >>= 1;
    }

    result = MLDLSetMemoryMPU(KEY_CFG_8, 5, regs);
    ERROR_CHECK(result);

    return SetFIFOFooter();
}

/** Sends raw data to the FIFO. 
 *  Should be called once after MLDmpOpen() and before MLDmpStart().
 *  @param[in] elements Which of the 7 elements to send. Use ML_ALL for all of them
 *            or ML_ELEMENT_1, ML_ELEMENT_2, ML_ELEMENT_3 ... ML_ELEMENT_7 or'd together
 *            for a subset. The first element is temperature, the next 3 are gyro data,
 *            and the last 3 accel data.
 *  @param  accuracy
 *              The element's accuracy, can be ML_16_BIT, ML_32_BIT, or 0 to turn off.
 *  @return 0 if successful, a non-zero error code otherwise.
 */
tMLError FIFOSendRaw(uint_fast16_t elements, uint_fast16_t accuracy)
{
    int result;
#ifdef M_HW
    unsigned char regs[7] = {DINAA0+3, DINAA0+3, DINAA0+3,
                             DINAA0+3, DINAA0+3, DINAA0+3,
                             DINAA0+3 };

    if (MLGetState() < ML_STATE_DMP_OPENED)
        return ML_ERROR_SM_IMPROPER_STATE;

    if (accuracy)
        accuracy = ML_16_BIT;

    elements = FIFOSetReference(elements, accuracy, REF_RAW, 7);

    if (elements & 1)
        FIFOData.mlFIFODataConfig[CONFIG_TEMPERATURE] = 1 | ML_16_BIT;
    else
        FIFOData.mlFIFODataConfig[CONFIG_TEMPERATURE] = 0;
    if (elements & 0x7e)
        FIFOData.mlFIFODataConfig[CONFIG_RAW_DATA] = (0x3f & (elements>>1)) | ML_16_BIT;
    else
        FIFOData.mlFIFODataConfig[CONFIG_RAW_DATA] = 0;

    if (elements & ML_ELEMENT_1) {
        regs[0] = DINACA;
    }
    if (elements & ML_ELEMENT_2) {
        regs[1] = DINBC4;
    }
    if (elements & ML_ELEMENT_3) {
        regs[2] = DINACC;
    }
    if (elements & ML_ELEMENT_4) {
        regs[3] = DINBC6;
    }
    if (elements & ML_ELEMENT_5) {
        regs[4] = DINBC0;
    }
    if (elements & ML_ELEMENT_6) {
        regs[5] = DINAC8;
    }
    if (elements & ML_ELEMENT_7) {
        regs[6] = DINBC2;
    }
    result = MLDLSetMemoryMPU(KEY_CFG_15, 7, regs);
    ERROR_CHECK(result);

    return SetFIFOFooter();

#else
    INVENSENSE_FUNC_START;
    unsigned char regs[4] = {DINAA0+3,
                             DINAA0+3,
                             DINAA0+3,
                             DINAA0+3};

    if (MLGetState() < ML_STATE_DMP_OPENED)
        return ML_ERROR_SM_IMPROPER_STATE;

    if (accuracy)
        accuracy = ML_16_BIT;

    elements = FIFOSetReference(elements, accuracy, REF_RAW, 7);

    if (elements & 0x03) {
        elements |= 0x03;
        regs[0] = DINA20;
    }
    if (elements & 0x0C) {
        elements |= 0x0C;
        regs[1] = DINA28;
    }
    if (elements & 0x30) {
        elements |= 0x30;
        regs[2] = DINA30;
    }
    if (elements & 0x40) {
        elements |= 0xC0;
        regs[3] = DINA38;
    }

    result = MLDLSetMemoryMPU(KEY_CFG_15, 4, regs);
    ERROR_CHECK(result);

    if (elements & 0x01)
        FIFOData.mlFIFODataConfig[CONFIG_TEMPERATURE] = 1 | ML_16_BIT;
    else
        FIFOData.mlFIFODataConfig[CONFIG_TEMPERATURE] = 0;
    if (elements & 0xfe)
        FIFOData.mlFIFODataConfig[CONFIG_RAW_DATA] = 
            (0x7f & (elements>>1)) | ML_16_BIT;
    else
        FIFOData.mlFIFODataConfig[CONFIG_RAW_DATA] = 0;

    return SetFIFOFooter();
#endif
}

/** Sends raw external data to the FIFO. 
 *  Should be called once after MLDmpOpen() and before MLDmpStart().
 *  @param[in] elements Which of the 3 elements to send. Use ML_ALL for all of them
 *            or ML_ELEMENT_1, ML_ELEMENT_2, ML_ELEMENT_3 or'd together
 *            for a subset. 
 *  @param[in] accuracy ML_16_BIT to send data, 0 to stop sending this data.
 *            Sending and Stop sending are reference counted, so data actually
 *            stops when the reference reaches zero.
 */
tMLError FIFOSendRawExternal(uint_fast16_t elements, uint_fast16_t accuracy)
{
#ifdef M_HW
    int result;
    unsigned char regs[3] = {DINAA0+3, DINAA0+3,
                             DINAA0+3 };

    if (MLGetState() < ML_STATE_DMP_OPENED)
        return ML_ERROR_SM_IMPROPER_STATE;

    if (accuracy)
        accuracy = ML_16_BIT;

    elements = FIFOSetReference(elements, accuracy, REF_RAW_EXTERNAL, 3);

    if (elements)
        FIFOData.mlFIFODataConfig[CONFIG_RAW_EXTERNAL] = elements | ML_16_BIT;
    else
        FIFOData.mlFIFODataConfig[CONFIG_RAW_EXTERNAL] = 0;

    if (elements & ML_ELEMENT_1) {
        regs[0] = DINBC2;
    }
    if (elements & ML_ELEMENT_2) {
        regs[1] = DINACA;
    }
    if (elements & ML_ELEMENT_3) {
        regs[2] = DINBC4;
    }

    result = MLDLSetMemoryMPU(KEY_CFG_EXTERNAL, 3, regs);
    ERROR_CHECK(result);

    return SetFIFOFooter();

#else
    return ML_ERROR_FEATURE_NOT_IMPLEMENTED; // Feature not supported
#endif
}


/**
 *  @brief  Send the Quantized Acceleromter data into the FIFO.  The data can be
 *          retrieved using FIFOGetQuantAccel() or FIFOGetDecodedAccel().
 *
 *  To be useful this should be set to mlFIFORate + 1 if less than
 *  ML_MAX_NUM_ACCEL_SAMPLES, otherwise it doesn't work.
 *
 *  @param  elements
 *              the components bitmask.
 *              To send all compoents use ML_ALL.
 *
 *  @param  accuracy
 *              Use ML_32_BIT for 32-bit data or ML_16_BIT for 
 *              16-bit data.
 *              Set to zero to remove it from the FIFO.
 *
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError FIFOSendQuantAccel(uint_fast16_t elements, uint_fast16_t accuracy)
{
    INVENSENSE_FUNC_START;
    unsigned char regs[5] = { DINAF8 + 1, DINA20, DINA28,
                              DINA30, DINA38 };
    unsigned char regs2[4] = { DINA20, DINA28,
                               DINA30, DINA38 };
    tMLError result;
    int_fast8_t kk;
    int_fast8_t ii;

    if (MLGetState() < ML_STATE_DMP_OPENED)
        return ML_ERROR_SM_IMPROPER_STATE;

    elements = FIFOSetReference(elements, accuracy, REF_QUANT_ACCEL, 8);

    if (elements) {
        FIFOData.mlFIFODataConfig[CONFIG_DMP_QUANT_ACCEL] = 
            (elements) | ML_32_BIT;
    } else {
        FIFOData.mlFIFODataConfig[CONFIG_DMP_QUANT_ACCEL] = 0;
    }

    for (kk = 0; kk < ML_MAX_NUM_ACCEL_SAMPLES; ++kk) {
        FIFOData.decoded[REF_QUANT_ACCEL + kk] = 0;
        for (ii = 0; ii < ACCEL_NUM_AXES; ii++) {
            FIFOData.decodedAccel[kk][ii] = 0;
        }
    }

    for (kk = 0; kk < 4; ++kk) {
        if ((elements & 1) == 0)
            regs[kk+1] = DINAA0+3;
        elements >>= 1;
    }

    result = MLDLSetMemoryMPU(KEY_CFG_TAP0, 5, regs);
    ERROR_CHECK(result);

    for (kk = 0; kk < 4; ++kk) {
        if ((elements & 1) == 0)
            regs2[kk] = DINAA0+3;
        elements >>= 1;
    }

    result = MLDLSetMemoryMPU(KEY_CFG_TAP4, 4, regs2);
    ERROR_CHECK(result);

    return SetFIFOFooter();
}

tMLError FIFOSendEis(uint_fast16_t elements, uint_fast16_t accuracy)
{
    INVENSENSE_FUNC_START;
    int_fast8_t kk;
    unsigned char regs[3] = { DINA28, DINA30, DINA38 };
    tMLError result;    

    if (MLGetState() < ML_STATE_DMP_OPENED)
        return ML_ERROR_SM_IMPROPER_STATE;

    if (accuracy) {
        accuracy = ML_32_BIT;
    }

    elements = FIFOSetReference(elements, accuracy, REF_EIS, 3);
    accuracy = FIFOSetAccuracy(elements, accuracy, CONFIG_EIS);

    FIFOData.mlFIFODataConfig[CONFIG_EIS] = elements | accuracy;

    for (kk = 0; kk < 3; ++kk) {
        if ((elements & 1) == 0)
            regs[kk] = DINAA0 + 7;
        elements >>= 1;
    }

    result = MLDLSetMemoryMPU(KEY_P_EIS_FIFO_XSHIFT, 3, regs);

    return SetFIFOFooter();
}

/** 
 * @brief       Returns 3-element vector of accelerometer data in body frame.
 *
 * @param[out]  data    3-element vector of accelerometer data in body frame.
 *                      One gee = 2^16.
 *  @return     0 on success or an error code.
 */
tMLError FIFOGetAccel(long *data)
{
    int kk;
    struct mldl_cfg *mldl_cfg = MLDLGetCfg();

    if (data == NULL)
        return ML_ERROR_INVALID_PARAMETER;

    if ((!FIFOData.mlFIFODataConfig[CONFIG_ACCEL] &&
         (mldl_cfg->requested_sensors & ML_DMP_PROCESSOR))
        ||
        (!(mldl_cfg->requested_sensors & ML_DMP_PROCESSOR) &&
         !AccelGetPresent()))
        return ML_ERROR_FEATURE_NOT_ENABLED;

    for (kk = 0; kk < ACCEL_NUM_AXES; ++kk) {
        data[kk] = FIFOData.decoded[REF_ACCEL + kk];
    }

    return ML_SUCCESS;
}

/** 
 *  @brief      Returns 4-element quaternion vector derived from 6-axis or 
 *  9-axis if 9-axis was implemented. 6-axis is gyros and accels. 9-axis is
 *  gyros, accel and compass.
 *
 *  @param[out] data    4-element quaternion vector. One is scaled to 2^30.
 *  @return     0 on success or an error code.
 */
tMLError FIFOGetQuaternion(long *data)
{
    int kk;

    if (data == NULL)
        return ML_ERROR_INVALID_PARAMETER;

    if (!FIFOData.mlFIFODataConfig[CONFIG_QUAT])
        return ML_ERROR_FEATURE_NOT_ENABLED;

    for (kk = 0; kk < 4; ++kk) {
        data[kk] = FIFOData.decoded[REF_QUATERNION + kk];
    }

    return ML_SUCCESS;
}

/** 
 *  @brief      Returns 4-element quaternion vector derived from 6 
 *              axis sensors (gyros and accels).
 *  @param[out] data    
 *                  4-element quaternion vector. One is scaled to 2^30.
 *  @return     0 on success or an error code.
 */
tMLError FIFOGetQuaternion6Axis(long *data)
{
    int kk;
    if (data == NULL)
        return ML_ERROR_INVALID_PARAMETER;

    if (!FIFOData.mlFIFODataConfig[CONFIG_QUAT])
        return ML_ERROR_FEATURE_NOT_ENABLED;

    for (kk = 0; kk < 4; ++kk) {
        data[kk] = FIFOData.decoded[REF_QUATERNION_6AXIS + kk];
    }

    return ML_SUCCESS;
}

tMLError FIFOGetRelativeQuaternion(long *data)
{
    if ( data == NULL )
        return ML_ERROR;
    data[0] = mlxData.mlRelativeQuat[0];
    data[1] = mlxData.mlRelativeQuat[1];
    data[2] = mlxData.mlRelativeQuat[2];
    data[3] = mlxData.mlRelativeQuat[3];
    return ML_SUCCESS;
}


/** 
 *  @brief  Returns 3-element vector of gyro data in body frame.
 *  @param[out] data    
 *              3-element vector of gyro data in body frame 
 *              with gravity removed. One degree per second = 2^16.
 *  @return 0 on success or an error code.
 */
tMLError FIFOGetGyro(long *data)
{
    int kk;
    if (data == NULL)
        return ML_ERROR_INVALID_PARAMETER;

    if (FIFOData.mlFIFODataConfig[CONFIG_GYROS]) {
        for (kk = 0; kk < 3; ++kk) {
            data[kk] = FIFOData.decoded[REF_GYROS + kk];
        }
        return ML_SUCCESS;
    } else {
        return ML_ERROR_FEATURE_NOT_ENABLED;
    }
}

/**
 *  @brief  Get the 3-element gravity vector from the FIFO expressed
 *          in coordinates relative to the body frame.
 *  @param  data    
 *              3-element vector of gravity in body frame.
 *  @return 0 on success or an error code.
 */
tMLError FIFOGetGravBody(long *data)
{
    long quat[4];
    int ii;
    tMLError result;

    if (data == NULL)
        return ML_ERROR_INVALID_PARAMETER;

    if (!FIFOData.mlFIFODataConfig[CONFIG_QUAT])
        return ML_ERROR_FEATURE_NOT_ENABLED;

    if ( (FIFOData.cache & FIFO_CACHE_GRAVITY_BODY) == 0 ) {
        FIFOData.cache |= FIFO_CACHE_GRAVITY_BODY;

        // Compute it from Quaternion
        result = FIFOGetQuaternion(quat);
        ERROR_CHECK(result);

        data[0] = q29_mult(quat[1], quat[3]) - q29_mult(quat[2], quat[0]);
        data[1] = q29_mult(quat[2], quat[3]) + q29_mult(quat[1], quat[0]);
        data[2] = (q29_mult(quat[3], quat[3]) + 
                   q29_mult(quat[0], quat[0])) - 1073741824L;

        for (ii = 0; ii < ACCEL_NUM_AXES; ii++) {
            data[ii] >>= 14;
            FIFOData.gravityCache[ii] = data[ii];
        }
    } else {
        data[0] = FIFOData.gravityCache[0];
        data[1] = FIFOData.gravityCache[1];
        data[2] = FIFOData.gravityCache[2];
    }

    return ML_SUCCESS;
}

/** 
* @brief        Sets the filter coefficent used for computing the acceleration
*               bias which is used to compute linear acceleration.
* @param[in] coef   Fitler coefficient. 0. means no filter, a small number means
*                   a small cutoff frequency with an increasing number meaning 
*                   an increasing cutoff frequency.
*/
tMLError FIFOSetLinearAccelFilterCoef(float coef)
{
    FIFOData.accFilterCoef = coef;
    return ML_SUCCESS;
}

/** 
 *  @brief      Returns 3-element vector of accelerometer data in body frame
 *              with gravity removed.
 *  @param[out] data    3-element vector of accelerometer data in body frame
 *                      with gravity removed. One g = 2^16.
 *  @return     0 on success or an error code. data unchanged on error.
 */
tMLError FIFOGetLinearAccel(long *data)
{
    int kk;
    long grav[3];
    long la[3];
    tMLError result;

    if (data == NULL)
        return ML_ERROR_INVALID_PARAMETER;

    result = FIFOGetGravBody(grav);
    ERROR_CHECK(result);
    result = FIFOGetAccel(la);
    ERROR_CHECK(result);

    if ( (FIFOData.cache & FIFO_CACHE_ACC_BIAS) == 0 ) {
        FIFOData.cache |= FIFO_CACHE_ACC_BIAS;

        for (kk = 0; kk < ACCEL_NUM_AXES; ++kk) {
            long x;
            x = la[kk] - grav[kk];
            FIFOData.accBiasFilt[kk] = (long)(x*FIFOData.accFilterCoef + 
                FIFOData.accBiasFilt[kk]*(1.f-FIFOData.accFilterCoef));
            data[kk] = x - FIFOData.accBiasFilt[kk];
        }
    } else {
        for (kk = 0; kk < ACCEL_NUM_AXES; ++kk) {
            data[kk] = la[kk] - grav[kk] - FIFOData.accBiasFilt[kk];
        }
    }
    return ML_SUCCESS;
}

/** 
 *  @brief      Returns 3-element vector of accelerometer data in world frame
 *              with gravity removed.
 *  @param[out] data    3-element vector of accelerometer data in world frame
 *                      with gravity removed. One g = 2^16.
 *  @return     0 on success or an error code.
 */
tMLError FIFOGetLinearAccelWorld(long *data)
{
    int kk;
    if (data == NULL)
        return ML_ERROR_INVALID_PARAMETER;
    if (FIFOData.mlFIFODataConfig[CONFIG_ACCEL] &&
        FIFOData.mlFIFODataConfig[CONFIG_QUAT])
    {
        long wtemp[4],qi[4],wtemp2[4];
        wtemp[0]=0;
        FIFOGetLinearAccel(&wtemp[1]);
        MLQMult(&FIFOData.decoded[REF_QUATERNION], wtemp, wtemp2);
        MLQInvert(&FIFOData.decoded[REF_QUATERNION], qi);
        MLQMult(wtemp2, qi, wtemp);
        for (kk = 0; kk < 3; ++kk) {
            data[kk] = wtemp[kk + 1];
        }
        return ML_SUCCESS;
    } else {
        return ML_ERROR_FEATURE_NOT_ENABLED;
    }
}

/**
 *  @brief      Returns 4-element vector of control data.
 *  @param[out] data    4-element vector of control data.
 *  @return     0 for succes or an error code.
 */
tMLError FIFOGetControlData(long *data)
{
    int kk;
    if (data == NULL)
        return ML_ERROR_INVALID_PARAMETER;

    if (!FIFOData.mlFIFODataConfig[CONFIG_CONTROL_DATA])
        return ML_ERROR_FEATURE_NOT_ENABLED;

    for (kk = 0; kk < 4; ++kk) {
        data[kk] = FIFOData.decoded[REF_CONTROL+kk];
    }

    return ML_SUCCESS;

}

/**
 *  @brief      Returns 3-element vector of EIS shfit data
 *  @param[out] data    3-element vector of EIS shift data.
 *  @return     0 for succes or an error code.
 */
tMLError FIFOGetEis(long *data)
{
    int kk;
    if (data == NULL)
        return ML_ERROR_INVALID_PARAMETER;

    if (!FIFOData.mlFIFODataConfig[CONFIG_EIS])
        return ML_ERROR_FEATURE_NOT_ENABLED;

    for (kk = 0; kk < 3; ++kk) {
        data[kk] = FIFOData.decoded[REF_EIS+kk];
    }

    return ML_SUCCESS;

}

/** 
 *  @brief      Returns 3-element vector of accelerometer data in body frame.
 *  @param[out] data    3-element vector of accelerometer data in body frame in g's.
 *  @return     0 for success or an error code.
 */
tMLError FIFOGetAccelFloat(float *data)
{
    long lData[3];
    int kk;
    int result;

    if (data == NULL)
        return ML_ERROR_INVALID_PARAMETER;

    result = FIFOGetAccel(lData);
    ERROR_CHECK(result);

    for (kk = 0; kk < ACCEL_NUM_AXES; ++kk) {
        data[kk] = lData[kk] / 65536.0f;
    }

    return ML_SUCCESS;
}

/** 
 *  @brief      Returns 4-element quaternion vector.
 *  @param[out] data    4-element quaternion vector.
 *  @return     0 on success, an error code otherwise.
 */
tMLError FIFOGetQuaternionFloat(float *data)
{
    int kk;

    if (data == NULL)
        return ML_ERROR_INVALID_PARAMETER;

    if (!FIFOData.mlFIFODataConfig[CONFIG_QUAT])
        return ML_ERROR_FEATURE_NOT_ENABLED;

    for (kk = 0; kk < 4; ++kk) {
        data[kk] = FIFOData.decoded[REF_QUATERNION + kk]/1073741824.0f;
    }

    return ML_SUCCESS;
}

/**
 * @brief   Command the MPU to put data in the FIFO at a particular rate.
 *
 *          The DMP will add fifo entries every fifoRate + 1 MPU cycles.  For
 *          example if the MPU is running at 200Hz the following values apply:
 *
 *          <TABLE>
 *          <TR><TD>fifoRate</TD><TD>DMP Sample Rate</TD><TD>FIFO update frequency</TD></TR>
 *          <TR><TD>0</TD><TD>200Hz</TD><TD>200Hz</TD></TR>
 *          <TR><TD>1</TD><TD>200Hz</TD><TD>100Hz</TD></TR>
 *          <TR><TD>2</TD><TD>200Hz</TD><TD>50Hz</TD></TR>
 *          <TR><TD>4</TD><TD>200Hz</TD><TD>40Hz</TD></TR>
 *          <TR><TD>9</TD><TD>200Hz</TD><TD>20Hz</TD></TR>
 *          <TR><TD>19</TD><TD>200Hz</TD><TD>10Hz</TD></TR>
 *          </TABLE>
 *
 *          Note: if the DMP is running, (state == ML_STATE_DMP_STARTED)
 *          then MLStateRunCallbacks() will be called to allow features
 *          that depend upon fundamental constants to be updated.
 *
 *  @pre    MLDmpOpen() with MLDmpDefaultOpen() or 
 *          MLDmpPedometerStandAloneOpen() and MLDmpStart() 
 *          must <b>NOT</b> have been called.
 *
 * @param   fifoRate    Divider value - 1.  Output rate is 
 *          (DMP Sample Rate) / (fifoRate + 1).
 *
 * @return  ML_SUCCESS if successful, ML error code on any failure.
 */
tMLError MLSetFIFORate(unsigned short fifoRate)
{
    INVENSENSE_FUNC_START;
    unsigned char regs[2];
    unsigned char state;
    tMLError result;

    state = MLGetState();
    if ( state != ML_STATE_DMP_OPENED && state != ML_STATE_DMP_STARTED)
        return ML_ERROR_SM_IMPROPER_STATE;

    FIFOData.mlFIFORate = fifoRate;

    regs[0] = (unsigned char)((fifoRate>>8) & 0xff);
    regs[1] = (unsigned char)(fifoRate & 0xff);
    
    
    result = MLDLSetMemoryMPU(KEY_D_0_22, 2, regs);
    ERROR_CHECK(result);

    if ( state == ML_STATE_DMP_STARTED )
        result = MLStateRunCallbacks( state );
    return result;
}

/**
 * @brief   Retrieve the current FIFO update divider - 1.
 *          See MLSetFIFORate() for how this value is used.
 *
 *          The fifo rate when there is no fifo is the equivilent divider when
 *          derived from the value set by SetSampleSteSizeMs()
 *          
 * @return  The value of the fifo rate divider or ML_INVALID_FIFO_RATE on error.
 */
unsigned short MLGetFIFORate(void)
{
    struct mldl_cfg *mldl_cfg = MLDLGetCfg();
    if (mldl_cfg->requested_sensors & ML_DMP_PROCESSOR)
        return FIFOData.mlFIFORate;
    else
        return (unsigned short)((1000 * FIFOData.sampleStepSizeMs) /
                                (SAMPLING_PERIOD_US(mldl_cfg))) - 1;
}

/**
 * @brief   Sets the step size when the FIFO is not used
 *          Typically set when using the accelerometer without the DMP.
 *
 * @return  step size for quaternion type data
 */
void SetSampleStepSizeMs(int_fast16_t ms)
{
    FIFOData.sampleStepSizeMs = ms;
}

/**
 * @brief   Returns the step size for quaternion type data.
 *
 * Typically the data rate for each FIFO packet. When the gryos are sleeping
 * this value will return the last value set by SetSampleStepSizeMs()
 *
 * @return  step size for quaternion type data
 */
int_fast16_t GetSampleStepSizeMs(void)
{
    struct mldl_cfg *mldl_cfg = MLDLGetCfg();

    if (mldl_cfg->requested_sensors & ML_DMP_PROCESSOR)
        return (FIFOData.mlFIFORate + 1) *
            (SAMPLING_PERIOD_US(mldl_cfg) / 1000);
    else
        return FIFOData.sampleStepSizeMs;
}

/**
 * @brief   Returns the step size for quaternion type data.
 *
 * Typically the data rate for each FIFO packet. When the gryos are sleeping
 * this value will return the last value set by SetSampleStepSizeMs()
 *
 * @return  step size for quaternion type data
 */
int_fast16_t GetSampleFrequencyHz(void)
{
    struct mldl_cfg *mldl_cfg = MLDLGetCfg();

    if (mldl_cfg->requested_sensors & ML_DMP_PROCESSOR)
        return (SAMPLING_RATE_HZ(mldl_cfg) / (FIFOData.mlFIFORate + 1));
    else
        return (1000 / FIFOData.sampleStepSizeMs);
}

/** 
 *  @brief  The gyro data magnitude squared : 
 *          (1 degree per second)^2 = 2^6 = 2^GYRO_MAG_SQR_SHIFT.
 *  @return the computed magnitude squared output of the gyroscope.
 */
unsigned long getGyroMagSqrd(void)
{
    unsigned long gmag = 0;
    long temp;
    int kk;

    for (kk = 0; kk < 3; ++kk) {
        temp = FIFOData.decoded[REF_GYROS + kk] >> 
            (16 - (GYRO_MAG_SQR_SHIFT / 2));
        gmag += temp * temp;
    }

    return gmag;
}

/** 
 *  @brief  The gyro data magnitude squared:
 *          (1 g)^2 = 2^16 = 2^ACC_MAG_SQR_SHIFT.
 *  @return the computed magnitude squared output of the accelerometer.
 */
unsigned long getAccMagSqrd(void)
{
    unsigned long amag = 0;
    long temp;
    int kk;
    long accel[3];
    tMLError result;

    result = FIFOGetAccel(accel);
    if (ML_SUCCESS != result) {
        return 0;
    }

    for (kk = 0; kk < 3; ++kk) {
        temp = accel[kk] >> (16 - (ACC_MAG_SQR_SHIFT / 2));
        amag += temp * temp;
    }
    return amag;
}

/**
 *  @internal
 */
void overRideQuaternion(float *q)
{
    int kk;
    for (kk = 0; kk < 4; ++kk) {
        FIFOData.decoded[REF_QUATERNION + kk] = (long)(q[kk] * (1L<<30));
    }
}

/**
 * @internal
 * @brief   This registers a function to be called for each set of 
 *          gyro/quaternion/rotation matrix/etc output.
 * @return  error code.
 */
tMLError RegisterHighRateProcess(tMlxdataFunction func)
{
    INVENSENSE_FUNC_START;
    tMLError result;

    result = MLOSLockMutex(mlxCallback.mutex);
    if (ML_SUCCESS != result) {
        return result;
    }

    // Make sure we have not filled up our number of allowable callbacks
    if (mlxCallback.numHighrateProcesses <= MAX_HIGH_RATE_PROCESSES-1) {
        int kk;
        // Make sure we haven't registered this function already
        for (kk = 0; kk < mlxCallback.numHighrateProcesses; ++kk) {
            if (mlxCallback.highrateProcess[kk] == func) {
                result = ML_ERROR_INVALID_PARAMETER;
                break;
            }
        }

        if (ML_SUCCESS == result) {
            // Add new callback
            mlxCallback.highrateProcess[mlxCallback.numHighrateProcesses] = func;
            mlxCallback.numHighrateProcesses++;
        }
    } else {
        result = ML_ERROR_MEMORY_EXAUSTED;
    }

    MLOSUnlockMutex(mlxCallback.mutex);
    return result;
}

/**
 * @internal
 * @brief   This unregisters a function to be called for each set of 
 *          gyro/quaternion/rotation matrix/etc output.
 * @return  error code.
 */
tMLError UnRegisterHighRateProcess(tMlxdataFunction func)
{
    INVENSENSE_FUNC_START;
    int kk,jj;
    tMLError result;

    result = MLOSLockMutex(mlxCallback.mutex);
    if (ML_SUCCESS != result) {
        return result;
    }

    // Make sure we haven't registered this function already
    result = ML_ERROR_INVALID_PARAMETER;
    for (kk = 0; kk < mlxCallback.numHighrateProcesses; ++kk) {
        if (mlxCallback.highrateProcess[kk] == func) {
            for (jj=kk+1; jj<mlxCallback.numHighrateProcesses; ++jj) {
                mlxCallback.highrateProcess[jj-1] =
                    mlxCallback.highrateProcess[jj];
            }
            mlxCallback.numHighrateProcesses--;
            result = ML_SUCCESS;
            break;
        }
    }

    MLOSUnlockMutex(mlxCallback.mutex);
    return result;

}

tMLError RunHighRateProcessFuncs(void)
{
    int kk;
    tMLError result,result2;
    
    result = MLOSLockMutex(mlxCallback.mutex);
    if (ML_SUCCESS != result) {
        MPL_LOGE("MLOsLockMutex returned %d\n", result);
        return result;
    }

    // User callbacks take priority over the highrateProcess callback
    if (FIFOData.FIFOProcessCB)
         FIFOData.FIFOProcessCB();

    for (kk = 0; kk < mlxCallback.numHighrateProcesses; ++kk) {
        if (mlxCallback.highrateProcess[kk]) {
            result2 = mlxCallback.highrateProcess[kk](&mlxData);
            if (result == ML_SUCCESS)
                result = result2;
        }
    }

    MLOSUnlockMutex(mlxCallback.mutex);
    return result;
}

/*********************/
/** \}*/ /* defgroup */
/*********************/
