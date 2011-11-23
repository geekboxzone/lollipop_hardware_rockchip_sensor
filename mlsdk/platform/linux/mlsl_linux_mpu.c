/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */

/******************************************************************************
 * $Id: mlsl_linux_mpu.c 5054 2011-03-22 17:50:42Z nroyer $
 *****************************************************************************/

/** 
 *  @defgroup MLSL (Motion Library - Serial Layer)
 *  @brief  The Motion Library System Layer provides the Motion Library the 
 *          interface to the system functions.
 *
 *  @{
 *      @file   mlsl_linux_dev.c
 *      @brief  The Motion Library System Layer.
 *
 */

/* ------------------ */
/* - Include Files. - */
/* ------------------ */
#include <stdio.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <signal.h>
#include <time.h>

#include "mpu.h"
#include "mlsl.h"
#include "mlos.h"
#include "mlmath.h"
#include "mlinclude.h"

#define MLCAL_ID      (0x0A0B0C0DL)
#define MLCAL_FILE    "/data/cal.bin"
#define MLCFG_ID      (0x01020304L)
#define MLCFG_FILE    "/data/cfg.bin"

#include <log.h>
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-mlsl"

#ifndef I2CDEV
#define I2CDEV "/dev/mpu"
#endif

#define SERIAL_FULL_DEBUG (0)

/* --------------- */
/* - Prototypes. - */
/* --------------- */

/* ----------------------- */
/* -  Function Pointers. - */
/* ----------------------- */

/* --------------------------- */
/* - Global and Static vars. - */
/* --------------------------- */

/* ---------------- */
/* - Definitions. - */
/* ---------------- */

/**
 *  @internal
 *  @brief  used to get the configuration data.
 *          Is called by the MPL to get the configuration data
 *          used by the motion library. 
 *          This data would typically be saved in non-volatile memory.
 *
 *  @param  cfg     Pointer to the configuration data.
 *  @param  len     Length of the configuration data.
 *
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError MLSLReadCfg(unsigned char *cfg, unsigned int len)
{
    FILE *fp;
    int bytesRead;

    fp = fopen(MLCFG_FILE, "rb");
    if (fp == NULL) {
        MPL_LOGE("Unable to open \"%s\" for read\n", MLCFG_FILE);
        return ML_ERROR_FILE_OPEN;
    }
    bytesRead = fread(cfg, 1, len, fp);
    if (bytesRead != len) {
        MPL_LOGE("bytes read (%d) don't match requested length (%d)\n",
                 bytesRead, len);
        return ML_ERROR_FILE_READ;
    }
    fclose(fp);

    if (((unsigned int)cfg[0] << 24 | cfg[1] << 16 | cfg[2] << 8 | cfg[3])
        != MLCFG_ID) {
        return ML_ERROR_ASSERTION_FAILURE;
    }

    return ML_SUCCESS;
}

/**
 *  @internal
 *  @brief  used to save the configuration data.
 *          Is called by the MPL to save the configuration data used by the 
 *          motion library.
 *          This data would typically be saved in non-volatile memory.
 *
 *  @param  cfg     Pointer to the configuration data.
 *  @param  len     Length of the configuration data.
 *
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError MLSLWriteCfg(unsigned char *cfg, unsigned int len)
{
    FILE *fp;
    int bytesWritten;
    unsigned char cfgId[4];
    
    fp = fopen(MLCFG_FILE,"wb");
    if (fp == NULL) {
        MPL_LOGE("Unable to open \"%s\" for write\n", MLCFG_FILE);
        return ML_ERROR_FILE_OPEN;
    }
    
    cfgId[0] = (unsigned char)(MLCFG_ID >> 24);
    cfgId[1] = (unsigned char)(MLCFG_ID >> 16);
    cfgId[2] = (unsigned char)(MLCFG_ID >> 8);
    cfgId[3] = (unsigned char)(MLCFG_ID);
    bytesWritten = fwrite(cfgId, 1, 4, fp);
    if (bytesWritten != 4) {
        MPL_LOGE("CFG ID could not be written on file\n");
        return ML_ERROR_FILE_WRITE;
    }
    
    bytesWritten = fwrite(cfg, 1, len, fp);
    if (bytesWritten != len) {
        MPL_LOGE("bytes write (%d) don't match requested length (%d)\n",
                 bytesWritten, len);
        return ML_ERROR_FILE_WRITE;
    }
    
    fclose(fp);
    
    return ML_SUCCESS;
}

/**
 *  @brief  used to get the calibration data.
 *          It is called by the MPL to get the calibration data used by the 
 *          motion library.
 *          This data would typically be saved in non-volatile memory.
 *
 *  @param  cal     Pointer to the calibration data.
 *  @param  len     Length of the calibration data.
 *
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError MLSLReadCal(unsigned char *cal, unsigned int len)
{
    FILE *fp;
    int bytesRead;
    tMLError result = ML_SUCCESS;

    fp = fopen(MLCAL_FILE,"rb");
    if (fp == NULL) {
        MPL_LOGE("Cannot open file \"%s\" for read\n", MLCAL_FILE);
        return ML_ERROR_FILE_OPEN;
    }
    bytesRead = fread(cal, 1, len, fp);
    if (bytesRead != len) {
        MPL_LOGE("bytes read (%d) don't match requested length (%d)\n",
                 bytesRead, len);
        result = ML_ERROR_FILE_READ;
        goto read_cal_end;
    }

    /* MLCAL_ID not used 
    if (((unsigned int)cal[0] << 24 | cal[1] << 16 | cal[2] << 8 | cal[3]) 
        != MLCAL_ID) {
        result = ML_ERROR_ASSERTION_FAILURE;
        goto read_cal_end;
    }
    */
read_cal_end:
    fclose(fp);
    return result;
}

/**
 *  @brief  used to save the calibration data.
 *          It is called by the MPL to save the calibration data used by the 
 *          motion library.
 *          This data would typically be saved in non-volatile memory.
 *
 *  @param cfg  Pointer to the calibration data.
 *  @param len  Length of the calibration data.
 *
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError MLSLWriteCal(unsigned char *cal, unsigned int len)
{
    FILE *fp;
    int bytesWritten;
    tMLError result = ML_SUCCESS;
    
    fp = fopen(MLCAL_FILE,"wb");
    if (fp == NULL) {
        MPL_LOGE("Cannot open file \"%s\" for write\n", MLCAL_FILE);
        return ML_ERROR_FILE_OPEN;
    }
    bytesWritten = fwrite(cal, 1, len, fp);
    if (bytesWritten != len) {
        MPL_LOGE("bytes written (%d) don't match requested length (%d)\n",
                 bytesWritten, len);
        result = ML_ERROR_FILE_WRITE;
    }
    fclose(fp);
    return result;
}

/**
 *  @brief Get the calibration length.
 *  @param  len 
 *              lenght to be returned
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError MLSLGetCalLength(unsigned int *len) 
{
    FILE *calFile;
    *len = 0;

    calFile = fopen(MLCAL_FILE, "rb");
    if (calFile == NULL) {
        MPL_LOGE("Cannot open file \"%s\" for read\n", MLCAL_FILE);
        return ML_ERROR_FILE_OPEN;
    }
    
    *len += (unsigned char)fgetc(calFile) << 24;
    *len += (unsigned char)fgetc(calFile) << 16;
    *len += (unsigned char)fgetc(calFile) << 8;
    *len += (unsigned char)fgetc(calFile);

    fclose(calFile);

    if (*len <= 0)
        return ML_ERROR_FILE_READ;

    return ML_SUCCESS;
}

/**
 *  @brief  used to open the I2C serial port.
 *          This port is used to send and receive data to the MPU device.
 *  @param  portNum 
 *              The COM port number associated with the device in use.
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError MLSLSerialOpen(char const *port, void **sl_handle)
{
    INVENSENSE_FUNC_START;

    if (NULL == port) {
        port = I2CDEV;
    }
    *sl_handle = (void*) open(port, O_RDWR);
    if(sl_handle < 0) {
        /* ERROR HANDLING; you can check errno to see what went wrong */
        MPL_LOGE("MLSLSerialOpen\n");
        MPL_LOGE("I2C Error %d: Cannot open Adapter %s\n", errno, port);
        return ML_ERROR_SERIAL_OPEN_ERROR;
    } else {
        MPL_LOGI("MLSLSerialOpen: %s\n", port);
    }

    return ML_SUCCESS;  
}


/**
 *  @brief  used to close the I2C serial port.
 *          This port is used to send and receive data to the MPU device.
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError MLSLSerialClose(void *sl_handle)
{
    INVENSENSE_FUNC_START;

    close((int)sl_handle);

    return ML_SUCCESS;
}

/**
 *  @brief  used to reset any buffering the driver may be doing
 *  @return Zero if the command is successful, an error code otherwise.
 */
tMLError MLSLSerialReset(void *sl_handle)
{
    return ML_ERROR_FEATURE_NOT_IMPLEMENTED;
}

/**
 *  @brief  used to write a single byte of data.
 *          It is called by the MPL to write a single byte of data to the MPU. 
 *          This should be sent by I2C.
 *
 *  @param  slaveAddr       I2C slave address of device.
 *  @param  registerAddr    Register address to write.
 *  @param  data            Single byte of data to write.
 *
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError MLSLSerialWriteSingle(void *sl_handle,
                               unsigned char slaveAddr, 
                               unsigned char registerAddr, 
                               unsigned char data)
{
    unsigned char buf[2];
    buf[0] = registerAddr;
    buf[1] = data;
    return MLSLSerialWrite(sl_handle, slaveAddr, 2, buf);
}

/**
 *  @brief  used to write multiple bytes of data to 
 *          registers.
 *          This should be sent by I2C.
 *
 *  @param  slaveAddr       I2C slave address of device.
 *  @param  registerAddr    Register address to write.
 *  @param  length          Length of burst data.
 *  @param  data            Pointer to block of data.
 *
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError MLSLSerialWrite(void *sl_handle,
                         unsigned char slaveAddr,
                         unsigned short length,
                         unsigned char const *data)
{
    INVENSENSE_FUNC_START;
    struct mpu_read_write msg;
    
    if (NULL == data) {
        return ML_ERROR_INVALID_PARAMETER;
    }
    
    msg.address = 0; /* not used */
    msg.length  = length;
    msg.data    = (unsigned char*)data;

    if (ioctl((int)sl_handle, MPU_WRITE, &msg) != ML_SUCCESS) {
        MPL_LOGE("I2C Error: could not write: R:%02x L:%d \n",
               data[0], length);
       return ML_ERROR_SERIAL_WRITE;
    } else if (SERIAL_FULL_DEBUG) {
        char data_buff[4096] = "";
        char strchar[3];
        int ii;
        for (ii = 0; ii < length; ii++) {
            snprintf(strchar, sizeof(strchar), "%02x", data[0]);
            strncat(data_buff, strchar, sizeof(data_buff));
        }
        MPL_LOGI("I2C Write Success %02x %02x: %s \n",
                 data[0], length, data_buff);
    }

    return ML_SUCCESS;
}

/**
 *  @brief  used to read multiple bytes of data to 
 *          registers.
 *          This should be sent by I2C.
 *
 *  @param  slaveAddr       I2C slave address of device.
 *  @param  registerAddr    Register address to read.
 *  @param  length          Length of burst data.
 *  @param  data            Pointer to block of data.
 *
 *  @return Zero if the command is successful; an error code otherwise
 */
tMLError MLSLSerialRead(void *sl_handle,
                        unsigned char  slaveAddr,
                        unsigned char  registerAddr,
                        unsigned short length,
                        unsigned char  *data)
{
    INVENSENSE_FUNC_START;
    int result = ML_SUCCESS;
    struct mpu_read_write msg;

    if (NULL == data) {
        return ML_ERROR_INVALID_PARAMETER;
    }

    msg.address = registerAddr;
    msg.length  = length;
    msg.data    = data;

    result = ioctl((int)sl_handle, MPU_READ, &msg);

    if (result != ML_SUCCESS) {
        MPL_LOGE("I2C Error %08x: could not read: R:%02x L:%d\n",
                 result, registerAddr, length);
        result = ML_ERROR_SERIAL_READ;
    } else if (SERIAL_FULL_DEBUG) {
        char data_buff[4096] = "";
        char strchar[3];
        int ii;
        for (ii = 0; ii < length; ii++) {
            snprintf(strchar, sizeof(strchar), "%02x", data[0]);
            strncat(data_buff, strchar, sizeof(data_buff));
        }
        MPL_LOGI("I2C Read  Success %02x %02x: %s \n",
                  registerAddr, length, data_buff);
    } 

    return (tMLError) result;
}

/**
 *  @brief  used to write multiple bytes of data to the memory.
 *          This should be sent by I2C.
 *
 *  @param  slaveAddr       I2C slave address of device.
 *  @param  memAddr         The location in the memory to write to.
 *  @param  length          Length of burst data.
 *  @param  data            Pointer to block of data.
 *
 *  @return Zero if successful; an error code otherwise
 */
tMLError MLSLSerialWriteMem(void *sl_handle,
                            unsigned char mpu_addr,
                            unsigned short memAddr,
                            unsigned short length,
                            const unsigned char *data)
{
    INVENSENSE_FUNC_START;
    struct mpu_read_write msg;
    int result;

    msg.address = memAddr;
    msg.length  = length;
    msg.data    = (unsigned char *)data;

    result = ioctl((int)sl_handle, MPU_WRITE_MEM, &msg);
    if (result != ML_SUCCESS) {
        MPL_LOGE("I2C Error: could not write memory: "
                 "S:%02x L:%d\n",
                 memAddr, length);
       return ML_ERROR_SERIAL_WRITE;
    } else if (SERIAL_FULL_DEBUG) {
        char data_buff[4096] = "";
        char strchar[3];
        int ii;
        for (ii = 0; ii < length; ii++) {
            snprintf(strchar, sizeof(strchar), "%02x", data[0]);
            strncat(data_buff, strchar, sizeof(data_buff));
        }
        MPL_LOGI("I2C WriteMem Success %04x %04x: %s \n",
                 memAddr, length, data_buff);
    }
    return ML_SUCCESS;
}

/**
 *  @brief  used to read multiple bytes of data from the 
 *          Invensense device's internal memory.
 *          This should be sent by I2C.
 *
 *  @param  slaveAddr       I2C slave address of device.
 *  @param  registerAddr    Register address to read.
 *  @param  length          Length of burst data.
 *  @param  data            Pointer to block of data.
 *
 *  @return Zero if the command is successful; an error code otherwise
 */
tMLError MLSLSerialReadMem(void *sl_handle,
                           unsigned char  mpu_addr,
                           unsigned short memAddr,
                           unsigned short length,
                           unsigned char  *data)
{
    INVENSENSE_FUNC_START;
    struct mpu_read_write msg;
    int result;

    if (NULL == data) {
        return ML_ERROR_INVALID_PARAMETER;
    }

    msg.address = memAddr;
    msg.length  = length;
    msg.data    = data;

    result = ioctl((int)sl_handle, MPU_READ_MEM, &msg);
    if (result != ML_SUCCESS) {
        MPL_LOGE("I2C Error %08x: could not read memory: A:%04x L:%d\n",
                 result, memAddr, length);
        return ML_ERROR_SERIAL_READ;
    } else if (SERIAL_FULL_DEBUG) {
        char data_buff[4096] = "";
        char strchar[3];
        int ii;
        for (ii = 0; ii < length; ii++) {
            snprintf(strchar, sizeof(strchar), "%02x", data[0]);
            strncat(data_buff, strchar, sizeof(data_buff));
        }
        MPL_LOGI("I2C ReadMem Success %04x %04x: %s\n",
                 memAddr, length, data_buff);
    }
    return ML_SUCCESS;
}

/**
 *  @brief  used to write multiple bytes of data from the 
 *          Invensense device's FIFO.
 *          This should be sent by I2C.
 *
 *  @param  slaveAddr       I2C slave address of device.
 *  @param  length          Length of burst data.
 *  @param  data            Pointer to block of data.
 *
 *  @return Zero if the command is successful; an error code otherwise
 */
tMLError MLSLSerialWriteFifo(void *sl_handle,
                             unsigned char mpu_addr,
                             unsigned short length,
                             const unsigned char *data)
{
    INVENSENSE_FUNC_START;
    struct mpu_read_write msg;
    int result;

    if (NULL == data) {
        return ML_ERROR_INVALID_PARAMETER;
    }

    msg.address = 0; /* Not used */
    msg.length  = length;
    msg.data    = (unsigned char *)data;

    result = ioctl((int)sl_handle, MPU_WRITE_FIFO, &msg);
    if (result != ML_SUCCESS) {
        MPL_LOGE("I2C Error: could not write fifo: %02x %02x\n",
                  MPUREG_FIFO_R_W, length);
        return ML_ERROR_SERIAL_WRITE;
    } else if (SERIAL_FULL_DEBUG) {
        char data_buff[4096] = "";
        char strchar[3];
        int ii;
        for (ii = 0; ii < length; ii++) {
            snprintf(strchar, sizeof(strchar), "%02x", data[0]);
            strncat(data_buff, strchar, sizeof(data_buff));
        }
        MPL_LOGI("I2C Write Success %02x %02x: %s\n",
                 MPUREG_FIFO_R_W, length, data_buff);
    }
    return (tMLError) result;
}

/**
 *  @brief  used to read multiple bytes of data from the 
 *          Invensense device's FIFO.
 *          This should be sent by I2C.
 *
 *  @param  slaveAddr       I2C slave address of device.
 *  @param  length          Length of burst data.
 *  @param  data            Pointer to block of data.
 *
 *  @return Zero if the command is successful; an error code otherwise
 */
tMLError MLSLSerialReadFifo(void *sl_handle,
                            unsigned char  mpu_addr,
                            unsigned short length,
                            unsigned char  *data)
{
    INVENSENSE_FUNC_START;
    struct mpu_read_write msg;
    int result;

    if (NULL == data) {
        return ML_ERROR_INVALID_PARAMETER;
    }

    msg.address = MPUREG_FIFO_R_W; /* Not used */
    msg.length  = length;
    msg.data    = data;

    result = ioctl((int)sl_handle, MPU_READ_FIFO, &msg);
    if (result != ML_SUCCESS) {
        MPL_LOGE("I2C Error %08x: could not read fifo: R:%02x L:%d\n",
                 result, MPUREG_FIFO_R_W, length);
        return ML_ERROR_SERIAL_READ;
    } else if (SERIAL_FULL_DEBUG) {
        char data_buff[4096] = "";
        char strchar[3];
        int ii;
        for (ii = 0; ii < length; ii++) {
            snprintf(strchar, sizeof(strchar), "%02x", data[0]);
            strncat(data_buff, strchar, sizeof(data_buff));
        }
        MPL_LOGI("I2C ReadFifo Success %02x %02x: %s\n",
                 MPUREG_FIFO_R_W, length, data_buff);
    }
    return ML_SUCCESS;
}

/**
 *  @}
 */


