/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */

/******************************************************************************
 * $Id: mlsl_linux.c 4639 2011-01-28 04:39:15Z yserlin $
 *****************************************************************************/

/** 
 *  @defgroup MLSL (Motion Library - Serial Layer)
 *  @brief  The Motion Library System Layer provides the Motion Library the 
 *          interface to the system functions.
 *
 *  @{
 *      @file   mlsl_linux.c
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
#include <pthread.h>

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

#if (defined(ML_USE_DMP_SIM) && !defined(ML_USE_DMP_SIM_POLL))

void* pthread_poll    (void *arg);
void  setup_polling   (void);
void  cleanup_polling (void);

#endif

static tMLError I2C_ReadBurst(void *sl_handle,
                              unsigned char   slaveAddr,
                              unsigned char   registerAddr,
                              unsigned short  length,
                              unsigned char  *data );

static tMLError I2C_WriteBurst(void *sl_handle,
                               unsigned char  slaveAddr,
                               unsigned short length,
                               unsigned char const *data );

static tMLError I2C_ReadMem(void *sl_handle,
                            unsigned char   slaveAddr,
                            unsigned short  address,
                            unsigned short  length,
                            unsigned char  *data );

static tMLError I2C_WriteMem(void *sl_handle,
                             unsigned char  slaveAddr,
                             unsigned short  address,
                             unsigned short length,
                             unsigned char const *data );

static tMLError I2C_ReadFifo(void *sl_handle,
                             unsigned char   slaveAddr,
                             unsigned short  length,
                             unsigned char  *data );

static tMLError I2C_WriteFifo(void *sl_handle,
                              unsigned char  slaveAddr,
                              unsigned short length,
                              unsigned char const *data );

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
static tMLError I2C_ReadBurst( void *sl_handle,
                               unsigned char  slaveAddr,
                               unsigned char  registerAddr,
                               unsigned short length,
                               unsigned char  *data )
{
    INVENSENSE_FUNC_START;
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset;
    int rc;
    tMLError result = ML_SUCCESS;
    int temp = 0 ;
    if (NULL == data) {
        MPL_LOGE("%s: NULL data\n",__func__);
        return ML_ERROR_INVALID_PARAMETER;
    }

    /* Write Message */
    msgs[0].addr = slaveAddr;
    msgs[0].flags = 0;
    msgs[0].buf = (unsigned char *) &registerAddr;
    msgs[0].len = 1;

    /* Read Message */
    msgs[1].addr = slaveAddr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].buf = data;
    msgs[1].len = length;

    msgset.msgs = msgs;
    msgset.nmsgs = 2;

    rc = ioctl((int)sl_handle, I2C_RDWR, &msgset);

    if (rc < 0) {
	temp = errno;
        MPL_LOGI("I2C Error %08x: could not read: S:%02x R:%02x L:%d\n",
                 rc, slaveAddr, registerAddr, length );
        result =  ML_ERROR_SERIAL_READ;
	MPL_LOGD("xxm  %s errno = %d\n",__func__,temp);
    } else if (SERIAL_FULL_DEBUG) {
        char data_buff[4096] = "";
        char strchar[3];
        int ii;
        for (ii = 0; ii < length; ii++) {
            snprintf(strchar, sizeof(strchar), "%02x", data[0]);
            strncat(data_buff, strchar, sizeof(data_buff));
        }
        MPL_LOGI("I2C Read  Success %02x %02x %02x: %s \n",
                  slaveAddr, registerAddr, length, data_buff);
    } 

    return result;
}

/**
 *  @internal
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
static tMLError I2C_WriteBurst( void *sl_handle,
                                unsigned char slaveAddr,
                                unsigned short length,
                                unsigned char const *data )
{
    INVENSENSE_FUNC_START;
    struct i2c_msg msg;
    struct i2c_rdwr_ioctl_data rdwr = { &msg, 1 };
    int temp = 0;
    if (NULL == data) {
        return ML_ERROR_INVALID_PARAMETER;
    }

    /* Write Message */
    msg.addr  = slaveAddr;
    msg.flags = 0;
    msg.buf   = (unsigned char*)data;
    msg.len   = length;

    if (ioctl((int)sl_handle, I2C_RDWR, &rdwr) < 0) {
	temp = errno;
        MPL_LOGI("I2C Error: could not write: S:%02x R:%02x L:%d \n",
               slaveAddr, data[0], length);
	MPL_LOGD("xxm  %s errno = %d\n",__func__,temp);	
       return ML_ERROR_SERIAL_WRITE;
    } else if (SERIAL_FULL_DEBUG) {
        char data_buff[4096] = "";
        char strchar[3];
        int ii;
        for (ii = 0; ii < length; ii++) {
            snprintf(strchar, sizeof(strchar), "%02x", data[0]);
            strncat(data_buff, strchar, sizeof(data_buff));
        }
        MPL_LOGI("I2C Write Success %02x %02x %02x: %s \n",
                 slaveAddr, data[0], length, data_buff);
    }

    return ML_SUCCESS;
}

/**
 *  @internal
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
static tMLError I2C_ReadMem( void *sl_handle,
                             unsigned char  mpu_addr,
                             unsigned short mem_addr,
                             unsigned short len,
                             unsigned char  *data )
{
    INVENSENSE_FUNC_START;
    unsigned char bank[2];
    unsigned char addr[2];
    unsigned char buf;

    struct i2c_msg msgs[4];
    struct i2c_rdwr_ioctl_data msgset = { msgs, 4 };

    int rc;

    if (NULL == data) {
        return ML_ERROR_INVALID_PARAMETER;
    }

    bank[0] = MPUREG_BANK_SEL;
    bank[1] = mem_addr >> 8;

    addr[0] = MPUREG_MEM_START_ADDR;
    addr[1] = mem_addr & 0xFF;

    buf = MPUREG_MEM_R_W;

    /* Write bank */
    msgs[0].addr = mpu_addr;
    msgs[0].flags = 0;
    msgs[0].buf = bank;
    msgs[0].len = sizeof(bank);
    /* Write memory start address */
    msgs[1].addr = mpu_addr;
    msgs[1].flags = 0;
    msgs[1].buf = addr;
    msgs[1].len = sizeof(addr);
    /* Read memory */
    msgs[2].addr = mpu_addr;
    msgs[2].flags = 0;
    msgs[2].buf = &buf;
    msgs[2].len = 1;

    msgs[3].addr = mpu_addr;
    msgs[3].flags = I2C_M_RD;
    msgs[3].buf = data;
    msgs[3].len = len;

    rc = ioctl((int)sl_handle, I2C_RDWR, &msgset);

    if (rc < 0) {
        MPL_LOGI("I2C Error %08x: could not read memory: "
                 "S:%02x B:%02x R:%02x L:%d\n",
             rc, mpu_addr, bank[1], addr[1], len );
          return ML_ERROR_SERIAL_READ;
    } else if (SERIAL_FULL_DEBUG) {
        char data_buff[4096] = "";
        char strchar[3];
        int ii;
        for (ii = 0; ii < len; ii++) {
            snprintf(strchar, sizeof(strchar), "%02x", data[0]);
            strncat(data_buff, strchar, sizeof(data_buff));
        }
        MPL_LOGI("I2C ReadMem Success %02x %02x %02x %02x: %s \n",
                 mpu_addr, bank[1], addr[1], len, data_buff);
    }
    return ML_SUCCESS;
}

/**
 *  @internal
 *  @brief  used to write multiple bytes of data from the 
 *          Invensense device's internal memory.
 *          This should be sent by I2C.
 *
 *  @param  slaveAddr       I2C slave address of device.
 *  @param  registerAddr    Register address to write.
 *  @param  length          Length of burst data.
 *  @param  data            Pointer to block of data.
 *
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
static tMLError I2C_WriteMem( void *sl_handle,
                              unsigned char mpu_addr,
                              unsigned short mem_addr,
                              unsigned short len,
                              const unsigned char *data )
{
    INVENSENSE_FUNC_START;
    unsigned char bank[2];
    unsigned char addr[2];
    unsigned char buf[513];

    struct i2c_msg msgs[3];
    struct i2c_rdwr_ioctl_data msgset = { msgs, 3 };
    int rc;

    if (len >= (sizeof(buf) - 1) || NULL == data)
        return ML_ERROR_INVALID_PARAMETER;

    bank[0] = MPUREG_BANK_SEL;
    bank[1] = mem_addr >> 8;

    addr[0] = MPUREG_MEM_START_ADDR;
    addr[1] = mem_addr & 0xFF;

    buf[0] = MPUREG_MEM_R_W;
    memcpy(&buf[1], data, len);

    /* Write bank */
    msgs[0].addr = mpu_addr;
    msgs[0].flags = 0;
    msgs[0].buf = bank;
    msgs[0].len = sizeof(bank);
    /* Write memory start addr */
    msgs[1].addr = mpu_addr;
    msgs[1].flags = 0;
    msgs[1].buf = addr;
    msgs[1].len = sizeof(addr);
    /* Write memory */
    msgs[2].addr = mpu_addr;
    msgs[2].flags = 0;
    msgs[2].buf = (unsigned char *) buf;
    msgs[2].len = len+1;

    rc = ioctl((int)sl_handle, I2C_RDWR, &msgset);
    if (rc < 0) {
        MPL_LOGI("I2C Error: could not write memory: "
                 "S:%02x B:%02x R:%02x L:%d\n",
                 mpu_addr, bank[1], addr[1], len);
       return ML_ERROR_SERIAL_WRITE;
    } else if (SERIAL_FULL_DEBUG) {
        char data_buff[4096] = "";
        char strchar[3];
        int ii;
        for (ii = 0; ii < len; ii++) {
            snprintf(strchar, sizeof(strchar), "%02x", data[0]);
            strncat(data_buff, strchar, sizeof(data_buff));
        }
        MPL_LOGI("I2C WriteMem Success %02x %02x %02x %02x: %s \n",
                 mpu_addr, bank[1], addr[1], len, data_buff);
    }
    return ML_SUCCESS;
}

/**
 *  @internal
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
static tMLError I2C_ReadFifo( void *sl_handle,
                              unsigned char  mpu_addr,
                              unsigned short len,
                              unsigned char  *data )
{
    INVENSENSE_FUNC_START;
    unsigned char buf;

    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset = { msgs, 2 };

    int rc;

    if (NULL == data) {
        return ML_ERROR_INVALID_PARAMETER;
    }

    buf = MPUREG_FIFO_R_W;

    msgs[0].addr = mpu_addr;
    msgs[0].flags = 0;
    msgs[0].buf = &buf;
    msgs[0].len = 1;

    msgs[1].addr = mpu_addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].buf = data;
    msgs[1].len = len;

    rc = ioctl((int)sl_handle, I2C_RDWR, &msgset);

    if (rc < 0) {
        MPL_LOGI("I2C Error %08x: could not read fifo: S:%02x R:%02x L:%d\n",
                 rc, mpu_addr, MPUREG_FIFO_R_W, len );
        return ML_ERROR_SERIAL_READ;
    } else if (SERIAL_FULL_DEBUG) {
        char data_buff[4096] = "";
        char strchar[3];
        int ii;
        for (ii = 0; ii < len; ii++) {
            snprintf(strchar, sizeof(strchar), "%02x", data[0]);
            strncat(data_buff, strchar, sizeof(data_buff));
        }
        MPL_LOGI("I2C ReadFifo Success %02x %02x %02x: %s \n",
                 mpu_addr, MPUREG_FIFO_R_W, len, data_buff);
    }
    return ML_SUCCESS;
}

/**
 *  @internal
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
static tMLError I2C_WriteFifo( void *sl_handle,
                               unsigned char mpu_addr,
                               unsigned short len,
                               const unsigned char *data )
{
    INVENSENSE_FUNC_START;
    unsigned char buf[FIFO_HW_SIZE+1];

    struct i2c_msg msgs[1];
    struct i2c_rdwr_ioctl_data msgset = { msgs, 1 };
    int rc;

    if (len >= (sizeof(buf) - 1) || NULL == data)
        return ML_ERROR_INVALID_PARAMETER;

    buf[0] = MPUREG_FIFO_R_W;
    memcpy(&buf[1], data, len);

    msgs[0].addr = mpu_addr;
    msgs[0].flags = 0;
    msgs[0].buf = (unsigned char *) buf;
    msgs[0].len = len+1;

    rc = ioctl((int)sl_handle, I2C_RDWR, &msgset);
    if (rc < 0) {
        MPL_LOGI("I2C Error: could not write fifo: %02x %02x %02x \n",
                  mpu_addr, MPUREG_FIFO_R_W, len);
        return ML_ERROR_SERIAL_WRITE;
    } else if (SERIAL_FULL_DEBUG) {
        char data_buff[4096] = "";
        char strchar[3];
        int ii;
        for (ii = 0; ii < len; ii++) {
            snprintf(strchar, sizeof(strchar), "%02x", data[0]);
            strncat(data_buff, strchar, sizeof(data_buff));
        }
        MPL_LOGI("I2C Write Success %02x %02x %02x: %s \n",
                 mpu_addr, MPUREG_FIFO_R_W, len, data_buff);
    }
    return ML_SUCCESS;
}

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

    fp = fopen(MLCAL_FILE,"rb");
    if (fp == NULL) {
        MPL_LOGE("Cannot open file \"%s\" for read\n", MLCAL_FILE);
        return ML_ERROR_FILE_OPEN;
    }
    bytesRead = fread(cal, 1, len, fp);
    if (bytesRead != len) {
        MPL_LOGE("bytes read (%d) don't match requested length (%d)\n",
                 bytesRead, len);
        return ML_ERROR_FILE_READ;
    }
    fclose(fp);

    /* MLCAL_ID not used 
    if (((unsigned int)cal[0] << 24 | cal[1] << 16 | cal[2] << 8 | cal[3]) 
        != MLCAL_ID) {
        return ML_ERROR_ASSERTION_FAILURE;
    }
    */
    return ML_SUCCESS;
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
    
    fp = fopen(MLCAL_FILE,"wb");
    if (fp == NULL) {
        MPL_LOGE("Cannot open file \"%s\" for write\n", MLCAL_FILE);
        return ML_ERROR_FILE_OPEN;
    }
    bytesWritten = fwrite(cal, 1, len, fp);
    if (bytesWritten != len) {
        MPL_LOGE("bytes written (%d) don't match requested length (%d)\n",
                 bytesWritten, len);
        return ML_ERROR_FILE_WRITE;
    }
    fclose(fp);

    return ML_SUCCESS;
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
    int res;

    if (NULL == port) {
        port = I2CDEV;
    }
    *sl_handle = (void*) open(port, O_RDWR );
    if(sl_handle < 0) {
        /* ERROR HANDLING; you can check errno to see what went wrong */
        MPL_LOGE("MLSLSerialOpen\n");
        MPL_LOGE("I2C Error: Cannot open Adapter %s\n", port);
        return ML_ERROR_SERIAL_OPEN_ERROR;
    } else {
        MPL_LOGI("MLSLSerialOpen: %s\n", port);
    }

    res = ioctl((int)*sl_handle, I2C_SLAVE, DEFAULT_MPU_SLAVEADDR );
    if (res < 0 ) {
        MPL_LOGI( "I2C Error: could not bind address %x: %d\n", 
                  DEFAULT_MPU_SLAVEADDR, res );
        close((int)sl_handle);
        return ML_ERROR_SERIAL_DEVICE_NOT_RECOGNIZED;
    }

#if (defined(ML_USE_DMP_SIM) && !defined(ML_USE_DMP_SIM_POLL))
    setup_polling();
#endif

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

#if (defined(ML_USE_DMP_SIM) && !defined(ML_USE_DMP_SIM_POLL))
    cleanup_polling();
#endif

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
    return MLSLSerialWrite( sl_handle, slaveAddr, 2, buf);
}


/**
 *  @brief  used to write multiple bytes of data from registers.
 *          This should be sent by I2C.
 *
 *  @param  slaveAddr       I2C slave address of device.
 *  @param  registerAddr    Register address to write.
 *  @param  length          Length of burst of data.
 *  @param  data            Pointer to block of data.
 *
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError MLSLSerialWrite( void *sl_handle,
                          unsigned char slaveAddr,
                          unsigned short length,
                          unsigned char const *data )
{
    tMLError result;
    const unsigned short dataLength = length-1;
    const unsigned char startRegAddr = data[0];
    unsigned char i2cWrite[SERIAL_MAX_TRANSFER_SIZE+1];
    uint_fast16_t bytesWritten = 0;

    while (bytesWritten<dataLength) {
        unsigned short thisLen = MIN(
            SERIAL_MAX_TRANSFER_SIZE, dataLength-bytesWritten
        );
        if (bytesWritten==0) {
            /*--- NOTE : first chunk or only chunk don't incurr in memcpy 
                         for better performance ---*/
            result = I2C_WriteBurst(sl_handle, slaveAddr, 1+thisLen, data);
        }
        else {
            // manually increment register addr between chunks
            i2cWrite[0] = startRegAddr+bytesWritten;    
            memcpy(&i2cWrite[1], &data[1+bytesWritten], thisLen);
            result = I2C_WriteBurst(sl_handle, slaveAddr, 1+thisLen, i2cWrite);
        }
        if (ML_SUCCESS != result)
            return result;
        bytesWritten += thisLen;
    }

    return ML_SUCCESS;
}



/**
 *  @brief  used to read multiple bytes of data from registers.
 *          This should be sent by I2C.
 *
 *  @param  slaveAddr       I2C slave address of device.
 *  @param  registerAddr    Register address to read.
 *  @param  length          Length of burst of data.
 *  @param  data            Pointer to block of data.
 *
 *  @return Zero if successful; an error code otherwise
 */
tMLError MLSLSerialRead( void *sl_handle,
                         unsigned char  slaveAddr,
                         unsigned char  registerAddr,
                         unsigned short length,
                         unsigned char  *data )
{
    tMLError result;
    uint_fast16_t bytesRead = 0;
    
    if (registerAddr==MPUREG_FIFO_R_W  ||  registerAddr==MPUREG_MEM_R_W) {
        return ML_ERROR_INVALID_PARAMETER;
    }
    while (bytesRead<length) {
        unsigned short thisLen = MIN(
            SERIAL_MAX_TRANSFER_SIZE, length-bytesRead
        );
        result = I2C_ReadBurst(sl_handle, slaveAddr, 
                               registerAddr+bytesRead, 
                               thisLen, &data[bytesRead]);
        if (ML_SUCCESS != result)
            return result;
        bytesRead += thisLen;
    }
    return ML_SUCCESS;
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
tMLError MLSLSerialWriteMem( void *sl_handle,
                             unsigned char slaveAddr,
                             unsigned short memAddr,
                             unsigned short length, 
                             unsigned char const *data )
{
    tMLError result;
    uint_fast16_t bytesWritten = 0;

    if ((memAddr & 0xFF) + length > MPU_MEM_BANK_SIZE) {
        MPL_LOGE("memory write length (%d B) extends beyond its limits (%d) "
                 "if started at location %d\n", 
                 length, MPU_MEM_BANK_SIZE, memAddr & 0xFF);
        return ML_ERROR_INVALID_PARAMETER;
    }
    while (bytesWritten<length) {
        unsigned short thisLen = MIN(
            SERIAL_MAX_TRANSFER_SIZE, length-bytesWritten
        );
        result = I2C_WriteMem(sl_handle, slaveAddr,
                              memAddr+bytesWritten, 
                              thisLen, &data[bytesWritten]);
        if (ML_SUCCESS != result)
            return result;
        bytesWritten += thisLen;
    }
    return ML_SUCCESS;
}



/**
 *  @brief  used to read multiple bytes of data from the memory.
 *          This should be sent by I2C.
 *
 *  @param  slaveAddr       I2C slave address of device.
 *  @param  memAddr         The location in the memory to read from.
 *  @param  length          Length of burst data.
 *  @param  data            Pointer to block of data.
 *
 *  @return Zero if successful; an error code otherwise
 */
tMLError MLSLSerialReadMem( void *sl_handle,
                            unsigned char  slaveAddr, 
                            unsigned short memAddr, 
                            unsigned short length, 
                            unsigned char *data )
{
    tMLError result;
    uint_fast16_t bytesRead = 0;

    if (memAddr & 0xFF + length > MPU_MEM_BANK_SIZE) {
        MPL_LOGE("memory read length (%d B) extends beyond its limits (%d) "
                 "if started at location %d\n", 
                 length, MPU_MEM_BANK_SIZE, memAddr & 0xFF);
        return ML_ERROR_INVALID_PARAMETER;
    }
    while (bytesRead<length) {
        unsigned short thisLen = MIN(
            SERIAL_MAX_TRANSFER_SIZE, length-bytesRead
        );
        result = I2C_ReadMem(sl_handle, slaveAddr, 
                             memAddr+bytesRead, 
                             thisLen, &data[bytesRead]);
        if (ML_SUCCESS != result)
            return result;
        bytesRead += thisLen;
    }
    return ML_SUCCESS;
}



/**
 *  @brief  used to write multiple bytes of data to the fifo.
 *          This should be sent by I2C.
 *
 *  @param  slaveAddr       I2C slave address of device.
 *  @param  length          Length of burst of data.
 *  @param  data            Pointer to block of data.
 *
 *  @return Zero if successful; an error code otherwise
 */
tMLError MLSLSerialWriteFifo( void *sl_handle,
                              unsigned char slaveAddr,
                              unsigned short length, 
                              unsigned char const *data )
{
    tMLError result;
    uint_fast16_t bytesWritten = 0;

    if (length>FIFO_HW_SIZE) {
        MPL_LOGE("maximum fifo write length is %d\n", FIFO_HW_SIZE);
        return ML_ERROR_INVALID_PARAMETER;
    }
    while (bytesWritten<length) {
        unsigned short thisLen = MIN(
            SERIAL_MAX_TRANSFER_SIZE, length-bytesWritten
        );
        result = I2C_WriteFifo(sl_handle, slaveAddr, 
                               thisLen, 
                               &data[bytesWritten]);
        if (ML_SUCCESS != result)
            return result;
        bytesWritten += thisLen;
    }
    return ML_SUCCESS;
}




/**
 *  @brief  used to read multiple bytes of data from the fifo.
 *          This should be sent by I2C.
 *
 *  @param  slaveAddr       I2C slave address of device.
 *  @param  length          Length of burst of data.
 *  @param  data            Pointer to block of data.
 *
 *  @return Zero if successful; an error code otherwise
 */
tMLError MLSLSerialReadFifo( void *sl_handle,
                             unsigned char  slaveAddr, 
                             unsigned short length, 
                             unsigned char *data )
{
    tMLError result;
    uint_fast16_t bytesRead = 0;
        
    if (length>FIFO_HW_SIZE) {
        MPL_LOGE("maximum fifo read length is %d\n", FIFO_HW_SIZE);
        return ML_ERROR_INVALID_PARAMETER;
    }
    while (bytesRead<length) {
        unsigned short thisLen = MIN(
            SERIAL_MAX_TRANSFER_SIZE, length-bytesRead
        );
        result = I2C_ReadFifo(sl_handle, slaveAddr,  
                              thisLen, &data[bytesRead]);
        if (ML_SUCCESS != result)
            return result;
        bytesRead += thisLen;
    }

    return ML_SUCCESS;
}



#if (defined(ML_USE_DMP_SIM) && !defined(ML_USE_DMP_SIM_POLL))

HANDLE ml_sim_mutex = 0;
int retry =0;
static pthread_t sim_thread;
static int exit_sim_thread = 0;

/**
 *  @internal
 */
void* pthread_poll(void *arg) 
{
    void MLSimHWDataInput(void);
    int res;

    usleep(200000);  //prevent hang at startup?
    while(!exit_sim_thread) {
        res = pthread_mutex_lock((pthread_mutex_t*)ml_sim_mutex);
        if(res == 0) {
            MLSimHWDataInput();
            pthread_mutex_unlock((pthread_mutex_t*)ml_sim_mutex);
        }
        usleep(4500); //about 200hz
    }
}



/**
 *  @internal
 */
void setup_polling(void)
{
    int ret;
    struct sigevent sigev;
    struct itimerspec itime;

    ml_sim_mutex = (HANDLE)malloc(sizeof(pthread_mutex_t));
    pthread_mutex_init((pthread_mutex_t*)ml_sim_mutex, NULL);
    pthread_create(&sim_thread, NULL, &pthread_poll, NULL);
}

/**
 *  @internal
 */
void cleanup_polling(void)
{
    exit_sim_thread = 1;
    pthread_join(sim_thread, NULL);
    pthread_mutex_destroy((pthread_mutex_t*)ml_sim_mutex);
    free((pthread_mutex_t*)ml_sim_mutex);
}

#endif

/**
 *  @}
 */


