/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */

/******************************************************************************
 *
 * $Id: mldl_cfg_mpu.c 5144 2011-04-05 18:09:41Z mcaramello $
 *
 *****************************************************************************/

/** 
 *  @addtogroup MLDL
 *
 *  @{
 *      @file   mldl_cfg_mpu.c
 *      @brief  The Motion Library Driver Layer.
 */

/* ------------------ */
/* - Include Files. - */
/* ------------------ */

#include <stddef.h>
#include "mldl_cfg.h"
#include "mlsl.h"
#include "mpu.h"

#ifdef LINUX
#include <sys/ioctl.h>
#endif

#include "log.h"
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-mldl_cfg_mpu:"

/* --------------------- */
/* -    Variables.     - */
/* --------------------- */


/* ---------------------- */
/* -  Static Functions. - */
/* ---------------------- */
void mpu_print_cfg(struct mldl_cfg * mldl_cfg)
{
    struct mpu3050_platform_data   *pdata   = mldl_cfg->pdata;
    struct ext_slave_platform_data *accel   = &mldl_cfg->pdata->accel;
    struct ext_slave_platform_data *compass = &mldl_cfg->pdata->compass;
    struct ext_slave_platform_data *pressure = &mldl_cfg->pdata->pressure;

    MPL_LOGD("mldl_cfg.addr             = %02x\n", mldl_cfg->addr);
    MPL_LOGD("mldl_cfg.int_config       = %02x\n", mldl_cfg->int_config);
    MPL_LOGD("mldl_cfg.ext_sync         = %02x\n", mldl_cfg->ext_sync);
    MPL_LOGD("mldl_cfg.full_scale       = %02x\n", mldl_cfg->full_scale);
    MPL_LOGD("mldl_cfg.lpf              = %02x\n", mldl_cfg->lpf);
    MPL_LOGD("mldl_cfg.clk_src          = %02x\n", mldl_cfg->clk_src);
    MPL_LOGD("mldl_cfg.divider          = %02x\n", mldl_cfg->divider);
    MPL_LOGD("mldl_cfg.dmp_enable       = %02x\n", mldl_cfg->dmp_enable);
    MPL_LOGD("mldl_cfg.fifo_enable      = %02x\n", mldl_cfg->fifo_enable);
    MPL_LOGD("mldl_cfg.dmp_cfg1         = %02x\n", mldl_cfg->dmp_cfg1);
    MPL_LOGD("mldl_cfg.dmp_cfg2         = %02x\n", mldl_cfg->dmp_cfg2);
    MPL_LOGD("mldl_cfg.offset_tc[0]     = %02x\n", mldl_cfg->offset_tc[0]);
    MPL_LOGD("mldl_cfg.offset_tc[1]     = %02x\n", mldl_cfg->offset_tc[1]);
    MPL_LOGD("mldl_cfg.offset_tc[2]     = %02x\n", mldl_cfg->offset_tc[2]);
    MPL_LOGD("mldl_cfg.silicon_revision = %02x\n", mldl_cfg->silicon_revision);
    MPL_LOGD("mldl_cfg.product_id       = %02x\n", mldl_cfg->product_id);
    MPL_LOGD("mldl_cfg.trim             = %02x\n", mldl_cfg->trim);

    if (mldl_cfg->accel) {
        MPL_LOGD("slave_accel->suspend      = %02x\n", (int)mldl_cfg->accel->suspend);
        MPL_LOGD("slave_accel->resume       = %02x\n", (int)mldl_cfg->accel->resume);
        MPL_LOGD("slave_accel->read         = %02x\n", (int)mldl_cfg->accel->read);
        MPL_LOGD("slave_accel->type         = %02x\n", mldl_cfg->accel->type);
        MPL_LOGD("slave_accel->reg          = %02x\n", mldl_cfg->accel->reg);
        MPL_LOGD("slave_accel->len          = %02x\n", mldl_cfg->accel->len);
        MPL_LOGD("slave_accel->endian       = %02x\n", mldl_cfg->accel->endian);
        MPL_LOGD("slave_accel->range.mantissa= %02x\n", (int)mldl_cfg->accel->range.mantissa);
        MPL_LOGD("slave_accel->range.fraction= %02x\n", (int)mldl_cfg->accel->range.fraction);
    } else {
        MPL_LOGD("slave_accel               = NULL\n");
    }

    if (mldl_cfg->compass) {
        MPL_LOGD("slave_compass->suspend    = %02x\n", (int)mldl_cfg->compass->suspend);
        MPL_LOGD("slave_compass->resume     = %02x\n", (int)mldl_cfg->compass->resume);
        MPL_LOGD("slave_compass->read       = %02x\n", (int)mldl_cfg->compass->read);
        MPL_LOGD("slave_compass->type       = %02x\n", mldl_cfg->compass->type);
        MPL_LOGD("slave_compass->reg        = %02x\n", mldl_cfg->compass->reg);
        MPL_LOGD("slave_compass->len        = %02x\n", mldl_cfg->compass->len);
        MPL_LOGD("slave_compass->endian     = %02x\n", mldl_cfg->compass->endian);
        MPL_LOGD("slave_compass->range.mantissa= %02x\n", (int)mldl_cfg->compass->range.mantissa);
        MPL_LOGD("slave_compass->range.fraction= %02x\n", (int)mldl_cfg->compass->range.fraction);
    } else {
        MPL_LOGD("slave_compass             = NULL\n");
    }

    if (mldl_cfg->pressure) {
        MPL_LOGD("slave_pressure->suspend    = %02x\n", (int)mldl_cfg->pressure->suspend);
        MPL_LOGD("slave_pressure->resume     = %02x\n", (int)mldl_cfg->pressure->resume);
        MPL_LOGD("slave_pressure->read       = %02x\n", (int)mldl_cfg->pressure->read);
        MPL_LOGD("slave_pressure->type       = %02x\n", mldl_cfg->pressure->type);
        MPL_LOGD("slave_pressure->reg        = %02x\n", mldl_cfg->pressure->reg);
        MPL_LOGD("slave_pressure->len        = %02x\n", mldl_cfg->pressure->len);
        MPL_LOGD("slave_pressure->endian     = %02x\n", mldl_cfg->pressure->endian);
        MPL_LOGD("slave_pressure->range.mantissa= %02x\n", (int)mldl_cfg->pressure->range.mantissa);
        MPL_LOGD("slave_pressure->range.fraction= %02x\n", (int)mldl_cfg->pressure->range.fraction);
    } else {
        MPL_LOGD("slave_pressure             = NULL\n");
    }

    MPL_LOGD("accel->get_slave_descr    = %x\n",(unsigned int) accel->get_slave_descr);
    MPL_LOGD("accel->adapt_num          = %02x\n", accel->adapt_num);
    MPL_LOGD("accel->bus                = %02x\n", accel->bus);
    MPL_LOGD("accel->address            = %02x\n", accel->address);
    MPL_LOGD("accel->orientation        = \n"
             "                            %2d %2d %2d\n"
             "                            %2d %2d %2d\n"
             "                            %2d %2d %2d\n",
             accel->orientation[0],accel->orientation[1],accel->orientation[2],
             accel->orientation[3],accel->orientation[4],accel->orientation[5],
             accel->orientation[6],accel->orientation[7],accel->orientation[8]);
    MPL_LOGD("compass->get_slave_descr  = %x\n",(unsigned int) compass->get_slave_descr);
    MPL_LOGD("compass->adapt_num        = %02x\n", compass->adapt_num);
    MPL_LOGD("compass->bus              = %02x\n", compass->bus);
    MPL_LOGD("compass->address          = %02x\n", compass->address);
    MPL_LOGD("compass->orientation      = \n"
             "                            %2d %2d %2d\n"
             "                            %2d %2d %2d\n"
             "                            %2d %2d %2d\n",
             compass->orientation[0],compass->orientation[1],compass->orientation[2],
             compass->orientation[3],compass->orientation[4],compass->orientation[5],
             compass->orientation[6],compass->orientation[7],compass->orientation[8]);
    MPL_LOGD("pressure->get_slave_descr  = %x\n",(unsigned int) pressure->get_slave_descr);
    MPL_LOGD("pressure->adapt_num        = %02x\n", pressure->adapt_num);
    MPL_LOGD("pressure->bus              = %02x\n", pressure->bus);
    MPL_LOGD("pressure->address          = %02x\n", pressure->address);
    MPL_LOGD("pressure->orientation      = \n"
             "                            %2d %2d %2d\n"
             "                            %2d %2d %2d\n"
             "                            %2d %2d %2d\n",
             pressure->orientation[0],pressure->orientation[1],pressure->orientation[2],
             pressure->orientation[3],pressure->orientation[4],pressure->orientation[5],
             pressure->orientation[6],pressure->orientation[7],pressure->orientation[8]);
    
    MPL_LOGD("pdata->int_config         = %02x\n", pdata->int_config);
    MPL_LOGD("pdata->level_shifter      = %02x\n", pdata->level_shifter);
    MPL_LOGD("pdata->orientation        = \n"
             "                            %2d %2d %2d\n"
             "                            %2d %2d %2d\n"
             "                            %2d %2d %2d\n",
             pdata->orientation[0],pdata->orientation[1],pdata->orientation[2],
             pdata->orientation[3],pdata->orientation[4],pdata->orientation[5],
             pdata->orientation[6],pdata->orientation[7],pdata->orientation[8]);

    MPL_LOGD("Struct sizes: mldl_cfg: %d, "
             "ext_slave_descr:%d, mpu3050_platform_data:%d: RamOffset: %d\n", 
             sizeof(struct mldl_cfg), sizeof(struct ext_slave_descr), 
             sizeof(struct mpu3050_platform_data), 
             offsetof(struct mldl_cfg, ram));
}

/******************************************************************************
 ******************************************************************************
 * Exported functions
 ******************************************************************************
 *****************************************************************************/

/** 
 * Initializes the pdata structure to defaults.
 *
 * Opens the device to read silicon revision, product id and whoami.  Leaves
 * the device in suspended state for low power.
 * 
 * @param mldl_cfg handle to the config structure
 * @param mlsl_handle handle to the mpu serial layer
 * @param accel_handle handle to the accel serial layer
 * @param compass_handle handle to the compass serial layer
 * @param pressure_handle handle to the pressure serial layer
 *
 * @return ML_SUCCESS if silicon revision, product id and woami are supported
 *         by this software.
 */
int mpu3050_open(struct mldl_cfg *mldl_cfg, 
                 void *mlsl_handle,
                 void *accel_handle,
                 void *compass_handle,
                 void *pressure_handle)
{
    int result;
    result = ioctl((int)mlsl_handle, MPU_GET_MPU_CONFIG, mldl_cfg);

    result = mpu3050_suspend(mldl_cfg, mlsl_handle, NULL, NULL, NULL,
                             TRUE, TRUE, TRUE, TRUE);
    ERROR_CHECK(result);
    return result;
}

/** 
 * Stub for driver close.  Just verify that the devices are suspended
 * 
 * @param mldl_cfg handle to the config structure
 * @param mlsl_handle handle to the mpu serial layer
 * @param accel_handle handle to the accel serial layer
 * @param compass_handle handle to the compass serial layer
 * @param pressure_handle handle to the compass serial layer
 * 
 * @return ML_SUCCESS or non-zero error code
 */
int mpu3050_close(struct mldl_cfg *mldl_cfg, 
		  void *mlsl_handle,
		  void *accel_handle,
		  void *compass_handle,
		  void *pressure_handle)
{
    int result = ML_SUCCESS;

    result = mpu3050_suspend(mldl_cfg, mlsl_handle, NULL, NULL, NULL,
                             TRUE, TRUE, TRUE, TRUE);
    return result;
}

int mpu3050_resume(struct mldl_cfg* mldl_cfg, 
                   void *mlsl_handle, 
                   void *accel_handle, 
                   void *compass_handle, 
                   void *pressure_handle, 
                   bool resume_gyro,
                   bool resume_accel,
                   bool resume_compass,
                   bool resume_pressure)
{
    int result;
    
    //mpu_print_cfg(mldl_cfg);
    result = ioctl((int)mlsl_handle, MPU_SET_MPU_CONFIG, mldl_cfg);
    ERROR_CHECK(result);
    result = ioctl((int)mlsl_handle, MPU_RESUME, NULL);
    ERROR_CHECK(result);
    result = ioctl((int)mlsl_handle, MPU_GET_MPU_CONFIG, mldl_cfg);
    ERROR_CHECK(result);
    MPL_LOGI("%s: Resuming to %04lx\n", __func__, mldl_cfg->requested_sensors);

    return result;
}


int mpu3050_suspend(struct mldl_cfg *mldl_cfg, 
          	    void *mlsl_handle,
                    void *accel_handle,
                    void *compass_handle,
                    void *pressure_handle,
 		    bool suspend_gyro,
                    bool suspend_accel,
                    bool suspend_compass,
                    bool suspend_pressure)
{
    int result;
    unsigned long sensors = (ML_THREE_AXIS_GYRO |
                             ML_THREE_AXIS_ACCEL |
                             ML_THREE_AXIS_COMPASS |
                             ML_THREE_AXIS_PRESSURE);
    unsigned long requested = mldl_cfg->requested_sensors;

    if (suspend_gyro)
        sensors &= ~ML_THREE_AXIS_GYRO;
    if (suspend_accel)
        sensors &= ~ML_THREE_AXIS_ACCEL;
    if (suspend_compass)
        sensors &= ~ML_THREE_AXIS_COMPASS;
    if (suspend_pressure)
        sensors &= ~ML_THREE_AXIS_PRESSURE;

    MPL_LOGI("%s: suspending sensors to %04lx\n", __func__, sensors);
    mldl_cfg->requested_sensors = sensors;

    result = ioctl((int)mlsl_handle, MPU_SET_MPU_CONFIG, mldl_cfg);
    ERROR_CHECK(result);
    result = ioctl((int)mlsl_handle, MPU_SUSPEND, NULL);
    ERROR_CHECK(result);
    result = ioctl((int)mlsl_handle, MPU_GET_MPU_CONFIG, mldl_cfg);
    ERROR_CHECK(result);

    mldl_cfg->requested_sensors = requested;
    MPL_LOGI("%s: Will resume next to %04lx\n", __func__, requested);

    return result;
}

int mpu3050_read_accel(struct mldl_cfg *mldl_cfg, void *mlsl_handle,
                       unsigned char * data)
{
    int result;
    result = ioctl((int)mlsl_handle, MPU_READ_ACCEL, data);
    return result;
}

int mpu3050_read_compass(struct mldl_cfg *mldl_cfg, void *mlsl_handle,
                         unsigned char * data)
{
    int result;
    result = ioctl((int)mlsl_handle, MPU_READ_COMPASS, data);
    return result;
}

int mpu3050_read_pressure(struct mldl_cfg *mldl_cfg, void *mlsl_handle,
                         unsigned char * data)
{
    int result;
    result = ioctl((int)mlsl_handle, MPU_READ_PRESSURE, data);
    return result;
}

/**
 * Request slave to change configuration
 *
 * @param mldl_cfg pointer to the mldl configuration structure
 * @param accel_handle handle to the accel sensor
 * @param data the data being requested.
 *
 * @return 0 or non-zero error code
 */
int mpu3050_config_accel(struct mldl_cfg *mldl_cfg,
                         void *accel_handle,
                         struct ext_slave_config *data)
{
    int result;
    result = ioctl((int)accel_handle, MPU_CONFIG_ACCEL, data);
    return result;
}

/**
 * Request slave to change configuration
 *
 * @param mldl_cfg pointer to the mldl configuration structure
 * @param compass_handle handle to the compass sensor
 * @param data the data being requested.
 *
 * @return 0 or non-zero error code
 */
int mpu3050_config_compass(struct mldl_cfg *mldl_cfg,
                           void *compass_handle,
                           struct ext_slave_config *data)
{
    int result;
    result = ioctl((int)compass_handle, MPU_CONFIG_COMPASS, data);
    return result;
}

/**
 * Request slave to change configuration
 *
 * @param mldl_cfg pointer to the mldl configuration structure
 * @param pressure_handle handle to the pressure sensor
 * @param data the data being requested.
 *
 * @return 0 or non-zero error code
 */
int mpu3050_config_pressure(struct mldl_cfg *mldl_cfg, 
                            void *pressure_handle,
                            struct ext_slave_config *data)
{
    int result;
    result = ioctl((int)pressure_handle, MPU_CONFIG_PRESSURE, data);
    return result;
}

/**
 * Request slave configuration information
 *
 * Use this specifically after requesting a slave configuration to see what the
 * slave accually accepted.
 *
 * @param mldl_cfg pointer to the mldl configuration structure
 * @param accel_handle handle to the accel sensor
 * @param data the data being requested.
 *
 * @return 0 or non-zero error code
 */
int mpu3050_get_config_accel(struct mldl_cfg *mldl_cfg,
                             void *accel_handle,
                             struct ext_slave_config *data)
{
    int result;
    result = ioctl((int)accel_handle, MPU_GET_CONFIG_ACCEL, data);
    return result;
}

/** 
 * Request slave configuration information
 *
 * Use this specifically after requesting a slave configuration to see what the
 * slave accually accepted.
 *
 * @param mldl_cfg pointer to the mldl configuration structure
 * @param compass_handle handle to the compass sensor
 * @param data the data being requested.
 *
 * @return 0 or non-zero error code
 */
int mpu3050_get_config_compass(struct mldl_cfg *mldl_cfg,
                               void *compass_handle,
                               struct ext_slave_config *data)
{
    int result;
    result = ioctl((int)compass_handle, MPU_GET_CONFIG_COMPASS, data);
    return result;
}

/** 
 * Request slave configuration information
 *
 * Use this specifically after requesting a slave configuration to see what the
 * slave accually accepted.
 *
 * @param mldl_cfg pointer to the mldl configuration structure
 * @param pressure_handle handle to the pressure sensor
 * @param data the data being requested.
 *
 * @return 0 or non-zero error code
 */
int mpu3050_get_config_pressure(struct mldl_cfg *mldl_cfg, 
                                void *pressure_handle,
                                struct ext_slave_config *data)
{
    int result;
    result = ioctl((int)pressure_handle, MPU_GET_CONFIG_PRESSURE, data);
    return result;
}

/**
 *@}
 */
