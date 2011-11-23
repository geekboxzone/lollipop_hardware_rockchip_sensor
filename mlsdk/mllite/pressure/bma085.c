/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */

/**
 *  @defgroup   ACCELDL (Motion Library - Pressure Driver Layer)
 *  @brief      Provides the interface to setup and handle a pressure
 *              connected to the secondary I2C interface of the gyroscope.
 *
 *  @{
 *      @file   bma085.c
 *      @brief  Pressure setup and handling methods.
 */

/* ------------------ */
/* - Include Files. - */
/* ------------------ */

#ifdef __KERNEL__
#include <linux/module.h>
#endif

#include "mpu.h"
#include "mlos.h"
#include "mlsl.h"

/* --------------------- */
/* -    Variables.     - */
/* --------------------- */

/** this structure holds all device specific calibration parameters
*/
struct bmp085_calibration_param_t {
   short ac1;
   short ac2;
   short ac3;
   unsigned short ac4;
   unsigned short ac5;
   unsigned short ac6;
   short b1;
   short b2;
   short mb;
   short mc;
   short md;
   long param_b5;
};

struct bmp085_calibration_param_t cal_param;

#define PRESSURE_BMA085_PARAM_MG      3038        /* calibration parameter */
#define PRESSURE_BMA085_PARAM_MH     -7357        /* calibration parameter */
#define PRESSURE_BMA085_PARAM_MI      3791        /* calibration parameter */

/*********************************************
    Pressure Initialization Functions
**********************************************/

static int bma085_suspend(void *mlsl_handle,
			  struct ext_slave_descr *slave,
			  struct ext_slave_platform_data *pdata)
{
	int result = ML_SUCCESS;
	return result;
}

#define PRESSURE_BMA085_PROM_START_ADDR  (0xAA)
#define PRESSURE_BMA085_PROM_DATA_LEN    (22)
#define PRESSURE_BMP085_CTRL_MEAS_REG    (0xF4)
/* temperature measurent */
#define PRESSURE_BMP085_T_MEAS           (0x2E)
/* pressure measurement; oversampling_setting */
#define PRESSURE_BMP085_P_MEAS_OSS_0     (0x34)
#define PRESSURE_BMP085_P_MEAS_OSS_1     (0x74)
#define PRESSURE_BMP085_P_MEAS_OSS_2     (0xB4)
#define PRESSURE_BMP085_P_MEAS_OSS_3     (0xF4)
#define PRESSURE_BMP085_ADC_OUT_MSB_REG  (0xF6)
#define PRESSURE_BMP085_ADC_OUT_LSB_REG  (0xF7)

static int bma085_resume(void *mlsl_handle,
			 struct ext_slave_descr *slave,
			 struct ext_slave_platform_data *pdata)
{
	int result;
	unsigned char data[PRESSURE_BMA085_PROM_DATA_LEN];

	result =
	    MLSLSerialRead(mlsl_handle, pdata->address,
			   PRESSURE_BMA085_PROM_START_ADDR,
			   PRESSURE_BMA085_PROM_DATA_LEN, data);
	ERROR_CHECK(result);

	/* parameters AC1-AC6 */
	cal_param.ac1 = (data[0] << 8) | data[1];
	cal_param.ac2 = (data[2] << 8) | data[3];
	cal_param.ac3 = (data[4] << 8) | data[5];
	cal_param.ac4 = (data[6] << 8) | data[7];
	cal_param.ac5 = (data[8] << 8) | data[9];
	cal_param.ac6 = (data[10] << 8) | data[11];

	/* parameters B1,B2 */
	cal_param.b1 = (data[12] << 8) | data[13];
	cal_param.b2 = (data[14] << 8) | data[15];

	/* parameters MB,MC,MD */
	cal_param.mb = (data[16] << 8) | data[17];
	cal_param.mc = (data[18] << 8) | data[19];
	cal_param.md = (data[20] << 8) | data[21];

	return result;
}

static int bma085_read(void *mlsl_handle,
		       struct ext_slave_descr *slave,
		       struct ext_slave_platform_data *pdata,
		       unsigned char *data)
{
	int result;
	unsigned char reg = 0;
	long pressure, x1, x2, x3, b3, b6;
	unsigned long b4, b7;
	unsigned long up;
	unsigned short ut;
	short oversampling_setting = 0;
	short temperature;
	long divisor;

	/* get temprature */
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				       PRESSURE_BMP085_CTRL_MEAS_REG,
				       PRESSURE_BMP085_T_MEAS);
	MLOSSleep(5);
	result =
	    MLSLSerialRead(mlsl_handle, pdata->address,
			   PRESSURE_BMP085_ADC_OUT_MSB_REG, 2,
			   (unsigned char *)data);
	ERROR_CHECK(result);
	ut = (data[0] << 8) | data[1];

	x1 = (((long) ut - (long)cal_param.ac6) * (long)cal_param.ac5) >> 15;
	divisor = x1 + cal_param.md;
	if (!divisor)
		return ML_ERROR_DIVIDE_BY_ZERO;

	x2 = ((long)cal_param.mc << 11) / (x1 + cal_param.md);
	cal_param.param_b5 = x1 + x2;
	/* temperature in 0.1 degree C */
	temperature = (short)((cal_param.param_b5 + 8) >> 4);

	/* get pressure */
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				       PRESSURE_BMP085_CTRL_MEAS_REG,
				       PRESSURE_BMP085_P_MEAS_OSS_0);
	MLOSSleep(5);
	result =
	    MLSLSerialRead(mlsl_handle, pdata->address,
			   PRESSURE_BMP085_ADC_OUT_MSB_REG, 2,
			   (unsigned char *)data);
	ERROR_CHECK(result);
	up = (((unsigned long) data[0] << 8) | ((unsigned long) data[1]));

	b6 = cal_param.param_b5 - 4000;
	/* calculate B3 */
	x1 = (b6*b6) >> 12;
	x1 *= cal_param.b2;
	x1 >>= 11;

	x2 = (cal_param.ac2*b6);
	x2 >>= 11;

	x3 = x1 + x2;

	b3 = (((((long)cal_param.ac1) * 4 + x3)
	    << oversampling_setting) + 2) >> 2;

	/* calculate B4 */
	x1 = (cal_param.ac3 * b6) >> 13;
	x2 = (cal_param.b1 * ((b6*b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (cal_param.ac4 * (unsigned long) (x3 + 32768)) >> 15;
	if (!b4)
		return ML_ERROR;

	b7 = ((unsigned long)(up - b3) * (50000>>oversampling_setting));
	if (b7 < 0x80000000)
		pressure = (b7 << 1) / b4;
	else
		pressure = (b7 / b4) << 1;

	x1 = pressure >> 8;
	x1 *= x1;
	x1 = (x1 * PRESSURE_BMA085_PARAM_MG) >> 16;
	x2 = (pressure * PRESSURE_BMA085_PARAM_MH) >> 16;
	/* pressure in Pa */
	pressure += (x1 + x2 + PRESSURE_BMA085_PARAM_MI) >> 4;

	data[0] = (unsigned char)(pressure >> 16);
	data[1] = (unsigned char)(pressure >> 8);
	data[2] = (unsigned char)(pressure & 0xFF);

	return result;
}

static struct ext_slave_descr bma085_descr = {
	/*.init             = */ NULL,
	/*.exit             = */ NULL,
	/*.suspend          = */ bma085_suspend,
	/*.resume           = */ bma085_resume,
	/*.read             = */ bma085_read,
	/*.config           = */ NULL,
	/*.name             = */ "bma085",
	/*.type             = */ EXT_SLAVE_TYPE_PRESSURE,
	/*.id               = */ PRESSURE_ID_BMA085,
	/*.reg              = */ 0xF6,
	/*.len              = */ 3,
	/*.endian           = */ EXT_SLAVE_BIG_ENDIAN,
	/*.range            = */ {0, 0},
};

struct ext_slave_descr *bma085_get_slave_descr(void)
{
	return &bma085_descr;
}
EXPORT_SYMBOL(bma085_get_slave_descr);

#ifdef __KERNEL__
MODULE_AUTHOR("Invensense");
MODULE_DESCRIPTION("User space IRQ handler for MPU3xxx devices");
MODULE_LICENSE("GPL");
MODULE_ALIAS("bma");
#endif

/**
 *  @}
**/
