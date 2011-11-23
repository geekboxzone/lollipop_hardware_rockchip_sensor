/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: mldl.h 5018 2011-03-17 02:00:49Z nroyer $
 *
 *******************************************************************************/

#ifndef MLDL_H
#define MLDL_H

#include "mltypes.h"
#include "mlsl.h"
#include "mpu.h"
#include "mldl_cfg.h"

    /* ------------ */
    /* - Defines. - */
    /* ------------ */


    typedef enum _DEVICE_CONFIG
    {
        DEVICE_MPU_ACCEL = 0,
        DEVICE_MPU,
        NUM_OF_DEVICES
    } DEVICE_CONFIG;

#define SERIAL_I2C                  0
#define SERIAL_SPI                  1

#define DATAMODE_MANUAL             0       // Manual data mode
#define DATAMODE_AUTO               1       // Auto data mode

#define DATASRC_IMMEDIATE           0       // Return data immediately
#define DATASRC_WHENREADY           1       // Only return data when new data is available
#define DATASRC_FIFO                2       // Use FIFO for data

#define SENSOR_DATA_GYROX           0x0001
#define SENSOR_DATA_GYROY           0x0002
#define SENSOR_DATA_GYROZ           0x0004
#define SENSOR_DATA_ACCELX          0x0008
#define SENSOR_DATA_ACCELY          0x0010
#define SENSOR_DATA_ACCELZ          0x0020
#define SENSOR_DATA_AUX1            0x0040
#define SENSOR_DATA_AUX2            0x0080
#define SENSOR_DATA_AUX3            0x0100
#define SENSOR_DATA_TEMP            0x0200

#define INTPIN_MPU                  0

#define INTLOGIC_HIGH               0
#define INTLOGIC_LOW                1

#define INTTYPE_PUSHPULL            0
#define INTTYPE_OPENDRAIN           1

#define INTLATCH_DISABLE            0
#define INTLATCH_ENABLE             1

#define MPUINT_MPU_READY            0x04
#define MPUINT_DMP_DONE             0x02
#define MPUINT_DATA_READY           0x01

#define INTLATCHCLEAR_READSTATUS    0
#define INTLATCHCLEAR_ANYREAD       1

#define DMP_DONTRUN                 0
#define DMP_RUN                     1

    /*---- defines for external interrupt status (via external call into library) ----*/
#define INT_CLEAR                   0
#define INT_TRIGGERED               1

typedef enum {
    INTSRC_MPU = 0,
    INTSRC_AUX1,
    INTSRC_AUX2,
    INTSRC_AUX3,
    INTSRC_TIMER,
    INTSRC_UNKNOWN,
    INTSRC_MPU_FIFO,
    INTSRC_MPU_MOTION,
    NUM_OF_INTSOURCES,
} INT_SOURCE;

    /* --------------- */
    /* - Structures. - */
    /* --------------- */

    /* --------------- */
    /* - Variables.  - */
    /* --------------- */

    /* --------------------- */
    /* - Function p-types. - */
    /* --------------------- */
#ifdef __cplusplus
extern "C" {
#endif

    tMLError MLDLOpen(void *mlslHandle);
    tMLError MLDLClose(void);

    tMLError MLDLDmpStart(unsigned long sensors);
    tMLError MLDLDmpStop(unsigned long sensors);

    struct mldl_cfg* MLDLGetCfg(void);

    unsigned char MLDLGetMPUSlaveAddr(void);
    tMLError MLDLCtrlDmp(unsigned char enableRun,  
                         unsigned char enableFIFO);
    tMLError MLDLCfgInt(unsigned char triggers);
    tMLError MLDLCfgSamplingMPU(unsigned char lpf, 
                                unsigned char divider);
    tMLError MLDLSetFullScaleMPU(float fullScale);
    tMLError MLDLSetExternalSyncMPU(unsigned char extSync);
    tMLError MLDLSetIgnoreSystemSuspend(unsigned char ignore);
    tMLError MLDLSetGyroPower(unsigned long xOn, 
                              unsigned long yOn, 
                              unsigned long zOn);
    tMLError MLDLClockSource(unsigned char clkSource);
    tMLError MLDLGetMemoryMPU(unsigned short key, 
                              unsigned short length, 
                              unsigned char* buffer);
    tMLError MLDLSetMemoryMPU(unsigned short key, 
                              unsigned short length, 
                              const unsigned char* buffer);
    tMLError MLDLLoadDMP( const unsigned char *buffer, 
                          unsigned short length, 
                          unsigned short startAddress );

    unsigned char MLDLGetSiliconRev(void);
    tMLError MLDLSetOffsetTC(unsigned char const *tc);
    tMLError MLDLSetOffset(unsigned short const *offset);

    /* Functions for setting and retrieving the DMP memory */
    tMLError  MLDLGetMemoryMPUOriginal(unsigned short key, 
                                       unsigned short length, 
                                       unsigned char* buffer);
    void MLDLSetGetAddress(unsigned short (*func)(unsigned short key));
    unsigned short MLDLGetAddress(unsigned short key);
    uint_fast8_t DmpFeatureSupported(unsigned short key);

    tMLError  MLDLGetIntStatus(unsigned char intPin, unsigned char *value);
    unsigned char MLDLGetIntTrigger(unsigned char index);
    void MLDLClearIntTrigger(unsigned char index);
    tMLError MLDLIntHandler(unsigned char intSource);


    /** Only exposed for testing purposes */
    tMLError MLDLSetMemoryMPUOneBank( unsigned char bank, 
                                  unsigned short memAddr, 
                                  unsigned short length, 
                                  unsigned char const* buffer );
    tMLError MLDLGetMemoryMPUOneBank( unsigned char  bank,   
                         unsigned char  memAddr, 
                         unsigned short length, 
                         unsigned char* buffer );
#ifdef __cplusplus
}
#endif

#endif // MLDL_H 

