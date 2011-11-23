/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */

#ifndef INVENSENSE_ML_FIFO_H__
#define INVENSENSE_ML_FIFO_H__

#include "mltypes.h"
#include "mlinclude.h"
#include "ml.h"

#ifdef __cplusplus
extern "C" {
#endif

    /**************************************************************************/
    /*  Elements                                                              */
    /**************************************************************************/

#define ML_ELEMENT_1                    (0x0001)
#define ML_ELEMENT_2                    (0x0002)
#define ML_ELEMENT_3                    (0x0004)
#define ML_ELEMENT_4                    (0x0008)
#define ML_ELEMENT_5                    (0x0010)
#define ML_ELEMENT_6                    (0x0020)
#define ML_ELEMENT_7                    (0x0040)
#define ML_ELEMENT_8                    (0x0080)

#define ML_ALL                          (0xFFFF)
#define ML_ELEMENT_MASK                 (0x00FF)

    /**************************************************************************/
    /*  Accuracy                                                              */
    /**************************************************************************/

#define ML_16_BIT                       (0x0100)
#define ML_32_BIT                       (0x0200)
#define ML_ACCURACY_MASK                (0x0300)

    /**************************************************************************/
    /*  Accuracy                                                              */
    /**************************************************************************/

#define ML_GYRO_FROM_RAW                (0x00)
#define ML_GYRO_FROM_QUATERNION         (0x01)

    /**************************************************************************/
    /*  Prototypes                                                            */
    /**************************************************************************/

    tMLError MLSetFIFORate(unsigned short fifoRate);
    unsigned short MLGetFIFORate(void);
    void SetSampleStepSizeMs(int_fast16_t ms);
    int_fast16_t GetSampleStepSizeMs(void);
    int_fast16_t GetSampleFrequencyHz(void);

    // Register callbacks after a packet of FIFO data is processed
    tMLError RegisterHighRateProcess( tMlxdataFunction func );
    tMLError UnRegisterHighRateProcess( tMlxdataFunction func );
    tMLError RunHighRateProcessFuncs(void);
    
    // Setup FIFO for various output
    tMLError FIFOSendQuaternion( uint_fast16_t accuracy );
    tMLError FIFOSendGyro(uint_fast16_t elements, uint_fast16_t accuracy);
    tMLError FIFOSendAccel(uint_fast16_t elements, uint_fast16_t accuracy);
    tMLError FIFOSendLinearAccel(uint_fast16_t elements,
                                 uint_fast16_t accuracy);
    tMLError FIFOSendLinearAccelWorld(uint_fast16_t elements,
                                      uint_fast16_t accuracy);
    tMLError FIFOSendControlData(uint_fast16_t elements,
                                 uint_fast16_t accuracy);
    tMLError FIFOSendRaw(uint_fast16_t elements, uint_fast16_t accuracy);
    tMLError FIFOSendRawExternal(uint_fast16_t elements,
                                 uint_fast16_t accuracy);
    tMLError FIFOSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
    tMLError FIFOSendDMPPacketNumber(uint_fast16_t accuracy);
    tMLError FIFOSendQuantAccel(uint_fast16_t elements, uint_fast16_t accuracy);
    tMLError FIFOSendEis(uint_fast16_t elements, uint_fast16_t accuracy);

    // Get Fixed Point data from FIFO
    tMLError FIFOGetAccel(long *data);
    tMLError FIFOGetQuaternion(long *data);
    tMLError FIFOGetQuaternion6Axis(long *data);
    tMLError FIFOGetRelativeQuaternion(long *data);
    tMLError FIFOGetGyro(long *data);
    tMLError FIFOSetLinearAccelFilterCoef(float coef);
    tMLError FIFOGetLinearAccel(long *data);
    tMLError FIFOGetLinearAccelWorld(long *data);
    tMLError FIFOGetControlData(long *data);
    tMLError FIFOGetSensorData(long *data);
    tMLError FIFOGetSensorGyroData(long *data);
    tMLError FIFOGetTemperature(long *data);
    tMLError FIFOGetGravBody(long *data);
    tMLError FIFOGetDecodedAccel(long *data);
    tMLError FIFOGetQuantAccel(long *data);
    tMLError FIFOGetExternalSensorData(long *data);
    tMLError FIFOGetEis(long *data);

    // Get Floating Point data from FIFO
    tMLError FIFOGetAccelFloat(float *data);
    tMLError FIFOGetQuaternionFloat(float *data);

    tMLError MLProcessFIFOData(const unsigned char *dmpData);
    tMLError readAndProcessFIFO(int_fast8_t numPackets, int_fast8_t *processed);

    tMLError MLSetProcessedDataCallback(void (*func)(void) );

    tMLError FIFOParamInit(void);
    tMLError FIFOClose(void);
    tMLError FIFOSetGyroDataSource(uint_fast8_t source);
    tMLError FIFODecodeQuantAccel(void);
    unsigned long getGyroMagSqrd(void);
    unsigned long getAccMagSqrd(void);
    void overRideQuaternion( float *q );

    uint_fast16_t FIFOGetPacketSize(void);
#ifdef __cplusplus
}
#endif

#endif // INVENSENSE_ML_FIFO_H__
