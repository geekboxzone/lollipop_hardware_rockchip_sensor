/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
#ifndef INVENSENSE_ML_FIFO_HW_H__
#define INVENSENSE_ML_FIFO_HW_H__

#include "mpu.h"
#include "mltypes.h"
#include "mlinclude.h"

#ifdef __cplusplus
extern "C" {
#endif

    // This is the maximum amount of FIFO data we would read in one packet
#define MAX_FIFO_LENGTH             (256)
    // This is the hardware size of the FIFO
#define FIFO_FOOTER_SIZE            (2)


    uint_fast16_t  MPUGetFIFO(uint_fast16_t length, unsigned char* buffer);
    tMLError       MLDLGetFIFOStatus( void );
    tMLError       MLDLGetFifoLength(uint_fast16_t * len);
    short          MLDLGetFIFOCount( void );
    tMLError       FIFOReset(void);
    void           FIFOHWInit();
    tMLError       MLDLReadFifo(unsigned char *data, uint_fast16_t len);

#ifdef __cplusplus
}
#endif

#endif // INVENSENSE_ML_FIFO_HW_H__
