/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: mleis.h 5044 2011-03-18 23:28:37Z nroyer $
 * 
 ******************************************************************************/

/** 
 *  @defgroup MLEIS
 *  @brief  The Motion Library Electronic Image Stabalization library process 
 *          gyroscope data and provides a pixel shift that is the result of 
 *          hand jitter.
 *
 *  @{
 *      @file mleis.h
 *      @brief Header file for the Motion Library Electronic Image Stabilization.
 */
#ifndef MLEIS_H
#define MLEIS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "mltypes.h"

/* ------------ */
/* - Defines. - */
/* ------------ */

/* - Module defines. - */
#define ML_EIS_MAX_DIM (3)
/* Two possible correction stages */
#define EIS_STAGE_STATIONARY (0)
#define EIS_STAGE_TILT_PAN   (1)
/* Value to set the MLEisSetTransitionTime to not transition */
#define EIS_PAN_SETTLE_STATIONARY (0xFFFFFFFF)

/******************************************************************************/
/*  Data Source                                                               */
/******************************************************************************/
#define ML_EIS_DATA_FIFO                (1)
#define ML_EIS_DATA_POLL                (2)

/******************************************************************************/
/*  Interrupt Source                                                          */
/******************************************************************************/
#define ML_INT_EIS                      (0x10)

/******************************************************************************/
/*  Accuracy                                                                  */
/******************************************************************************/
#define ML_EIS_16_BIT                   (0x4000)
#define ML_EIS_32_BIT                   (0x8000)

#define ML_FIFO_FSYNC                   (0x0008)

#define ML_EIS_SET_PIXEL_SHIFT          (0x0040)
#define ML_EIS_SET_RAW_DATA             (0x0080)
#define ML_EIS_SET_FOOTER               (0x8000)

/******************************************************************************/
/*  Flags                                                                     */
/******************************************************************************/
#define ML_EIS_RAW_DATA_READY           (0x0001)
#define ML_EIS_PROCESSED_DATA_READY     (0x0002)

/******************************************************************************/
/*  FIFO Styles for MLEisSetFiforate                                          */
/******************************************************************************/
#define ML_EIS_FIFO_FULL                (0)
#define ML_EIS_FIFO_FSYNC               (33)

/* --------------- */
/* - Structures. - */
/* --------------- */

/** Pixel shift data for EIS */
typedef struct {
    int fsync;
    long mShiftAngleLsb[ML_EIS_MAX_DIM];

} tMlEisData;

/** Pointer to an EIS Callback function that is passed a tEisData pointer */
typedef void (*tMlEisCallback)(tMlEisData * data);

/* --------------------- */
/* - Function p-types. - */
/* --------------------- */

tMLError MLDmpEisOpen(void);
tMLError MLDmpEisStart(void);
tMLError MLDmpEisStop(void);
tMLError MLDmpEisClose(void);

/*API for handling the buffer*/
int MLEisUpdateData( void );

/*API for handling polling*/
int MLEisDataReady(void);

/* API For handling the callback */
int MLEisSetCallback(tMlEisCallback eisCallback);

/* Functions for handling augmented data */
int MLEisGetArray(long *data);
int MLEisGetFloatArray(float *data);

/*API for configuring MLEIS interrupt and data mode*/
int MLEisSetInterrupt(int on);
int MLEisSetFifoRate(int fifoRate);

/* Setting of parameters that control the behavior */
int MLEisSetRollOff(float frequency, int stage);
int MLEisSetTransitionTime(unsigned int timeMs);
int MLEisSetClipThreshold(int axis, float deg);

#ifdef __cplusplus
}
#endif

#endif /* MLEIS_H */
