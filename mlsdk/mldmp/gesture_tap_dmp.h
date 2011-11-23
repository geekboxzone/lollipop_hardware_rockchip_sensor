/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 * $Id: gesture_tap_dmp.h 3863 2010-10-08 22:05:31Z nroyer $ 
 ******************************************************************************/
/**
 * @defgroup ML_GESTUREX
 * @brief  Propritary gesture algorithms
 *
 * @{
 * @file     gesture_tap_dmp.h
 * @brief    Tap Algorithm using the DMP
 *
 * Algorithm for detecting tap and multitap using the DMP.
 */

#ifndef GESTURE_TAP_DMP_H
#define GESTURE_TAP_DMP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint_invensense.h"
#include "mltypes.h"
#include "ml.h"
#include "gesture.h"

/* ------------ */
/* - Defines. - */
/* ------------ */

/** Minimum time between taps in ms*/
#define ML_GSTR_MULTI_TAP_DEFAULT_IDLE (160)
/** Maximum time between taps in ms*/
#define ML_GSTR_MULTI_TAP_DEFAULT_WAIT (1000)
/** Maximm number of taps to report */
#define ML_GSTR_MULTI_TAP_DEFAULT_MAX  (2)

/* --------------- */
/* - Structures. - */
/* --------------- */

/** Parameters for multi-tap detection */
typedef struct {
    int_fast16_t tapThresh[ML_NUM_TAP_AXES];// The threshold for detecting a tap.
    int_fast16_t tapTime;     // The delay before a tap will be registered.
    
    int_fast16_t mtWait; /**< The amount of time(ms) to wait before reporting
                            the current set of taps */
    int_fast16_t mtMax;  /**< The maximum number of taps to report at a time */
}   tMLGstrMultiTapParams;

/** Report of taps detected */
typedef struct {
    int_fast16_t numTaps;      /**< Total number of taps detected */
    int_fast8_t  tapDirection; /**< Direction of the taps detected 
                                  ML_GSTR_TAP_DIRECTION_NO_TAP and similar
                                  macros */
} tMLGstrTapReport;

typedef uintptr_t tMLGstrMultiTapHandle;

/******************************************************************************/
/* MLGSTRX_Data_t Structure.                                                  */
/******************************************************************************/

/* --------------------- */
/* - Function p-types. - */
/* --------------------- */

tMLGstrMultiTapHandle MLGstrMultiTapOpen(
    tMLGstrMultiTapParams const * const params);

int  MLGstrMultiTapReset(
    tMLGstrMultiTapHandle               handle);

int  MLGstrMultiTapSetParams(
    tMLGstrMultiTapHandle               handle,
    tMLGstrMultiTapParams const * const params);

void MLGstrMultiTapClose(
    tMLGstrMultiTapHandle               handle);

int  DetectSingleTap(
    tMLGstrMultiTapHandle               handle,
    int                   const * const tapData,
    int                         * const direction);

int DetectMultiTaps(
    /* Input */
    tMLGstrMultiTapHandle               handle,
    int                                 timeSinceLastSample,
    int                   const * const tapData,
    float                               maxGyroPeak,
    float                               maxGyroPeakThreshold,
    /* output */
    tMLGstrTapReport            * const mtReport
    );


#ifdef __cplusplus
}
#endif

#endif /* GESTURE_TAP_DMP_H */

/**
 * @}
 */
