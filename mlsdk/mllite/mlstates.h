/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: mlstates.h 4084 2010-11-17 02:37:28Z nroyer $
 *
 *******************************************************************************/

#ifndef ML_STATES_H
#define ML_STATES_H

#include "mltypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* See MLStateTransition for the transition mask */
#define ML_STATE_SERIAL_CLOSED      (0)
#define ML_STATE_SERIAL_OPENED      (1)
#define ML_STATE_DMP_OPENED         (2)
#define ML_STATE_DMP_STARTED        (3)
#define ML_STATE_DMP_STOPPED        (ML_STATE_DMP_OPENED)
#define ML_STATE_DMP_CLOSED         (ML_STATE_SERIAL_OPENED)

#define ML_STATE_NAME(x)            (#x)

    typedef tMLError(*tMLStateChangeCallback) (unsigned char newState);

    char*           MLStateName(unsigned char x);
    tMLError        MLStateTransition(unsigned char newState);
    unsigned char   MLGetState(void);
    tMLError        MLStateRegisterCallback(tMLStateChangeCallback callback);
    tMLError        MLStateUnRegisterCallback(tMLStateChangeCallback callback);
    tMLError        MLStateRunCallbacks(unsigned char newState);

#ifdef __cplusplus
}
#endif

#endif // ML_STATES_H

