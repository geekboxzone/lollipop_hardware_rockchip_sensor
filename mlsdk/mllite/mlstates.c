/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: mlstates.c 4595 2011-01-25 01:43:03Z mcaramello $
 *
 *******************************************************************************/

/** 
 *  @defgroup MLSTATES
 *  @brief  Basic state machine definition and support for the Motion Library.
 *
 *  @{
 *      @file mlstates.c
 *      @brief The Motion Library state machine definition.
 */

#define ML_C

/* ------------------ */
/* - Include Files. - */
/* ------------------ */

#include <stdio.h>
#include <string.h>

#include "mlstates.h"
#include "mltypes.h"
#include "mlinclude.h"
#include "ml.h"
#include "mlos.h"

#include <log.h>
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-mlstates"

#define _stateDebug(x) //{x}

#define MAX_STATE_CHANGE_PROCESSES (8)

typedef struct {
    int_fast8_t numStateChangeCallbacks;
    HANDLE mutex;
    tMLStateChangeCallback stateChangeCallbacks[MAX_STATE_CHANGE_PROCESSES];
} tMLStateCallbacks;

static tMLStateCallbacks sStateChangeCallbacks = {0};

/* --------------- */
/* -  Functions. - */
/* --------------- */

static tMLError MLStateInitCallbacks(void) 
{
    memset(&sStateChangeCallbacks,0,sizeof(sStateChangeCallbacks));
    return MLOSCreateMutex(&sStateChangeCallbacks.mutex);
}

static tMLError MLStateCloseCallbacks(void) 
{
    tMLError result;
    result = MLOSDestroyMutex(sStateChangeCallbacks.mutex);
    memset(&sStateChangeCallbacks,0,sizeof(sStateChangeCallbacks));
    return result;
}

/**
 *  @internal
 *  @brief  return a string containing the label assigned to the given state.
 *  @param  state   The state of which the label has to be returned.
 *  @return A string containing the state label.
**/
char* MLStateName(unsigned char state) 
{
    switch(state) {
        case ML_STATE_SERIAL_CLOSED: 
            return ML_STATE_NAME(ML_STATE_SERIAL_CLOSED); 
            break;
        case ML_STATE_SERIAL_OPENED: 
            return ML_STATE_NAME(ML_STATE_SERIAL_OPENED); 
            break;
        case ML_STATE_DMP_OPENED: 
            return ML_STATE_NAME(ML_STATE_DMP_OPENED); 
            break;
        case ML_STATE_DMP_STARTED: 
            return ML_STATE_NAME(ML_STATE_DMP_STARTED); 
            break;
        default:
            return NULL;
    }
}

/**
 *  @internal
 *  @brief  Perform a transition from the current state to newState.
 *          Check for the correctness of the transition.
 *          Print out an error message if the transition is illegal .
 *          This routine is also called if a certain normally constant parameters
 *          are changed such as the FIFO Rate.
 *  @param  newState    state we are transitioning to.
 *  @return  
**/
tMLError MLStateTransition(unsigned char newState)
{
    tMLError result = ML_SUCCESS;

    if ( newState == ML_STATE_SERIAL_CLOSED ) {
        // Always allow transition to closed
    } else if ( newState == ML_STATE_SERIAL_OPENED ) {
        MLStateInitCallbacks(); // Always allow first transition to start over
    } else if (((newState == ML_STATE_DMP_OPENED) && 
                 ((mlParams.mlState == ML_STATE_SERIAL_OPENED) ||
                  (mlParams.mlState == ML_STATE_DMP_STARTED)))
                 ||
                ((newState == ML_STATE_DMP_STARTED) && 
                 (mlParams.mlState == ML_STATE_DMP_OPENED))) {
        // Valid transitions but no special action required
    } else {
        // All other combinations are illegal
        MPL_LOGE("Error : illegal state transition from %s to %s\n", MLStateName(mlParams.mlState), MLStateName(newState));
        result = ML_ERROR_SM_TRANSITION;
    }

    if ( result == ML_SUCCESS ) {
        _stateDebug(MPL_LOGV("ML State transition from %s to %s\n", MLStateName(mlParams.mlState), MLStateName(newState)));
        result = MLStateRunCallbacks(newState);
        if (ML_SUCCESS == result && 
            newState == ML_STATE_SERIAL_CLOSED) {
            MLStateCloseCallbacks();
        }
        mlParams.mlState = newState;
    }
    return result;
}

/**
 *  @internal
 *  @brief  To be moved in mlstates.c
**/
unsigned char MLGetState(void)
{
    return (mlParams.mlState);
}

/**
 * @internal
 * @brief   This registers a function to be called each time the state 
 *          changes. It may also be called when the FIFO Rate is changed.
 *          It will be called at the start of a state change before the
 *          state change has taken place. See Also MLStateUnRegisterCallback()
 *          The FIFO does not have to be on for this callback.
 * @param func Function to be called when a DMP interrupt occurs.
 * @return ML_SUCCESS or non-zero error code.
 */

tMLError MLStateRegisterCallback(tMLStateChangeCallback callback)
{
    INVENSENSE_FUNC_START;
    int kk;
    tMLError result;
    
    result = MLOSLockMutex(sStateChangeCallbacks.mutex);
    if (ML_SUCCESS != result) {
        return result;
    }

    // Make sure we have not filled up our number of allowable callbacks
    if ( sStateChangeCallbacks.numStateChangeCallbacks < MAX_STATE_CHANGE_PROCESSES ) {
        // Make sure we haven't registered this function already
        for ( kk=0; kk < sStateChangeCallbacks.numStateChangeCallbacks; ++kk) {
            if ( sStateChangeCallbacks.stateChangeCallbacks[kk] == callback ) {
                result = ML_ERROR_INVALID_PARAMETER;
                break;
            }
        }

        if (ML_SUCCESS == result) {
            // Add new callback
            sStateChangeCallbacks.stateChangeCallbacks[sStateChangeCallbacks.numStateChangeCallbacks] = callback;
            sStateChangeCallbacks.numStateChangeCallbacks++;
        }
    } else {
        result = ML_ERROR_MEMORY_EXAUSTED;
    }

    MLOSUnlockMutex(sStateChangeCallbacks.mutex);
    return result;
}

/**
 * @internal
 * @brief   This unregisters a function to be called each time the state 
 *          changes. See Also MLStateRegisterCallback()
 *          The FIFO does not have to be on for this callback.
 * @return ML_SUCCESS or non-zero error code.
 */
tMLError MLStateUnRegisterCallback( tMLStateChangeCallback callback )
{
    INVENSENSE_FUNC_START;
    int kk,jj;
    tMLError result;

    result = MLOSLockMutex(sStateChangeCallbacks.mutex);
    if (ML_SUCCESS != result) {
        return result;
    }

    // Make sure we haven't registered this function already
    result = ML_ERROR_INVALID_PARAMETER;
    for (kk=0; kk<sStateChangeCallbacks.numStateChangeCallbacks; ++kk) {
        if ( sStateChangeCallbacks.stateChangeCallbacks[kk] == callback ) {
            for (jj=kk+1; jj<sStateChangeCallbacks.numStateChangeCallbacks; ++jj) {
                sStateChangeCallbacks.stateChangeCallbacks[jj-1] = sStateChangeCallbacks.stateChangeCallbacks[jj];
            }
            sStateChangeCallbacks.numStateChangeCallbacks--;
            result = ML_SUCCESS;
            break;
        }
    }

    MLOSUnlockMutex(sStateChangeCallbacks.mutex);
    return result;
}

tMLError MLStateRunCallbacks(unsigned char newState)
{
    int kk;
    tMLError result;
    
    result = MLOSLockMutex(sStateChangeCallbacks.mutex);
    if (ML_SUCCESS != result) {
        MPL_LOGE("MLOsLockMutex returned %d\n", result);
        return result;
    }

    for (kk=0; kk<sStateChangeCallbacks.numStateChangeCallbacks; ++kk) {
        if ( sStateChangeCallbacks.stateChangeCallbacks[kk] ) {
            result = sStateChangeCallbacks.stateChangeCallbacks[kk]( newState );
            if (ML_SUCCESS != result) {
                break;  // Can't return, must release mutex
            }
        }
    }

    MLOSUnlockMutex(sStateChangeCallbacks.mutex);
    return result;
}
