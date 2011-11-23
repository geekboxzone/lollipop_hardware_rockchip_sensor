/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */

/*******************************************************************************
 *
 * $Id: helper-customer.h 5146 2011-04-05 18:23:24Z mcaramello $
 *
 *******************************************************************************/

#ifndef HELPER_C_H
#define HELPER_C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mltypes.h"
#include "mlerrorcode.h"

/*
    Defines 
*/

#define CALL_N_CHECK(f) {                                                   \
    unsigned int r35uLt = f;                                                \
    if(ML_SUCCESS != r35uLt) {                                              \
        printf("Error in file %s, line %d : %s returned code %s (#%d)\n",   \
                __FILE__, __LINE__, #f, MLErrorCode(r35uLt), r35uLt);       \
    }                                                                       \
}

#define CALL_CHECK_N_RETURN_ERROR(f) {                                      \
    unsigned int r35uLt = f;                                                \
    if(ML_SUCCESS != r35uLt) {                                              \
        printf("Error in file %s, line %d : %s returned code %s (#%d)\n",   \
                __FILE__, __LINE__, #f, MLErrorCode(r35uLt), r35uLt);       \
        return r35uLt;                                                      \
    }                                                                       \
}

// for functions returning void
#define CALL_CHECK_N_RETURN(f) {                                            \
    unsigned int r35uLt = f;                                                \
    if(ML_SUCCESS != r35uLt) {                                              \
        printf("Error in file %s, line %d : %s returned code %s (#%d)\n",   \
                __FILE__, __LINE__, #f, MLErrorCode(r35uLt), r35uLt);       \
        return;                                                             \
    }                                                                       \
}

#define CALL_CHECK_N_EXIT(f) {                                              \
    unsigned int r35uLt = f;                                                \
    if(ML_SUCCESS != r35uLt) {                                              \
        printf("Error in file %s, line %d : %s returned code %s (#%d)\n",   \
                __FILE__, __LINE__, #f, MLErrorCode(r35uLt), r35uLt);       \
        exit (r35uLt);                                                      \
    }                                                                       \
}

    
#define CALL_CHECK_N_CALLBACK(f, cb) {                                      \
    unsigned int r35uLt = f;                                                \
    if(ML_SUCCESS != r35uLt) {                                              \
        printf("Error in file %s, line %d : %s returned code %s (#%d)\n",   \
                __FILE__, __LINE__, #f, MLErrorCode(r35uLt), r35uLt);       \
        cb;                                                                 \
    }                                                                       \
}

#define CALL_CHECK_N_GOTO(f, label) {                                       \
    unsigned int r35uLt = f;                                                \
    if(ML_SUCCESS != r35uLt) {                                              \
        printf("Error in file %s, line %d : %s returned code %s (#%d)\n",   \
                __FILE__, __LINE__, #f, MLErrorCode(r35uLt), r35uLt);       \
        goto label;                                                         \
    }                                                                       \
}

#define DEFAULT_ACCEL_ID        ACCEL_ID_KXTF9
#define DEFAULT_COMPASS_ID      COMPASS_ID_AKM

#define DataLogger(x)           NULL
#define DataLoggerSelector(x)   //
#define DataLoggerCb(x)         NULL
#define findComm()              (9)
#define MenuHwChoice(x,y)       (*x = DEFAULT_ACCEL_ID, *y = DEFAULT_COMPASS_ID, 1)

    char ConsoleGetChar(void);
    int ConsoleKbhit(void);
    struct mpuirq_data **InterruptPoll(
        int *handles, int numHandles, long tv_sec, long tv_usec);
    void InterruptPollDone(struct mpuirq_data ** data);

#ifdef __cplusplus
}
#endif

#endif // HELPER_C_H
