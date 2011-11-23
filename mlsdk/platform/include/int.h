/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: int.h 4805 2011-02-18 22:24:04Z nroyer $
 *
 *******************************************************************************/

#ifndef _INT_H
#define _INT_H

#include "mltypes.h"
#include "mpu.h"

#ifdef __cplusplus
extern "C" {
#endif

    /* ------------ */
    /* - Defines. - */
    /* ------------ */

    /* ---------- */
    /* - Enums. - */
    /* ---------- */

    /* --------------- */
    /* - Structures. - */
    /* --------------- */

    /* --------------------- */
    /* - Function p-types. - */
    /* --------------------- */

    void IntOpen(const char **ints,
                 int *handles,
                 int numHandles);
    int IntProcess(int *handles, int numHandles,
                   struct mpuirq_data **data, 
                   long tv_sec, long tv_usec);
    tMLError IntClose(int *handles, int numHandles);
    tMLError IntSetTimeout(int handle, int timeout);

#ifdef __cplusplus
}
#endif

#endif /* _TEMPLATE_H */
