/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: accel.h 4580 2011-01-22 03:19:23Z prao $
 *
 *******************************************************************************/

#ifndef ACCEL_H
#define ACCEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mltypes.h"
#include "mpu.h"

    /* ------------ */
    /* - Defines. - */
    /* ------------ */


    /* --------------- */
    /* - Structures. - */
    /* --------------- */

    /* --------------------- */
    /* - Function p-types. - */
    /* --------------------- */

    unsigned char  AccelGetPresent    ( void );
    unsigned char  AccelGetSlaveAddr  ( void );
    tMLError       AccelGetData       ( long* data );
    unsigned short AccelGetId         ( void );

#ifdef __cplusplus
}
#endif

#endif // ACCEL_H
