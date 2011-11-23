/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: compass.h 5043 2011-03-18 22:08:25Z nroyer $
 *
 *******************************************************************************/

#ifndef COMPASS_H
#define COMPASS_H

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

    unsigned char  CompassGetPresent    ( void );
    unsigned char  CompassGetSlaveAddr  ( void );
    tMLError       CompassGetData       ( long* data );
    tMLError       CompassSetBias       ( long *bias );
    unsigned short CompassGetId         ( void );
    tMLError       CompassWriteReg(unsigned char reg, unsigned char val);
    tMLError       CompassReadReg(unsigned char reg, unsigned char *val);

#ifdef __cplusplus
}
#endif

#endif // COMPASS_H
