/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: pressure.h 4092 2010-11-17 23:49:22Z kkeal $
 *
 *******************************************************************************/

#ifndef PRESSURE_H
#define PRESSURE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mltypes.h"
#include "mpu.h"

    /* ------------ */
    /* - Defines. - */
    /* ------------ */

#define USE_PRESSURE_BMA                    0


#define PRESSURE_SLAVEADDR_INVALID          0x00
#define PRESSURE_SLAVEADDR_BMA085           0x77

/*
    Define default pressure to use if no selection is made
*/
 #if USE_PRESSURE_BMA
 #define DEFAULT_PRESSURE_TYPE              PRESSURE_ID_BMA
 #endif

     /* --------------- */
    /* - Structures. - */
    /* --------------- */

    /* --------------------- */
    /* - Function p-types. - */
    /* --------------------- */

    unsigned char  PressureGetPresent    ( void );
    unsigned char  PressureGetSlaveAddr  ( void );
    tMLError       PressureSuspend       ( void );
    tMLError       PressureResume        ( void );
    tMLError       PressureGetData       ( long* data );
    unsigned short PressureGetId         ( void );

#ifdef __cplusplus
}
#endif

#endif // PRESSURE_H
