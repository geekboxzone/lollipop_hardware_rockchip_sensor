/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
 
/*******************************************************************************
 *
 * $Id: mlsetup.h 4766 2011-02-15 01:12:29Z mcaramello $
 *
 *******************************************************************************/

#ifndef MLSETUP_H
#define MLSETUP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mltypes.h"

#define PLATFORM_ID_MSB               (0x0001) // Multi sensors testing board
#define PLATFORM_ID_ST_6AXIS          (0x0002) // 6 Axis board with ST accelerometer
#define PLATFORM_ID_DONGLE            (0x0003) // 9 Axis USB dongle with:
                                               //   Kionix accelerometer and
                                               //   AKM compass
#define PLATFORM_ID_MANTIS_PROTOTYPE  (0x0004) // Mantis prototype board
#define PLATFORM_ID_MANTIS_MSB        (0x0005) // Mantis on Multi sensor testing board
#define PLATFORM_ID_MANTIS_USB_DONGLE (0x0006) // Mantis and AKM on USB dongle.
#define PLATFORM_ID_MSB_10AXIS        (0x0007) // Multi sensors testing board with pressure sensor
#define PLATFORM_ID_DRAGON_PROTOTYPE  (0x0008) // Dragon prototype board

    // Main entry APIs
tMLError SetupPlatform(unsigned short platformId, 
                       unsigned short accelSelection, 
                       unsigned short compassSelection);

#ifdef __cplusplus
}
#endif

#endif /* MLSETUP_H */
