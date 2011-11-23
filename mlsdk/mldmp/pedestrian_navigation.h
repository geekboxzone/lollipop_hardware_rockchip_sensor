/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */


/******************************************************************************
 *
 * $Id: pedestrian_navigation.h 4552 2011-01-21 05:56:13Z mcaramello $
 *
 *****************************************************************************/

#ifndef ML_PEDESTRIAN_NAVIGATION_H__
#define ML_PEDESTRIAN_NAVIGATION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "mltypes.h"

int MLEnablePedestrianNavigation(void);
int MLDisablePedestrianNavigation(void);
int MLPedestrianNavigationSetCallback( void (*func) (float x, float y, float heading) );
int MLPedestrianNavigationSetPosition(float x, float y);
int MLPedestrianNavigationSetHeading(void);
int MLPedestrianNavigationSetStepSize(float stepsize);
void MLPedestrianNavigationGetUserLocation(float *x, float *y, float *heading);

#ifdef __cplusplus
}
#endif


#endif // ML_PEDESTRIAN_NAVIGATION_H__
