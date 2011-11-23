/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/***************************************************************************** *
 * $Id: gestureMenu.h 4574 2011-01-22 01:39:28Z nroyer $ 
 ******************************************************************************/
/**
 * @defgroup 
 * @brief  
 *
 * @{
 *      @file     gestureMenu.h
 *      @brief    
 *
 *
 */

#ifndef __GESTUREMENU_H__
#define __GESTUREMENU_H__

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
    typedef struct sGestureMenuParams {
        /* Tap Params */
        int xTapThreshold;
        int yTapThreshold;
        int zTapThreshold;
        int tapTime;
        int nextTapTime;
        int maxTaps;
        float shakeRejectValue;

        /* Shake Params */
        int xShakeThresh;
        int yShakeThresh;
        int zShakeThresh;
        int xSnapThresh;
        int ySnapThresh;
        int zSnapThresh;
        int shakeTime;
        int nextShakeTime;
        int shakeFunction;
        int maxShakes;

        /* Yaw rotate params */
        int yawRotateTime;
        int yawRotateThreshold;

        /* Orientation */
        float orientationThreshold;
        int sensorsIndex;
    } tGestureMenuParams;

    void     PrintGestureMenu(tGestureMenuParams const * const params) ;
    tMLError GestureMenuProcessChar(tGestureMenuParams * const params,char ch);
    void     PrintGesture(gesture_t* gesture);
    void     PrintOrientation(unsigned short orientation);
    void     GestureMenuSetDefaults(tGestureMenuParams * const params);
    tMLError GestureMenuSetMpl(tGestureMenuParams const * const params);

/******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif // __GESTUREMENU_H__
