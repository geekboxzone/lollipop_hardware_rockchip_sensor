/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: gesture.h 4568 2011-01-21 23:20:41Z nroyer $
 *
 ******************************************************************************/

/** 
 *  @defgroup ML_GESTURE
 *  @brief    The Gesture Library processes gyroscopes and accelerometers to 
 *            provide recognition of a set of gestures, including tapping, 
 *            shaking along various axes, and rotation about a horizontal axis.
 *
 *   @{
 *       @file mlgesture.h
 *       @brief Header file for the Gesture Library.
 *
**/

#ifndef GESTURE_H
#define GESTURE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mltypes.h"
#include "stdint_invensense.h"

    /* ------------ */
    /* - Defines. - */
    /* ------------ */

    /**************************************************************************/
    /* Gesture Types.                                                         */
    /**************************************************************************/

#define ML_PITCH_SHAKE                  0x01
#define ML_ROLL_SHAKE                   0x02
#define ML_YAW_SHAKE                    0x04
#define ML_TAP                          0x08
#define ML_YAW_IMAGE_ROTATE             0x10
#define ML_SHAKE_ALL                    0x07
#define ML_GESTURE_ALL                       \
    ML_PITCH_SHAKE                      | \
    ML_ROLL_SHAKE                       | \
    ML_YAW_SHAKE                        | \
    ML_TAP                              | \
    ML_YAW_IMAGE_ROTATE

    /**************************************************************************/
    /* Shake Functions.                                                       */
    /**************************************************************************/

#define ML_SOFT_SHAKE                   0x0000
#define ML_HARD_SHAKE                   0x0001
#define ML_NO_RETRACTION                0x0000
#define ML_RETRACTION                   0x0002

    /**************************************************************************/
    /* Data Enumerations.                                                     */
    /**************************************************************************/
#define ML_NUM_TAP_AXES (3)

#define ML_TAP_AXIS_X                        0x1
#define ML_TAP_AXIS_Y                        0x2
#define ML_TAP_AXIS_Z                        0x4
#define ML_TAP_AXIS_ALL                      \
    (ML_TAP_AXIS_X                            |   \
     ML_TAP_AXIS_Y                            |   \
     ML_TAP_AXIS_Z)
/** The direction of a tap */
#define     ML_GSTR_TAP_DIRECTION_NO_TAP (0)
#define     ML_GSTR_TAP_DIRECTION_NEGETIVE_X (-1)
#define     ML_GSTR_TAP_DIRECTION_NEGETIVE_Y (-2)
#define     ML_GSTR_TAP_DIRECTION_NEGETIVE_Z (-3)
#define     ML_GSTR_TAP_DIRECTION_X (1)
#define     ML_GSTR_TAP_DIRECTION_Y (2)
#define     ML_GSTR_TAP_DIRECTION_Z (3)

    /**************************************************************************/
    /* Data selection options.                                                */
    /**************************************************************************/

#define ML_GSTR_YAW_ROTATION        0x0000
#define ML_GSTR_DATA_STRUCT         0x0001

    /**************************************************************************/
    /* MLGSTR_Params_t structure default values.                              */
    /**************************************************************************/

#define MLGSTR_TAP_THRESH_DEFAULT                           (2046)
#define MLGSTR_TAP_TIME_DEFAULT                             (40)
#define MLGSTR_NEXT_TAP_TIME_DEFAULT                        (200)
#define MLGSTR_MAX_TAPS_DEFAULT                             (3)
#define MLGSTR_TAP_INTERPOLATION_DEFAULT                    (2)
#define MLGSTR_SHAKE_MASK_DEFAULT                           (0)
#define MLGSTR_SHAKE_MAXIMUM_DEFAULT                        (3)
#define MLGSTR_SHAKE_THRESHOLD_0_DEFAULT                    (4.0)
#define MLGSTR_SHAKE_THRESHOLD_1_DEFAULT                    (4.0)
#define MLGSTR_SHAKE_THRESHOLD_2_DEFAULT                    (4.0)
#define MLGSTR_SNAP_THRESHOLD_0_DEFAULT                     (1000.0)
#define MLGSTR_SNAP_THRESHOLD_1_DEFAULT                     (1000.0)
#define MLGSTR_SNAP_THRESHOLD_2_DEFAULT                     (1000.0)
#define MLGSTR_SHAKE_REJECT_THRESHOLD_0_DEFAULT             (2.0f)
#define MLGSTR_SHAKE_REJECT_THRESHOLD_1_DEFAULT             (2.0f)
#define MLGSTR_SHAKE_REJECT_THRESHOLD_2_DEFAULT             (2.0f)
#define MLGSTR_SHAKE_REJECT_THRESHOLD_3_DEFAULT             (1.500f)
#define MLGSTR_SHAKE_REJECT_THRESHOLD_4_DEFAULT             (1.501f)
#define MLGSTR_SHAKE_REJECT_THRESHOLD_5_DEFAULT             (1.502f)
#define MLGSTR_LINEAR_SHAKE_DEADZONE_0_DEFAULT              (0.1f)
#define MLGSTR_LINEAR_SHAKE_DEADZONE_1_DEFAULT              (0.1f)
#define MLGSTR_LINEAR_SHAKE_DEADZONE_2_DEFAULT              (0.1f)
#define MLGSTR_SHAKE_TIME_DEFAULT                           (160)
#define MLGSTR_NEXT_SHAKE_TIME_DEFAULT                      (160)
#define MLGSTR_YAW_ROTATE_THRESHOLD_DEFAULT                 (70.0)
#define MLGSTR_YAW_ROTATE_TIME_DEFAULT                      (10)
#define MLGSTR_GESTURE_MASK_DEFAULT                         (0)
#define MLGSTR_GESTURE_CALLBACK_DEFAULT                     (0)

    /* --------------- */
    /* - Structures. - */
    /* --------------- */

    /**************************************************************************/
    /* Gesture Structure                                                      */
    /**************************************************************************/

    /** Gesture Description */
    typedef struct  {

        unsigned short type;
        short          strength;
        short          speed;
        unsigned short num;
        short          meta;
        short          reserved;

    }   tGesture,               // new type definition
        tGestureShake,          // Shake structure
        tGestureTap,            // Tap structure
        tGestureYawImageRotate, // Yaw Image Rotate Structure
        gesture_t;              // backward-compatible definition

    /**************************************************************************/
    /* Gesture Parameters Structure.                                          */
    /**************************************************************************/

    typedef struct {

        unsigned short tapThresh[ML_NUM_TAP_AXES];// The threshold for detecting a tap.
        unsigned short tapTime;            // The delay before a tap will be registered.
        unsigned short nextTapTime;        // The time interval required for the tap number to increase.
        unsigned short maxTaps;            // The max taps to record before reporting and resetting the count
        unsigned int   tapInterpolation;
        unsigned long  tapElements;
        unsigned short shakeMask;          // The shake detection functions.
        unsigned int   shakeMax[3];        // Pitch, Roll, and Yaw axis shake detection maximums
        float          shakeThreshold[3];           // Pitch, Roll, and Yaw axis shake detection thresholds.
        float          snapThreshold[3];           // Pitch, Roll, and Yaw axis shake detection thresholds.
        float          shakeRejectThreshold[6];     // Pitch, Roll, and Yaw axis shake detection thresholds.
        float          linearShakeDeadzone[3];     // Pitch, Roll, and Yaw axis shake detection thresholds.
        unsigned short shakeTime;          // The delay before a shake will be registered.
        unsigned short nextShakeTime;      // The time interval required for the shake number to increase.

        float          yawRotateThreshold;          // The threshold for detecting a yaw image rotation.
        unsigned short yawRotateTime;      // The time threshold for detecting a yaw image rotation.

        unsigned short gestureMask;        // A gesture or bitwise OR of gestures to be detected.
        void (*gestureCallback)(           // User defined callback function that will be run when a gesture is detected.
            tGesture *gesture);            // Gesture data structure.
        void (*gesturePedometerCallback)(  // Pedometer callback function that will be run when a gesture is detected.
            tGesture *gesture);            // Gesture data structure.
        int_fast16_t   suspend;            // Used to turn off gesture engine

    }   tMLGstrParams,      // new type definition
        MLGSTR_Params_t;    // backward-compatible definition


    /* --------------------- */
    /* - Function p-types. - */
    /* --------------------- */

    /**************************************************************************/
    /* ML Gesture Functions                                                   */
    /**************************************************************************/


    /*API For detecting tapping*/
    int MLSetTapThreshByAxis(unsigned int axis, unsigned short threshold);
    /* Deprecated.  Use MLSetTapThreshByAxis */
#define MLSetTapThresh(threshold) \
        MLSetTapThreshByAxis(ML_TAP_AXIS_X | ML_TAP_AXIS_Y | ML_TAP_AXIS_Z, \
                             threshold)
    int MLSetTapTime(unsigned short time);
    int MLSetNextTapTime(unsigned short time);
    int MLSetMaxTaps(unsigned short max);
    int MLResetTap(void);
    int MLSetTapShakeReject(float value);

    tMLError MLSetTapInterrupt(unsigned char on);

    /*API for detecting shaking*/
    int MLSetShakeFunc(unsigned short function);
    int MLSetShakeThresh(unsigned short axis, unsigned short threshold);
    int MLSetHardShakeThresh(unsigned short axis, unsigned short threshold);
    int MLSetShakeTime(unsigned short time);
    int MLSetNextShakeTime(unsigned short time);
    int MLSetMaxShakes(int axis, int max);
    int MLResetShake(int axis);

    tMLError MLSetShakePitchInterrupt(unsigned char on);
    tMLError MLSetShakeRollInterrupt(unsigned char on);
    tMLError MLSetShakeYawInterrupt(unsigned char on);

    /*API for detecting yaw image rotation*/
    int MLSetYawRotateThresh(unsigned short threshold);
    int MLSetYawRotateTime(unsigned short time);
    int MLGetYawRotation();

    /*API for registering gestures to be detected*/
    int MLSetGestures(unsigned short gestures);
    int MLEnableGesture();
    int MLDisableGesture();
    int MLSetGestureCallback(void (*callback)(tGesture *gesture) );
    int MLGetGesture(tGesture *gesture);
    int MLGetGestureState(int *state);
    int MLSetGesturePedometer( void (*callback)(tGesture *gesture) );
    int MLDisableGesturePedometer();

    tMLError MLGestureTapSetQuantized(void);
#ifdef __cplusplus
}
#endif

#endif // GESTURE_H
