/*
 * Copyright (C) 2011 Invensense, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ANDROID_MPL_SENSOR_H
#define ANDROID_MPL_SENSOR_H

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>
#include <poll.h>
#include <utils/Vector.h>

#include "sensors.h"
#include "SensorBase.h"

/*****************************************************************************/
typedef void(*hfunc_t)(sensors_event_t*, uint32_t*, int);
/** MPLSensor implementation which fits into the HAL example for crespo provided
 * * by Google.
 * * WARNING: there may only be one instance of MPLSensor, ever.
 */
class MPLSensor : public SensorBase {
public:
            MPLSensor();
    virtual ~MPLSensor();

    enum {
#ifdef ENABLE_GESTURE_MANAGER
          Tap             = 0, //0 Tap
          Shake           = 1, //1 Shake
          YawIR           = 2, //2 Yaw Image Rotate
          Orient6         = 3, //3 orientation
          GridNum         = 4, //4 grid num
          GridDelta       = 5, //5 grid change
          CtrlSig         = 6, //6 ctrl sig
          Motion          = 7, //7 motion
          Step            = 8, //8 step
          Snap            = 9, //9 snap
          RotationVector  = 10,
#else
          RotationVector  = 0,
#endif
          LinearAccel,
          Gravity,
          Gyro,
          Accelerometer,
          MagneticField,
          Orientation,
          numSensors
    };
    virtual int setDelay(int32_t handle, int64_t ns);
    virtual int enable(int32_t handle, int enabled);
    virtual int readEvents(sensors_event_t *data, int count);
    virtual int getFd() const;
    virtual int getAccelFd() const;
    virtual int getTimerFd() const;
    virtual int getPowerFd() const;
    virtual int getPollTime();
    virtual bool hasPendingEvents() const;
    virtual void handlePowerEvent();
    virtual void sleepEvent();
    virtual void wakeEvent();
private:
    int update_delay();
    int accel_fd;
    int timer_fd;

    uint32_t mEnabled;
    uint32_t mPendingMask;
    sensors_event_t mPendingEvents[numSensors];
    uint64_t mDelays[numSensors];
    hfunc_t mHandlers[numSensors];
    bool mForceSleep;
    long int mOldEnabledMask;

};

/*****************************************************************************/

#endif  // ANDROID_MPL_SENSOR_H
