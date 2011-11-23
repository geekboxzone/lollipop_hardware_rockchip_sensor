# Copyright (C) 2008 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


LOCAL_PATH := $(call my-dir)

ifeq ($(BOARD_USE_MPU3050),true)
ifneq ($(TARGET_SIMULATOR),true)

$(info LOCAL_PATH=$(LOCAL_PATH)) 
# HAL module implemenation, not prelinked, and stored in
# hw/<SENSORS_HARDWARE_MODULE_ID>.<ro.product.board>.so
include $(CLEAR_VARS)

LOCAL_MODULE := sensors.rk29board

LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw

LOCAL_MODULE_TAGS := optional

LOCAL_CFLAGS := -DLOG_TAG=\"Sensors\"
LOCAL_SRC_FILES := 						\
				sensors_mpl.cpp 		\
				SensorBase.cpp			\
				AkmSensor.cpp                   \
				GyroSensor.cpp                  \
                InputEventReader.cpp            \
                MPLSensor.cpp					\
                MPLGesture.cpp          \
               LightSensor.cpp			\
               ProximitySensor.cpp

LOCAL_C_INCLUDES += $(LOCAL_PATH)/mlsdk/platform/include
LOCAL_C_INCLUDES += $(LOCAL_PATH)/mlsdk/platform/linux
LOCAL_C_INCLUDES += $(LOCAL_PATH)/mlsdk/mllite
LOCAL_C_INCLUDES += $(LOCAL_PATH)/mlsdk/mldmp

LOCAL_PREBUILT_LIBS :=  libmpl.so libmllite.so libmlplatform.so
LOCAL_SHARED_LIBRARIES := liblog libcutils libutils libdl libmpl libmllite libmlplatform
LOCAL_CPPFLAGS+=-DLINUX=1 
#LOCAL_CPPFLAGS+=-DENABLE_GESTURE_MANAGER
#LOCAL_CPPFLAGS+=-DENABLE_GESTURE_MANAGER
LOCAL_PRELINK_MODULE := false

include $(BUILD_SHARED_LIBRARY)
include $(BUILD_MULTI_PREBUILT)
endif # !TARGET_SIMULATOR



#include $(CLEAR_VARS)
#LOCAL_MODULE_TAGS := optional
#LOCAL_SRC_FILES:=        \
#  timerirq_test.c
#
#LOCAL_SHARED_LIBRARIES:= libcutils libutils libbinder
#
#LOCAL_MODULE:= timerirq_test
#
#LOCAL_CFLAGS+=-DLOG_TAG=\"power_test\"
#LOCAL_PRELINK_MODULE:=false
#include $(BUILD_EXECUTABLE)

include $(call all-subdir-makefiles)

endif 
