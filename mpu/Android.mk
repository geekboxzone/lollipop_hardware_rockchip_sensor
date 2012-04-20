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
# Modified 2011 by InvenSense, Inc


LOCAL_PATH := $(call my-dir)

ifeq ($(BOARD_SENSOR_MPU),true)

# InvenSense fragment of the HAL
include $(CLEAR_VARS)

LOCAL_MODULE := sensors.$(TARGET_BOARD_HARDWARE)

LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw

LOCAL_MODULE_TAGS := optional

LOCAL_CFLAGS := -DLOG_TAG=\"Sensors\"
LOCAL_CFLAGS += -DCONFIG_MPU_SENSORS_MPU3050=1

LOCAL_SRC_FILES := \
	SensorBase.cpp \
	MPLSensor.cpp \
	sensors_mpl.cpp \
	#InputEventReader.cpp \
	#LightSensor.cpp \
	#ProximitySensor.cpp \
	#PressureSensor.cpp \
	#SamsungSensorBase.cpp 
	

LOCAL_C_INCLUDES += $(LOCAL_PATH)/mlsdk/platform/include
LOCAL_C_INCLUDES += $(LOCAL_PATH)/mlsdk/platform/include/linux
LOCAL_C_INCLUDES += $(LOCAL_PATH)/mlsdk/platform/linux
LOCAL_C_INCLUDES += $(LOCAL_PATH)/mlsdk/mllite
LOCAL_C_INCLUDES += $(LOCAL_PATH)/mlsdk/mldmp
LOCAL_C_INCLUDES += $(LOCAL_PATH)/mlsdk/external/aichi
LOCAL_C_INCLUDES += $(LOCAL_PATH)/mlsdk/external/akmd

LOCAL_PREBUILT_LIBS :=  libmplmpu.so libmllite.so libmlplatform.so
LOCAL_SHARED_LIBRARIES := liblog libcutils libutils libdl libmplmpu libmllite libmlplatform
LOCAL_CPPFLAGS+=-DLINUX=1
LOCAL_CPPFLAGS += -DMPL_LIB_NAME=\"libmplmpu.so\"
LOCAL_CPPFLAGS += -DAICHI_LIB_NAME=\"libami.so\"
LOCAL_CPPFLAGS += -DAKM_LIB_NAME=\"libakmd.so\"
LOCAL_LDFLAGS:=-rdynamic
LOCAL_PRELINK_MODULE := false

include $(BUILD_SHARED_LIBRARY)
include $(BUILD_MULTI_PREBUILT)
#include $(call all-subdir-makefiles)
endif # !BOARD_SENSOR_MPU
