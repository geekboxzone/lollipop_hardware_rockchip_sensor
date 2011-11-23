LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE_TAGS := optional

LOCAL_MODULE := mlplatform
#modify these to point to the mpl source installation
MLSDK_ROOT = mlsdk
MLPLATFORM_DIR = $(MLSDK_ROOT)/platform/linux

LOCAL_CFLAGS += -D_REENTRANT -DLINUX -DANDROID
LOCAL_CFLAGS += -I$(LOCAL_PATH)/$(MLSDK_ROOT)/platform/include
LOCAL_CFLAGS += -I$(LOCAL_PATH)/$(MLPLATFORM_DIR)
LOCAL_CFLAGS += -I$(LOCAL_PATH)/$(MLPLATFORM_DIR)/kernel
LOCAL_CFLAGS += -I$(LOCAL_PATH)/$(MLSDK_ROOT)/mllite

ML_SOURCES = \
    $(MLPLATFORM_DIR)/int_linux.c \
    $(MLPLATFORM_DIR)/mlos_linux.c \
    $(MLPLATFORM_DIR)/mlsl_linux_mpu.c

LOCAL_SRC_FILES := $(ML_SOURCES)

LOCAL_SHARED_LIBRARIES := liblog libm libutils libcutils
LOCAL_PRELINK_MODULE := false
include $(BUILD_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE_TAGS := optional

LOCAL_MODULE := mllite
#modify these to point to the mpl source installation
MLSDK_ROOT = mlsdk
MLPLATFORM_DIR = $(MLSDK_ROOT)/platform
MLLITE_DIR = $(MLSDK_ROOT)/mllite
MPL_DIR = $(MLSDK_ROOT)/mldmp

LOCAL_CFLAGS += -DNDEBUG
LOCAL_CFLAGS += -D_REENTRANT -DLINUX -DANDROID
LOCAL_CFLAGS += -DUNICODE -D_UNICODE -DSK_RELEASE
LOCAL_CFLAGS += -DI2CDEV=\"/dev/mpu\"
LOCAL_CFLAGS += -I$(LOCAL_PATH)/$(MPL_DIR) -I$(LOCAL_PATH)/$(MLLITE_DIR) -I$(LOCAL_PATH)/$(MLPLATFORM_DIR)/include
LOCAL_CFLAGS += -I$(LOCAL_PATH)/$(MLSDK_ROOT)/mlutils -I$(LOCAL_PATH)/$(MLSDK_ROOT)/mlapps/common

ML_SOURCES = \
    $(MLLITE_DIR)/accel.c \
    $(MLLITE_DIR)/accel/kxtf9.c \
    $(MLLITE_DIR)/accel/kxsd9.c \
    $(MLLITE_DIR)/accel/mma8450.c \
    $(MLLITE_DIR)/accel/mma845x.c   \
    $(MLLITE_DIR)/accel/bma150.c \
    $(MLLITE_DIR)/accel/bma222.c \
    $(MLLITE_DIR)/accel/adxl346.c \
    $(MLLITE_DIR)/accel/lis331.c \
    $(MLLITE_DIR)/accel/lis3dh.c \
    $(MLLITE_DIR)/accel/lsm303a.c \
    \
    $(MLLITE_DIR)/compass.c \
    $(MLLITE_DIR)/compass/ak8975.c \
    $(MLLITE_DIR)/compass/ami30x.c \
    $(MLLITE_DIR)/compass/ami306.c \
    $(MLLITE_DIR)/compass/hmc5883.c \
    $(MLLITE_DIR)/compass/yas529.c \
    $(MLLITE_DIR)/compass/mmc314x.c \
    $(MLLITE_DIR)/compass/lsm303m.c \
    $(MLLITE_DIR)/compass/hscdtd002b.c \
    $(MLLITE_DIR)/compass/hscdtd004a.c \
    \
    $(MLLITE_DIR)/pressure.c \
    $(MLLITE_DIR)/pressure/bma085.c \
    \
    $(MLLITE_DIR)/mldl_cfg_mpu.c \
    $(MLLITE_DIR)/dmpDefault.c \
    $(MLLITE_DIR)/ml.c \
    $(MLLITE_DIR)/mlFIFO.c \
    $(MLLITE_DIR)/mlFIFOHW.c \
    $(MLLITE_DIR)/mlMathFunc.c \
    $(MLLITE_DIR)/ml_stored_data.c \
    $(MLLITE_DIR)/mlcontrol.c \
    $(MLLITE_DIR)/mldl.c \
    $(MLLITE_DIR)/mldmp.c \
    $(MLLITE_DIR)/mlstates.c \
    $(MLLITE_DIR)/mlsupervisor.c \
    $(MLLITE_DIR)/ml_mputest.c \
    \
    $(MLSDK_ROOT)/mlutils/mputest.c \
    $(MLSDK_ROOT)/mlutils/checksum.c

ifeq ($(HARDWARE),M_HW)
    ML_SOURCES += $(MLLITE_DIR)/accel/mantis.c
endif

LOCAL_SRC_FILES := $(ML_SOURCES) 
LOCAL_SHARED_LIBRARIES := libm libutils libcutils liblog libmlplatform
LOCAL_PRELINK_MODULE := false
include $(BUILD_SHARED_LIBRARY)

#include $(LOCAL_PATH)/build_mldmp.mk
