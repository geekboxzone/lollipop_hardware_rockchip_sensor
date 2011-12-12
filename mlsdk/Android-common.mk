# Use bash for additional echo fancyness
SHELL = /bin/bash

####################################################################################################
## defines

## libraries ##
LIB_PREFIX = lib

STATIC_LIB_EXT = a
SHARED_LIB_EXT = so

# normally, overridden from outside 
# =? assignment sets it only if not already defined
MLPLATFORM_LIB_NAME ?= mlplatform
MLLITE_LIB_NAME     ?= mllite
MPL_LIB_NAME        ?= mpl
AICHI_LIB_NAME      ?= ami
AKM_LIB_NAME        ?= akmd

AICHI_EXT_LIB       ?= libami_sensor_mw.so
AKM_EXT_LIB         ?= libAK8975.a


## applications ##
SHARED_APP_SUFFIX = -shared
STATIC_APP_SUFFIX = -static

####################################################################################################
## includes and linker

ANDROID_LINK  = -L$(ANDROID_ROOT)/out/target/product/$(PRODUCT)/system/lib
ANDROID_LINK += -L$(ANDROID_ROOT)/out/target/product/$(PRODUCT)/obj/lib 

ANDROID_INCLUDES  = -I$(ANDROID_ROOT)/system/core/include
ANDROID_INCLUDES += -I$(ANDROID_ROOT)/hardware/libhardware/include
ANDROID_INCLUDES += -I$(ANDROID_ROOT)/hardware/ril/include
ANDROID_INCLUDES += -I$(ANDROID_ROOT)/dalvik/libnativehelper/include
ANDROID_INCLUDES += -I$(ANDROID_ROOT)/frameworks/base/include
ANDROID_INCLUDES += -I$(ANDROID_ROOT)/external/skia/include
ANDROID_INCLUDES += -I$(ANDROID_ROOT)/out/target/product/generic/obj/include
ANDROID_INCLUDES += -I$(ANDROID_ROOT)/bionic/libc/arch-arm/include
ANDROID_INCLUDES += -I$(ANDROID_ROOT)/bionic/libc/include
ANDROID_INCLUDES += -I$(ANDROID_ROOT)/bionic/libstdc++/include
ANDROID_INCLUDES += -I$(ANDROID_ROOT)/bionic/libc/kernel/common
ANDROID_INCLUDES += -I$(ANDROID_ROOT)/bionic/libc/kernel/arch-arm
ANDROID_INCLUDES += -I$(ANDROID_ROOT)/bionic/libm/include
ANDROID_INCLUDES += -I$(ANDROID_ROOT)/bionic/libm/include/arch/arm
ANDROID_INCLUDES += -I$(ANDROID_ROOT)/bionic/libthread_db/include
ANDROID_INCLUDES += -I$(ANDROID_ROOT)/bionic/libm/arm
ANDROID_INCLUDES += -I$(ANDROID_ROOT)/bionic/libm
ANDROID_INCLUDES += -I$(ANDROID_ROOT)/out/target/product/generic/obj/SHARED_LIBRARIES/libm_intermediates

KERNEL_INCLUDES  = -I$(KERNEL_ROOT)/include

MLSDK_INCLUDES  = -I$(MLSDK_ROOT)/platform/include
MLSDK_INCLUDES += -I$(MLSDK_ROOT)/platform/include/linux
MLSDK_INCLUDES += -I$(MLSDK_ROOT)/platform/linux
MLSDK_INCLUDES += -I$(MLSDK_ROOT)/platform/linux/kernel

MLSDK_DEFINES += -DINV_CACHE_DMP=1

####################################################################################################
## macros

ifndef echo_in_colors
define echo_in_colors
	echo -ne "\e[1;32m"$(1)"\e[0m"
endef 
endif



