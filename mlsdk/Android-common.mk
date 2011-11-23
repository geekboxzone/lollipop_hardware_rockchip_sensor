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

## applications ##
SHARED_APP_SUFFIX = -shared
STATIC_APP_SUFFIX = -static


####################################################################################################
## includes and linker

ANDROID_LINK = \
    -L$(ANDROID_ROOT)/out/target/product/$(PRODUCT)/system/lib \
    -L$(ANDROID_ROOT)/out/target/product/$(PRODUCT)/obj/lib 

ANDROID_INCLUDES = \
	-I$(ANDROID_ROOT)/system/core/include \
	-I$(ANDROID_ROOT)/hardware/libhardware/include \
	-I$(ANDROID_ROOT)/hardware/ril/include \
	-I$(ANDROID_ROOT)/dalvik/libnativehelper/include \
	-I$(ANDROID_ROOT)/frameworks/base/include \
	-I$(ANDROID_ROOT)/external/skia/include \
	-I$(ANDROID_ROOT)/out/target/product/generic/obj/include \
	-I$(ANDROID_ROOT)/bionic/libc/arch-arm/include \
	-I$(ANDROID_ROOT)/bionic/libc/include \
	-I$(ANDROID_ROOT)/bionic/libstdc++/include \
	-I$(ANDROID_ROOT)/bionic/libc/kernel/common \
	-I$(ANDROID_ROOT)/bionic/libc/kernel/arch-arm \
	-I$(ANDROID_ROOT)/bionic/libm/include \
	-I$(ANDROID_ROOT)/bionic/libm/include/arch/arm \
	-I$(ANDROID_ROOT)/bionic/libthread_db/include \
	-I$(ANDROID_ROOT)/bionic/libm/arm \
	-I$(ANDROID_ROOT)/bionic/libm \
	-I$(ANDROID_ROOT)/out/target/product/generic/obj/SHARED_LIBRARIES/libm_intermediates 

KERNEL_INCLUDES = \
	-I$(KERNEL_ROOT)/include   

MLSDK_INCLUDES = \
	-I$(MLSDK_ROOT)/platform/include \
	-I$(MLSDK_ROOT)/platform/linux \
	-I$(MLSDK_ROOT)/platform/linux/kernel

####################################################################################################
## macros

define echo_in_colors
	@echo -ne "\e[1;32m"$(1)"\e[0m"
endef 

####################################################################################################
## flags

ifeq ($(HARDWARE),M_HW)
	CFLAGS += -DM_HW
endif



