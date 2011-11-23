# Use bash for additional echo fancyness
SHELL = /bin/bash

LIBRARY = libmllite.so
TARGET = android
PRODUCT = beagleboard

OBJFOLDER = $(CURDIR)/obj


MLSDK_ROOT = ../../..
ML_DIR  = ../..
ML_PLATFORM = $(MLSDK_ROOT)/platform
ML_PLATFORM_LIB = $(ML_PLATFORM)/linux/libmlplatform.so
MPL_LIB  = ../../../mldmp/mpl/android/libmpl.so

CROSS   = $(ANDROID_ROOT)/prebuilt/linux-x86/toolchain/arm-eabi-4.4.0/bin/arm-eabi-
COMP    = $(CROSS)gcc

LINK_DIR = -L$(ANDROID_ROOT)/out/target/product/$(PRODUCT)/system/lib -L$(ANDROID_ROOT)/out/target/product/$(PRODUCT)/obj/lib 

LINK    = $(CROSS)gcc -nostdlib -shared -lgcc -lc -lm -lutils -lcutils -lgcc 
LINK    += -Wl,-rpath,$(ANDROID_ROOT)/out/target/product/$(PRODUCT)/obj/lib:$(ANDROID_ROOT)/out/target/product/$(PRODUCT)/system/lib -L$(ML_PLATFORM)/linux -lmlplatform
LINK    +=  -Wl,-T,$(ANDROID_ROOT)/build/core/armelf.xsc -Wl,--no-whole-archive -Wl,--gc-sections -Wl,--no-whole-archive -Wl,-dynamic-linker,/system/bin/linker -Wl,-rpath, --Wl,-rpath-link -Wl,-soname -Wl,-T,$(ANDROID_ROOT)/build/core/armelf.xsc 
LINK    += -Wl,--gc-sections -Wl,-shared,-Bsymbolic $(LINK_DIR)  -g -mfpu=neon -march=armv7-a -mfloat-abi=softfp -Wl,-soname,$(LIBRARY)


CFLAGS   = -fpic
CFLAGS  += -Wall 
CFLAGS  += -D_REENTRANT -DLINUX 
CFLAGS  += -DUNICODE -D_UNICODE -DANDROID -DSK_RELEASE -DNDEBUG -D__ARM_ARCH_7TE__ -DI2CDEV=\"/dev/mpu\" -UDEBUG
CFLAGS  += -mthumb-interwork -fpic -fno-exceptions -ffunction-sections -funwind-tables -fstack-protector -g -mfpu=neon -march=armv7-a -mfloat-abi=softfp ../../../platform/linux/libmlplatform.so  -fmessage-length=0
CFLAGS  += -rdynamic -nostdlib -fpic -nostdlib -fpic 
#CFLAGS  += -DMLMATH 
CFLAGS  += -I$(ML_DIR) -I$(ML_PLATFORM)/include
CFLAGS  += -I$(MLSDK_ROOT)/mlutils -I$(MLSDK_ROOT)/mlapps/common
CFLAGS  += -fno-short-enums

VPATH   += $(ML_DIR) 
VPATH   += $(ML_DIR)/pressure $(ML_DIR)/accel $(ML_DIR)/compass $(ML_DIR)/log 
VPATH   += $(MLSDK_ROOT)/mlutils

ML_LIBS = \
	$(ML_DIR)/mldmp/mpl/$(TARGET)/libmpl.so \
	$(ML_PLATFORM_DIR)/linux/libmlplatform.so \

####################################################################################################
## sources

MK_NAME = $(notdir $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST)))

ANDROID_SRC = \
	-I${ANDROID_ROOT}/system/core/include \
	-I$(ANDROID_ROOT)/hardware/libhardware/include\
	-I$(ANDROID_ROOT)/hardware/ril/include\
	-I$(ANDROID_ROOT)/dalvik/libnativehelper/include\
	-I$(ANDROID_ROOT)/frameworks/base/include\
	-I$(ANDROID_ROOT)/external/skia/include\
	-I$(ANDROID_ROOT)/out/target/product/generic/obj/include\
	-I$(ANDROID_ROOT)/bionic/libc/arch-arm/include\
	-I$(ANDROID_ROOT)/bionic/libc/include\
	-I$(ANDROID_ROOT)/bionic/libstdc++/include\
	-I$(ANDROID_ROOT)/bionic/libc/kernel/common\
	-I$(ANDROID_ROOT)/bionic/libc/kernel/arch-arm\
	-I$(ANDROID_ROOT)/bionic/libm/include\
	-I$(ANDROID_ROOT)/bionic/libm/include/arch/arm\
	-I$(ANDROID_ROOT)/bionic/libthread_db/include\
	-I$(ANDROID_ROOT)/bionic/libm/arm\
	-I$(ANDROID_ROOT)/bionic/libm\
	-I$(ANDROID_ROOT)/out/target/product/generic/obj/SHARED_LIBRARIES/libm_intermediates\
    -I$(MLSDK_ROOT)/platform/include\
    -I$(MLSDK_ROOT)/platform/linux\


ML_SRCS = \
	$(ML_DIR)/accel/kxtf9.c \
	$(ML_DIR)/accel/kxsd9.c \
	$(ML_DIR)/accel/mma8450.c \
	$(ML_DIR)/accel/mma8451.c	\
	$(ML_DIR)/accel/bma150.c \
	$(ML_DIR)/accel/adxl346.c \
	$(ML_DIR)/compass/ak8975.c \
	$(ML_DIR)/compass/ami30x.c \
	$(ML_DIR)/compass/hmc5883.c \
	$(ML_DIR)/compass/yas529.c \
	$(ML_DIR)/compass/mmc314x.c \
	$(ML_DIR)/accel/bma222.c \
	$(ML_DIR)/accel/lis331.c \
	$(ML_DIR)/accel/lsm303a.c \
	$(ML_DIR)/compass/lsm303m.c \
	$(ML_DIR)/compass/hscdtd00xx.c \
	$(ML_DIR)/pressure/bma085.c \
    \
	$(ML_DIR)/mldl_cfg_mpu.c \
	$(ML_DIR)/compass.c \
	$(ML_DIR)/pressure.c \
	$(ML_DIR)/dmpDefault.c \
	$(ML_DIR)/ml.c \
	$(ML_DIR)/mlFIFO.c \
	$(ML_DIR)/mlFIFOHW.c \
	$(ML_DIR)/mlMathFunc.c \
	$(ML_DIR)/ml_stored_data.c \
	$(ML_DIR)/mlcontrol.c \
	$(ML_DIR)/mldl.c \
	$(ML_DIR)/mldmp.c \
	$(ML_DIR)/mlstates.c \
	$(ML_DIR)/mlsupervisor.c \
	$(ML_DIR)/ml_mputest.c \
	$(MLSDK_ROOT)/mlutils/mputest.c \
	$(MLSDK_ROOT)/mlutils/checksum.c


ML_OBJS := $(addsuffix .o,$(ML_SRCS))
ML_OBJS_DST = $(addprefix $(OBJFOLDER)/,$(addsuffix .o, $(notdir $(ML_SRCS))))

####################################################################################################
## macros

define echo_in_colors
@echo -ne "\e[1;32m"$(1)"\e[0m"
endef 

####################################################################################################
## rules

.PHONY: all clean cleanall

all: $(LIBRARY) $(MK_NAME)

$(LIBRARY) : $(OBJFOLDER) $(ML_OBJS_DST) $(MK_NAME)
	@$(call echo_in_colors, "\n<linking $(LIBRARY) with objects $(ML_OBJS_DST)\n")
	@$(call echo_in_colors, "\n<linking $(EXEC) with objects $(ML_OBJS_DST) and libraries $(ML_LIBS)\n")
	$(LINK) -o $(LIBRARY) $(ML_OBJS_DST)

$(OBJFOLDER) : 
	@$(call echo_in_colors, "\n<creating object's folder 'obj/'>\n")
	mkdir obj

$(ML_OBJS_DST) : $(OBJFOLDER)/%.c.o : %.c  $(MK_NAME)
	@$(call echo_in_colors, "\n<compile $< to $(OBJFOLDER)/$(notdir $@)>\n")
	$(COMP) $(ANDROID_SRC) $(CFLAGS) -o $@ -c $<

clean : 
	rm -fR $(OBJFOLDER)

cleanall : 
	rm -fR $(LIBRARY) $(OBJFOLDER)

