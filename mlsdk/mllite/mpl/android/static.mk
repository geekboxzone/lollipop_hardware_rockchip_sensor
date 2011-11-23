MLLITE_LIB_NAME = mllite
LIBRARY = $(LIB_PREFIX)$(MLLITE_LIB_NAME).$(STATIC_LIB_EXT)

MK_NAME = $(notdir $(CURDIR)/$(firstword $(MAKEFILE_LIST)))

OBJFOLDER = $(CURDIR)/obj

CROSS = arm-none-linux-gnueabi-
COMP= $(CROSS)gcc
LINK= $(CROSS)ar cr

MLSDK_ROOT = ../../..
MLLITE_DIR = ../..
MPL_DIR = $(MLSDK_ROOT)/mldmp
MLPLATFORM_DIR = $(MLSDK_ROOT)/platform

include $(MLSDK_ROOT)/Android-common.mk

CFLAGS += $(CMDLINE_CFLAGS)
CFLAGS += -Wall -fpic
CFLAGS += -mthumb-interwork 
CFLAGS += -fno-exceptions -ffunction-sections -funwind-tables 
CFLAGS += -fstack-protector -fno-short-enums -fmessage-length=0
CFLAGS += -DNDEBUG
CFLAGS += -D_REENTRANT -DLINUX -DANDROID
#CFLAGS += -DBEAGLEBOARD
#CFLAGS += -DMLMATH 
CFLAGS += -I$(MLLITE_DIR) -I$(MLPLATFORM_DIR)/include -I$(MPL_DIR)
CFLAGS += -I$(MLSDK_ROOT)/mlutils -I$(MLSDK_ROOT)/mlapps/common

VPATH += $(MLLITE_DIR) 
VPATH += $(MLSDK_ROOT)/mlutils
VPATH += $(MLLITE_DIR)/accel $(MLLITE_DIR)/compass $(MLLITE_DIR)/pressure 
VPATH += $(MLLITE_DIR)/mlapps/common

####################################################################################################
## sources

MLPLATFORM_LIB = \
	$(MLPLATFORM_DIR)/linux/$(LIB_PREFIX)$(MLPLATFORM_LIB_NAME).$(STATIC_LIB_EXT)

ML_SOURCES = \
	$(MLLITE_DIR)/accel.c \
	$(MLLITE_DIR)/compass.c \
	$(MLLITE_DIR)/pressure.c \
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
	\
	$(MLLITE_DIR)/ml_mputest.c \
	$(MLSDK_ROOT)/mlutils/mputest.c \
	$(MLSDK_ROOT)/mlutils/checksum.c

#ML_SOURCES += \
	$(MLLITE_DIR)/accel/kxtf9.c \
	$(MLLITE_DIR)/accel/kxsd9.c \
	$(MLLITE_DIR)/accel/mma8450.c \
	$(MLLITE_DIR)/accel/mma845x.c	\
	$(MLLITE_DIR)/accel/bma150.c \
	$(MLLITE_DIR)/accel/bma222.c \
	$(MLLITE_DIR)/accel/adxl346.c \
	$(MLLITE_DIR)/accel/lis331.c \
	$(MLLITE_DIR)/accel/lis3dh.c \
	$(MLLITE_DIR)/accel/lsm303a.c \
	\
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
	$(MLLITE_DIR)/pressure/bma085.c 

ifeq ($(HARDWARE),M_HW)
	ML_SOURCES += $(MLLITE_DIR)/accel/mantis.c
endif

ML_OBJS := $(addsuffix .o,$(ML_SOURCES))
ML_OBJS_DST = $(addprefix $(OBJFOLDER)/,$(addsuffix .o, $(notdir $(ML_SOURCES))))

####################################################################################################
## rules

.PHONY: all clean cleanall

all: $(LIBRARY) $(MK_NAME)

$(LIBRARY) : $(OBJFOLDER) $(ML_OBJS_DST) $(MLPLATFORM_LIB) $(MK_NAME)
	@$(call echo_in_colors, "\n<linking $(LIBRARY) with objects $(ML_OBJS_DST)\n")
	$(LINK) $(LIBRARY) $(ML_OBJS_DST)
	$(CROSS)ranlib $(LIBRARY)

$(OBJFOLDER) : 
	@$(call echo_in_colors, "\n<creating object's folder 'obj/'>\n")
	mkdir obj

$(ML_OBJS_DST) : $(OBJFOLDER)/%.c.o : %.c  $(MK_NAME)
	@$(call echo_in_colors, "\n<compile $< to $(OBJFOLDER)/$(notdir $@)>\n")
	$(COMP) $(CFLAGS) $(ANDROID_INCLUDES) $(KERNEL_INCLUDES) $(MLSDK_INCLUDES) -o $@ -c $<

clean : 
	rm -fR $(OBJFOLDER)

cleanall : 
	rm -fR $(LIBRARY) $(OBJFOLDER)

