# Use bash for additional echo fancyness
SHELL = /bin/bash

LIBRARY = libmllite.a
TARGET = android

OBJFOLDER = $(CURDIR)/obj

CROSS = arm-none-linux-gnueabi-
COMP= $(CROSS)gcc
LINK= $(CROSS)ar cr

MLSDK_ROOT = ../../..
ML_DIR = ../..
ML_PLATFORM = $(MLSDK_ROOT)/platform
ML_PLATFORM_LIB = $(ML_PLATFORM)/linux/libplatform.a


#CFLAGS  = -fpic -static 
CFLAGS  = -fpic
CFLAGS += -Wall 
CFLAGS += -D_REENTRANT -DLINUX 
CFLAGS += -DBB_DEMO -DBB_RS232 -DBB_I2C -DBEAGLEBOARD
#CFLAGS += -DMLMATH 
CFLAGS += -I$(ML_DIR) -I$(ML_PLATFORM)/include
CFLAGS += -I$(MLSDK_ROOT)/mlutils -I$(MLSDK_ROOT)/mlapps/common

VPATH += $(ML_DIR) 
VPATH += $(ML_DIR)/mlapps/common $(ML_DIR)/accel $(ML_DIR)/pressure $(ML_DIR)/compass $(ML_DIR)/log 
VPATH += $(MLSDK_ROOT)/mlutils


####################################################################################################
## sources

MK_NAME = $(notdir $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST)))

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
	$(LINK) $(LIBRARY) $(ML_OBJS_DST)
	$(CROSS)ranlib $(LIBRARY)

$(OBJFOLDER) : 
	@$(call echo_in_colors, "\n<creating object's folder 'obj/'>\n")
	mkdir obj

$(ML_OBJS_DST) : $(OBJFOLDER)/%.c.o : %.c  $(MK_NAME)
	@$(call echo_in_colors, "\n<compile $< to $(OBJFOLDER)/$(notdir $@)>\n")
	$(COMP) $(CFLAGS) -o $@ -c $<

clean : 
	rm -fR $(OBJFOLDER)

cleanall : 
	rm -fR $(LIBRARY) $(OBJFOLDER)

