# Use bash for additional echo fancyness
SHELL = /bin/bash

LIBRARY = libmlplatform.a
TARGET = android

OBJFOLDER = $(CURDIR)/obj

CROSS = arm-none-linux-gnueabi-
COMP= $(CROSS)gcc
LINK= $(CROSS)ar cr

MLSDK_ROOT = ../..
ML_DIR = .

CFLAGS  = -static -Wall -fpic
CFLAGS += -D_REENTRANT -DBB_DEMO -DBB_RS232 -DBB_I2C -DLINUX -DBEAGLEBOARD
CFLAGS += -I$(ML_DIR) -I$(ML_DIR)/kernel -I$(ML_DIR)/../include -I$(MLSDK_ROOT)/mllite 
#CFLAGS += -DMLMATH 
CFLAGS += -I/home/chenhui/fih/gingerbread/bionic/libc/kernel/common

VPATH += $(ML_DIR) $(ML_DIR)/log

####################################################################################################
## sources

MK_NAME = Makefile

ML_SRCS = \
	$(ML_DIR)/log_linux.c \
	$(ML_DIR)/log_printf_linux.c \
	$(ML_DIR)/int_linux.c \
	$(ML_DIR)/mlos_linux.c \
	$(ML_DIR)/mlsl_linux.c

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
