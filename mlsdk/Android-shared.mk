SHELL=/bin/bash

TARGET 	     = android
PRODUCT      = beagleboard
ANDROID_ROOT = /Software/Android/trunk/0xdroid/beagle-eclair
KERNEL_ROOT  = /Software/Android/trunk/0xdroid/kernel

ifeq ($(VERBOSE),1)
	DUMP=2>/dev/stderr
else
	DUMP=2>/dev/null
endif

include Android-common.mk

#############################################################################
## targets

LIB_FOLDERS = \
	platform/linux \
	mllite/mpl/$(TARGET) \
	mldmp/mpl/$(TARGET)

APP_FOLDERS = \
	mlapps/DemoAppConsole/ConsoleUnix \
	mlapps/DemoAppPedometer/PedometerStandAloneUnix \

#	mlapps/DemoAppEis/EisUnix

ifneq ($(HARDWARE),M_HW)
	APP_FOLDERS += mltools/driver_selftest
endif

INSTALL_DIR = $(CURDIR)

####################################################################################################
## macros

ifndef echo_in_colors
define echo_in_colors
	echo -ne "\e[1;34m"$(1)"\e[0m"
endef	
endif

define maker_libs
	echo "MLPLATFORM_LIB_NAME = $(MLPLATFORM_LIB_NAME)"
	echo "MLLITE_LIB_NAME     = $(MLLITE_LIB_NAME)"
	echo "MPL_LIB_NAME        = $(MPL_LIB_NAME)"

	$(call echo_in_colors, "\n<making '$(1)' in folder 'platform/linux'>\n"); \
	make MLPLATFORM_LIB_NAME=$(MLPLATFORM_LIB_NAME) ANDROID_ROOT=$(ANDROID_ROOT) KERNEL_ROOT=$(KERNEL_ROOT) PRODUCT=$(PRODUCT) -C platform/linux -f Android-shared.mk $@ $(DUMP)

	$(call echo_in_colors, "\n<making '$(1)' in folder 'mllite/mpl/$(TARGET)'>\n"); \
	make MLLITE_LIB_NAME=$(MLLITE_LIB_NAME) ANDROID_ROOT=$(ANDROID_ROOT) KERNEL_ROOT=$(KERNEL_ROOT) PRODUCT=$(PRODUCT) -C mllite/mpl/$(TARGET) -f Android-shared.mk $@ $(DUMP)

	if test -f mldmp/mpl/$(TARGET)/Android-shared.mk; then \
		$(call echo_in_colors, "\n<making '$(1)' in folder 'mldmp/mpl/$(TARGET)'>\n"); \
	        make MPL_LIB_NAME=$(MPL_LIB_NAME) ANDROID_ROOT=$(ANDROID_ROOT) KERNEL_ROOT=$(KERNEL_ROOT) PRODUCT=$(PRODUCT) -C mldmp/mpl/$(TARGET) -f Android-shared.mk $@ $(DUMP); \
	fi
endef

define maker_apps
	for dir in $(APP_FOLDERS); do \
		$(call echo_in_colors, "\n<making '$(1)' in folder $$dir>\n"); \
		make -C $$dir TARGET=$(TARGET) -f Android-shared.mk $@ $(DUMP); \
	done
endef 

#############################################################################
## rules

.PHONY : all $(LIB_FOLDERS) $(APP_FOLDERS) clean cleanall install

all : 
	@$(call maker_libs,$@)
	@$(call maker_apps,$@)

clean : 
	@$(call maker_libs,$@)
	@$(call maker_apps,$@)

cleanall : 
	@$(call maker_libs,$@)
	@$(call maker_apps,$@)

