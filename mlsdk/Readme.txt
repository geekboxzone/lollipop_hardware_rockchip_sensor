**************************************************************************
**   InvenSense Motion Processing Library - README.TXT
**************************************************************************

This file briefly explains how to use the InvenSense Motion Processing
Library (MPL) and examples.  This document corresponds with Android Beagle 
MPL Alpha Release.

MPL contains the code for controlling the InvenSense MPU-3050 series
gyroscope, including activating and managing built in motion processing
features.  All of the application source code is in ANSI C and can be 
compiled in C or C++ environments.  This code is designed to work with 
all MPU-3050 devices revision K or earlier.

*************************************************************************

Build instructions :

the static and shared makefiles build against the Android Framework structure.  
The makefiles expect to receive:
- the location of the Android framework root in the ANDROID_ROOT variable;
- the location of the kernel root folder in the KERNEL_ROOT variable;
- the location of the Android cross compiling toolchain, in the CROSS variable;
- the target platform of the build, in the PRODUCT variable;
- the target OS, in the TARGET variable;
- an optional VERBOSE variable to enable verbose output from the build.

See the top-level Makefile for details: 
- Android-static.mk
- Android-shaerd.mk

Example on how to run the build processes:

MAKE_CMD="make \
    VERBOSE=0 \  
    TARGET=android \
    CROSS=/Software/Android/trunk/0xdroid/beagle-eclair/prebuilt/linux-x86/toolchain/arm-eabi-4.4.0/bin/arm-eabi- \ 
    ANDROID_ROOT=/Software/Android/trunk/0xdroid/beagle-eclair \
    KERNEL_ROOT=/Software/Android/trunk/0xdroid/kernel \
    PRODUCT=beagleboard \
"
eval $MAKE_CMD -f Android-static.mk
eval $MAKE_CMD -f Android-static.mk clean
eval $MAKE_CMD -f Android-shared.mk
eval $MAKE_CMD -f Android-shared.mk clean

sudo make VERBOSE=1 TARGET=android CROSS=/home/lllyx/share/rk29/gingerbread/prebuilt/linux-x86/toolchain/arm-eabi-4.4.0/bin/arm-eabi-  ANDROID_ROOT=/home/lllyx/share/rk29/gingerbread  KERNEL_ROOT=/home/lllyx/share/rk29/kernel  PRODUCT=rk29sdk  -f Android-static.mk
