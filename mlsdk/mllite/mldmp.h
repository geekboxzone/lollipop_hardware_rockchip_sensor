/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/***************************************************************************** *
 * $Id: mldmp.h 3863 2010-10-08 22:05:31Z nroyer $
 ******************************************************************************/

/**
 * @defgroup MLDMP
 * @brief 
 *
 *  These are the top level functions that define how to load the MPL.  In order
 *  to use most of the features, the DMP must be loaded with some code.  The 
 *  loading procedure takes place when calling MLDmpOpen with a given DMP set 
 *  function, after having open the serial communication with the device via 
 *  MLSerialOpen().  
 *  The DMP set function will load the DMP memory and enable a certain
 *  set of features.
 *
 *  First select a DMP version from one of the released DMP sets.  
 *  These could be:
 *  - DMP default to load and use the default DMP code featuring pedometer, 
 *    gestures, and orientation.  Use MLDmpOpen().
 *  - DMP pedometer stand-alone to load and use the standalone pedometer
 *    implementation. Use MLDmpPedometerStandAloneOpen().
 *  <!-- - DMP EIS ... Use MLDmpEisOpen(). -->
 *
 *  After MLDmpOpenXXX any number of appropriate initialization and configuration 
 *  routines can be called. Each one of these routines will return an error code 
 *  and will check to make sure that it is compatible with the the DMP version 
 *  selected during the call to MLDmpOpen.
 *
 *  Once the configuration is complete, make a call to MLDmpStart(). This will
 *  finally turn on the DMP and run the code previously loaded.
 *
 *  While the DMP is running, all data fetching, polling or other functions can 
 *  be called and will return valid data. Some parameteres can be changed while 
 *  the DMP is runing, while others cannot.  Therefore it is important to always 
 *  check the return code of each function.  Check the error code list in mltypes
 *  to know what each returned error corresponds to.
 *
 *  When no more motion processing is required, the library can be shut down and
 *  the DMP turned off.  We can do that by calling MLDmpClose().  Note that 
 *  MLDmpClose() will not close the serial communication automatically, which will
 *  remain open an active, in case another module needs to be loaded instead.
 *  If the intention is shutting down the MPL as well, an explicit call to 
 *  MLSerialClose() following MLDmpClose() has to be made.
 *
 *  The MPL additionally implements a basic state machine, whose purpose is to
 *  give feedback to the user on whether he is following all the required 
 *  initialization steps.  If an anomalous transition is detected, the user will
 *  be warned by a terminal message with the format:
 *
 *  <tt>"Error : illegal state transition from STATE_1 to STATE_3"</tt>
 *
 *  @{
 *      @file     mldmp.h
 *      @brief    Top level entry functions to the MPL library with DMP support
 */

#ifndef MLDMP_H
#define MLDMP_H

#ifdef __cplusplus
extern "C" {
#endif

    tMLError MLDmpOpen(void);
    tMLError MLDmpStart(void);
    tMLError MLDmpStop(void);
    tMLError MLDmpClose(void);

#ifdef __cplusplus
}
#endif

#endif /* MLDMP_H */

/**
 * @}
**/

