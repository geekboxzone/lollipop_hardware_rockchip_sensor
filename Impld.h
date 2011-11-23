/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/**

  Binder interface class.  Shared by clients and server.  Defines the IPC calls that the server exports.

*/


#ifndef IMPLD_H_
#define IMPLD_H_

#include <utils/RefBase.h>
#include <binder/IInterface.h>
#include <binder/Parcel.h>
#include <binder/IMemory.h>
#include <utils/Timers.h>
#include <cutils/native_handle.h>


namespace android {

class Impld: public IInterface
{
public:
  DECLARE_META_INTERFACE(mpld);

  virtual int rpcGyroOn() = 0;
  virtual int rpcGyroOff() = 0;
  virtual int rpcCompassOn() = 0;
  virtual int rpcCompassOff() = 0;
  virtual int rpcGetBiases(float* b) = 0;
  virtual int rpcSetBiases(float* b) = 0;

};

/* --- Server side --- */

class Bnmpld: public BnInterface<Impld> {
public:
  virtual status_t    onTransact( uint32_t code,
                                  const Parcel& data,
                                  Parcel* reply,
                                  uint32_t flags = 0); };

}; // namespace android

#endif /* IMPLD_H_ */

