/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/**
 *  Implementation of mpld exported IPC interface
 */
 
 
#include <utils/Log.h>
#include <stdint.h>
#include <sys/types.h>

#include <binder/MemoryHeapBase.h>
#include <Impld.h>

namespace android {

enum {
    GYRO_ON,
    GYRO_OFF,
    COMPASS_ON,
    COMPASS_OFF,
    GET_BIASES,
    SET_BIASES,
};

/* --- Client side --- */
class Bpmpld: public BpInterface<Impld> {
public:
  Bpmpld(const sp<IBinder>& impl) : BpInterface<Impld>(impl)
    {
    }


  virtual int rpcGyroOn() {
      Parcel data, reply;
      int rv;
      data.writeInterfaceToken(Impld::getInterfaceDescriptor());
      remote()->transact(GYRO_ON, data, &reply);
      rv = reply.readInt32();
      return rv;
  }
  virtual int rpcGyroOff() {
      Parcel data, reply;
      int rv;
      data.writeInterfaceToken(Impld::getInterfaceDescriptor());
      remote()->transact(GYRO_OFF, data, &reply);
      rv = reply.readInt32();
      return rv;
  }
  virtual int rpcCompassOn() {
      Parcel data, reply;
      int rv;
      data.writeInterfaceToken(Impld::getInterfaceDescriptor());
      remote()->transact(COMPASS_ON, data, &reply);
      rv = reply.readInt32();
      return rv;
  }
  virtual int rpcCompassOff(){
      Parcel data, reply;
      int rv;
      data.writeInterfaceToken(Impld::getInterfaceDescriptor());
      remote()->transact(COMPASS_OFF, data, &reply);
      rv = reply.readInt32();
      return rv;
  }
  virtual int rpcGetBiases(float* b){
      Parcel data, reply;
      int rv;
      int i;
      int len = 9;
      data.writeInterfaceToken(Impld::getInterfaceDescriptor());
      remote()->transact(GET_BIASES, data, &reply);
      for (i = 0; i < len; i++)
                b[i]=reply.readFloat();
      rv = reply.readInt32();
      return rv;
  }
  virtual int rpcSetBiases(float* b){
      Parcel data, reply;
      int rv;
      int i;
      int len = 9;
      data.writeInterfaceToken(Impld::getInterfaceDescriptor());
      for (i = 0; i < len; i++)
          data.writeFloat(b[i]);
      remote()->transact(SET_BIASES, data, &reply);
      rv = reply.readInt32();
      return rv;
  }
};

IMPLEMENT_META_INTERFACE(mpld, "android.vendor.Impld");

/* --- Server side --- */

status_t Bnmpld::onTransact(uint32_t code, const Parcel& data, Parcel* reply, uint32_t flags) {
  switch (code)
  {
 
    case GYRO_ON:
    {
        CHECK_INTERFACE(Impld, data, reply);
        int rv;
        rv = rpcGyroOn();
        reply->writeInt32(rv);
        return NO_ERROR;
        break;
    }
    case GYRO_OFF:
    {
        CHECK_INTERFACE(Impld, data, reply);
        int rv;
        rv = rpcGyroOff();
        reply->writeInt32(rv);
        return NO_ERROR;
        break;
    }
    case COMPASS_ON:
    {
        CHECK_INTERFACE(Impld, data, reply);
        int rv;
        rv = rpcCompassOn();
        reply->writeInt32(rv);
        return NO_ERROR;
        break;
    }
    case COMPASS_OFF:
    {
        CHECK_INTERFACE(Impld, data, reply);
        int rv;
        rv = rpcCompassOff();
        reply->writeInt32(rv);
        return NO_ERROR;
        break;
    }
    case GET_BIASES:
    {
        CHECK_INTERFACE(Impld, data, reply);
        int rv,i;
        float b[9];
        rv = rpcGetBiases(b);
        for(i=0;i<9;i++)
            reply->writeFloat(b[i]);
        reply->writeInt32(rv);
        return NO_ERROR;
        break;
    }
    case SET_BIASES:
    {
        CHECK_INTERFACE(Impld, data, reply);
        int rv,i;
        float b[9];
        for(i=0;i<9;i++)
            b[i] = data.readFloat();
        rv = rpcSetBiases(b);
        reply->writeInt32(rv);
        return NO_ERROR;
        break;
    }
    default:
      return BBinder::onTransact(code, data, reply, flags);
  }
}

}; // namespace android


