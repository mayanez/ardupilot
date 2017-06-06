#pragma once

#include "Protocol.h"

#include <AP_Common/AP_Common.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_HAL/AP_HAL.h>

using namespace SensorHead;

/*
 * Base Class for a Message Handler
 */
template <class T>
class AP_SensorHead_Handler {
public:
    virtual void handle(typename T::data_t *data) = 0;
};

class AP_SensorHead {
public:

    AP_SensorHead &baro(AP_Baro *baro) {
        _baro = baro;
        return *this;
    }

    AP_SensorHead &ins(AP_InertialSensor *ins) {
        _ins = ins;
        return *this;
    }

    // TODO: Will this be too cumbersome? Template approach might be too memory
    // intensive.
    AP_SensorHead &
    registerHandler(AP_SensorHead_Handler<BaroMessage> *handler) {
      _baroHandler = handler;
      return *this;
    }

    AP_SensorHead &
    registerHandler(AP_SensorHead_Handler<InertialSensorMessage> *handler) {
      _insHandler = handler;
      return *this;
    }

    AP_SensorHead &
    registerHandler(AP_SensorHead_Handler<UnknownMessage> *handler) {
        _defaultHandler = handler;
        return *this;
    }

    /* Singleton methods */
    static AP_SensorHead *init();
    static AP_SensorHead *get_instance() {
        if (_initialized) {
            return &_instance;
        }
        return nullptr;
    }

    /*
     * Given a valid packet, determine its Message type & call appropriate
     * handler.
     */
    void handlePacket(Packet::raw_t *packet);
    bool recv(uint8_t *buf, uint32_t len);

    /*
     * Poll AP_Baro & create BaroMessage & write to stream.
     */
    bool send_baro(AP_HAL::Stream *stream);
    bool send_ins(AP_HAL::Stream *stream);

  private:
    static bool _initialized;
    static AP_SensorHead _instance;
    // TODO: Add more sensors.
    AP_Baro *_baro;
    AP_InertialSensor *_ins;
    AP_SensorHead_Handler<BaroMessage> *_baroHandler;
    AP_SensorHead_Handler<InertialSensorMessage> *_insHandler;
    AP_SensorHead_Handler<UnknownMessage> *_defaultHandler;
};
