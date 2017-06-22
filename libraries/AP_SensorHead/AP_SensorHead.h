#pragma once

#include "Protocol.h"

#include <AP_Common/AP_Common.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
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

/*
 * SensorHead Main Class
 * NOTE: Must be initialized before the Sensors, otherwise registerSensor will
 * fail.
 */
class AP_SensorHead {
public:
    // TODO: What is the right number here?
    /* Receive Thread Rate */
    static const uint16_t UPDATE_RATE_HZ = 400;

    AP_SensorHead();

    /* Sensor Registration */
    void registerSensor(AP_Baro *baro)
    {
        _baro = baro;
    }
    void registerSensor(AP_InertialSensor *ins)
    {
        _ins = ins;
    }

    void registerSensor(Compass *compass)
    {
        _compass = compass;
    }

    void registerSensor(AP_GPS *gps)
    {
        _gps = gps;
    }


    /* Handler Registration */
    void registerHandler(AP_SensorHead_Handler<BaroMessage> *handler)
    {
        _baroHandler = handler;
    }

    void
    registerHandler(AP_SensorHead_Handler<InertialSensorMessage> *handler)
    {
        _insHandler = handler;
    }

    void registerHandler(AP_SensorHead_Handler<CompassMessage> *handler)
    {
        _compassHandler = handler;
    }

    void registerHandler(AP_SensorHead_Handler<GPSMessage> *handler)
    {
        _gpsHandler = handler;
    }

    bool isReady() {
        return handlerReady() && sensorReady();
    }

    /* Singleton methods */
    static AP_SensorHead *init(); // TODO: Rename to init_instance()
    static AP_SensorHead *get_instance()
    {
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

    /*
     * Given a buffer decode packet & call handler.
     */
    bool read(uint8_t *buf, size_t len);

    /*
     * Each Message type implements a specialized write()
     * NOTE: Taking buf as argument allows for greater flexibility, but maybe it
     * makes sense make member variable.
     */
    template <class T>
    void  write(uint8_t *buf, size_t len);

private:
    static bool _initialized;
    static AP_SensorHead _instance;

    bool handlerReady() {
        return _baroHandler
            && _insHandler
            && _compassHandler
            && _gpsHandler;
    }

    bool sensorReady() {
        return _baro
            && _compass
            && _ins
            && _gps;
    }

    // TODO: Add more sensors.
    AP_Baro *_baro;
    Compass *_compass;
    AP_InertialSensor *_ins;
    AP_GPS *_gps;

    /* Message Handlers */
    AP_SensorHead_Handler<BaroMessage> *_baroHandler;
    AP_SensorHead_Handler<InertialSensorMessage> *_insHandler;
    AP_SensorHead_Handler<CompassMessage> *_compassHandler;
    AP_SensorHead_Handler<UnknownMessage> _defaultHandler;
    AP_SensorHead_Handler<GPSMessage> *_gpsHandler;


    AP_HAL::Util::perf_counter_t _perf_read;
    AP_HAL::Util::perf_counter_t _perf_write;
};
