#pragma once

#include "Protocol.h"

#include <AP_Common/AP_Common.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>

#if HAL_SHEAD_ENABLED
using namespace SensorHead;

/*
 * Base Class for a Message Handler
 */
template <class T>
class AP_SensorHead_Handler {
public:
    virtual void handle(typename T::data_t *data) = 0;
    virtual bool isValid(typename T::data_t *data) = 0;
};

/*
 * SensorHead Main Class
 * NOTE: Must be initialized before the Sensors, otherwise registerSensor will
 * fail.
 */
class AP_SensorHead {
public:
    static const uint16_t UPDATE_RATE_HZ = 1000;

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

    /* Singleton methods */
    static AP_SensorHead *init_instance();
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
    bool handlePacket(Packet::raw_t *packet);

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
    bool  write(uint8_t *buf, size_t len);

    bool init();

private:
    static bool _initialized;
    static AP_SensorHead _instance;

    // TODO: Add more sensors.
    AP_Baro *_baro;
    Compass *_compass;
    AP_InertialSensor *_ins;
    AP_GPS *_gps;

    /* Message Handlers */

    class UnknownMessageHandler : public AP_SensorHead_Handler<UnknownMessage> {
    public:
        virtual void handle(UnknownMessage::data_t *data) {}
        virtual bool isValid(UnknownMessage::data_t *data) { return true; }
    };

    AP_SensorHead_Handler<BaroMessage> *_baroHandler;
    AP_SensorHead_Handler<InertialSensorMessage> *_insHandler;
    AP_SensorHead_Handler<CompassMessage> *_compassHandler;
    AP_SensorHead_Handler<GPSMessage> *_gpsHandler;
    UnknownMessageHandler _defaultHandler;

    /* Utils */
    AP_HAL::Util::perf_counter_t _perf_read;
    AP_HAL::Util::perf_counter_t _perf_write;

    /* Helper Methods */
    template <class MessageType, class MessageHandler>
    void _handlePacketHelper(Packet::raw_t *packet, MessageHandler *handler)
    {
        if (!Message::verify<MessageType>(packet)) {
            return;
        }
        typename MessageType::data_t data{};
        Message::decode<MessageType>(packet, &data);
        if (!handler) {
            return;
        }
        handler->handle(&data);
    }
};
#endif // HAL_SHEAD_ENABLED
