#pragma once

#include "Protocol.h"

#include <AP_Common/AP_Common.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_HAL/AP_HAL.h>

using namespace SensorHead;

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

    AP_SensorHead &stream(AP_HAL::Stream *stream) {
        _stream = stream;
        return *this;
    }

    // TODO: There are ways to get rid of this duplication with templating, but
    // has high stack cost. Since it should only be a few types of messages this
    // doesn't seem too bad. Otherwise, it can be revisited.
    // Solution is something like this: https://stackoverflow.com/questions/27941661/generating-one-class-member-per-variadic-template-argument
    AP_SensorHead &registerHandler(AP_SensorHead_Handler<BaroMessage> *handler) {
        _baroHandler = handler;
        return *this;
    }

    AP_SensorHead &registerHandler(AP_SensorHead_Handler<InertialSensorMessage> *handler) {
        _insHandler = handler;
        return *this;
    }

    void handlePacket(Packet::raw_t *packet) {

        switch(Packet::id(packet)) {
        case msgid_t::BARO:
        {
            if(!Packet::verify<BaroMessage>(packet)) return;
            BaroMessage::data_t *data = BaroMessage::decode(packet);
            _baroHandler->handle(data);
            break;
        }
        case msgid_t::INS:
        {
            if(!Packet::verify<InertialSensorMessage>(packet)) return;
            InertialSensorMessage::data_t *data =
                InertialSensorMessage::decode(packet);
            _insHandler->handle(data);
            break;
        }
        }
    }

    void baro_update() {
        BaroMessage msg;
        msg.pressure(_baro->get_pressure()).
            temperature(_baro->get_temperature());

        Packet *packet = msg.encode();

        _stream->write(reinterpret_cast<uint8_t *>(packet->raw()),
                       packet->length());
    }

    void ins_update() {
        InertialSensorMessage msg;
        msg.gyro(_ins->get_gyro());

        Packet *packet = msg.encode();

        _stream->write(reinterpret_cast<uint8_t *>(packet->raw()),
                       packet->length());
    }

    // TODO: To be called from a thread. What this does might depend on
    // underlying stream? Essentially should ready a chunk from the stream and
    // decode to a Packet::raw_t and call handlePacket().
    void recv() {};

  private:
    AP_HAL::Stream *_stream;
    AP_Baro *_baro;
    AP_SensorHead_Handler<BaroMessage> *_baroHandler;
    AP_SensorHead_Handler<InertialSensorMessage> *_insHandler;
    AP_InertialSensor *_ins;
};
