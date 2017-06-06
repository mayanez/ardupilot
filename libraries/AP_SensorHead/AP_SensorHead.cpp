#include "AP_SensorHead.h"

using namespace SensorHead;

AP_SensorHead AP_SensorHead::_instance{};
bool AP_SensorHead::_initialized = false;

AP_SensorHead *AP_SensorHead::init() {
    _initialized = true;
    return &_instance;
}

void AP_SensorHead::handlePacket(Packet::raw_t *packet) {

    switch(Packet::id(packet)) {
    case msgid_t::BARO:
    {
        if(!Message::verify<BaroMessage>(packet)) return;
        BaroMessage::data_t *data = Message::decode<BaroMessage>(packet);
        _baroHandler->handle(data);
        break;
    }
    case msgid_t::INS:
    {
        if(!Message::verify<InertialSensorMessage>(packet)) return;
        InertialSensorMessage::data_t *data =
            Message::decode<InertialSensorMessage>(packet);
        _insHandler->handle(data);
        break;
    }
    default:
        _defaultHandler->handle(nullptr);
        break;
    }
}

bool AP_SensorHead::recv(uint8_t *buf, uint32_t len) {
    Packet::raw_t *p = nullptr;
    bool decoded = Packet::decode(p, buf, len);
    if(decoded) {
        handlePacket(p);
    }
    return decoded;
}

bool AP_SensorHead::send_baro(AP_HAL::Stream *stream) {
    BaroMessage msg;
    msg.pressure(_baro->get_pressure()).
        temperature(_baro->get_temperature());

    Packet *packet = msg.encode();

    size_t bytes = stream->write(reinterpret_cast<uint8_t *>(packet->raw()),
                    packet->length());

    return bytes == packet->length();
}

bool AP_SensorHead::send_ins(AP_HAL::Stream *stream) {
    InertialSensorMessage msg;
    msg.gyro(_ins->get_gyro());

    Packet *packet = msg.encode();

    size_t bytes = stream->write(reinterpret_cast<uint8_t *>(packet->raw()),
                    packet->length());

    return bytes == packet->length();
}
