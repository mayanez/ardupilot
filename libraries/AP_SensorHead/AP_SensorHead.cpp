#include "AP_SensorHead.h"

extern const AP_HAL::HAL& hal;

using namespace SensorHead;

AP_SensorHead AP_SensorHead::_instance{};
bool AP_SensorHead::_initialized = false;

AP_SensorHead *AP_SensorHead::init()
{
    _initialized = true;
    return &_instance;
}

AP_SensorHead::AP_SensorHead()
{
    // TODO: Segfault for some reason.
    // _perf_read = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "shead_read");
    // _perf_write = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "shead_write");
}

void AP_SensorHead::handlePacket(Packet::raw_t *packet)
{

    // TODO: Order according to frequency of appearance.
    switch (Packet::id(packet)) {
    case msgid_t::INS: {
        if (!Message::verify<InertialSensorMessage>(packet)) {
            return;
        }
        InertialSensorMessage::data_t *data =
            Message::decode<InertialSensorMessage>(packet);
        _insHandler->handle(data);
        break;
    }
    case msgid_t::BARO: {
        if (!Message::verify<BaroMessage>(packet)) {
            return;
        }
        BaroMessage::data_t *data = Message::decode<BaroMessage>(packet);
        _baroHandler->handle(data);
        break;
    }
    case msgid_t::COMPASS: {
        hal.console->printf("Compass: %f\n", (double)AP_HAL::micros64());
        if (!Message::verify<CompassMessage>(packet)) {
            return;
        }
        CompassMessage::data_t *data = Message::decode<CompassMessage>(packet);
        _compassHandler->handle(data);
        break;
    }
    case msgid_t::GPS: {
        //hal.console->printf("GPS: %f\n", (double)AP_HAL::micros64());
        if (!Message::verify<GPSMessage>(packet)) {
            return;
        }
        GPSMessage::data_t *data = Message::decode<GPSMessage>(packet);
        _gpsHandler->handle(data);
        break;
    }
    default:
        _defaultHandler->handle(nullptr);
        break;
    }
}

bool AP_SensorHead::read(uint8_t *buf, size_t len)
{
    // TODO: Is there a decorator for these?
    // hal.util->perf_begin(_perf_read);

    Packet::raw_t p;
    bool decoded = Packet::decode(&p, buf, len);
    if (decoded) {
        handlePacket(&p);
    }

    // hal.util->perf_end(_perf_read);

    return decoded;
}

template <>
void AP_SensorHead::write<BaroMessage>(uint8_t *buf, size_t len)
{
    // hal.util->perf_begin(_perf_write);

    BaroMessage msg{buf, len};
    msg.setPressure(_baro->get_pressure());
    msg.setTemperature(_baro->get_temperature());
    msg.encode();

    // hal.util->perf_end(_perf_write);
}

template <>
void AP_SensorHead::write<InertialSensorMessage>(uint8_t *buf, size_t len)
{
    // hal.util->perf_begin(_perf_write);

    InertialSensorMessage msg{buf, len};
    msg.setGyro(_ins->get_gyro());
    msg.setAccel(_ins->get_accel());
    msg.encode();

    // hal.util->perf_end(_perf_write);
}

template <>
void AP_SensorHead::write<CompassMessage>(uint8_t *buf, size_t len)
{
    CompassMessage msg{buf, len};
    msg.setField(_compass->get_raw_field());
    msg.encode();
}
template <>
void AP_SensorHead::write<GPSMessage>(uint8_t *buf, size_t len)
{
    GPSMessage msg{buf, len};
    msg.setState(_gps->get_state());
    msg.encode();
}
