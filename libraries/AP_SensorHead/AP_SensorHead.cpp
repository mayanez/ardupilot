#include "AP_SensorHead.h"

extern const AP_HAL::HAL& hal;

#if HAL_SHEAD_ENABLED
using namespace SensorHead;

AP_SensorHead AP_SensorHead::_instance{};
bool AP_SensorHead::_initialized = false;

AP_SensorHead *AP_SensorHead::init_instance()
{
    _initialized = true;
    return &_instance;
}

bool AP_SensorHead::init()
{
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_SHEAD_SLAVE
    _perf_read = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "shead_read");
    _perf_write = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "shead_write");
#endif
    return true;
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
        if (!_insHandler) {
            return;
        }
        _insHandler->handle(data);
        break;
    }
    case msgid_t::BARO: {
        if (!Message::verify<BaroMessage>(packet)) {
            return;
        }
        BaroMessage::data_t *data = Message::decode<BaroMessage>(packet);
        if (!_baroHandler) {
            return;
        }
        _baroHandler->handle(data);
        break;
    }
    case msgid_t::COMPASS: {
        if (!Message::verify<CompassMessage>(packet)) {
            return;
        }
        CompassMessage::data_t *data = Message::decode<CompassMessage>(packet);
        if (!_compassHandler) {
            return;
        }
        _compassHandler->handle(data);
        break;
    }
    case msgid_t::GPS: {
        if (!Message::verify<GPSMessage>(packet)) {
            return;
        }
        GPSMessage::data_t *data = Message::decode<GPSMessage>(packet);
        if (!_gpsHandler) {
            return;
        }
        _gpsHandler->handle(data);
        break;
    }
    default:
        _defaultHandler.handle(nullptr);
        break;
    }
}

bool AP_SensorHead::read(uint8_t *buf, size_t len)
{
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_SHEAD_SLAVE
    hal.util->perf_begin(_perf_read);
#endif

    Packet::raw_t p;
    bool decoded = Packet::decode(&p, buf, len);
    if (decoded) {
        handlePacket(&p);
    }

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_SHEAD_SLAVE
    hal.util->perf_end(_perf_read);
#endif

    return decoded;
}

template <>
void AP_SensorHead::write<BaroMessage>(uint8_t *buf, size_t len)
{
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_SHEAD_SLAVE
    hal.util->perf_begin(_perf_write);
#endif
    if (!_baro) {
        return;
    }

    BaroMessage msg{buf, len};
    msg.setPressure(_baro->get_pressure());
    msg.setTemperature(_baro->get_temperature());
    msg.encode();

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_SHEAD_SLAVE
    hal.util->perf_end(_perf_write);
#endif
}

template <>
void AP_SensorHead::write<InertialSensorMessage>(uint8_t *buf, size_t len)
{
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_SHEAD_SLAVE
    hal.util->perf_begin(_perf_write);
#endif
    if (!_ins) {
        return;
    }

    InertialSensorMessage msg{buf, len};
    msg.setGyro(_ins->get_gyro());
    msg.setAccel(_ins->get_accel());
    msg.encode();

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_SHEAD_SLAVE
    hal.util->perf_end(_perf_write);
#endif
}

template <>
void AP_SensorHead::write<CompassMessage>(uint8_t *buf, size_t len)
{
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_SHEAD_SLAVE
    hal.util->perf_begin(_perf_write);
#endif
    if (!_compass) {
        return;
    }

    CompassMessage msg{buf, len};
    msg.setField(_compass->get_raw_field());
    msg.encode();

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_SHEAD_SLAVE
    hal.util->perf_end(_perf_write);
#endif
}
template <>
void AP_SensorHead::write<GPSMessage>(uint8_t *buf, size_t len)
{
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_SHEAD_SLAVE
    hal.util->perf_begin(_perf_write);
#endif
    if (!_gps) {
        return;
    }

    GPSMessage msg{buf, len};
    msg.setState(_gps->get_state());
    msg.encode();

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_SHEAD_SLAVE
    hal.util->perf_end(_perf_write);
#endif
}
#endif // HAL_SHEAD_ENABLED
