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
//TODO: remove ifdefs when AP_Perf is merged
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_SHEAD_SLAVE
    _perf_read = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "shead_read");
    _perf_write = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "shead_write");
#endif
    return true;
}

bool AP_SensorHead::handlePacket(Packet::raw_t *packet)
{

    // TODO: Order according to frequency of appearance.
    switch (Packet::id(packet)) {
    case msgid_t::INS: {
        _handlePacketHelper<InertialSensorMessage,
                            AP_SensorHead_Handler<InertialSensorMessage> >(packet, _insHandler);
        return true;
        break;
    }
    case msgid_t::BARO: {
        _handlePacketHelper<BaroMessage,
                            AP_SensorHead_Handler<BaroMessage> >(packet, _baroHandler);
        return true;
        break;
    }
    case msgid_t::COMPASS: {
        _handlePacketHelper<CompassMessage,
                            AP_SensorHead_Handler<CompassMessage> >(packet, _compassHandler);
        return true;
        break;
    }
    case msgid_t::GPS: {
        _handlePacketHelper<GPSMessage,
                            AP_SensorHead_Handler<GPSMessage> >(packet, _gpsHandler);
        return true;
        break;
    }
    default:
        _defaultHandler.handle(nullptr);
        return false;
        break;
    }
}

bool AP_SensorHead::read(uint8_t *buf, size_t len)
{
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_SHEAD_SLAVE
    hal.util->perf_begin(_perf_read);
#endif

    Packet::raw_t p;
    bool handled = false;
    bool decoded = Packet::decode(&p, buf, len);

    if (decoded) {
        handled = handlePacket(&p);
    }

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_SHEAD_SLAVE
    hal.util->perf_end(_perf_read);
#endif

    return handled;
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
