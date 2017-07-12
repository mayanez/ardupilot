#include "AP_SensorHub.h"

#if HAL_SENSORHUB_ENABLED

extern const AP_HAL::HAL& hal;

using namespace SensorHub;

AP_SensorHub AP_SensorHub::_instance{};
bool AP_SensorHub::_initialized = false;

AP_SensorHub *AP_SensorHub::init_instance()
{
    _initialized = true;
    return &_instance;
}

bool AP_SensorHub::init()
{
//TODO: Add perf when AP_Perf PR is merged.
    return true;
}

void AP_SensorHub::registerIO(AP_SensorHub_IO *io)
{
    if (_io_count < SENSORHUB_MAX_PORTS) {
        _port[_io_count++] = io;
        io->setSensorHub(this);
    } else {
        AP_HAL::panic("Too many IO ports\n.");
    }
}

bool AP_SensorHub::handlePacket(Packet::packet_t *packet)
{

    // NOTE: These should be ordered from most to least frequent.
    switch (Packet::id(packet)) {
    case msgid_t::GYRO: {
        _handlePacketHelper<GyroMessage,
                            AP_SensorHub_Handler<GyroMessage> >(packet, _gyroHandler);
        return true;
        break;
    }
    case msgid_t::ACCEL: {
        _handlePacketHelper<AccelMessage,
                            AP_SensorHub_Handler<AccelMessage> >(packet, _accelHandler);
        return true;
        break;
    }
    case msgid_t::COMPASS: {
        _handlePacketHelper<CompassMessage,
                            AP_SensorHub_Handler<CompassMessage> >(packet, _compassHandler);
        return true;
        break;
    }
    case msgid_t::BARO: {
        _handlePacketHelper<BaroMessage,
                            AP_SensorHub_Handler<BaroMessage> >(packet, _baroHandler);
        return true;
        break;
    }
    case msgid_t::GPS: {
        _handlePacketHelper<GPSMessage,
                            AP_SensorHub_Handler<GPSMessage> >(packet, _gpsHandler);
        return true;
        break;
    }
    default:
        _defaultHandler.handle(nullptr);
        return false;
        break;
    }
}

int AP_SensorHub::read(uint8_t *buf, size_t len)
{
    Packet::packet_t p;
    auto decoded = Packet::decode(&p, buf, len);

    if (static_cast<decode_t>(decoded) == decode_t::SUCCESS) {
        handlePacket(&p);
    }

    return decoded;
}


#endif
