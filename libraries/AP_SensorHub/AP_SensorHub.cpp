#include "AP_SensorHub.h"
#include <DataFlash/DataFlash.h>
#include <AP_Math/AP_Math.h>

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

    #if SENSORHUB_DEBUG
    hal.console->printf("AP_SensorHub - Handling %s\n", msgid_t_names[static_cast<int>(Packet::id(packet))]);
    #endif

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
    auto begin = AP_HAL::micros64();
    auto decoded = Packet::decode(&p, buf, len);
    auto decode_status = static_cast<decode_t>(decoded) == decode_t::SUCCESS;
    if (decode_status) {
        handlePacket(&p);

        if (!_notFirstPacket) {
            _notFirstPacket = true;
        } else {
            if (p.hdr.seq != _readSeq + 1) {
                // A packet was dropped.

                if (p.hdr.seq < _readSeq) {
                    // Seq has wrapped around.
                    _packetLoss += UINT32_MAX - _readSeq;
                } else {
                    _packetLoss += p.hdr.seq - _readSeq;
                }

                #if SENSORHUB_DEBUG
                hal.console->printf("AP_SensorHub - Packet Lost: %u\n", _packetLoss);
                #endif
            }
        }

        _readSeq = p.hdr.seq;
        _lastPacketTime = begin;
    }

    if (_dataflash) {
        auto now = AP_HAL::micros64();
        auto processTime = now - begin;
        struct log_SHUB_RW pkt = {
            LOG_PACKET_HEADER_INIT((uint8_t)(LOG_SHUB_RW_MSG)),
            time_us : now,
            process_time : processTime,
            packet_loss : _packetLoss,
            msg_id : static_cast<uint8_t>(Packet::id(&p)),
            status : decode_status,
            rw  : 0
        };
        _dataflash->WriteBlock(&pkt, sizeof(pkt));
    }

    return decoded;
}

void AP_SensorHub::write(Packet::packet_t *packet) {
    auto begin = AP_HAL::micros64();

    for (int i = 0; i < SENSORHUB_MAX_PORTS; i++) {
        if (_port[i]) {
            _port[i]->write(packet, Packet::length(packet));
        }
    }

    if (_dataflash) {
        auto now = AP_HAL::micros64();
        auto processTime = now - begin;
        struct log_SHUB_RW pkt = {
            LOG_PACKET_HEADER_INIT((uint8_t)(LOG_SHUB_RW_MSG)),
            time_us : now,
            process_time : processTime,
            packet_loss : _packetLoss,
            msg_id : static_cast<uint8_t>(Packet::id(packet)),
            status : 1, // TODO: consider making write return a value to use.
            rw : 1
        };
        _dataflash->WriteBlock(&pkt, sizeof(pkt));
    }
}

#endif
