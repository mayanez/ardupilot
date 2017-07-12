#include "AP_SensorHub.h"
#include <DataFlash/DataFlash.h>
#include <AP_Math/AP_Math.h>

#if HAL_SENSORHUB_ENABLED

#include "AP_SensorHub_IO_FileDescriptor.h"

#if SENSORHUB_DEBUG
static const uint8_t N_SAMPLES = 100;
static const uint8_t MSG_TYPES = static_cast<uint8_t>(msgid_t::LAST_MSG_ENTRY);
static uint32_t last_packet_us[MSG_TYPES];
static uint32_t packet_us_sample[MSG_TYPES][N_SAMPLES];
static uint32_t packet_us_sum[MSG_TYPES];
static uint32_t avg_iters[MSG_TYPES] = {1,1,1,1,1,1,1};
static uint32_t handle_iters[MSG_TYPES];
static uint32_t last_print_us;
#endif

extern const AP_HAL::HAL& hal;

using namespace SensorHub;

AP_SensorHub AP_SensorHub::_instance{};
bool AP_SensorHub::_initialized = false;

AP_SensorHub *AP_SensorHub::init_instance()
{
    _initialized = true;
    return &_instance;
}

bool AP_SensorHub::init(AP_SerialManager &serial_manager)
{
    //TODO: Add perf when AP_Perf PR is merged.

    auto shub_uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_SENSORHUB, 0);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_SENSORHUB_SINK
    // TODO: Make parameter
    this->setSinkMode();
#else
    // TODO: Make parameter
    this->setSourceMode();

#endif

    auto shub_io = new AP_SensorHub_IO_FileDescriptor();
    auto uart_fd = shub_uart->claim();
    shub_io->registerInput(uart_fd);
    shub_io->registerOutput(uart_fd);
    // TODO: For now the source does not process messages.
    if (!this->isSource()) {
        hal.scheduler->register_timer_process(FUNCTOR_BIND(shub_io, &AP_SensorHub_IO_FileDescriptor::read, void));
    }

    this->registerIO(shub_io);

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
    uint8_t id = static_cast<uint8_t>(Packet::id(packet));
    auto now = AP_HAL::micros();

    // TODO: Add min/max tracking per message.

    uint32_t diff;
    if (now > last_packet_us[id]) {
        diff = now - last_packet_us[id];
    } else {
        diff = last_packet_us[id] - now;
    }

    if (avg_iters[id] < N_SAMPLES) {
        packet_us_sample[id][avg_iters[id]++] = diff;
        packet_us_sum[id] += diff;
    } else {
        auto oldest = packet_us_sample[id][avg_iters[id] % N_SAMPLES];
        packet_us_sum[id] -= oldest;
        packet_us_sum[id] += diff;
        packet_us_sample[id][avg_iters[id] % N_SAMPLES] = diff;
        avg_iters[id]++;
    }

    handle_iters[id]++;
    last_packet_us[id] = now;

    if (now - last_print_us > 1000*1000) {
        for (int i = 0; i < MSG_TYPES; i++) {
            uint32_t div = avg_iters[i] > N_SAMPLES ? N_SAMPLES : avg_iters[i];
            uint64_t avg = packet_us_sum[i] / div;
            #if SENSORHUB_DEBUG == SENSORHUB_DEBUG_CONSOLE
            if (avg > 0) {
                // hal.console->printf("AP_SensorHub - handlePacket Id: %s Avg: %u (recv)\n", msgid_t_names[i], avg);
            }
            #endif
        }
        last_print_us = now;
    }
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

int AP_SensorHub::read(uint8_t *buf, size_t len, uint32_t &p_len)
{
    Packet::packet_t p;
    auto begin = AP_HAL::micros64();
    auto decoded = Packet::decode(&p, buf, len);
    auto decode_status = static_cast<decode_t>(decoded) == decode_t::SUCCESS;
    if (decode_status) {
        handlePacket(&p);
        p_len = Packet::length(&p);

        // NOTE: Wait until vehicle is fully initialized.
        if (isReady()) {
            if (!_notFirstPacket) {
                _notFirstPacket = true;
            } else {
                if (p.hdr.seq != (Packet::seq_t)(_readSeq + 1)) {
                    // NOTE: Can't arbitrarily say the number of lost packets.
                    // However, we can count the instances in which packets
                    // are not received sequentially.
                    // This should be sufficient for our needs.
                    _packetLoss++;
                    #if SENSORHUB_DEBUG == SENSORHUB_DEBUG_CONSOLE
                    hal.console->printf("AP_SensorHub - Packet Loss Event Count: %u\n", _packetLoss);
                    #endif
                }
            }

            }
        }

            _readSeq = p.hdr.seq;
            _lastPacketTime = begin;
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
            msg_id : static_cast<uint8_t>(Packet::id(&p)),
            status : decode_status,
            rw  : 0
        };
        _dataflash->WriteBlock(&pkt, sizeof(pkt));
    }

    return decoded;
}

void AP_SensorHub::write(Packet::packet_t *packet) {

    for (int i = 0; i < SENSORHUB_MAX_PORTS; i++) {
        if (_port[i]) {
            auto begin = AP_HAL::micros64();
            auto success = _port[i]->write(packet, Packet::length(packet));
            if (success) {

            if (_dataflash) {
                auto now = AP_HAL::micros64();
                auto processTime = now - begin;
                struct log_SHUB_RW pkt = {
                    LOG_PACKET_HEADER_INIT((uint8_t)(LOG_SHUB_RW_MSG)),
                    time_us : now,
                    process_time : processTime,
                    packet_loss : _packetLoss,
                    msg_id : static_cast<uint8_t>(Packet::id(packet)),
                    status : success,
                    rw : 1
                };
                _dataflash->WriteBlock(&pkt, sizeof(pkt));
            }

        }
    }

}

#endif
