#include "AP_SensorHub.h"
#include "AP_SensorHub_IO_FileDescriptor.h"

#if HAL_SENSORHUB_ENABLED

extern const AP_HAL::HAL& hal;

#if SENSORHUB_DEBUG
static uint64_t read_total_bytes;
static uint64_t read_packets;
static uint32_t read_last_print_us;

static uint32_t write_total_bytes;
static uint64_t write_packets;
static uint32_t write_last_print_us;
static uint32_t write_miss_bytes;
#endif

void AP_SensorHub_IO_FileDescriptor::read()
{
    if (!_isInputInitialized()) {
        return;
    }

    int read_bytes = 0;

    while ((read_bytes = ::read(_inputFd, &dataBuffer[0], sizeof(dataBuffer))) > 0 ) {
        recvBuffer.write(&dataBuffer[0], read_bytes);

#if SENSORHUB_DEBUG
        read_total_bytes += read_bytes;
#endif
    }

#if SENSORHUB_DEBUG
    uint32_t now = AP_HAL::micros();
    if (now - read_last_print_us > 1000*1000) {
        #if SENSORHUB_DEBUG == SENSORHUB_DEBUG_CONSOLE
        hal.console->printf("AP_SensorHub_IO_FD - Read: %lu bytes/s \n", read_total_bytes);
        hal.console->printf("AP_SensorHub_IO_FD - Read : %lu packets/s \n", read_packets);
        #endif
        read_total_bytes = 0;
        read_packets = 0;
        read_last_print_us = now;
    }
#endif

    while (recvBuffer.available() >= Packet::EMPTY_PACKET_LEN) {
        auto peekedBytes = recvBuffer.peekbytes(&dataBuffer[0], sizeof(dataBuffer));
        uint32_t packet_len = 0;
        auto receivedPacket = static_cast<decode_t>(_shub->read(&dataBuffer[0], peekedBytes, packet_len));
        if (receivedPacket == decode_t::SUCCESS) {
            recvBuffer.advance(packet_len);
            #if SENSORHUB_DEBUG
            read_packets++;
            #endif
        } else if (receivedPacket == decode_t::FAIL_ADV) {
            recvBuffer.advance(peekedBytes);
        } else {
            break;
        }
    }
}

bool AP_SensorHub_IO_FileDescriptor::write(Packet::packet_t *packet, size_t len)
{
    if (!_isOutputInitialized()) {
        return false;
    }

#if SENSORHUB_DEBUG

#endif

    bool status = true;
    if (_sem_write->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        Packet::commit(packet, &writeBuffer[0], packet->hdr.len);
        auto write_bytes = ::write(_outputFd, &writeBuffer[0], len);
        if (write_bytes < 0 || (write_bytes > 0 && static_cast<size_t>(write_bytes) != len)) {
            _write_error++;
            #if SENSORHUB_DEBUG
            write_miss_bytes += len;
            #endif
            status = false;
        }

#if SENSORHUB_DEBUG
        write_total_bytes += write_bytes;
        // NOTE: Every call to write() corresponds to a packet write.
        write_packets++;

        uint32_t now = AP_HAL::micros();
        if (now - write_last_print_us > 1000*1000) {
            #if SENSORHUB_DEBUG == SENSORHUB_DEBUG_CONSOLE
            hal.console->printf("AP_SensorHub_IO_FD - Written: %u bytes/s \n", write_total_bytes);
            hal.console->printf("AP_SensorHub_IO_FD - Write: %lu packets/s \n", write_packets);
            hal.console->printf("AP_SensorHub_IO_FD - Write Error (count): %u\n", _write_error);
            hal.console->printf("AP_SensorHub_IO_FD - Write Error (missed bytes): %u\n", write_miss_bytes);
            #endif
            write_total_bytes = 0;
            write_packets = 0;
            write_last_print_us = now;
        }
#endif

        _sem_write->give();
    }

    return status;
}
#endif