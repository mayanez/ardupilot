#include "AP_SensorHub.h"
#include "AP_SensorHub_IO_Stream.h"

#if HAL_SENSORHUB_ENABLED

extern const AP_HAL::HAL& hal;

void AP_SensorHub_IO_Stream::read()
{
    if (!_isInputInitialized()) {
        return;
    }

    int16_t byte = 0;

    while ((byte = _inputStream->read()) != -1) {

        uint8_t b = byte;
        recvBuffer.write(&b, 1);

        if (recvBuffer.available() >= Packet::EMPTY_PACKET_LEN) {
            uint32_t nBytes = recvBuffer.available();
            // Due to RingBuffer we must have an extra write here.
            // TODO: Possible future optimization.
            recvBuffer.peekbytes(&dataBuffer[0], nBytes);
            auto receivedPacket = static_cast<decode_t>(_shub->read(&dataBuffer[0], nBytes));
            if (receivedPacket == decode_t::SUCCESS || receivedPacket == decode_t::FAIL_ADV) {
                recvBuffer.advance(nBytes);
            }
        }
    }
}

void AP_SensorHub_IO_Stream::write(Packet::packet_t *packet, size_t len)
{
    if (!_isOutputInitialized()) {
        return;
    }

    if (_sem_write->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {

        if (_outputStream->txspace() >= len) {
            Packet::commit(packet, &writeBuffer[0], packet->hdr.len);
            auto write_bytes = _outputStream->write(&writeBuffer[0], len);
            if (write_bytes != len) {
                _write_error++;
                #if SENSORHUB_DEBUG
                hal.console->printf("AP_SensorHub - Write Error: %u\n", _write_error);
                #endif
            }
        } else {
            _write_drop++;
            #if SENSORHUB_DEBUG
            hal.console->printf("AP_SensorHub - Seq: %u Write Drop: %u\n", packet->hdr.seq, _write_drop);
            #endif
        }

        _sem_write->give();
    }
}
#endif
