#include "AP_SensorHub.h"
#include "AP_SensorHub_IO_Stream.h"

#if HAL_SENSORHUB_ENABLED

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
#endif
