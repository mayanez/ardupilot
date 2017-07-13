#include "AP_SensorHub.h"
#include "AP_SensorHub_IO_Stream.h"

#if HAL_SENSORHUB_ENABLED

void AP_SensorHub_IO_Stream::read()
{
    if (!_isInputInitialized()) {
        return;
    }

    int16_t byte = 0;

    while (byte != -1) {
        byte = _inputStream->read();

        uint8_t b = byte;
        recvBuffer.write(&b, 1);

        // TODO: Optimization - also check if _packetBytes == MSG::PACKET_LENGTH
        // that way we dont even bother doing the peeek or read().
        if (recvBuffer.available() >= Packet::EMPTY_PACKET_LEN) {
            uint32_t nBytes = recvBuffer.available();
            // Due to RingBuffer we must have an extra write here.
            // TODO: Possible future optimization.
            recvBuffer.peekbytes(&dataBuffer[0], nBytes);
            bool receivedPacket = _shub->read(&dataBuffer[0], nBytes);
            if (receivedPacket || nBytes == Packet::MAX_PACKET_LEN) {
                // If we successfully decode a packet we advance the read pointer.
                // or if we already gathered enough data for the maximum packet
                // and have not decoded a packet.
                recvBuffer.advance(nBytes);
            }
        }
    }
}
#endif
