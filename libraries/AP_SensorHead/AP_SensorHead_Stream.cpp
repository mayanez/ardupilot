#include "AP_SensorHead_Stream.h"

#if HAL_SHEAD_ENABLED
bool AP_SensorHead_Stream::init()
{
    _shead = AP_SensorHead::get_instance();

    return _shead
        && _inputStream
        && _outputStream;
}

void AP_SensorHead_Stream::read()
{
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
            bool receivedPacket = _shead->read(&dataBuffer[0], nBytes);
            if (receivedPacket || nBytes == Packet::MAX_PACKET_LEN) {
                // If we successfully decode a packet we advance the read pointer.
                // or if we already gathered enough data for the maximum packet
                // and have not decoded a packet.
                recvBuffer.advance(nBytes);
            }
        }
    }
}
#endif // HAL_SHEAD_ENABLED
