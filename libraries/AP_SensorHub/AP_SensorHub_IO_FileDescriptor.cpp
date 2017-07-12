#include "AP_SensorHub.h"
#include "AP_SensorHub_IO_FileDescriptor.h"

#if HAL_SENSORHUB_ENABLED
void AP_SensorHub_IO_FileDescriptor::read()
{
    if (!_isInputInitialized()) {
        return;
    }

    uint8_t b = 0;

    while (::read(_inputFd, &b, 1) == 1) {

        recvBuffer.write(&b, 1);

        if (recvBuffer.available() >= Packet::EMPTY_PACKET_LEN) {
            uint32_t nBytes = recvBuffer.available();
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