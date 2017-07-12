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
            auto receivedPacket = static_cast<decode_t>(_shub->read(&dataBuffer[0], nBytes));
            if (receivedPacket == decode_t::SUCCESS || receivedPacket == decode_t::FAIL_ADV) {
                recvBuffer.advance(nBytes);
            }
        }
    }
}
#endif