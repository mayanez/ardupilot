#include "AP_SensorHead_Stream.h"

bool AP_SensorHead_Stream::init()
{
    _shead = AP_SensorHead::get_instance();

    return _shead
        && _inputStream
        && _outputStream;
}

void AP_SensorHead_Stream::read()
{

    if (!_shead->isReady()) {
        return;
    }

    int16_t byte = _inputStream->read();
    if (byte != -1) {
        uint8_t b = byte;
        recvBuffer.write(&b, 1);
    }

    if (recvBuffer.available() >= Packet::EMPTY_PACKET_LEN) {
        uint32_t nBytes = recvBuffer.available();
        // Due to RingBuffer we must have an extra write here.
        // TODO: Possible future optimization.
        recvBuffer.peekbytes(&dataBuffer[0], nBytes);
        bool receivedPacket = _shead->read(&dataBuffer[0], nBytes);
        if (receivedPacket) {
            // If we successfully decode a packet we advance the read pointer.
            recvBuffer.advance(nBytes);
        }
    }
}
