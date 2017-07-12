#pragma once

#if HAL_SENSORHUB_ENABLED

#include "Protocol.h"
#include "AP_SensorHub_IO.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>

class AP_SensorHub;

class AP_SensorHub_IO_Stream : public AP_SensorHub_IO {
public:

    void registerInputStream(AP_HAL::Stream *inputStream) {
        _inputStream = inputStream;
    }

    void registerOutputStream(AP_HAL::Stream *outputStream) {
        _outputStream = outputStream;
    }

    virtual void read();

    virtual void write(Packet::packet_t *packet, size_t len)
    {
        if (!_isOutputInitialized()) {
            return;
        }

        Packet::commit(packet, &writeBuffer[0], packet->hdr.len);
        _outputStream->write(&writeBuffer[0], len);
    }

private:
    bool _isInputInitialized() {
        return _shub && _inputStream;
    }

    bool _isOutputInitialized() {
        return _shub && _outputStream;
    }

    ByteBuffer recvBuffer {2*Packet::MAX_PACKET_LEN};
    uint8_t dataBuffer[Packet::MAX_PACKET_LEN];
    uint8_t writeBuffer[Packet::MAX_PACKET_LEN];

    AP_HAL::Stream *_inputStream;
    AP_HAL::Stream *_outputStream;
};
#endif
