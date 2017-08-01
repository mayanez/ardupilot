#pragma once

#include <AP_HAL/AP_HAL.h>

#if HAL_SENSORHUB_ENABLED

#include "Protocol.h"
#include "AP_SensorHub_IO.h"

#include <AP_HAL/utility/RingBuffer.h>

#include <unistd.h>

class AP_SensorHub;

class AP_SensorHub_IO_FileDescriptor : public AP_SensorHub_IO {
public:

    void registerInput(int inputFd) {
        _inputFd = inputFd;
    }

    void registerOutput(int outputFd) {
        _outputFd = outputFd;
    }

    virtual void read();

    virtual void write(Packet::packet_t *packet, size_t len)
    {
        if (!_isOutputInitialized()) {
            return;
        }

        if (_sem_write->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
            Packet::commit(packet, &writeBuffer[0], packet->hdr.len);
            auto write_bytes = ::write(_outputFd, &writeBuffer, len);
            if (write_bytes != len) {
                _write_error++;
            }
            _sem_write->give();
        }
    }

private:
    bool _isInputInitialized() {
        return _shub && _inputFd > 0;
    }

    bool _isOutputInitialized() {
        return _shub && _outputFd > 0;
    }

    ByteBuffer recvBuffer {2*Packet::MAX_PACKET_LEN};
    uint8_t dataBuffer[Packet::MAX_PACKET_LEN];
    uint8_t writeBuffer[Packet::MAX_PACKET_LEN];

    int _inputFd;
    int _outputFd;
};
#endif
