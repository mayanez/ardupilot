#pragma once

#include "AP_SensorHead.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>

/*
 * Auxiliary class for using AP_SensorHead with Streams (eg. UART).
 */
class AP_SensorHead_Stream {
public:
    AP_SensorHead_Stream(AP_SensorHead *shead, AP_HAL::Stream *readStream, AP_HAL::Stream *writeStream) :
        _shead(shead),
        _readStream(readStream),
        _writeStream(writeStream)
    {}

    void read();

    template <class T>
    void write()
    {
        _shead->write<T>(&writeBuffer[0], sizeof(writeBuffer));
        _writeStream->write(&writeBuffer[0], T::PACKET_LENGTH);
    }

private:
    AP_SensorHead *_shead;

    ByteBuffer recvBuffer {Packet::MAX_PACKET_LEN};
    uint8_t dataBuffer[Packet::MAX_PACKET_LEN];
    uint8_t writeBuffer[Packet::MAX_PACKET_LEN];

    AP_HAL::Stream *_readStream;
    AP_HAL::Stream *_writeStream;
};
