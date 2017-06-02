#pragma once

#include "Packet.h"
#include "Message.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#include <vector>

class AP_SensorHead {
public:

    AP_SensorHead &setUART(AP_HAL::UARTDriver *uart) {
        _uart = uart;
        _packetStream = new BSStream(_uart);
        return *this;
    }

    static AP_SensorHead *get_instance() {
        if(!_instance) {
            _instance = new AP_SensorHead();
        }
        return _instance;
    }

    template <typename T>
    size_t send(T *msg) {
        std::vector<uint8_t> packed_msg;
        _pack<T>(packed_msg, msg);
        return _packetStream->write(packed_msg);
    }

private:
    static AP_SensorHead *_instance;
    AP_HAL::UARTDriver *_uart;
    BSStream *_packetStream;

    void _decode(std::vector<uint8_t> &buf);

    template <typename T>
    inline void _pack(std::vector<uint8_t> &dst, T *data) {
        uint8_t *src = reinterpret_cast<uint8_t *>(data);
        dst.insert(dst.end(), src, src + sizeof(T));
    }

    template <typename T>
    inline void _unpack(std::vector<uint8_t> &src, T *data) {
        copy(&src[0], &src[sizeof(T)], data);
    }

};
