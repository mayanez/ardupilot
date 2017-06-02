#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#include <vector>

#define MAX_PACKET_SIZE 10

/*
  START | DATA | END
*/
class Packet {
public:
    static const uint8_t START = 0xFE;
    static const uint8_t END = 0xED;
    static const uint8_t ESC = 0xEC;
};

class PacketStream {
public:
    static const int EOF = -1;
    static const int EOP = -2;

    /*
      Returns DATA byte by byte inside Packet.
      EOF if we have reached the end of the stream
      EOP if the current packet is complete
    */
    virtual int read() = 0;
};

class PacketPrint {
public:
    virtual size_t write(std::vector<uint8_t> &vals) = 0;
};

/* Byte Stuffing Stream */
class BSStream : public PacketStream, public PacketPrint {
private:
    AP_HAL::Stream *_base;
    bool _escape_pending;

public:

    BSStream(AP_HAL::Stream *base) : _base(base) {}

    virtual int read() override;
    virtual size_t write(std::vector<uint8_t> &vals) override;
};
