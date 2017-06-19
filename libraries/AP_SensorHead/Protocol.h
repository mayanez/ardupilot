#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

namespace SensorHead {

enum class msgid_t : uint8_t;

/* Packet structure */
class Packet {
public:
    static const uint8_t MAGIC = 0xFC;
    static const uint8_t VERSION = 0;

    // float conforms to IEEE 754.
    // NOTE: May not be entirely reliable, but should be sufficient.
    // https://stackoverflow.com/questions/5777484/how-to-check-if-c-compiler-uses-ieee-754-floating-point-standard
    static_assert(std::numeric_limits<float>::is_iec559);

    typedef struct PACKED {
        uint8_t magic;
        uint8_t ver;
        uint8_t len;
        msgid_t id;
    } shead_hdr_t;

    typedef uint16_t shead_crc_t;

    // This is to allow packets to be read/write from buffer.
    // NOTE: Stack usage for message is constant.
    typedef struct PACKED {
        shead_hdr_t *hdr;
        uint8_t *data;
        shead_crc_t *crc;
    } shead_packet_t;

    typedef shead_packet_t raw_t;

    static const size_t MAX_DATA_LEN = UINT8_MAX;

    static const size_t MAX_PACKET_LEN =
        sizeof(shead_hdr_t) + MAX_DATA_LEN + sizeof(shead_crc_t);

    static const size_t EMPTY_PACKET_LEN =
        sizeof(shead_hdr_t) + sizeof(shead_crc_t);

    static size_t length(raw_t *p)
    {
        return sizeof(shead_hdr_t) + p->hdr->len + sizeof(shead_crc_t);
    }

    /*
     * Given a buffer point p to appropriate offsets.
     */
    // TODO: Consider changing them all to use Iterators & std::array
    template <class T> static bool setup(raw_t *p, uint8_t *buf, size_t len);

    /*
     * Encodes a message of type T into a packet.
     */
    template <class T> static void encode(raw_t *p);

    /* Verifies that a raw_t is valid. */
    static bool verify(raw_t *p);

    /*
     * Points p to proper offsets inside buf.
     */
    static bool decode(raw_t *p, uint8_t *buf, size_t len);

    static msgid_t id(raw_t *p)
    {
        return p->hdr->id;
    }

private:
    /*
     * Find first instance of MAGIC.
     */
    static uint8_t *findMagic(uint8_t *buf, size_t len);
};

// TODO: update this according to handler order. Refer to loop frequencies
// for different vehicles.
enum class msgid_t : uint8_t {
    UNKNOWN,
    INS,
    RCIN,
    RCOUT,
    BARO,
    COMPASS,
};
// TODO: Define remaining message types.

class Message {
public:
    /*
     * Encodes message contents inside buffer
     */
    virtual Packet::raw_t *encode() = 0;

    /*
     * Retrieve pointer to data inside of packet.
     */
    template <class T> static typename T::data_t *decode(Packet::raw_t *packet)
    {
        return reinterpret_cast<typename T::data_t *>(packet->data);
    }

    /*
     * Verify if a valid packet is actually of type T
     */
    template <class T> static bool verify(Packet::raw_t *packet)
    {
        return packet->hdr->id == T::ID &&
               packet->hdr->len == sizeof(typename T::data_t);
    }

protected:
    Packet::raw_t _packet;
};

class UnknownMessage : public Message {
public:
    static const msgid_t ID = msgid_t::UNKNOWN;
    typedef uint8_t data_t;

    virtual Packet::raw_t *encode()
    {
        return nullptr;
    }
};

class BaroMessage : public Message {
public:
    // TODO: complete definition
    typedef struct PACKED {
        float pressure;
        float temperature;
    } data_t;

    // NOTE: If this fails, need to change len data type
    static_assert(sizeof(data_t) < Packet::MAX_DATA_LEN);

    static const size_t PACKET_LENGTH = Packet::EMPTY_PACKET_LEN + sizeof(data_t);

    static const msgid_t ID = msgid_t::BARO;

    /* Messages write data directly to buffer. */
    BaroMessage(uint8_t *buf, size_t len);

    void setPressure(float pressure)
    {
        _data->pressure = pressure;
    }

    void setTemperature(float temperature)
    {
        _data->temperature = temperature;
    }

    virtual Packet::raw_t *encode()
    {
        Packet::encode<BaroMessage>(&_packet);
        return &_packet;
    }

private:
    data_t *_data;
};

class InertialSensorMessage : public Message {
public:
    // TODO: complete definition
    typedef struct PACKED {
        float gyrox;
        float gyroy;
        float gyroz;
        float accelx;
        float accely;
        float accelz;
    } data_t;

    static_assert(sizeof(data_t) < Packet::MAX_DATA_LEN);

    static const size_t PACKET_LENGTH = Packet::EMPTY_PACKET_LEN + sizeof(data_t);
    static const msgid_t ID = msgid_t::INS;

    InertialSensorMessage(uint8_t *buf, size_t len);

    void setGyro(const Vector3f &vec)
    {
        _data->gyrox = vec.x;
        _data->gyroy = vec.y;
        _data->gyroz = vec.z;
    }

    void setAccel(const Vector3f &vec)
    {
        _data->accelx = vec.x;
        _data->accely = vec.y;
        _data->accelz = vec.z;
    }

    virtual Packet::raw_t *encode()
    {
        Packet::encode<InertialSensorMessage>(&_packet);
        return &_packet;
    }

private:
    data_t *_data;
};

class CompassMessage : public Message {
public:

    // These are the raw fields in body frame.
    typedef struct {
        float magx;
        float magy;
        float magz;
    } data_t;

    static_assert(sizeof(data_t) < Packet::MAX_DATA_LEN);

    static const size_t PACKET_LENGTH = Packet::EMPTY_PACKET_LEN + sizeof(data_t);
    static const msgid_t ID = msgid_t::COMPASS;

    CompassMessage(uint8_t *buf, size_t len);

    void setField(const Vector3f &field) {
        _data->magx = field.x;
        _data->magy = field.y;
        _data->magz = field.z;
    }

    virtual Packet::raw_t *encode()
    {
        Packet::encode<CompassMessage>(&_packet);
        return &_packet;
    }

private:
    data_t *_data;
};

} // namespace SensorHead
