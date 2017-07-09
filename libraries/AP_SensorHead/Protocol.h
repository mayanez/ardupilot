#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>

#if HAL_SHEAD_ENABLED
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

    // NOTE: data will belong to Message. This avoid unnecessary stack usage.
    typedef struct PACKED {
        shead_hdr_t hdr;
        uint8_t *data;
        shead_crc_t crc;
    } shead_packet_t;

    typedef shead_packet_t raw_t;

    static const size_t MAX_DATA_LEN = UINT8_MAX;

    static const size_t MAX_PACKET_LEN =
        sizeof(shead_hdr_t) + MAX_DATA_LEN + sizeof(shead_crc_t);

    static const size_t EMPTY_PACKET_LEN =
        sizeof(shead_hdr_t) + sizeof(shead_crc_t);

    static size_t length(raw_t *p)
    {
        return sizeof(shead_hdr_t) + p->hdr.len + sizeof(shead_crc_t);
    }

    /*
     * Encodes a message of type T into a packet.
     */
    template <class T> static void encode(raw_t *p);

    /*
     * Writes the packet to the specified buffer.
     */
    static inline void commit(raw_t *p, uint8_t *buf, size_t len)
    {
        memcpy(buf, &p->hdr, sizeof(shead_hdr_t));
        memcpy(buf + sizeof(shead_hdr_t), p->data, p->hdr.len);
        memcpy(buf + sizeof(shead_hdr_t) + p->hdr.len, &p->crc, sizeof(shead_crc_t));
    }

    /* Verifies that a raw_t is valid. */
    static bool verify(raw_t *p);

    /*
     * Decodes packet from raw buffer.
     * NOTE: Packet data will point to contents inside of buffer.
     */
    static bool decode(raw_t *p, uint8_t *buf, size_t len);

    static msgid_t id(raw_t *p)
    {
        return p->hdr.id;
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
    GPS,
};
// TODO: Define remaining message types.

class Message {
public:
    /*
     * Encodes message into packet.
     */
    virtual Packet::raw_t *encode() = 0;

    /*
     * Verify if a valid packet is actually of type T
     */
    template <class T> static bool verify(Packet::raw_t *packet)
    {
        return packet->hdr.id == T::ID &&
               packet->hdr.len == sizeof(typename T::data_t);
    }

    /*
     * Decode Message data from packet.
     */
    template <class T>
    static inline void decode(Packet::raw_t *packet, typename T::data_t *data)
    {
        memcpy(data, packet->data, sizeof(typename T::data_t));
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

    BaroMessage() {
        _packet.data = reinterpret_cast<uint8_t *>(&_data);
    }

    void setPressure(float pressure)
    {
        _data.pressure = pressure;
    }

    void setTemperature(float temperature)
    {
        _data.temperature = temperature;
    }

    virtual Packet::raw_t *encode()
    {
        Packet::encode<BaroMessage>(&_packet);
        return &_packet;
    }


private:
    data_t _data;
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

    InertialSensorMessage() {
        _packet.data = reinterpret_cast<uint8_t *>(&_data);
    }

    void setGyro(const Vector3f &vec)
    {
        _data.gyrox = vec.x;
        _data.gyroy = vec.y;
        _data.gyroz = vec.z;
    }

    void setAccel(const Vector3f &vec)
    {
        _data.accelx = vec.x;
        _data.accely = vec.y;
        _data.accelz = vec.z;
    }

    virtual Packet::raw_t *encode()
    {
        Packet::encode<InertialSensorMessage>(&_packet);
        return &_packet;
    }

private:
    data_t _data;
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

    CompassMessage() {
        _packet.data = reinterpret_cast<uint8_t *>(&_data);
    }

    void setField(const Vector3f &field) {
        _data.magx = field.x;
        _data.magy = field.y;
        _data.magz = field.z;
    }

    virtual Packet::raw_t *encode()
    {
        Packet::encode<CompassMessage>(&_packet);
        return &_packet;
    }

private:
    data_t _data;
};

class GPSMessage : public Message {
public:

    typedef struct PACKED {
        AP_GPS::GPS_Status status;
        uint32_t time_week_ms;
        uint16_t time_week;
        // NOTE: This can be serialized.
        Location location;
        float ground_speed;
        float ground_course;
        uint16_t hdop;
        uint16_t vdop;
        uint8_t num_sats;
        float velocityx;
        float velocityy;
        float velocityz;
        float speed_accuracy;
        float horizontal_accuracy;
        float vertical_accuracy;
        bool have_vertical_velocity:1;
        bool have_speed_accuracy:1;
        bool have_horizontal_accuracy:1;
        bool have_vertical_accuracy:1;
    } data_t;

    static_assert(sizeof(data_t) < Packet::MAX_DATA_LEN);

    static const size_t PACKET_LENGTH = Packet::EMPTY_PACKET_LEN + sizeof(data_t);
    static const msgid_t ID = msgid_t::GPS;

    GPSMessage() {
        _packet.data = reinterpret_cast<uint8_t *>(&_data);
    }

    void setState(const AP_GPS::GPS_State &state) {
        _data.status                   = state.status;
        _data.time_week_ms             = state.time_week_ms;
        _data.time_week                = state.time_week;
        _data.location                 = state.location;
        _data.ground_speed             = state.ground_speed;
        _data.ground_course            = state.ground_course;
        _data.hdop                     = state.hdop;
        _data.vdop                     = state.vdop;
        _data.num_sats                 = state.num_sats;
        _data.velocityx                = state.velocity.x;
        _data.velocityy                = state.velocity.y;
        _data.velocityz                = state.velocity.z;
        _data.speed_accuracy           = state.speed_accuracy;
        _data.horizontal_accuracy      = state.horizontal_accuracy;
        _data.vertical_accuracy        = state.vertical_accuracy;
        _data.have_vertical_velocity   = state.have_vertical_velocity;
        _data.have_speed_accuracy      = state.have_speed_accuracy;
        _data.have_horizontal_accuracy = state.have_horizontal_accuracy;
        _data.have_vertical_accuracy   = state.have_vertical_accuracy;
    }

    virtual Packet::raw_t *encode()
    {
        Packet::encode<GPSMessage>(&_packet);
        return &_packet;
    }

private:
    data_t _data;
};

} // namespace SensorHead
#endif // HAL_SHEAD_ENABLED
