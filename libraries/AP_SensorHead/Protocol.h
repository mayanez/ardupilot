#pragma once

#include <AP_Math/AP_Math.h>

namespace SensorHead {

    enum class msgid_t : uint8_t;

    /* Packet structure */
    class Packet {
    public:
        static const uint8_t MAGIC = 0xFC;
        static const uint8_t VERSION = 0;

        typedef struct PACKED {
            uint8_t magic;
            uint8_t ver;
            uint8_t len;
            msgid_t id;
        } shead_hdr_t;

        typedef uint16_t shead_crc_t;

        typedef struct PACKED {
            shead_hdr_t *hdr;
            uint8_t *data;
            shead_crc_t *crc;
        } shead_packet_t;

        typedef shead_packet_t raw_t;

        static const size_t MAX_PACKET_LEN =
            sizeof(shead_hdr_t) + UINT8_MAX + sizeof(shead_crc_t);
        static const size_t EMPTY_PACKET_LEN = sizeof(shead_hdr_t) + sizeof(shead_crc_t);

        Packet();

        /*
         * raw_t is used by receiver to make decode from buffer simpler and avoid
         * copy overheads.
         */
        raw_t *raw() { return &_packet; }

        msgid_t id() { return _packet.hdr->id; }

        size_t length() {
            return sizeof(shead_hdr_t) + _packet.hdr->len + sizeof(shead_crc_t);
        }

        uint8_t *data() { return _packet.data; }

        /*
         * Encodes a message of type T into a packet.
         */
        template <class T>
        void encode(T *msg);

        /* Verifies that a raw_t is valid. */
        static bool verify(raw_t *p);

        /*
         * Points p to proper contents inside buf.
         * NOTE: This is done to avoid copy overhead.
         */
        static bool decode(raw_t *p, uint8_t *buf, uint16_t len);

        static msgid_t id(raw_t *p) {
            return p->hdr->id;
        }

    private:
        raw_t _packet;
        uint8_t _data[UINT8_MAX];
        shead_hdr_t _hdr;
        shead_crc_t _crc;

        /*
         * Find first instance of MAGIC.
         */
        static uint8_t *findMagic(uint8_t *buf, uint16_t len);
    };

    /* Message Structure */
    enum class msgid_t : uint8_t {
        UNKNOWN,
        BARO,
        INS,
    };

    class Message {
    public:
        /*
         * Encode Message inside packet.
         */
        virtual Packet *encode() = 0;

        template <class T>
        static typename T::data_t *decode(Packet::raw_t *packet) {
            return reinterpret_cast<typename T::data_t *>(packet->data);
        }

        /*
         * Verify if a valid packet is actually of type T
         */
        template <class T>
        static bool verify(Packet::raw_t *packet) {
            return packet->hdr->id == T::ID
                && packet->hdr->len == sizeof(typename T::data_t);
        }

    protected:
        Packet _packet;
    };

    class UnknownMessage : public Message {
    public:
        static const msgid_t ID = msgid_t::UNKNOWN;
        typedef uint8_t data_t;

        // Not used.
        Packet *encode() { return nullptr; };
    };

    /* Baro */
    class BaroMessage : public Message {
    public:

        // TODO: complete definition
        typedef struct PACKED {
            float pressure;
            float temperature;
        } data_t;

        data_t *data = reinterpret_cast<data_t *>(_packet.data());
        static const msgid_t ID = msgid_t::BARO;

        BaroMessage &pressure(float pressure) {
            data->pressure = pressure;
            return *this;
        }

        BaroMessage &temperature(float temperature) {
            data->temperature = temperature;
            return *this;
        }

        Packet *encode() {
            _packet.encode<BaroMessage>(this);
            return &_packet;
        }
    };

    /* INS */
    class InertialSensorMessage : public Message {
    public:

        // TODO: complete definition
        typedef struct PACKED {
            float gyrox;
            float gyroy;
            float gyroz;
            uint64_t time;
        } data_t;

        data_t *data = reinterpret_cast<data_t *>(_packet.data());
        static const msgid_t ID = msgid_t::INS;

        InertialSensorMessage &gyro(const Vector3f &vec) {
            data->gyrox = vec.x;
            data->gyroy = vec.y;
            data->gyroz = vec.z;
            return *this;
        }

        Packet *encode() {
            _packet.encode<InertialSensorMessage>(this);
            return &_packet;
        }

    };

}
