#pragma once

#include <AP_Math/AP_Math.h>

namespace SensorHead {

    enum class msgid_t : uint8_t;

    /* Packet structure */
    class Packet {
    public:
        static const uint8_t MAGIC = 0xFE;
        static const uint8_t VERSION = 0;

        typedef struct PACKED {
            uint8_t magic;
            uint8_t ver;
            uint8_t len;
            msgid_t id;
        } shead_hdr_t;

        typedef uint16_t shead_crc_t;

        typedef struct PACKED {
            shead_hdr_t hdr;
            uint8_t *data;
            shead_crc_t crc;
        } shead_packet_t;

        typedef shead_packet_t raw_t;

        static const size_t MAX_PACKET_LEN = sizeof(shead_hdr_t) + UINT8_MAX + sizeof(shead_crc_t);

        raw_t packet;

        raw_t *raw() { return &packet; }

        msgid_t id() { return packet.hdr.id; }

        size_t length() {
            return sizeof(shead_hdr_t) + packet.hdr.len + sizeof(shead_crc_t);
        }

        template <class T>
        void encode(T *msg) {
            packet.data = reinterpret_cast<uint8_t *>(&msg->data);
            packet.hdr.magic = MAGIC;
            packet.hdr.ver = VERSION;
            packet.hdr.id = T::ID;
            packet.hdr.len = sizeof(typename T::data_t);
            packet.crc = crc16_ccitt(
                reinterpret_cast<uint8_t *>(&packet),
                sizeof(shead_hdr_t) + sizeof(typename T::data_t), 0);
        }

        template <class T>
        bool verify() {
          shead_crc_t crc =
              crc16_ccitt(reinterpret_cast<uint8_t *>(&packet),
                          sizeof(shead_hdr_t) + sizeof(typename T::data_t), 0);

          return length() == sizeof(typename T::data_t) && crc == packet.crc;
        }

        /* raw_t packet helpers */
        template <class T>
        static bool verify(raw_t *packet) {
            shead_crc_t crc =
                crc16_ccitt(reinterpret_cast<uint8_t *>(packet),
                            sizeof(shead_hdr_t) + sizeof(typename T::data_t), 0);

            return packet->hdr.len == sizeof(typename T::data_t) && crc == packet->crc;
        }

        template <class T>
        static raw_t *decode(uint8_t *buf, size_t len) {

            if (len > MAX_PACKET_LEN) return nullptr;

            // Assumes that buf one packet. Might need to revist this.
            raw_t *packet;
            shead_hdr_t *hdr = reinterpret_cast<shead_hdr_t *>(buf);
            &packet->hdr = hdr;
            packet->data = buf + sizeof(shead_hdr_t);
            &packet->crc = buf + sizeof(shead_hdr_t) + hdr->len;
            return packet;
        }

        static msgid_t id(raw_t *packet) {
            return packet->hdr.id;
        }
    };

    /* Message Structure */
    enum class msgid_t : uint8_t {
        BARO,
        INS,
    };

    class Message {
    public:
        virtual Packet *encode() = 0;

    protected:
        Packet _packet;
    };

    /* Baro */
    class BaroMessage : public Message {
    public:

        // TODO: complete definition
        typedef struct PACKED {
            float pressure;
            float temperature;
            uint64_t time;
        } data_t;

        static const msgid_t ID = msgid_t::BARO;
        data_t data;

        BaroMessage &pressure(float pressure) {
            data.pressure = pressure;
            return *this;
        }

        BaroMessage &temperature(float temperature) {
            data.temperature = temperature;
            return *this;
        }

        Packet *encode() {
            _packet.encode<BaroMessage>(this);
            return &_packet;
        }

        static data_t *decode(Packet::raw_t *packet) {
            return reinterpret_cast<data_t *>(packet->data)
        }

        static data_t *decode(Packet *packet) {
            return Packet::decode<BaroMessage>(packet->raw());
        }
    };

    /* INS */
    class InertialSensorMessage : public Message {
    public:

        // TODO: complete definition
        typedef struct PACKED {
            float xgyro;
            float ygyro;
            float zgyro;
            uint64_t time;
        } data_t;

        static const msgid_t ID = msgid_t::INS;
        data_t data;

        InertialSensorMessage &gyro(const Vector3f &vec) {
            data.xgyro = vec.x;
            data.ygyro = vec.y;
            data.zgyro = vec.z;
            return *this;
        }

        Packet *encode() {
            _packet.encode<InertialSensorMessage>(this);
            return &_packet;
        }

        static data_t *decode(Packet::raw_t *packet) {
            return reinterpret_cast<data_t *>(packet->data)
        }

        static data_t *decode(Packet *packet) {
            return Packet::decode<InertialSensorMessage>(packet->raw());
        }

    };

}
