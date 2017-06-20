#include "Protocol.h"

using namespace SensorHead;

template <class T>
bool Packet::setup(raw_t *p, uint8_t *buf, size_t len)
{
    if (len < T::PACKET_LENGTH) {
        return false;
    }

    p->hdr  = reinterpret_cast<shead_hdr_t *>(buf);
    p->data = reinterpret_cast<uint8_t *>(buf + sizeof(shead_hdr_t));
    p->crc  = reinterpret_cast<shead_crc_t *>(buf + sizeof(shead_hdr_t) +
              sizeof(typename T::data_t));

    return true;
}

template <class T>
void Packet::encode(raw_t *p)
{
    p->hdr->magic = MAGIC;
    p->hdr->ver   = VERSION;
    p->hdr->id    = T::ID;
    p->hdr->len   = sizeof(typename T::data_t);

    *p->crc = crc16_ccitt(reinterpret_cast<uint8_t *>(p->hdr),
                          sizeof(shead_hdr_t), 0);
    *p->crc = crc16_ccitt(p->data, p->hdr->len, *p->crc);
}

bool Packet::verify(raw_t *p)
{

    shead_crc_t calc_crc = 0;
    bool preCheck = p->hdr->magic == MAGIC && p->hdr->ver == VERSION;

    // No point in calculating CRC otherwise.
    if (preCheck) {
        calc_crc = crc16_ccitt(reinterpret_cast<uint8_t *>(p->hdr),
                               sizeof(shead_hdr_t), 0);
        calc_crc = crc16_ccitt(p->data, p->hdr->len, calc_crc);
    }

    return preCheck && calc_crc == *p->crc;
}

bool Packet::decode(raw_t *p, uint8_t *buf, size_t len)
{

    // Assumes packet & data in buf. (eg. HDR | DATA | CRC)
    if (len > MAX_PACKET_LEN) {
        return false;
    }

    bool verified = false;

    for (int i = 0; i < len; i++) {
        // Find first instance of MAGIC & assume it is the beginning of a packet.
        uint8_t *packetStart = findMagic(buf + i, len - i*sizeof(uint8_t));
        if (!packetStart) {
            return false;
        }

        shead_hdr_t *h = reinterpret_cast<shead_hdr_t *>(packetStart);
        p->hdr         = h;
        p->data        = packetStart + sizeof(shead_hdr_t);
        p->crc = reinterpret_cast<shead_crc_t *>(packetStart + sizeof(shead_hdr_t) +
                h->len);

        // Verify if our assumption is correct.
        verified = Packet::verify(p);
        if (verified) {
            return true;
        }
    }

    return false;
}

uint8_t *Packet::findMagic(uint8_t *buf, size_t len)
{

    // Don't bother searching
    if (len < EMPTY_PACKET_LEN) {
        return nullptr;
    }

    uint8_t byte = 0;
    size_t remainingBytes = len;

    for (size_t i = 0; i < len; i++) {
        if (remainingBytes < EMPTY_PACKET_LEN) {
            // Impossible to find a valid packet.
            break;
        }

        byte = *(buf + i);
        if (byte == MAGIC) {
            return (buf + i);
        }
    }

    return nullptr;
}

/* Template Specializations */
template void Packet::encode<BaroMessage>(raw_t *p);
template void Packet::encode<InertialSensorMessage>(raw_t *p);
template void Packet::encode<CompassMessage>(raw_t *p);
template void Packet::encode<GPSMessage>(raw_t *p);

template bool Packet::setup<BaroMessage>(raw_t *p, uint8_t *buf, size_t len);
template bool Packet::setup<InertialSensorMessage>(raw_t *p, uint8_t *buf, size_t len);
template bool Packet::setup<CompassMessage>(raw_t *p, uint8_t *buf, size_t len);
template bool Packet::setup<GPSMessage>(raw_t *p, uint8_t *buf, size_t len);

/* Message Constructors */
// TODO: Refactor this into base class constructor.
BaroMessage::BaroMessage(uint8_t *buf, size_t len)
{
    bool init = Packet::setup<BaroMessage>(&_packet, buf, len);
    _data = reinterpret_cast<data_t *>(_packet.data);

    if (!init) {
        AP_HAL::panic("Failed to setup");
    }
}

InertialSensorMessage::InertialSensorMessage(uint8_t *buf, size_t len)
{
    bool init = Packet::setup<InertialSensorMessage>(&_packet, buf, len);
    _data = reinterpret_cast<data_t *>(_packet.data);

    if (!init) {
        AP_HAL::panic("Failed to setup");
    }
}

CompassMessage::CompassMessage(uint8_t *buf, size_t len)
{
    bool init = Packet::setup<CompassMessage>(&_packet, buf, len);
    _data = reinterpret_cast<data_t *>(_packet.data);

    if (!init) {
        AP_HAL::panic("Failed to setup");
    }
}

GPSMessage::GPSMessage(uint8_t *buf, size_t len)
{
    bool init = Packet::setup<GPSMessage>(&_packet, buf, len);
    _data = reinterpret_cast<data_t *>(_packet.data);

    if (!init) {
        AP_HAL::panic("Failed to setup");
    }
}
