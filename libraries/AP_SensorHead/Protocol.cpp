#include "Protocol.h"

using namespace SensorHead;

Packet::Packet()
{
    _packet.hdr = &_hdr;
    _packet.data = reinterpret_cast<uint8_t *>(&_data);
    _packet.crc = &_crc;
}

template <class T>
void Packet::encode(T *msg) {
    _packet.hdr->magic = MAGIC;
    _packet.hdr->ver = VERSION;
    _packet.hdr->id = T::ID;
    _packet.hdr->len = sizeof(typename T::data_t);

    *_packet.crc = crc16_ccitt(
        reinterpret_cast<uint8_t *>(_packet.hdr),
        sizeof(shead_hdr_t), 0);
    *_packet.crc = crc16_ccitt(_packet.data, _packet.hdr->len, *_packet.crc);
}

bool Packet::verify(raw_t *p) {

    shead_crc_t calc_crc = 0;
    bool preCheck = p->hdr->magic == MAGIC
        && p->hdr->ver == VERSION;

    // No point in calculating CRC otherwise.
    if (preCheck) {
        calc_crc = crc16_ccitt(reinterpret_cast<uint8_t *>(p->hdr),
                                         sizeof(shead_hdr_t), 0);
        calc_crc = crc16_ccitt(p->data, p->hdr->len, calc_crc);
    }

    return preCheck && calc_crc == *p->crc;
}

bool Packet::decode(raw_t *p, uint8_t *buf, uint16_t len) {

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

uint8_t *Packet::findMagic(uint8_t *buf, uint16_t len) {

    // Don't bother searching
    if(len < EMPTY_PACKET_LEN) {
        return nullptr;
    }

    uint8_t byte = 0;
    size_t remainingBytes = len;

    for (int i = 0; i < len; i++) {
        if (remainingBytes < EMPTY_PACKET_LEN) {
            // Impossible to find a valid packet.
            break;
        }

        byte = *(buf + i);
        if(byte == MAGIC) {
            return (buf + i);
        }
    }

    return nullptr;
}

template void Packet::encode<BaroMessage>(BaroMessage *msg);
template void Packet::encode<InertialSensorMessage>(InertialSensorMessage *msg);
