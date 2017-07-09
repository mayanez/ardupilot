#include "Protocol.h"

#if HAL_SHEAD_ENABLED

using namespace SensorHead;

template <class T>
void Packet::encode(raw_t *p)
{
    p->hdr.magic = MAGIC;
    p->hdr.ver   = VERSION;
    p->hdr.id    = T::ID;
    p->hdr.len   = sizeof(typename T::data_t);

    p->crc = crc16_ccitt(reinterpret_cast<uint8_t *>(&p->hdr),
                          sizeof(shead_hdr_t), 0);
    p->crc = crc16_ccitt(p->data, p->hdr.len, p->crc);
}

bool Packet::verify(raw_t *p)
{

    shead_crc_t calc_crc = 0;
    bool preCheck = p->hdr.magic == MAGIC && p->hdr.ver == VERSION;

    // No point in calculating CRC otherwise.
    if (preCheck) {
        // NOTE: In case of collision we handle whether data is valid in the
        // individual message's handler.
        calc_crc = crc16_ccitt(reinterpret_cast<uint8_t *>(&p->hdr),
                               sizeof(shead_hdr_t), 0);
        calc_crc = crc16_ccitt(p->data, p->hdr.len, calc_crc);
    }

    return preCheck && calc_crc == p->crc;
}

bool Packet::decode(raw_t *p, uint8_t *buf, size_t len)
{

    // Assumes packet & data in buf. (eg. HDR | DATA | CRC)
    if (len > MAX_PACKET_LEN) {
        return false;
    }

    bool verified = false;
    size_t remainingBytes = 0;
    for (int i = 0; i < len; i++) {
        remainingBytes = len - i*sizeof(uint8_t);
        // Find first instance of MAGIC & assume it is the beginning of a packet.
        uint8_t *packetStart = findMagic(buf + i, remainingBytes);
        if (!packetStart) {
            return false;
        }
        // We found magic but there aren't enough bytes in the buffer to
        // consider it valid.
        if (remainingBytes < EMPTY_PACKET_LEN) {
            return false;
        }

        memcpy(&p->hdr, packetStart, sizeof(p->hdr));
        if (remainingBytes < (EMPTY_PACKET_LEN + p->hdr.len)) {
            // A valid packet might still exist, therefore we continue.
            continue;
        }

        // buffer contains enough data for packet with decoded length.
        p->data        = packetStart + sizeof(shead_hdr_t); //Don't copy data yet.
        memcpy(&p->crc, packetStart + sizeof(shead_hdr_t) +
               p->hdr.len, sizeof(shead_crc_t));

        // Verify it is a valid packet.
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

#endif // HAL_SHEAD_ENABLED