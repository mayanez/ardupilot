#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_SensorHead/Protocol.h>
#include <AP_SensorHead/AP_SensorHead.h>

#if HAL_SHEAD_ENABLED
using namespace SensorHead;
const AP_HAL::HAL &hal = AP_HAL::get_HAL();

class ProtocolTest : public testing::Test {
protected:
  uint8_t buf[BaroMessage::PACKET_LENGTH];
  BaroMessage msg;
  Packet::shead_hdr_t e_hdr{Packet::MAGIC, Packet::VERSION,
                            sizeof(BaroMessage::data_t), BaroMessage::ID};
  BaroMessage::data_t e_data{0xCA, 0xFE};
  Packet::shead_crc_t e_crc;
  uint8_t e_raw_offset_garbage[sizeof(e_hdr) + sizeof(e_data) + sizeof(e_crc) + 10]{};

  virtual void SetUp() {
    msg.setPressure(0xCA);
    msg.setTemperature(0xFE);
    e_crc = crc16_ccitt(reinterpret_cast<uint8_t *>(&e_hdr), sizeof(e_hdr), 0);
    e_crc = crc16_ccitt(reinterpret_cast<uint8_t *>(&e_data), sizeof(e_data),
                        e_crc);

    // Mimics receiver buffer with packet inside: HDR | DATA | CRC
    e_raw_offset_garbage[1] = Packet::MAGIC;
    memcpy(&e_raw_offset_garbage[3], &e_hdr, sizeof(e_hdr));
    memcpy(&e_raw_offset_garbage[sizeof(e_hdr) + 3], &e_data, sizeof(e_data));
    memcpy(&e_raw_offset_garbage[sizeof(e_hdr) + sizeof(e_data) + 3], &e_crc,
        sizeof(e_crc));
  }
};

/*
 * Test Protocol Encoding functionality.
 */
TEST_F(ProtocolTest, encode) {
    auto packet = msg.encode();

    EXPECT_TRUE(0 == memcmp(&packet->hdr, &e_hdr, sizeof(e_hdr)));
    EXPECT_TRUE(0 == memcmp(packet->data, reinterpret_cast<uint8_t *>(&e_data),
                            sizeof(e_data)));
    EXPECT_EQ(e_crc, packet->crc);
}

/*
 * Test Protocol Decoding functionality.
 */
TEST_F(ProtocolTest, decode) {

    Packet::raw_t a_packet;

    // Moves past garbage.
    bool decoded = Packet::decode(&a_packet, &e_raw_offset_garbage[0],
                             sizeof(e_raw_offset_garbage));

    EXPECT_TRUE(decoded);

    // Verify hdr & crc are expected.
    EXPECT_TRUE(0 == memcmp(&e_hdr, &a_packet.hdr, sizeof(e_hdr)));
    EXPECT_TRUE(0 == memcmp(&e_crc, &a_packet.crc, sizeof(e_crc)));

    auto isValid = Message::verify<BaroMessage>(&a_packet);
    EXPECT_TRUE(isValid);

    BaroMessage::data_t data {};
    Message::decode<BaroMessage>(&a_packet, &data);
    EXPECT_TRUE(0 == memcmp(&e_data, &data, sizeof(e_data)));
}

TEST_F(ProtocolTest, decode_magic_end_buffer) {
    uint8_t magic_end[BaroMessage::PACKET_LENGTH];
    memset(&magic_end[0], 0, sizeof(magic_end) - 1);
    magic_end[sizeof(magic_end) - 1] = Packet::MAGIC;

    Packet::raw_t a_packet;
    bool decoded = Packet::decode(&a_packet, &magic_end[0],
                                  sizeof(magic_end));
    EXPECT_FALSE(decoded);
}

TEST_F(ProtocolTest, decode_incomplete_packet) {
    uint8_t incomplete_packet[BaroMessage::PACKET_LENGTH];
    Packet::shead_hdr_t inc_hdr{Packet::MAGIC, Packet::VERSION,
            sizeof(BaroMessage::data_t) + 3, BaroMessage::ID};
    memcpy(&incomplete_packet[0], &inc_hdr, sizeof(inc_hdr));

    Packet::raw_t a_packet;
    bool decoded = Packet::decode(&a_packet, &incomplete_packet[0],
                                  sizeof(incomplete_packet));

    EXPECT_FALSE(decoded);
}
#endif

AP_GTEST_MAIN()