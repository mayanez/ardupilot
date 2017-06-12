#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_SensorHead/Protocol.h>

using namespace SensorHead;
const AP_HAL::HAL &hal = AP_HAL::get_HAL();

class ProtocolTest : public testing::Test {
protected:
  uint8_t buf[BaroMessage::PACKET_LENGTH];
  BaroMessage msg{&buf[0], sizeof(buf)};
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

TEST_F(ProtocolTest, encode_time) {
    // TODO: Is this time accurate?
    auto start = AP_HAL::micros();
    BaroMessage msg2{&buf[0], sizeof(buf)};
    msg.setPressure(0xCA);
    msg.setPressure(0xFE);
    msg.encode();
    auto end = AP_HAL::micros() - start;
    printf("Encode Time (us): %d\n", end);
}

/*
 * Test Protocol Encoding functionality.
 */
TEST_F(ProtocolTest, encode) {
    auto packet = msg.encode(); // points to offset within buffer.

    EXPECT_TRUE(0 == memcmp(packet->hdr, &e_hdr, sizeof(e_hdr)));
    EXPECT_TRUE(0 == memcmp(packet->data, reinterpret_cast<uint8_t *>(&e_data),
                            sizeof(e_data)));
    EXPECT_EQ(e_crc, *packet->crc);
}

/*
 * Test Protocol Decoding functionality.
 */
TEST_F(ProtocolTest, decode) {

    Packet::raw_t a_packet; // points to relevant structures within buffer.

    auto start = AP_HAL::micros();
    // Moves past garbage.
    bool decoded = Packet::decode(&a_packet, &e_raw_offset_garbage[0],
                             sizeof(e_raw_offset_garbage));

    auto end = AP_HAL::micros() - start;
    printf("Decode Time (us): %d\n", end);

    EXPECT_TRUE(decoded);

    EXPECT_TRUE(0 == memcmp(&e_hdr, a_packet.hdr, sizeof(e_hdr)));
    EXPECT_TRUE(0 == memcmp(&e_crc, a_packet.crc, sizeof(e_crc)));

    auto isValid = Message::verify<BaroMessage>(&a_packet);
    EXPECT_TRUE(isValid);

    auto data = Message::decode<BaroMessage>(&a_packet);
    EXPECT_TRUE(0 == memcmp(data, &e_data, sizeof(e_data)));
}

AP_GTEST_MAIN()
