#include <AP_gtest.h>

#include <AP_SensorHead/AP_SensorHead.h>

using namespace SensorHead;
const AP_HAL::HAL &hal = AP_HAL::get_HAL();

class SensorHeadTest : public testing::Test {
protected:
    BaroMessage msg;
    AP_SensorHead sHead;

    virtual void SetUp() {
      msg.pressure(0xCA).temperature(0xFE);
    }
};

class BaroMessageHandler : public AP_SensorHead_Handler<BaroMessage> {
public:
    bool messageHandled;

    void handle(BaroMessage::data_t *data) {
        messageHandled = true;
    }

    bool getMessageHandled() { return messageHandled; }
};

class UnknownMessageHandler : public AP_SensorHead_Handler<UnknownMessage> {
public:
    bool messageHandled;

    void handle(BaroMessage::data_t *data) {
        messageHandled = true;
    }

    bool getMessageHandled() { return messageHandled; }
};

TEST_F(SensorHeadTest, handler)
{
    BaroMessageHandler handler;
    Packet *p = msg.encode();
    Packet::raw_t *packet = p->raw();

    sHead.registerHandler(&handler);
    EXPECT_FALSE(handler.getMessageHandled());
    sHead.handlePacket(packet);
    EXPECT_TRUE(handler.getMessageHandled());
}

AP_GTEST_MAIN()
