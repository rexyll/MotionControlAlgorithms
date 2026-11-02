#include <chrono>
#include <cstdlib>
#include <thread>
#include <type_traits>

#include <gtest/gtest.h>

#include "mca/hardware/socketcan_transport.hpp"

namespace {

using mca::hardware::CanopenCanFrame;
using mca::hardware::CanopenTransportState;
using mca::hardware::SocketCanTransport;

static_assert(!std::is_copy_constructible<SocketCanTransport>::value,
              "Socket descriptors must not be copied");
static_assert(!std::is_copy_assignable<SocketCanTransport>::value,
              "Socket descriptors must not be copy-assigned");
static_assert(!std::is_move_constructible<SocketCanTransport>::value,
              "Socket descriptor ownership is intentionally non-movable");

TEST(SocketCanTransport, RejectsAnEmptyInterfaceWithoutOpeningAFileDescriptor) {
  SocketCanTransport transport("");
  EXPECT_EQ(transport.initialization_status().code,
            mca::core::StatusCode::InvalidArgument);
  EXPECT_FALSE(transport.is_open());
  EXPECT_EQ(transport.state(), CanopenTransportState::Offline);
}

TEST(SocketCanTransport, ExchangesClassicCanFramesOverVcanWhenProvisioned) {
  if (std::getenv("MCA_TEST_VCAN") == nullptr) {
    GTEST_SKIP() << "CI runner did not provide a configured vcan0 interface";
  }

  SocketCanTransport receiver("vcan0");
  SocketCanTransport sender("vcan0");
  ASSERT_TRUE(receiver.initialization_status().ok());
  ASSERT_TRUE(sender.initialization_status().ok());
  ASSERT_EQ(receiver.state(), CanopenTransportState::Online);
  ASSERT_EQ(sender.state(), CanopenTransportState::Online);

  CanopenCanFrame received;
  bool has_frame = true;
  ASSERT_TRUE(receiver.receive(received, has_frame).ok());
  EXPECT_FALSE(has_frame);

  CanopenCanFrame sent;
  sent.cob_id = 0x321U;
  sent.dlc = 4U;
  sent.data = {{0x12U, 0x34U, 0x56U, 0x78U, 0U, 0U, 0U, 0U}};
  ASSERT_TRUE(sender.send(sent).ok());

  bool observed = false;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(500);
  while (std::chrono::steady_clock::now() < deadline) {
    ASSERT_TRUE(receiver.receive(received, has_frame).ok());
    if (has_frame) {
      observed = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  ASSERT_TRUE(observed) << "vcan0 did not deliver the transmitted frame";
  EXPECT_EQ(received.cob_id, sent.cob_id);
  EXPECT_EQ(received.dlc, sent.dlc);
  EXPECT_EQ(received.data, sent.data);
}

}  // namespace
