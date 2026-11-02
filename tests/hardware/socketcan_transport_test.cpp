#include <type_traits>

#include <gtest/gtest.h>

#include "mca/hardware/socketcan_transport.hpp"

namespace {

using mca::hardware::CanopenTransportState;
using mca::hardware::SocketCanTransport;

static_assert(!std::is_copy_constructible<SocketCanTransport>::value,
              "Socket descriptors must not be copied");
static_assert(!std::is_copy_assignable<SocketCanTransport>::value,
              "Socket descriptors must not be copy-assigned");

TEST(SocketCanTransport, RejectsAnEmptyInterfaceWithoutOpeningAFileDescriptor) {
  SocketCanTransport transport("");
  EXPECT_EQ(transport.initialization_status().code,
            mca::core::StatusCode::InvalidArgument);
  EXPECT_FALSE(transport.is_open());
  EXPECT_EQ(transport.state(), CanopenTransportState::Offline);
}

}  // namespace
