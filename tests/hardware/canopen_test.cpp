#include <cstdint>

#include <gtest/gtest.h>

#include "mca/hardware/canopen.hpp"

namespace {

using namespace mca::hardware;

CanopenCanFrame nmt(std::uint8_t command, std::uint8_t node = 1U) {
  CanopenCanFrame frame;
  frame.cob_id = 0x000U;
  frame.dlc = 2U;
  frame.data[0] = command;
  frame.data[1] = node;
  return frame;
}

CanopenCanFrame sdo(std::uint8_t command, std::uint16_t index,
                    std::uint8_t subindex, std::uint32_t value = 0U) {
  CanopenCanFrame frame;
  frame.cob_id = 0x601U;
  frame.dlc = 8U;
  frame.data[0] = command;
  frame.data[1] = static_cast<std::uint8_t>(index & 0xFFU);
  frame.data[2] = static_cast<std::uint8_t>(index >> 8U);
  frame.data[3] = subindex;
  for (std::size_t i = 0; i < 4U; ++i)
    frame.data[4U + i] = static_cast<std::uint8_t>((value >> (8U * i)) & 0xFFU);
  return frame;
}

class FakeCanopenTransport final : public ICanopenTransport {
 public:
  mca::core::Status send(const CanopenCanFrame& frame) noexcept override {
    if (!frame.valid()) return {mca::core::StatusCode::InvalidArgument};
    last_sent_ = frame;
    sent_ = true;
    return {};
  }
  mca::core::Status receive(CanopenCanFrame& frame,
                       bool& has_frame) noexcept override {
    has_frame = pending_;
    if (pending_) {
      frame = incoming_;
      pending_ = false;
    }
    return {};
  }
  CanopenTransportState state() const noexcept override { return state_; }
  void queue(const CanopenCanFrame& frame) noexcept {
    incoming_ = frame;
    pending_ = true;
  }
  void set_state(CanopenTransportState state) noexcept { state_ = state; }
  bool sent() const noexcept { return sent_; }
  const CanopenCanFrame& last_sent() const noexcept { return last_sent_; }
 private:
  CanopenCanFrame incoming_{};
  CanopenCanFrame last_sent_{};
  CanopenTransportState state_{CanopenTransportState::Online};
  bool pending_{false};
  bool sent_{false};
};

TEST(CanopenTransport, SeparatesEmptyNonblockingReadFromTransportFailure) {
  FakeCanopenTransport transport;
  EXPECT_EQ(transport.state(), CanopenTransportState::Online);
  CanopenCanFrame received;
  bool has_frame = true;
  ASSERT_TRUE(transport.receive(received, has_frame).ok());
  EXPECT_FALSE(has_frame);

  CanopenCanFrame outgoing;
  outgoing.cob_id = 0x123U;
  outgoing.dlc = 1U;
  outgoing.data[0] = 0x5AU;
  ASSERT_TRUE(transport.send(outgoing).ok());
  EXPECT_TRUE(transport.sent());
  EXPECT_EQ(transport.last_sent().data[0], 0x5AU);

  transport.queue(outgoing);
  ASSERT_TRUE(transport.receive(received, has_frame).ok());
  EXPECT_TRUE(has_frame);
  EXPECT_EQ(received.cob_id, 0x123U);
  transport.set_state(CanopenTransportState::BusOff);
  EXPECT_EQ(transport.state(), CanopenTransportState::BusOff);
}

TEST(CanopenFrame, ValidatesStandardCanIdentifierAndDlc) {
  CanopenCanFrame frame;
  EXPECT_TRUE(frame.valid());
  frame.cob_id = 0x800U;
  EXPECT_FALSE(frame.valid());
  frame.cob_id = 0x7FFU;
  frame.dlc = 9U;
  EXPECT_FALSE(frame.valid());
}

TEST(CanopenNmt, HandlesStartStopPreOperationalAndResetCommands) {
  CanopenVirtualNode<8> node(1U);
  EXPECT_EQ(node.nmt_state(), CanopenNmtState::PreOperational);
  CanopenCanFrame response;
  bool has_response = true;

  EXPECT_TRUE(node.process(nmt(0x01U), response, has_response).ok());
  EXPECT_EQ(node.nmt_state(), CanopenNmtState::Operational);
  EXPECT_FALSE(has_response);
  EXPECT_TRUE(node.process(nmt(0x02U), response, has_response).ok());
  EXPECT_EQ(node.nmt_state(), CanopenNmtState::Stopped);
  EXPECT_TRUE(node.process(nmt(0x80U), response, has_response).ok());
  EXPECT_EQ(node.nmt_state(), CanopenNmtState::PreOperational);
  EXPECT_TRUE(node.process(nmt(0x81U), response, has_response).ok());
  EXPECT_EQ(node.nmt_state(), CanopenNmtState::Initialising);
  EXPECT_EQ(node.heartbeat().data[0], 0x00U);
  EXPECT_EQ(node.process(nmt(0x01U), response, has_response).code,
            mca::core::StatusCode::RuntimeError);
  node.complete_initialization();
  EXPECT_EQ(node.nmt_state(), CanopenNmtState::PreOperational);
  EXPECT_EQ(node.heartbeat().data[0], 0x7FU);
  EXPECT_EQ(node.process(nmt(0x03U), response, has_response).code,
            mca::core::StatusCode::InvalidArgument);
  EXPECT_EQ(node.process(nmt(0x01U, 2U), response, has_response).code,
            mca::core::StatusCode::InvalidArgument);
}

TEST(CanopenHeartbeat, ObservesProducerStateAndDetectsTimeoutRecovery) {
  CanopenHeartbeatMonitor monitor(7U, 100U);
  EXPECT_TRUE(monitor.valid());
  CanopenCanFrame heartbeat;
  heartbeat.cob_id = 0x707U;
  heartbeat.dlc = 1U;
  heartbeat.data[0] = 0x05U;
  EXPECT_TRUE(monitor.observe(heartbeat, 1000U));
  EXPECT_EQ(monitor.last_state(), 0x05U);
  EXPECT_FALSE(monitor.poll(1099U));
  EXPECT_TRUE(monitor.poll(1100U));
  EXPECT_TRUE(monitor.timed_out());
  EXPECT_TRUE(monitor.observe(heartbeat, 1200U));
  EXPECT_FALSE(monitor.timed_out());
  EXPECT_FALSE(monitor.poll(1199U));
  heartbeat.data[0] = 0x06U;
  EXPECT_FALSE(monitor.observe(heartbeat, 1300U));
}

TEST(CanopenHeartbeat, DetectsMissingInitialHeartbeatAfterExplicitStart) {
  CanopenHeartbeatMonitor monitor(7U, 100U);
  ASSERT_TRUE(monitor.start(500U));
  EXPECT_FALSE(monitor.seen());
  EXPECT_FALSE(monitor.poll(599U));
  EXPECT_TRUE(monitor.poll(600U));
  EXPECT_TRUE(monitor.timed_out());
  EXPECT_FALSE(monitor.seen());

  CanopenCanFrame heartbeat;
  heartbeat.cob_id = 0x707U;
  heartbeat.dlc = 1U;
  heartbeat.data[0] = 0x7FU;
  EXPECT_TRUE(monitor.observe(heartbeat, 650U));
  EXPECT_TRUE(monitor.seen());
  EXPECT_FALSE(monitor.timed_out());
  EXPECT_FALSE(monitor.poll(749U));
  EXPECT_TRUE(monitor.poll(750U));
}

TEST(CanopenEmcy, DecodesErrorRegisterAndPreservesManufacturerBytes) {
  CanopenCanFrame frame;
  frame.cob_id = 0x081U;
  frame.dlc = 8U;
  frame.data = {0x34U, 0x12U, 0x08U, 1U, 2U, 3U, 4U, 5U};
  CanopenEmcy emcy;
  ASSERT_TRUE(decode_canopen_emcy(frame, 1U, emcy));
  EXPECT_EQ(emcy.error_code, 0x1234U);
  EXPECT_EQ(emcy.error_register, 0x08U);
  EXPECT_EQ(emcy.manufacturer_data[0], 1U);
  EXPECT_EQ(emcy.manufacturer_data[4], 5U);
  frame.dlc = 7U;
  EXPECT_FALSE(decode_canopen_emcy(frame, 1U, emcy));
  EXPECT_EQ(emcy.error_code, 0x1234U);
}

TEST(CanopenSdo, SupportsExpeditedUploadDownloadAndAbortCodes) {
  CanopenVirtualNode<8> node(1U);
  ASSERT_TRUE(node.add_object({0x6040U, 0U, 0x000FU, 2U, true, true}).ok());
  CanopenCanFrame response;
  bool has_response = false;

  ASSERT_TRUE(node.process(sdo(0x40U, 0x6040U, 0U), response, has_response).ok());
  ASSERT_TRUE(has_response);
  EXPECT_EQ(response.cob_id, 0x581U);
  EXPECT_EQ(response.data[0], 0x4BU);
  EXPECT_EQ(response.data[4], 0x0FU);
  EXPECT_EQ(response.data[5], 0x00U);

  ASSERT_TRUE(node.process(sdo(0x2BU, 0x6040U, 0U, 0x0006U),
                           response, has_response).ok());
  EXPECT_EQ(response.data[0], 0x60U);
  ASSERT_TRUE(node.process(sdo(0x40U, 0x6040U, 0U), response, has_response).ok());
  EXPECT_EQ(response.data[4], 0x06U);

  ASSERT_TRUE(node.process(sdo(0x40U, 0x2000U, 0U), response, has_response).ok());
  EXPECT_EQ(response.data[0], 0x80U);
  EXPECT_EQ(response.data[4], 0x00U);
  EXPECT_EQ(response.data[5], 0x00U);
  EXPECT_EQ(response.data[6], 0x02U);
  EXPECT_EQ(response.data[7], 0x06U);

  ASSERT_TRUE(node.add_object({0x2001U, 0U, 0x12345678U, 4U, true, false}).ok());
  ASSERT_TRUE(node.process(sdo(0x23U, 0x2001U, 0U, 0U),
                           response, has_response).ok());
  EXPECT_EQ(response.data[0], 0x80U);
  EXPECT_EQ(response.data[4], 0x02U);
  EXPECT_EQ(response.data[5], 0x00U);
  EXPECT_EQ(response.data[6], 0x01U);
  EXPECT_EQ(response.data[7], 0x06U);
}

TEST(CanopenSdo, SegmentedEightByteUploadDownloadAndToggleValidation) {
  CanopenVirtualNode<8> node(1U);
  constexpr std::uint64_t original = 0x8877665544332211ULL;
  ASSERT_TRUE(node.add_object(
      {0x2002U, 0U, original, 8U, true, true}).ok());
  CanopenCanFrame response;
  bool has_response = false;

  auto initiate_upload = sdo(0x40U, 0x2002U, 0U);
  ASSERT_TRUE(node.process(initiate_upload, response, has_response).ok());
  ASSERT_TRUE(has_response);
  EXPECT_EQ(response.data[0], 0x41U);
  EXPECT_EQ(response.data[4], 8U);

  CanopenCanFrame upload_segment;
  upload_segment.cob_id = 0x601U;
  upload_segment.dlc = 8U;
  upload_segment.data[0] = 0x60U;
  ASSERT_TRUE(node.process(upload_segment, response, has_response).ok());
  EXPECT_EQ(response.data[0], 0x00U);
  EXPECT_EQ(response.data[1], 0x11U);
  EXPECT_EQ(response.data[7], 0x77U);

  upload_segment.data[0] = 0x70U;
  ASSERT_TRUE(node.process(upload_segment, response, has_response).ok());
  EXPECT_EQ(response.data[0], 0x1DU);
  EXPECT_EQ(response.data[1], 0x88U);

  CanopenCanFrame initiate_download;
  initiate_download.cob_id = 0x601U;
  initiate_download.dlc = 8U;
  initiate_download.data[0] = 0x21U;
  initiate_download.data[1] = 0x02U;
  initiate_download.data[2] = 0x20U;
  initiate_download.data[3] = 0U;
  initiate_download.data[4] = 8U;
  ASSERT_TRUE(node.process(initiate_download, response, has_response).ok());
  EXPECT_EQ(response.data[0], 0x60U);

  CanopenCanFrame download_segment;
  download_segment.cob_id = 0x601U;
  download_segment.dlc = 8U;
  download_segment.data = {0x00U, 0x01U, 0x02U, 0x03U,
                           0x04U, 0x05U, 0x06U, 0x07U};
  ASSERT_TRUE(node.process(download_segment, response, has_response).ok());
  EXPECT_EQ(response.data[0], 0x20U);

  download_segment.data = {0x1DU, 0x08U, 0U, 0U, 0U, 0U, 0U, 0U};
  ASSERT_TRUE(node.process(download_segment, response, has_response).ok());
  EXPECT_EQ(response.data[0], 0x30U);

  ASSERT_TRUE(node.process(initiate_upload, response, has_response).ok());
  upload_segment.data[0] = 0x60U;
  ASSERT_TRUE(node.process(upload_segment, response, has_response).ok());
  EXPECT_EQ(response.data[1], 0x01U);
  EXPECT_EQ(response.data[7], 0x07U);
  upload_segment.data[0] = 0x70U;
  ASSERT_TRUE(node.process(upload_segment, response, has_response).ok());
  EXPECT_EQ(response.data[1], 0x08U);

  // A wrong toggle aborts the transfer and leaves the object unchanged.
  ASSERT_TRUE(node.process(initiate_download, response, has_response).ok());
  download_segment.data = {0x00U, 0xAAU, 0xBBU, 0xCCU, 0xDDU,
                           0xEEU, 0xFFU, 0x11U};
  ASSERT_TRUE(node.process(download_segment, response, has_response).ok());
  download_segment.data[0] = 0x00U;
  ASSERT_TRUE(node.process(download_segment, response, has_response).ok());
  EXPECT_EQ(response.data[0], 0x80U);
  EXPECT_EQ(response.data[1], 0x02U);
  EXPECT_EQ(response.data[2], 0x20U);
  EXPECT_EQ(response.data[3], 0x00U);
  EXPECT_EQ(response.data[4], 0x00U);
  EXPECT_EQ(response.data[5], 0x00U);
  EXPECT_EQ(response.data[6], 0x03U);
  EXPECT_EQ(response.data[7], 0x05U);
  ASSERT_TRUE(node.process(initiate_upload, response, has_response).ok());
  upload_segment.data[0] = 0x60U;
  ASSERT_TRUE(node.process(upload_segment, response, has_response).ok());
  EXPECT_EQ(response.data[1], 0x01U);
  EXPECT_EQ(response.data[7], 0x07U);
}

TEST(CanopenSdo, ClientAbortTerminatesSegmentedTransferWithoutResponse) {
  CanopenVirtualNode<4> node(1U);
  ASSERT_TRUE(node.add_object(
      {0x2003U, 0U, 0x8877665544332211ULL, 8U, true, true}).ok());
  CanopenCanFrame request;
  request.cob_id = 0x601U;
  request.dlc = 8U;
  request.data[0] = 0x21U;
  request.data[1] = 0x03U;
  request.data[2] = 0x20U;
  request.data[4] = 8U;
  CanopenCanFrame response;
  bool has_response = false;
  ASSERT_TRUE(node.process(request, response, has_response).ok());
  ASSERT_TRUE(has_response);
  EXPECT_EQ(response.data[0], 0x60U);

  request.data[0] = 0x80U;
  request.data[4] = 0U;
  ASSERT_TRUE(node.process(request, response, has_response).ok());
  EXPECT_FALSE(has_response);
}

TEST(CanopenCiA402Node, ControlwordWritesAdvanceVirtualPdsState) {
  CanopenVirtualNode<8> node(1U);
  ASSERT_TRUE(node.add_object({0x6040U, 0U, 0U, 2U, true, true}).ok());
  ASSERT_TRUE(node.add_object({0x6041U, 0U, 0x0040U, 2U, true, false}).ok());
  CanopenCanFrame response;
  bool has_response = false;

  const auto write_controlword = [&](std::uint16_t value) {
    return node.process(sdo(0x2BU, 0x6040U, 0U, value), response, has_response);
  };
  const auto read_state = [&]() {
    const auto status = node.process(sdo(0x40U, 0x6041U, 0U),
                                     response, has_response);
    EXPECT_TRUE(status.ok());
    EXPECT_TRUE(has_response);
    const auto raw = static_cast<std::uint16_t>(
        static_cast<std::uint32_t>(response.data[4]) |
        (static_cast<std::uint32_t>(response.data[5]) << 8U));
    return decode_cia402_pds_state(raw);
  };

  ASSERT_TRUE(write_controlword(0x0006U).ok());
  EXPECT_EQ(read_state(), Cia402PdsState::ReadyToSwitchOn);
  ASSERT_TRUE(write_controlword(0x0007U).ok());
  EXPECT_EQ(read_state(), Cia402PdsState::SwitchedOn);
  ASSERT_TRUE(write_controlword(0x000FU).ok());
  EXPECT_EQ(read_state(), Cia402PdsState::OperationEnabled);
  ASSERT_TRUE(write_controlword(0x0002U).ok());
  EXPECT_EQ(read_state(), Cia402PdsState::QuickStopActive);

  ASSERT_TRUE(node.set_object_value(0x6041U, 0U, 0x0008U).ok());
  ASSERT_TRUE(write_controlword(0x0080U).ok());
  EXPECT_EQ(read_state(), Cia402PdsState::SwitchOnDisabled);
}

TEST(CanopenPdo, ValidatesTransmissionTypeAndSyncCadence) {
  EXPECT_TRUE(canopen_tpdo_transmission_type_valid(0U));
  EXPECT_TRUE(canopen_tpdo_transmission_type_valid(1U));
  EXPECT_TRUE(canopen_tpdo_transmission_type_valid(240U));
  EXPECT_FALSE(canopen_tpdo_transmission_type_valid(241U));
  EXPECT_FALSE(canopen_tpdo_transmission_type_valid(252U));
  EXPECT_FALSE(canopen_tpdo_transmission_type_valid(253U));
  EXPECT_TRUE(canopen_tpdo_transmission_type_valid(254U));
  EXPECT_TRUE(canopen_tpdo_transmission_type_valid(255U));

  EXPECT_TRUE(canopen_tpdo_is_synchronous(0U));
  EXPECT_TRUE(canopen_tpdo_is_synchronous(240U));
  EXPECT_FALSE(canopen_tpdo_is_synchronous(254U));
  EXPECT_TRUE(canopen_tpdo_is_event_driven(254U));
  EXPECT_TRUE(canopen_tpdo_is_event_driven(255U));
  EXPECT_FALSE(canopen_tpdo_sync_due(0U, 10U));
  EXPECT_TRUE(canopen_tpdo_sync_due(1U, 1U));
  EXPECT_TRUE(canopen_tpdo_sync_due(4U, 8U));
  EXPECT_FALSE(canopen_tpdo_sync_due(4U, 7U));
  EXPECT_FALSE(canopen_tpdo_sync_due(255U, 8U));

  CanopenVirtualNode<4> node(1U);
  CanopenPdoCommunicationParameters parameters;
  parameters.transmission_type = 4U;
  parameters.inhibit_time_100us = 10U;
  parameters.event_timer_ms = 50U;
  ASSERT_TRUE(node.set_tpdo1_communication(parameters).ok());
  EXPECT_EQ(node.tpdo1_communication().transmission_type, 4U);
  parameters.transmission_type = 252U;
  EXPECT_EQ(node.set_tpdo1_communication(parameters).code,
            mca::core::StatusCode::InvalidArgument);
}

TEST(CanopenPdo, AppliesRpdoAtomicallyAndBuildsTpdo) {
  CanopenVirtualNode<8> node(1U);
  ASSERT_TRUE(node.add_object({0x2000U, 0U, 0U, 2U, true, true}).ok());
  ASSERT_TRUE(node.add_object({0x2001U, 0U, 0U, 2U, true, true}).ok());
  ASSERT_TRUE(node.add_object({0x2002U, 0U, 0x12345678U, 4U, true, false}).ok());

  CanopenPdoMapping<4U> rpdo;
  rpdo.count = 2U;
  rpdo.entries[0] = {0x2000U, 0U, 16U};
  rpdo.entries[1] = {0x2001U, 0U, 16U};
  ASSERT_TRUE(rpdo.valid());
  node.set_rpdo1_mapping(rpdo);

  CanopenPdoMapping<4U> tpdo;
  tpdo.count = 1U;
  tpdo.entries[0] = {0x2002U, 0U, 32U};
  ASSERT_TRUE(tpdo.valid());
  node.set_tpdo1_mapping(tpdo);

  CanopenCanFrame response;
  bool has_response = false;
  ASSERT_TRUE(node.process(nmt(0x01U), response, has_response).ok());

  CanopenCanFrame rpdo_frame;
  rpdo_frame.cob_id = 0x201U;
  rpdo_frame.dlc = 4U;
  rpdo_frame.data = {0x34U, 0x12U, 0xCDU, 0xABU, 0U, 0U, 0U, 0U};
  ASSERT_TRUE(node.process(rpdo_frame, response, has_response).ok());

  CanopenCanFrame tpdo_frame;
  ASSERT_TRUE(node.make_tpdo1(tpdo_frame));
  EXPECT_EQ(tpdo_frame.cob_id, 0x181U);
  EXPECT_EQ(tpdo_frame.dlc, 4U);
  EXPECT_EQ(tpdo_frame.data[0], 0x78U);
  EXPECT_EQ(tpdo_frame.data[3], 0x12U);

  auto invalid_map = rpdo;
  invalid_map.entries[1].index = 0x2FFFU;
  node.set_rpdo1_mapping(invalid_map);
  rpdo_frame.data[0] = 0xFFU;
  EXPECT_EQ(node.process(rpdo_frame, response, has_response).code,
            mca::core::StatusCode::ConstraintViolation);
}

TEST(CanopenRecovery, BusOffInhibitsProtocolAndRecoveryRestoresIt) {
  CanopenVirtualNode<4> node(1U);
  CanopenCanFrame response;
  bool has_response = false;
  node.set_bus_off();
  EXPECT_TRUE(node.communication_faulted());
  EXPECT_EQ(node.process(nmt(0x01U), response, has_response).code,
            mca::core::StatusCode::RuntimeError);
  node.set_bus_recovered();
  EXPECT_FALSE(node.communication_faulted());
  EXPECT_TRUE(node.process(nmt(0x01U), response, has_response).ok());
  EXPECT_EQ(node.nmt_state(), CanopenNmtState::Operational);
}

}  // namespace
