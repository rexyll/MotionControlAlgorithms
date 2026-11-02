#include <cstdint>

#include <gtest/gtest.h>

#include "mca/hardware/ethercat_mailbox_queue.hpp"

namespace {

using Queue = mca::hardware::EthercatMailboxQueue<2U, 8U>;
using Request = Queue::Request;
using Result = Queue::Result;
using State = mca::hardware::EthercatMailboxResultState;

Request make_read(std::uint64_t id, std::uint64_t deadline) {
  Request request;
  request.request_id = id;
  request.index = 0x6041U;
  request.operation = mca::hardware::EthercatSdoOperation::Read;
  request.max_response_size = 2U;
  request.deadline_timestamp_ns = deadline;
  return request;
}

TEST(EthercatMailboxQueue, RejectsDuplicateLiveIdsAndAppliesBackpressure) {
  Queue queue;
  const auto request = make_read(1U, 100U);
  ASSERT_TRUE(queue.enqueue(request, 10U).ok());
  EXPECT_EQ(queue.enqueue(request, 10U).code,
            mca::core::StatusCode::RuntimeError);
  ASSERT_TRUE(queue.enqueue(make_read(2U, 100U), 10U).ok());
  EXPECT_EQ(queue.enqueue(make_read(3U, 100U), 10U).code,
            mca::core::StatusCode::RuntimeError);
}

TEST(EthercatMailboxQueue, PollsPendingAndCompletedUntilExplicitRetirement) {
  Queue queue;
  ASSERT_TRUE(queue.enqueue(make_read(7U, 100U), 10U).ok());
  Result output;
  ASSERT_TRUE(queue.poll(7U, 20U, output).ok());
  EXPECT_EQ(output.state, State::Pending);

  Result completed;
  completed.request_id = 7U;
  completed.state = State::Completed;
  completed.payload[0] = 0x34U;
  completed.payload_size = 1U;
  ASSERT_TRUE(queue.complete(7U, 30U, completed).ok());
  ASSERT_TRUE(queue.poll(7U, 40U, output).ok());
  EXPECT_EQ(output.state, State::Completed);
  EXPECT_EQ(output.payload[0], 0x34U);
  ASSERT_TRUE(queue.poll(7U, 50U, output).ok());
  EXPECT_EQ(output.state, State::Completed);
  EXPECT_TRUE(queue.retire(7U).ok());
  EXPECT_EQ(queue.poll(7U, 60U, output).code,
            mca::core::StatusCode::InvalidArgument);
  EXPECT_TRUE(queue.enqueue(make_read(7U, 100U), 60U).ok());
}

TEST(EthercatMailboxQueue, TimesOutAtDeadlineAndRejectsLateCompletion) {
  Queue queue;
  ASSERT_TRUE(queue.enqueue(make_read(9U, 100U), 10U).ok());
  Result output;
  ASSERT_TRUE(queue.poll(9U, 99U, output).ok());
  EXPECT_EQ(output.state, State::Pending);
  ASSERT_TRUE(queue.poll(9U, 100U, output).ok());
  EXPECT_EQ(output.state, State::TimedOut);
  EXPECT_TRUE(output.valid());

  Result completed;
  completed.request_id = 9U;
  completed.state = State::Completed;
  completed.payload_size = 1U;
  EXPECT_EQ(queue.complete(9U, 101U, completed).code,
            mca::core::StatusCode::RuntimeError);
  EXPECT_TRUE(queue.retire(9U).ok());
}

TEST(EthercatMailboxQueue, WriteCompletionCannotReturnReadPayload) {
  Queue queue;
  Request request;
  request.request_id = 11U;
  request.index = 0x6040U;
  request.operation = mca::hardware::EthercatSdoOperation::Write;
  request.payload[0] = 0x06U;
  request.payload_size = 1U;
  request.deadline_timestamp_ns = 100U;
  ASSERT_TRUE(queue.enqueue(request, 10U).ok());

  Result completed;
  completed.request_id = 11U;
  completed.state = State::Completed;
  completed.payload[0] = 0x06U;
  completed.payload_size = 1U;
  EXPECT_EQ(queue.complete(11U, 20U, completed).code,
            mca::core::StatusCode::InvalidArgument);
  completed.payload_size = 0U;
  EXPECT_TRUE(queue.complete(11U, 20U, completed).ok());
}

TEST(EthercatMailboxQueue, RejectsExpiredRequestsAndInvalidTransitions) {
  Queue queue;
  EXPECT_EQ(queue.enqueue(make_read(1U, 10U), 10U).code,
            mca::core::StatusCode::InvalidArgument);
  ASSERT_TRUE(queue.enqueue(make_read(2U, 100U), 10U).ok());
  EXPECT_EQ(queue.retire(2U).code, mca::core::StatusCode::RuntimeError);
  Result invalid;
  invalid.request_id = 2U;
  invalid.state = State::Pending;
  EXPECT_EQ(queue.complete(2U, 20U, invalid).code,
            mca::core::StatusCode::InvalidArgument);
}

}  // namespace
