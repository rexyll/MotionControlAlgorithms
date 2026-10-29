#include <atomic>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <gz/msgs/double.pb.h>
#include <gz/transport/Node.hh>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: mca_transport_subscriber_probe TOPIC\n";
    return 2;
  }
  gz::transport::Node node;
  std::atomic<bool> received{false};
  const std::string topic(argv[1]);
  const bool subscribed = node.Subscribe<gz::msgs::Double>(
      topic, [&received](const gz::msgs::Double& message) {
        received.store(true, std::memory_order_relaxed);
        std::cout << "probe callback received gz.msgs.Double data="
                  << message.data() << std::endl;
      });
  std::cout << "probe subscribed=" << (subscribed ? "true" : "false")
            << " topic=" << topic
            << " partition=" << node.Options().Partition() << std::endl;
  if (!subscribed) return 3;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  while (std::chrono::steady_clock::now() < deadline &&
         !received.load(std::memory_order_relaxed)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::cout << "probe callback_received="
            << (received.load(std::memory_order_relaxed) ? "true" : "false")
            << std::endl;
  return received.load(std::memory_order_relaxed) ? 0 : 1;
}
