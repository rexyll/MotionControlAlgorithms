#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include <gz/msgs/double.pb.h>
#include <gz/transport/Node.hh>

int main(int argc, char** argv) {
  if (argc != 3 && argc != 4) {
    std::cerr << "usage: mca_transport_command_publisher TOPIC VALUE [COUNT]\n";
    return 2;
  }

  errno = 0;
  char* end = nullptr;
  const double value = std::strtod(argv[2], &end);
  if (end == argv[2] || *end != '\0' || errno == ERANGE) {
    std::cerr << "invalid numeric VALUE: " << argv[2] << std::endl;
    return 2;
  }

  // The watchdog re-arm probe intentionally omits COUNT and is stopped by the
  // workflow after the plugin acknowledges a fresh valid command. Keep that
  // finite-value stream alive long enough for the workflow to perform the
  // hand-off to malformed traffic without the watchdog expiring in between.
  // Explicit COUNT remains authoritative, while malformed values keep the
  // short default burst used by the rejection tests.
  int publish_count = std::isfinite(value) ? 2000 : 20;
  if (argc == 4) {
    errno = 0;
    char* count_end = nullptr;
    const long parsed_count = std::strtol(argv[3], &count_end, 10);
    if (count_end == argv[3] || *count_end != '\0' || errno == ERANGE ||
        parsed_count < 0 || parsed_count > 100000) {
      std::cerr << "invalid non-negative COUNT: " << argv[3] << std::endl;
      return 2;
    }
    publish_count = static_cast<int>(parsed_count);
  }

  gz::transport::Node node;
  const std::string topic(argv[1]);
  auto publisher = node.Advertise<gz::msgs::Double>(topic);
  if (!publisher) {
    std::cerr << "failed to advertise Gazebo Transport topic: " << topic << std::endl;
    return 3;
  }

  // Advertising a topic does not mean a subscriber is connected yet.
  // Gazebo Transport discovery is asynchronous, and messages published before
  // the adapter subscription is established can be dropped. Wait for the
  // actual subscriber before starting finite or continuous command streams.
  constexpr auto kConnectionTimeout = std::chrono::seconds(10);
  const auto connection_deadline =
      std::chrono::steady_clock::now() + kConnectionTimeout;
  while (!publisher.HasConnections() &&
         std::chrono::steady_clock::now() < connection_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!publisher.HasConnections()) {
    std::cerr << "timed out waiting for a Gazebo Transport subscriber on "
              << topic << " after " << kConnectionTimeout.count()
              << " seconds" << std::endl;
    return 5;
  }
  std::cout << "Gazebo Transport subscriber connected on " << topic
            << std::endl;

  gz::msgs::Double message;
  message.set_data(value);
  int published_count = 0;
  while (publish_count == 0 || published_count < publish_count) {
    if (!publisher.Publish(message)) {
      std::cerr << "failed to publish command on " << topic
                << " after " << published_count << " successful publishes" << std::endl;
      return 4;
    }
    ++published_count;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::cout << "published " << published_count
            << " Gazebo Transport Double commands value=" << value
            << " topic=" << topic << std::endl;
  return 0;
}
