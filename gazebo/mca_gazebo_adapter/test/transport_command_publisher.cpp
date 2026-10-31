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

  int publish_count = 20;
  if (argc == 4) {
    errno = 0;
    char* count_end = nullptr;
    const long parsed_count = std::strtol(argv[3], &count_end, 10);
    if (count_end == argv[3] || *count_end != '\0' || errno == ERANGE ||
        parsed_count <= 0 || parsed_count > 100000) {
      std::cerr << "invalid positive COUNT: " << argv[3] << std::endl;
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

  gz::msgs::Double message;
  message.set_data(value);
  for (int i = 0; i < publish_count; ++i) {
    if (!publisher.Publish(message)) {
      std::cerr << "failed to publish command on " << topic << std::endl;
      return 4;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }
  std::cout << "published " << publish_count << " Gazebo Transport Double commands value=" << value
            << " topic=" << topic << std::endl;
  return 0;
}
