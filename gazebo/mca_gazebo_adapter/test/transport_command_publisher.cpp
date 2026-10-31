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
  if (argc != 3) {
    std::cerr << "usage: mca_transport_command_publisher TOPIC VALUE\n";
    return 2;
  }

  errno = 0;
  char* end = nullptr;
  const double value = std::strtod(argv[2], &end);
  if (end == argv[2] || *end != '\\0' || errno == ERANGE) {
    std::cerr << "invalid numeric VALUE: " << argv[2] << std::endl;
    return 2;
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
  for (int i = 0; i < 20; ++i) {
    if (!publisher.Publish(message)) {
      std::cerr << "failed to publish command on " << topic << std::endl;
      return 4;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }
  std::cout << "published 20 Gazebo Transport Double commands value=" << value
            << " topic=" << topic << std::endl;
  return 0;
}
