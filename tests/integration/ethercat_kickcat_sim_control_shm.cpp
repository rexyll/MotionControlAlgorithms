#include <iostream>
#include <string>

#include "kickcat/OS/SharedMemory.h"
#include "kickcat/SimulatorControl.h"

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: " << argv[0] << " init|unlink SHM_NAME\n";
    return 2;
  }

  const std::string operation(argv[1]);
  const std::string name(argv[2]);
  if (name.empty() || name.front() != '/') {
    std::cerr << "shared-memory name must begin with '/'\n";
    return 2;
  }

  if (operation == "init") {
    try {
      kickcat::sim::ControlChannel channel;
      channel.create(name);
    } catch (const std::exception& error) {
      std::cerr << "failed to initialise simulator control channel: "
                << error.what() << '\n';
      return 1;
    }
    std::cout << "PASS: simulator control shared memory initialised: "
              << name << '\n';
    return 0;
  }

  if (operation == "unlink") {
    kickcat::SharedMemory::unlink(name);
    std::cout << "PASS: simulator control shared memory unlinked: "
              << name << '\n';
    return 0;
  }

  std::cerr << "unknown operation: " << operation << '\n';
  return 2;
}
