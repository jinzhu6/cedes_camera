#include <iostream>
#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>

#include "interface.hpp"

namespace Cedes {

void Interface::getDistance() {
  std::vector<uint8_t> payload = {0x00, 0x02, 0x00};
  tcpConnection.sendCommand(payload);
}

Interface::~Interface() {
  serverThread->interrupt();
  ioService.stop();
}
}
