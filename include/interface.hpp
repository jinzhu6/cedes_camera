#ifndef __CEDES_INTERFACE_H__
#define __CEDES_INTERFACE_H__

#include <iostream>
#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>

#include "tcp_connection.hpp"
#include "udp_server.hpp"

namespace Cedes {
struct Frame {
  uint16_t width;
  uint16_t height;
  static const int HEADER_SIZE = 20;
  std::vector<uint16_t> data;
  
  Frame(uint16_t, uint16_t);

  void addDataAtOffset(Packet, uint16_t);
};

class Interface {
public:
  Interface();
  ~Interface();

  void getDistance();
private:
  Frame* currentFrame;
  boost::asio::io_service ioService;
  boost::scoped_ptr<boost::thread> serverThread;
  TcpConnection tcpConnection;
  UdpServer udpServer;
  Packet recvBuff;
};
}
#endif
