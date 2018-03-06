#ifndef __CEDES_UDPSERVER_H__
#define __CEDES_UDPSERVER_H__

#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/signals2.hpp>

using boost::asio::ip::udp;

namespace Cedes {

class UdpServer {
public:
  UdpServer(boost::asio::io_service& ios)
    : socket(ios, udp::endpoint(udp::v4(), PORT)),
      totalBytes(0) {
    startReceive();
  } 
  ~UdpServer();

  boost::signals2::signal<void ()> dataReady;

private:
  void startReceive();
  void handleReceive(const boost::system::error_code &, std::size_t);

  size_t totalBytes;
  static const int MAX_BUF_SIZE = 2048;
  static const int PORT = 45454;
  udp::socket socket;
  udp::endpoint remoteEndpoint;
  boost::array<uint8_t, MAX_BUF_SIZE> recvBuffer;
};
}
#endif
