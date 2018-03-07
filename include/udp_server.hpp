#ifndef __CEDES_UDPSERVER_H__
#define __CEDES_UDPSERVER_H__

#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/signals2.hpp>

using boost::asio::ip::udp;

namespace Cedes {

//typedef boost::array<uint8_t, 2048> Packet;
typedef std::vector<uint8_t> Packet;

class UdpServer {
public:
  UdpServer(boost::asio::io_service &);
  ~UdpServer();

  boost::signals2::signal<void (Packet, size_t)> dataReady;

private:
  void startReceive();
  void handleReceive(const boost::system::error_code &, std::size_t);

  static const int PORT = 45454;
  udp::socket socket;
  udp::endpoint remoteEndpoint;
  Packet recvBuffer;
};
}
#endif
