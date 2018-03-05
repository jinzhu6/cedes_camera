#include <iostream>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

namespace Cedes {

class UDPServer {
public:
  UDPServer(boost::asio::io_service& io_service)
    : socket(io_service, udp::endpoint(udp::v4(), UDP_PORT)),
      totalBytesReceived(0) {} 

  start() {
    socket.async_receive_from(
      boost::asio::buffer(recv_buffer),
      remote_endpoint,
      boost::bind(
        &UDPServer::handle_receive,
        this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
  }
private:
  udp::socket socket;
  udp::endpoint remote_endpoint;
  int totalBytesReceived;

  void handle_receive(
    const boost::system::error_code& error,
    std::size_t bytes_received) {
    
    if (!error || error == boost::asio::error::message_size) {
      totalBytesReceived += bytesReceived;
      std::cout << "Total bytes received: " << totalBytesReceived << std::endl;
    }
    start();
  }
};
}
