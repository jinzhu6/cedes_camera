#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>

#include "udp_server.hpp"

using boost::asio::ip::udp;

namespace Cedes {

UdpServer::~UdpServer() {
  boost::system::error_code error;
  socket.shutdown(boost::asio::ip::udp::socket::shutdown_both, error);  
  if (!error) {
    socket.close(error);
  }
}

void UdpServer::startReceive() {
  socket.async_receive_from(
    boost::asio::buffer(recvBuffer),
    remoteEndpoint,
    boost::bind(
      &UdpServer::handleReceive,
      this,
      boost::asio::placeholders::error,
      boost::asio::placeholders::bytes_transferred));
}

void UdpServer::handleReceive(
  const boost::system::error_code& error,
  std::size_t bytesReceived) {
  if (!error || error == boost::asio::error::message_size) {
    totalBytes += bytesReceived;
    dataReady();
  }
  startReceive();
}
}
