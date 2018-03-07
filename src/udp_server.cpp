#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>

#include "udp_server.hpp"

using boost::asio::ip::udp;

namespace Cedes {

UdpServer::UdpServer(boost::asio::io_service& ios)
  : socket(ios, udp::endpoint(udp::v4(), PORT)),
    recvBuffer(Packet(2048)) {
  startReceive();
} 

UdpServer::~UdpServer() {
  boost::system::error_code error;
  socket.shutdown(boost::asio::ip::udp::socket::shutdown_both, error);  
  if (!error) {
    socket.close(error);
  }
}

boost::signals2::connection UdpServer::subscribe(std::function<void(Packet, size_t)> onDataReady) {
  return dataReady.connect(onDataReady);
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
    dataReady(recvBuffer, bytesReceived);
  }
  startReceive();
}
}
