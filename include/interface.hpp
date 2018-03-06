#ifndef __CEDES_INTERFACE_H__
#define __CEDES_INTERFACE_H__

#include <iostream>
#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>

#include "tcp_connection.hpp"
#include "udp_server.hpp"

namespace Cedes {

class Interface {
public:
  Interface()
    : tcpConnection(ioService),
      udpServer(ioService) {
    tcpConnection.connect();
    serverThread.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, &ioService)));
    udpServer.dataReady.connect(helloWorld);
  }
  ~Interface();

  void getDistance();
private:
  boost::asio::io_service ioService;
  TcpConnection tcpConnection;
  UdpServer udpServer;
  boost::scoped_ptr<boost::thread> serverThread;
  
  struct HelloWorld {
    void operator()() const {
      std::cout << "Data ready" << std::endl;
    }
  };

  HelloWorld helloWorld;
};
}
#endif
