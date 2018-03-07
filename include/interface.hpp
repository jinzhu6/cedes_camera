#ifndef __CEDES_INTERFACE_H__
#define __CEDES_INTERFACE_H__

#include <iostream>
#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>

#include "tcp_connection.hpp"
#include "udp_server.hpp"

namespace Cedes {

typedef std::vector<uint8_t> Packet;

struct Frame {
  static const int HEADER_SIZE = 20;

  uint16_t width;
  uint16_t height;
  std::vector<uint16_t> data;
  
  Frame(uint16_t, uint16_t);

  void addDataAtOffset(Packet, uint16_t);
};

class Interface {
public:
  Interface();
  ~Interface();

  void stopStream();
  void streamDistance();
  void streamGrayscale();
  void getDistanceFrame();
  void getGrayscaleFrame();
  void setIntegrationTime(uint16_t, uint16_t, uint16_t, uint16_t);
  boost::signals2::connection subscribe(std::function<void (Frame)>);
  void printCameraSettings();

private:
  bool isStreaming;
  Frame* currentFrame;
  boost::asio::io_service ioService;
  boost::scoped_ptr<boost::thread> serverThread;
  boost::signals2::signal<void (Frame)> frameReady;
  TcpConnection tcpConnection;
  UdpServer udpServer;
};
}
#endif
