#ifndef __CEDES_INTERFACE_H__
#define __CEDES_INTERFACE_H__

#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>

#include "tcp_connection.hpp"
#include "udp_server.hpp"

namespace Cedes {

typedef std::vector<uint8_t> Packet;

struct CameraInfo {
  uint16_t width;
  uint16_t height;
  uint16_t roiX0;
  uint16_t roiY0;
  uint16_t roiX1;
  uint16_t roiY1;
};

struct Frame {
  static const int HEADER_SIZE = 20;

  uint64_t frame_id;
  uint16_t width;
  uint16_t height;
  uint32_t px_size;

  std::vector<uint8_t> data;
  //std::vector<uint8_t>::iterator frame_iter;
  size_t frame_iter;
  
  Frame(uint64_t, uint16_t, uint16_t);

  void addDataAtOffset(Packet, uint16_t);
};

class Interface {
public:
  Interface();
  ~Interface();

  void stopStream();
  void streamDistance();
  void streamAmplitude();
  void streamGrayscale();
  void getDistanceFrame();
  void getGrayscaleFrame();
  void setIntegrationTime(uint16_t, uint16_t, uint16_t, uint16_t);
  boost::signals2::connection subscribeFrame(std::function<void (Frame)>);
  boost::signals2::connection subscribeCameraInfo(std::function<void (CameraInfo)>);
  CameraInfo getCameraInfo();

private:
  bool isStreaming;
  uint64_t currentFrame_id;

  Frame* currentFrame;
  boost::asio::io_service ioService;
  boost::scoped_ptr<boost::thread> serverThread;
  boost::signals2::signal<void (Frame)> frameReady;
  boost::signals2::signal<void (CameraInfo)> cameraInfoReady;
  TcpConnection tcpConnection;
  UdpServer udpServer;
};
}
#endif
