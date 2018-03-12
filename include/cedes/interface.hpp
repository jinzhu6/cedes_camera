#ifndef __CEDES_INTERFACE_H__
#define __CEDES_INTERFACE_H__

#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>

#include "frame.hpp"
#include "camera_info.hpp"
#include "tcp_connection.hpp"
#include "udp_server.hpp"

namespace Cedes {

typedef std::vector<uint8_t> Packet;

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
  boost::signals2::connection subscribeFrame(std::function<void (std::shared_ptr<Frame>)>);
  boost::signals2::connection subscribeCameraInfo(std::function<void (CameraInfo)>);
  CameraInfo getCameraInfo();

private:
  bool isStreaming;
  uint64_t currentFrame_id;

  std::shared_ptr<Frame> currentFrame;
  boost::asio::io_service ioService;
  boost::scoped_ptr<boost::thread> serverThread;
  boost::signals2::signal<void (std::shared_ptr<Frame>)> frameReady;
  boost::signals2::signal<void (CameraInfo)> cameraInfoReady;
  TcpConnection tcpConnection;
  UdpServer udpServer;

  void streamMeasurement(uint8_t);
};
}
#endif
