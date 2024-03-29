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
  void streamAmplitude();
  void streamDistance();
  void streamGrayscale();
  void setIntegrationTime(uint16_t, uint16_t, uint16_t, uint16_t);
  boost::signals2::connection subscribeFrame(std::function<void (std::shared_ptr<Frame>)>);
  boost::signals2::connection subscribeCameraInfo(std::function<void (std::shared_ptr<CameraInfo>)>);
  std::shared_ptr<CameraInfo> getCameraInfo(const Packet &);

private:
  bool isStreaming;
  uint8_t dataType;
  uint64_t currentFrame_id;

  const static uint8_t COMMAND_SET_INT_TIMES = 1;
  const static uint8_t COMMAND_GET_DIST_AND_AMP = 2;
  const static uint8_t COMMAND_GET_DISTANCE = 3;
  const static uint8_t COMMAND_GET_GRAYSCALE = 5;
  const static uint8_t COMMAND_STOP_STREAM = 6;
  const static uint8_t COMMAND_CALIBRATE = 30;

  std::shared_ptr<Frame> currentFrame;
  boost::asio::io_service ioService;
  boost::scoped_ptr<boost::thread> serverThread;
  boost::signals2::signal<void (std::shared_ptr<Frame>)> frameReady;
  boost::signals2::signal<void (std::shared_ptr<CameraInfo>)> cameraInfoReady;
  TcpConnection tcpConnection;
  UdpServer udpServer;

  void setDataType(uint8_t);
  void streamMeasurement(uint8_t);
};
}
#endif
