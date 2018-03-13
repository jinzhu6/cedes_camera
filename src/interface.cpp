#include <iostream>
#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>

#include "frame.hpp"
#include "interface.hpp"

namespace Cedes {

Interface::Interface() 
  : tcpConnection(ioService),
    udpServer(ioService),
    isStreaming(false),
    dataType(0),
    currentFrame_id(0)  {
  serverThread.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, &ioService)));
  udpServer.subscribe(
    [&](const Packet& p) -> void {
      uint32_t packetNum  = (p[16] << 24) + (p[17] << 16) + (p[18] << 8) + p[19];

      if (packetNum == 0) { // new frame
        uint16_t width  = (p[23] << 8) + p[24];
        uint16_t height = (p[25] << 8) + p[26];
        currentFrame = std::shared_ptr<Frame>(new Frame(dataType, currentFrame_id++, width, height));
        currentFrame->addDataAtStart(p);

        cameraInfoReady(getCameraInfo(p));
      } else {
        uint32_t numPackets = (p[12] << 24) + (p[13] << 16) + (p[14] << 8) + p[15];
        currentFrame->addDataAtOffset(p);
        if (packetNum == numPackets - 1) {
          frameReady(currentFrame);
        }  
      }
    });
}

Interface::~Interface() {
  stopStream();
  serverThread->interrupt();
  ioService.stop();
}

void Interface::stopStream() {
  if (!isStreaming) { return; }
  std::vector<uint8_t> payload = {0x00, 0x06};
  tcpConnection.sendCommand(payload);
  isStreaming = false;
}

void Interface::streamAmplitude() {
  setDataType(0);
  streamMeasurement(0x02);
}

void Interface::streamDistance() {
  setDataType(1);
  streamMeasurement(0x03);
}

void Interface::streamGrayscale() {
  setDataType(1);
  streamMeasurement(0x05);
}

void Interface::streamMeasurement(uint8_t cmd) {
  tcpConnection.sendCommand(std::vector<uint8_t>({0x00, cmd, 0x01}));
  isStreaming = true;
}

void Interface::getDistanceFrame() {
  stopStream();
  std::vector<uint8_t> payload = {0x00, 0x03, 0x00};
  tcpConnection.sendCommand(payload);
}

void Interface::getGrayscaleFrame() {
  stopStream();
  std::vector<uint8_t> payload = {0x00, 0x05, 0x00};
  tcpConnection.sendCommand(payload);
}

void Interface::setIntegrationTime(uint16_t low, uint16_t mid, uint16_t high, uint16_t gray) {
  std::vector<uint8_t> payload = {
    0x00, 0x01,
    static_cast<uint8_t>(low >> 8), static_cast<uint8_t>(low & 0x00ff),
    static_cast<uint8_t>(mid >> 8), static_cast<uint8_t>(mid & 0x00ff),
    static_cast<uint8_t>(high >> 8), static_cast<uint8_t>(high & 0x00ff),
    static_cast<uint8_t>(gray >> 8), static_cast<uint8_t>(gray & 0x00ff)};
  tcpConnection.sendCommand(payload);
}

boost::signals2::connection Interface::subscribeCameraInfo(
  std::function<void (std::shared_ptr<CameraInfo>)> onCameraInfoReady) {
  cameraInfoReady.connect(onCameraInfoReady);
}

boost::signals2::connection Interface::subscribeFrame(
  std::function<void (std::shared_ptr<Frame>)> onFrameReady) {
  frameReady.connect(onFrameReady);
}

void Interface::setDataType(uint8_t d) {
  dataType = d;
}

std::shared_ptr<CameraInfo> Interface::getCameraInfo(const Packet& p) {
  std::shared_ptr<CameraInfo> camInfo(new CameraInfo);

  int offset = 23;
  camInfo->width  = (p[offset++] << 8) + p[offset++];
  camInfo->height = (p[offset++] << 8) + p[offset++];
  camInfo->roiX0  = (p[offset++] << 8) + p[offset++];
  camInfo->roiY0  = (p[offset++] << 8) + p[offset++];
  camInfo->roiX1  = (p[offset++] << 8) + p[offset++];
  camInfo->roiY1  = (p[offset++] << 8) + p[offset++];

  return camInfo;
}
}
