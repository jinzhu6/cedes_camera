#include <iostream>
#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>

#include "interface.hpp"

namespace Cedes {

Frame::Frame(uint64_t frame_id, uint16_t width, uint16_t height)
  : frame_id(frame_id),
    width(width),
    height(height),
    px_size(sizeof(uint16_t)),
    data(std::vector<uint8_t>(width*height*px_size)),
    frame_iter(0) {}

void Frame::addDataAtOffset(Packet p, uint16_t offsetInPacket) {
  uint32_t payloadSize   = (p[6] << 8) + p[7];
  uint32_t dataSize = payloadSize - offsetInPacket;
  size_t dataStart = HEADER_SIZE + offsetInPacket;

  if (dataStart % 2 == 1) {
    data[frame_iter++] = p[dataStart+1];
  }
  for (int i = dataStart; i < dataStart + dataSize - 1; i += 2) {
    data[frame_iter++] = p[i];
    data[frame_iter++] = p[i+1];
  }
  if (dataSize % 2 == 1) {
    data[frame_iter++] = p[dataStart + dataSize - 1];
  }
}

Interface::Interface() 
  : tcpConnection(ioService),
    udpServer(ioService),
    isStreaming(false),
    currentFrame_id(0)  {
  serverThread.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, &ioService)));
  udpServer.subscribe(
    [&](Packet p, size_t packetSize) -> void {
      uint32_t numPackets = (p[12] << 24) + (p[13] << 16) + (p[14] << 8) + p[15];
      uint32_t packetNum  = (p[16] << 24) + (p[17] << 16) + (p[18] << 8) + p[19];

      uint16_t offsetInPacket = 0;
      if (packetNum == 0) {
        uint16_t width  = (p[23] << 8) + p[24];
        uint16_t height = (p[25] << 8) + p[26];
        offsetInPacket += (p[43] << 8) + p[44];
        currentFrame = new Frame(currentFrame_id++, width, height);
      }
      currentFrame->addDataAtOffset(p, offsetInPacket);
      if (packetNum == numPackets - 1) {
        frameReady(*currentFrame);
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

void Interface::streamDistance() {
  std::vector<uint8_t> payload = {0x00, 0x03, 0x01};
  tcpConnection.sendCommand(payload);
  isStreaming = true;
}

void Interface::streamAmplitude() {
  std::vector<uint8_t> payload = {0x00, 0x04, 0x01};
  tcpConnection.sendCommand(payload);
  isStreaming = true;
}

void Interface::streamGrayscale() {
  std::vector<uint8_t> payload = {0x00, 0x05, 0x01};
  tcpConnection.sendCommand(payload);
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
  std::function<void (CameraInfo)> onCameraInfoReady) {
  cameraInfoReady.connect(onCameraInfoReady);
}

boost::signals2::connection Interface::subscribeFrame(
  std::function<void (Frame)> onFrameReady) {
  frameReady.connect(onFrameReady);
}

CameraInfo Interface::getCameraInfo() {
  bool hasCapturedInfo = false;
  stopStream();
  CameraInfo camInfo;

  boost::signals2::connection c;
  c = udpServer.subscribe(
    [&](Packet p, size_t packetSize) -> void {
      if (!hasCapturedInfo) {
        int offset = 20; // payload offset
      /*camInfo.version =*/ p[offset++];
      /*camInfo.dataType =*/(p[offset++] << 8) + p[offset++];
        camInfo.width   = (p[offset++] << 8) + p[offset++];
        camInfo.height  = (p[offset++] << 8) + p[offset++];
        camInfo.roiX0   = (p[offset++] << 8) + p[offset++];
        camInfo.roiY0   = (p[offset++] << 8) + p[offset++];
        camInfo.roiX1   = (p[offset++] << 8) + p[offset++];
        camInfo.roiY1   = (p[offset++] << 8) + p[offset++];
        // camInfo.int_time_low = (p[offset++] << 8) + p[offset++] << std::endl;
        // camInfo.int_time_mid = (p[offset++] << 8) + p[offset++] << std::endl;
        // camInfo.int_time_high = (p[offset++] << 8) + p[offset++] << std::endl;
        // camInfo.mgx = (p[offset++] << 8) + p[offset++] << std::endl;
        // camInfo.offset = (p[offset++] << 8) + p[offset++] << std::endl;
        hasCapturedInfo = true;
        c.disconnect();
        cameraInfoReady(camInfo);
      }
    });
    getDistanceFrame();
}
}
