#include <iostream>
#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>

#include "interface.hpp"

namespace Cedes {

Frame::Frame(uint16_t width, uint16_t height)
  : width(width),
    height(height),
    data(std::vector<uint16_t>(width*height)) {}

void Frame::addDataAtOffset(Packet p, uint16_t offsetInPacket) {
  uint32_t payloadSize   = (p[6] << 8) + p[7];
  uint32_t offsetInFrame = (p[8] << 24) + (p[9] << 16) + (p[10] << 8) + p[11];
  for (int i = HEADER_SIZE + offsetInPacket, j = offsetInFrame/2; i < payloadSize; i += 2, ++j) {
    data[j] = ((p[i] & 0x3F) << 8) + p[i+1];
  }
}

Interface::Interface() 
  : tcpConnection(ioService),
    udpServer(ioService),
    isStreaming(false) {
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
        currentFrame = new Frame(width, height);
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

void Interface::subscribe(std::function<void (Frame)> onFrameReady) {
  frameReady.connect(onFrameReady);
}
}
