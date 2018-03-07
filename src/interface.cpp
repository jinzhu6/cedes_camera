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

void Interface::setIntegrationTime(uint16_t low, uint16_t mid, uint16_t high, uint16_t gray) {
  std::vector<uint8_t> payload = {
    0x00, 0x01,
    static_cast<uint8_t>(low >> 8), static_cast<uint8_t>(low & 0x00ff),
    static_cast<uint8_t>(mid >> 8), static_cast<uint8_t>(mid & 0x00ff),
    static_cast<uint8_t>(high >> 8), static_cast<uint8_t>(high & 0x00ff),
    static_cast<uint8_t>(gray >> 8), static_cast<uint8_t>(gray & 0x00ff)};
  tcpConnection.sendCommand(payload);
}

boost::signals2::connection Interface::subscribe(
  std::function<void (Frame)> onFrameReady) {
  frameReady.connect(onFrameReady);
}

void Interface::printCameraSettings() {
  bool hasCapturedSettings = false;
  stopStream();
  boost::signals2::connection c = udpServer.subscribe(
    [&](Packet p, size_t packetSize) -> void {
      if (!hasCapturedSettings) {
        int i = 20; // payload offset
        std::cout << "Version: "       << +p[i++] << std::endl;
        std::cout << "Datatype: "      << (p[i++] << 8) + p[i++] << std::endl;
        std::cout << "Width: "         << (p[i++] << 8) + p[i++] << std::endl;
        std::cout << "Height: "        << (p[i++] << 8) + p[i++] << std::endl;
        std::cout << "RoiX0: "         << (p[i++] << 8) + p[i++] << std::endl;
        std::cout << "RoiY0: "         << (p[i++] << 8) + p[i++] << std::endl;
        std::cout << "RoiX1: "         << (p[i++] << 8) + p[i++] << std::endl;
        std::cout << "RoiY1: "         << (p[i++] << 8) + p[i++] << std::endl;
        std::cout << "Int time low: "  << (p[i++] << 8) + p[i++] << std::endl;
        std::cout << "Int time mid: "  << (p[i++] << 8) + p[i++] << std::endl;
        std::cout << "Int time high: " << (p[i++] << 8) + p[i++] << std::endl;
        std::cout << "MGX: "           << (p[i++] << 8) + p[i++] << std::endl;
        std::cout << "Offset: "        << (p[i++] << 8) + p[i++] << std::endl;
        hasCapturedSettings = true;
      }
    });
    getDistanceFrame();
    while (!hasCapturedSettings);
    c.disconnect();
}
}
