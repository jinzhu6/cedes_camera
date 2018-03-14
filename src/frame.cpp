#include "frame.hpp"
#include <stdint.h>

namespace Cedes {

Frame::Frame(uint16_t dataType, uint64_t frame_id, uint16_t width, uint16_t height)
  : frame_id(frame_id),
    width(width),
    height(height),
    px_size(sizeof(uint16_t)),
    data(std::vector<uint8_t>(width*height*px_size)),
    frameIter(data.begin()) {
      if (dataType == 0) {
        px_mask = 0x0F;
        stride = 4;
      } else {
        px_mask = 0x3F;
        stride = 2;
      }
}

void Frame::addDataAtStart(const Packet& p) {
  payloadHeaderOffset = (p[43] << 8) + p[44];

  const uint16_t payloadSize = (p[6] << 8) + p[7];

  const uint32_t payloadDataStart = UDP_HEADER_OFFSET + payloadHeaderOffset;
  const uint32_t payloadDataEnd = UDP_HEADER_OFFSET + payloadSize;
  const uint32_t payloadDataSize = payloadSize - payloadHeaderOffset;

  const uint32_t frameDataStart = 0;
  const uint32_t frameDataEnd = frameDataStart + payloadSize - payloadHeaderOffset;

  int i, j;
  for (i = payloadDataStart, j = 2*frameDataStart/stride; i + 1 < payloadDataEnd; i += stride, j += 2) {
    data[j] = p[i];
    data[j+1] = p[i+1] & px_mask;
  }
  for (; j < 2*frameDataEnd/stride; ++i, ++j) {
    data[j] = p[i];
  }
}

void Frame::addDataAtOffset(const Packet& p) {
  const uint16_t payloadSize = (p[6] << 8) + p[7];

  const uint32_t payloadDataStart = UDP_HEADER_OFFSET;
  const uint32_t payloadDataEnd = UDP_HEADER_OFFSET + payloadSize;

  const uint32_t frameDataStart = (p[8] << 24) + (p[9] << 16) + (p[10] << 8) 
    + p[11] - payloadHeaderOffset;
  const uint32_t frameDataEnd = frameDataStart + payloadSize - payloadHeaderOffset;

  int i, j;
  for (i = payloadDataStart, j = 2*frameDataStart/stride; i + 1 < payloadDataEnd; i += stride, j += 2) {
    data[j] = p[i] & px_mask;
    data[j+1] = p[i+1];
  }
  for (; j < 2*frameDataEnd/stride; ++i, ++j) {
    data[j] = p[i] & px_mask;
  }
}
}
