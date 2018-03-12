#ifndef __CEDES_FRAME_H__
#define __CEDES_FRAME_H__

#include <cstdint>
#include <vector>

namespace Cedes {

typedef std::vector<uint8_t> Packet;

struct Frame {
  
  enum DataType {
    AMPLITUDE,
    DISTANCE,
    GRAYSCALE
  };

  static const int UDP_HEADER_OFFSET = 20;
  uint16_t payloadHeaderOffset;
  
  uint64_t cnt;

  uint16_t dataType;
  uint64_t frame_id;
  uint16_t width;
  uint16_t height;
  uint32_t px_size;
  uint8_t px_mask;
  uint8_t stride;

  std::vector<uint8_t> data;
  std::vector<uint8_t>::iterator frameIter;
  
  Frame(uint16_t, uint64_t, uint16_t, uint16_t);

  void addDataAtStart(const Packet &);
  void addDataAtOffset(const Packet &);
};
}
#endif // __CEDES_FRAME_H__
