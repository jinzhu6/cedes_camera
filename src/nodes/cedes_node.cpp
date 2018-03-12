#include <iostream>
#include <ros/ros.h>
#include <cedes/interface.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace Cedes {

class CedesNode {
public:

  enum Command {
    CAPTURE_DISTANCE,
    STREAM_DISTANCE,
    STREAM_GRAYSCALE,
    STREAM_AMPLITUDE,
    CAMERA_INFO
  };

  CedesNode(ros::NodeHandle* nh)
    : seq(0),
      nh(nh) {
    publisher = nh->advertise<sensor_msgs::Image>("cedes", 1000);
    connectionCameraInfo = iface.subscribeCameraInfo(
      [&](CameraInfo camInfo) -> void {
        std::cout << "I got camera info!" << std::endl;
      }
    );
    connectionFrames = iface.subscribeFrame(
      [&](std::shared_ptr<Frame> f) {
        sensor_msgs::Image img;
        img.header.seq = seq++;
        img.header.stamp = ros::Time::now();
        img.header.frame_id = std::to_string(f->frame_id);
        img.height = static_cast<uint32_t>(f->height);
        img.width = static_cast<uint32_t>(f->width);
        img.encoding = sensor_msgs::image_encodings::MONO16;
        img.step = img.width * f->px_size;
        img.is_bigendian = 0;
        img.data = f->data;
//        if (seq == 1) {
//          for (int i = 0; i < f.height; ++i) {
//            for (int j = 0; j < f.width*2; j += 2) {
//              std::cout << (f.data[i*f.width*2 + j + 1] << 8) + f.data[i*f.width*2 + j] << " ";
//            } std::cout << std::endl;
//          }
//        }

        publisher.publish(img);
      });
    ROS_INFO("[+] Cedes camera node started");
  }

  ~CedesNode() {
    publisher.shutdown();
    connectionFrames.disconnect();
    connectionCameraInfo.disconnect();
  }

  void setIntegrationTime(uint16_t low, uint16_t mid, uint16_t high, uint16_t gray) {
    iface.setIntegrationTime(low, mid, high, gray);
  }

  void sendCommand(Command c) {
    switch(c) {
      case CAPTURE_DISTANCE:
        iface.getDistanceFrame();
        break;
      case STREAM_DISTANCE:
        iface.streamDistance();
        break;
      case STREAM_GRAYSCALE:
        iface.streamGrayscale();
        break;
      case STREAM_AMPLITUDE:
        iface.streamAmplitude();
        break;
      case CAMERA_INFO:
        iface.getCameraInfo();
        break;
      default:
        break;
    }
  }

private:
  uint32_t seq;
  Cedes::Interface iface;
  ros::NodeHandle* nh;
  ros::Publisher publisher;
  ros::ServiceServer service;
  boost::signals2::connection connectionFrames;
  boost::signals2::connection connectionCameraInfo;
};
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "cedes");
  ros::NodeHandle nh;

  Cedes::CedesNode node(&nh);
  node.sendCommand(node.STREAM_DISTANCE);

  // Let ROS handle all callbacks.
  ros::spin();

  return 0;
}  // end main()
