#include <iostream>
#include <ros/ros.h>
#include <cedes/interface.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace Cedes {

class CedesNode {
public:

  enum Command {
    STREAM_DISTANCE,
    STREAM_GRAYSCALE
  };

  CedesNode(ros::NodeHandle* nh)
    : seq(0),
      nh(nh) {
    publisher = nh->advertise<sensor_msgs::Image>("cedes", 1000);
    ROS_INFO("[+] Cedes camera node started");
    conn = iface.subscribe(
      [&](Cedes::Frame f) {
        sensor_msgs::Image img;
        img.header.seq = seq++;
        img.header.stamp = ros::Time::now();
        img.header.frame_id = std::to_string(f.frame_id);
        img.height = static_cast<uint32_t>(f.height);
        img.width = static_cast<uint32_t>(f.width);
        img.encoding = sensor_msgs::image_encodings::MONO16;
        img.step = img.width * f.px_size;
        img.is_bigendian = 1;
        img.data = f.data;
         if (seq == 10) {
         for (int i = 0; i < f.height; ++i) {
         for (int j = 0; j < f.width*2; j += 2) {
           std::cout << (f.data[i*f.width*2 + j] << 8) + f.data[i*f.width*2 + j + 1] << " ";
         } std::cout << std::endl;
         }
         }

        publisher.publish(img);
      });

  }

  ~CedesNode() {
    publisher.shutdown();
    conn.disconnect();
  }

  void sendCommand(Command c) {
    switch(c) {
      case STREAM_DISTANCE:
        iface.streamDistance();
        break;
      case STREAM_GRAYSCALE:
        iface.streamGrayscale();
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
  boost::signals2::connection conn;
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
