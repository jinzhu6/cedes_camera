#include <map>
#include <iostream>
#include <ros/ros.h>
#include <cedes/interface.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>

namespace Cedes {

class CedesNode {
public:

  CedesNode()
    : frameSeq(0),
      cameraSeq(0),
      nh("~") {

    nh.getParam("stream", streamType);
    nh.getParam("int0", int0);
    nh.getParam("int1", int1);
    nh.getParam("int2", int2);
    nh.getParam("intGr", intGr);

    imagePublisher = nh.advertise<sensor_msgs::Image>("image_raw", 1000);
    connectionCameraInfo = iface.subscribeCameraInfo(
      [&](std::shared_ptr<CameraInfo> ci) -> void {
        cameraInfo.width = ci->width;
        cameraInfo.height = ci->height;
        cameraInfo.roi.x_offset = ci->roiX0;
        cameraInfo.roi.y_offset = ci->roiY0;
        cameraInfo.roi.width = ci->roiX1 - ci->roiX0;
        cameraInfo.roi.height = ci->roiY1 - ci->roiY0;
      }
    );
    cameraInfoPublisher = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1000);

    cameraInfoService = nh.advertiseService("set_camera_info",
      &Cedes::CedesNode::setCameraInfo, this);

    connectionFrames = iface.subscribeFrame(
      [&](std::shared_ptr<Frame> f) -> void {
        sensor_msgs::Image img;
        img.header.seq = frameSeq++;
        img.header.stamp = ros::Time::now();
        img.header.frame_id = std::to_string(f->frame_id);
        img.height = static_cast<uint32_t>(f->height);
        img.width = static_cast<uint32_t>(f->width);
        img.encoding = sensor_msgs::image_encodings::MONO16;
        img.step = img.width * f->px_size;
        img.is_bigendian = 0;
        img.data = f->data;
        imagePublisher.publish(img);
      });
    ROS_INFO("[Cedes] Camera node started");
  }

  void run() {
    setIntegrationTimes();
    sendCommand();
    ROS_INFO("[Cedes] Streaming %s", streamType.c_str());  
  }

  ~CedesNode() {
    imagePublisher.shutdown();
    cameraInfoPublisher.shutdown();
    connectionFrames.disconnect();
    connectionCameraInfo.disconnect();
  }

private:
  enum Command {
    STREAM_AMPLITUDE,
    STREAM_DISTANCE,
    STREAM_GRAYSCALE,
  };

  int int0, int1, int2, intGr;
  uint32_t frameSeq, cameraSeq;
  std::string streamType;
  std::map<std::string, Command> commandMap =
    {{"amplitude", STREAM_AMPLITUDE},
     {"distance", STREAM_DISTANCE}, 
     {"grayscale", STREAM_GRAYSCALE}}; 

  ros::NodeHandle nh;
  ros::Publisher imagePublisher;
  ros::Publisher cameraInfoPublisher;
  ros::ServiceServer cameraInfoService;

  sensor_msgs::CameraInfo cameraInfo;

  Cedes::Interface iface;

  boost::signals2::connection connectionFrames;
  boost::signals2::connection connectionCameraInfo;

  void setIntegrationTimes() {
    iface.setIntegrationTime(int0, int1, int2, intGr);
  }

  void sendCommand() {
    switch(commandMap[streamType]) {
      case STREAM_AMPLITUDE:
        iface.streamAmplitude();
        break;
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

  bool setCameraInfo(
    sensor_msgs::SetCameraInfo::Request& req,
    sensor_msgs::SetCameraInfo::Response& res) {
    req.camera_info.width = cameraInfo.width;
    req.camera_info.height = cameraInfo.height;
    req.camera_info.roi = cameraInfo.roi;

    cameraInfoPublisher.publish(req.camera_info);

    res.success = true;
    res.status_message = "";
    
    return true;
  }
};
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cedes");

  Cedes::CedesNode node;
  node.run();

  ros::spin();

  return 0;
}  // end main()
