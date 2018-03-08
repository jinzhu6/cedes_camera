#include <cedes/talker.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "talker");
  ros::NodeHandle nh;

  // Create a new cedes::Talker object.
  cedes::ExampleTalker node(nh);

  // Let ROS handle all callbacks.
  ros::spin();

  return 0;
}  // end main()
