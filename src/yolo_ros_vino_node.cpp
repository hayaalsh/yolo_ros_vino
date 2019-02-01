#include <yolo_ros_vino/yolo_ros_vino.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "darknet_ros");
  ros::NodeHandle nodeHandle("~");
  YoloRosVino YoloRosVino(nodeHandle);
  ros::spin();
  return 0;
}