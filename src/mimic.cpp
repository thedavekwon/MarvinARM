#include "mimic.h"

#include <exception>

bool markers_fetched;

visualization_msgs::MarkerArray currentMarkerArray;

int main(int argc, char ** argv) {
  ros::init(argc, argv, "mimic");

  int arm_type;
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  nh.getParam("arm_type", arm_type);
  std::cout << "arm_type " << arm_type << std::endl;
  if (arm_type == LEFT) std::cout << "LEFT" << std::endl;
  else if (arm_type == RIGHT) std::cout << "RIGHT" << std::endl;
  std::cout << arm_type << std::endl;
  ros::Subscriber markerarrays_sub = n.subscribe("/body_tracking_data", 100, &getMarkerArraysFromKinect);

  Arm marvin_arm  = Arm(arm_type);

  ros::Rate loop_rate(10);
  while ( ros::ok() ) {
    if (markers_fetched) {
      marvin_arm.calculateJointAngle(currentMarkerArray);
      marvin_arm.setJointSpacePath(1);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void getMarkerArraysFromKinect(const visualization_msgs::MarkerArray::ConstPtr &msg) {
  markers_fetched = true;
  if(msg) currentMarkerArray = *msg;
}
