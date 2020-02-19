#include "mimic.h"

#include <exception>

bool markers_fetched;

visualization_msgs::MarkerArray currentMarkerArray;

int main(int argc, char ** argv) {
  ros::init(argc, argv, "mimic");

  ros::NodeHandle n;
  ros::Subscriber markerarrays_sub = n.subscribe("/body_tracking_data", 100, &getMarkerArraysFromKinect);

  Arm marvin_right = Arm(RIGHT);
  // Arm marvin_left  = Arm(LEFT);

  ros::Rate loop_rate(10);
  while ( ros::ok() ) {
    if (markers_fetched) {
      marvin_right.calculateJointAngle(currentMarkerArray);
      // marvin_left.calculateJointAngle(currentMarkerArray);
      marvin_right.setJointSpacePath(1);
      // marvin_left.setJointSpacePath(1);
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
