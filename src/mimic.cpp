#include <cmath>
#include <sstream>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

#include "open_manipulator_msgs/OpenManipulatorState.h"
#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
#include "open_manipulator_msgs/SetDrawingTrajectory.h"
#include "open_manipulator_msgs/SetActuatorState.h"

#define NUM_OF_JOINT_AND_TOOL 5

void manipulatorStatesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr &msg);
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);

bool setTaskSpacePath(ros::ServiceClient &service, std::vector<double> kinematics_pose, double path_time);
bool setJointSpacePath(double path_time);

void joint1CallBack(const geometry_msgs::TransformStamped::ConstPtr &msg);
void joint2CallBack(const geometry_msgs::TransformStamped::ConstPtr &msg);
void joint3CallBack(const geometry_msgs::TransformStamped::ConstPtr &msg);
void calculateJoints();
void calculateJointsMarker();

void getMarkerArraysFromKinect(const visualization_msgs::MarkerArray::ConstPtr &msg);

std::vector<double> present_joint_angle_;
std::vector<double> present_kinematic_position_;
std::vector<double> update_joint_angle(4, 0);
std::vector<std::string> joint_name;

open_manipulator_msgs::KinematicsPose kinematics_pose_;

bool open_manipulator_is_moving_ = false;
bool open_manipulator_actuator_enabled_ = false;

ros::ServiceClient goal_task_space_path_position_only_client_;
ros::ServiceClient goal_joint_space_path_client_ ;

geometry_msgs::TransformStamped joint1_tf;
geometry_msgs::TransformStamped joint2_tf;
geometry_msgs::TransformStamped joint3_tf;

visualization_msgs::MarkerArray currentMarkerArray;

bool joint1_fetched;
bool joint2_fetched;
bool joint3_fetched;

bool markers_fetched;


int main(int argc, char ** argv) {
  joint_name.push_back("joint1");
  joint_name.push_back("joint2");
  joint_name.push_back("joint3");
  joint_name.push_back("joint4");
  
  ros::init(argc, argv, "mimic");

  ros::NodeHandle n;

  ros::Subscriber open_manipulator_states_sub_       = n.subscribe("states", 10, &manipulatorStatesCallback);
  ros::Subscriber open_manipulator_joint_states_sub_ = n.subscribe("joint_states", 10, &jointStatesCallback);
  ros::Subscriber open_manipulator_kinematics_pose_sub_ = n.subscribe("kinematics_pose", 10, &kinematicsPoseCallback);

  // ros::Subscriber joint1_sub = n.subscribe("/vicon/joint_0/joint_0", 10, &joint1CallBack);
  // ros::Subscriber joint2_sub = n.subscribe("/vicon/joint_1/joint_1", 10, &joint2CallBack);
  // ros::Subscriber joint3_sub = n.subscribe("/vicon/joint_2/joint_2", 10, &joint3CallBack);

  ros::Subscriber markerarrays_sub = n.subscribe("/body_tracking_data", 10, &getMarkerArraysFromKinect);

  goal_task_space_path_position_only_client_ = n.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_position_only");
  goal_joint_space_path_client_ = n.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");

  ros::Rate loop_rate(10);
  while ( ros::ok() ) {
    // if (joint1_fetched && joint2_fetched && joint3_fetched) {
    if (markers_fetched) {
      calculateJointsMarker();
      setJointSpacePath(1);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void getMarkerArraysFromKinect(const visualization_msgs::MarkerArray::ConstPtr &msg) {
  markers_fetched = true;
  currentMarkerArray = *msg;
}

// Update Joint 1
void joint1CallBack(const geometry_msgs::TransformStamped::ConstPtr &msg) {
  joint1_tf = *msg;
  static tf::TransformBroadcaster br1;
  tf::Transform tmp(tf::Quaternion(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w), tf::Vector3(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z));
  br1.sendTransform(tf::StampedTransform(tmp, ros::Time::now(), "world", "/vicon/joint_0/joint_0"));
  joint1_fetched = true;
}

// Update Joint 2
void joint2CallBack(const geometry_msgs::TransformStamped::ConstPtr &msg) {
  joint2_tf = *msg;
  static tf::TransformBroadcaster br2;
  tf::Transform tmp(tf::Quaternion(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w), tf::Vector3(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z));
  br2.sendTransform(tf::StampedTransform(tmp, ros::Time::now(), "world", "/vicon/joint_1/joint_1"));
  joint2_fetched = true;
}

// Update Joint 3
void joint3CallBack(const geometry_msgs::TransformStamped::ConstPtr &msg) {
  joint3_tf = *msg;
  static tf::TransformBroadcaster br3;
  tf::Transform tmp(tf::Quaternion(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w), tf::Vector3(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z));
  br3.sendTransform(tf::StampedTransform(tmp, ros::Time::now(), "world", "/vicon/joint_2/joint_2"));
  joint3_fetched = true;
}

//
// void calculateJoints() {
//   tf::Vector3 joint1(joint1_tf.transform.translation.x, joint1_tf.transform.translation.y, joint1_tf.transform.translation.z);
//   tf::Vector3 joint2(joint2_tf.transform.translation.x, joint2_tf.transform.translation.y, joint2_tf.transform.translation.z);
//   tf::Vector3 joint3(joint3_tf.transform.translation.x, joint3_tf.transform.translation.y, joint3_tf.transform.translation.z);

//   tf::Vector3 link12_normed = (joint2-joint1).normalize();
//   tf::Vector3 link23_normed = (joint3-joint2).normalize();

//   // update_joint_angle[1] = acos((link12_normed).dot(joint1.normalize()))-M_PI;
//   update_joint_angle[1] = -0.2;
//   update_joint_angle[2] = acos((link23_normed).dot(link12_normed))*180/M_PI;
// }

void calculateJointsMarker() {
  tf::Vector3 joint2(currentMarkerArray.markers[2].pose.position.x, currentMarkerArray.markers[2].pose.position.y, currentMarkerArray.markers[2].pose.position.z);
  tf::Vector3 joint11(currentMarkerArray.markers[11].pose.position.x, currentMarkerArray.markers[11].pose.position.y, currentMarkerArray.markers[11].pose.position.z);
  tf::Vector3 joint12(currentMarkerArray.markers[12].pose.position.x, currentMarkerArray.markers[12].pose.position.y, currentMarkerArray.markers[12].pose.position.z);
  tf::Vector3 joint13(currentMarkerArray.markers[13].pose.position.x, currentMarkerArray.markers[13].pose.position.y, currentMarkerArray.markers[13].pose.position.z);
  tf::Vector3 joint14(currentMarkerArray.markers[14].pose.position.x, currentMarkerArray.markers[14].pose.position.y, currentMarkerArray.markers[14].pose.position.z);
  tf::Vector3 joint15(currentMarkerArray.markers[15].pose.position.x, currentMarkerArray.markers[15].pose.position.y, currentMarkerArray.markers[15].pose.position.z);
  tf::Vector3 joint16(currentMarkerArray.markers[16].pose.position.x, currentMarkerArray.markers[16].pose.position.y, currentMarkerArray.markers[16].pose.position.z);

  // comment 4 now: tf::Vector3 link11_2_normed = (joint11-joint2).normalize();
  tf::Vector3 link02_11_normed = (joint2-joint11).normalize();
  tf::Vector3 link11_12_normed = (joint11-joint12).normalize();
  tf::Vector3 link12_13_normed = (joint12-joint13).normalize();
  tf::Vector3 link13_14_normed = (joint13-joint14).normalize();
  tf::Vector3 link14_15_normed = (joint14-joint15).normalize();
  tf::Vector3 link14_16_normed = (joint14-joint16).normalize();

  // update_joint_angle[1] = acos((link12_normed).dot(joint1.normalize()))-M_PI;
  update_joint_angle[1] = -0.3;
  update_joint_angle[3] = acos((-1*link02_11_normed).dot(link11_12_normed)); //shoulder angle
  //update_joint_angle[?] = acos((-1*link11_12_normed).dot(link12_13_normed)); //im a little confused about this one
  //update_joint_angle[?] = acos((-1*link12_13_normed).dot(link13_14_normed)); //elbow angle
  //update_joint_angle[?] = acos((-1*link13_14_normed).dot(link14_15_normed)); //wrist angle
  //update_joint_angle[?] = acos((link14_15_normed).dot(link14_16_normed)); //hand angle (can only control open/closed)
  std::cout << update_joint_angle[3]*180/M_PI << std::endl;
}

// Update New Joint States
bool setJointSpacePath(double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = update_joint_angle;
  srv.request.path_time = path_time;

  if(goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

// Update Manipulator States
void manipulatorStatesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr &msg)
{
  if(msg->open_manipulator_moving_state == msg->IS_MOVING)
    open_manipulator_is_moving_ = true;
  else
    open_manipulator_is_moving_ = false;

  if(msg->open_manipulator_actuator_state == msg->ACTUATOR_ENABLED)
    open_manipulator_actuator_enabled_ = true;
  else
    open_manipulator_actuator_enabled_ = false;
}

// Update Joint States
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT_AND_TOOL);
  for(int i = 0; i < msg->name.size(); i ++)
  {
    if(!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("gripper"))  temp_angle.at(4) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;
}

// Update Current Pose
void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  
  present_kinematic_position_ = temp_position;

  kinematics_pose_.pose = msg->pose;
}

// Set Destination for End Effector
bool setTaskSpacePath(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;

  srv.request.end_effector_name = "gripper";

  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

  srv.request.kinematics_pose.pose.orientation.w = kinematics_pose_.pose.orientation.w;
  srv.request.kinematics_pose.pose.orientation.x = kinematics_pose_.pose.orientation.x;
  srv.request.kinematics_pose.pose.orientation.y = kinematics_pose_.pose.orientation.y;
  srv.request.kinematics_pose.pose.orientation.z = kinematics_pose_.pose.orientation.z;

  srv.request.path_time = path_time;

  if(goal_task_space_path_position_only_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}
