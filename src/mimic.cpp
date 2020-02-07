#include "mimic.h"

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
    if(!msg->name.at(i).compare("joint1")) temp_angle.at(0) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joivisualization_msgs::MarkerArraynt4"))  temp_angle.at(3) = (msg->position.at(i));
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

Arm::Arm(int type) {
	if (type == LEFT) {
		idx_neck = 3;
		idx_spinechest = 2;
		idx_clavicle = 4;
		idx_thumb = 10;
	} else if (type == RIGHT) {
		idx_neck = 3;
		idx_spinechest = 2;
		idx_clavicle = 11;
		idx_shoulder = 12;
		idx_elbow = 13;
		idx_wrist = 14;
		idx_hand = 15;
		idx_handtip = 16;
		idx_thumb = 17;
	}
}

void Arm::calculateJointAngle(std::vector<double> &update_angle) {
  visualization_msgs::MarkerArray markerarray = currentMarkerArray;

	Eigen::Vector3f neck = marker2Vector3(markerarray, idx_neck); //3
	Eigen::Vector3f spinechest = marker2Vector3(markerarray, idx_spinechest); //2
	Eigen::Vector3f clavicle = marker2Vector3(markerarray, idx_clavicle); //11
	Eigen::Vector3f shoulder = marker2Vector3(markerarray, idx_shoulder); //12
	Eigen::Vector3f elbow = marker2Vector3(markerarray, idx_elbow); //13
	Eigen::Vector3f wrist = marker2Vector3(markerarray, idx_wrist); //14
	Eigen::Vector3f hand = marker2Vector3(markerarray, idx_hand); //15
	Eigen::Vector3f handtip = marker2Vector3(markerarray, idx_handtip);
  Eigen::Vector3f thumb = marker2Vector3(markerarray, idx_thumb);

	Eigen::Vector3f link12 = (elbow - shoulder).normalized();
	Eigen::Vector3f link23 = (wrist - elbow).normalized();
	Eigen::Vector3f link34 = (handtip - wrist).normalized();
	Eigen::Vector3f link35 = (thumb - wrist).normalized();

	Eigen::Vector3f plane1 = (neck - shoulder).normalized();
	Eigen::Vector3f plane2 = (spinechest - shoulder).normalized();
	Eigen::Vector3f ortho_space = (plane1.cross(plane2)).normalized();
	Eigen::Vector3f ortho_vect = (ortho_space * ortho_space.transpose()) * link12;
	Eigen::Vector3f parall_vect = link12 - ortho_vect;



	update_joint_angle[0] = acos((plane1).dot(ortho_vect));
	update_joint_angle[1] = acos((plane1).dot(parall_vect))-M_PI/2;
	update_joint_angle[2] = -M_PI/2;//acos((-1*link12).dot(link23));
	update_joint_angle[3] = 0;//acos((-1*link23).dot(link34));

  std::cout << update_joint_angle[0]*180/M_PI << ", " << update_joint_angle[1]*180/M_PI << ", " << update_joint_angle[2]*180/M_PI << ", " << update_joint_angle[3]*180/M_PI << std::endl;
}

Eigen::Vector3f marker2Vector3(const visualization_msgs::MarkerArray& markerarray, int idx) {
	return Eigen::Vector3f(markerarray.markers[idx].pose.position.x, markerarray.markers[idx].pose.position.y, markerarray.markers[idx].pose.position.y);
}

int main(int argc, char ** argv) {
  joint_name.push_back("joint1");
  joint_name.push_back("joint2");
  joint_name.push_back("joint3");
  joint_name.push_back("joint4");
  
  ros::init(argc, argv, "mimic");

  ros::NodeHandle n;

  ros::Subscriber open_manipulator_states_sub_          = n.subscribe("states", 10, &manipulatorStatesCallback);
  ros::Subscriber open_manipulator_joint_states_sub_    = n.subscribe("joint_states", 10, &jointStatesCallback);
  ros::Subscriber open_manipulator_kinematics_pose_sub_ = n.subscribe("kinematics_pose", 10, &kinematicsPoseCallback);

  // ros::Subscriber joint1_sub = n.subscribe("/vicon/joint_0/joint_0", 10, &joint1CallBack);
  // ros::Subscriber joint2_sub = n.subscribe("/vicon/joint_1/joint_1", 10, &joint2CallBack);
  // ros::Subscriber joint3_sub = n.subscribe("/vicon/joint_2/joint_2", 10, &joint3CallBack);

  ros::Subscriber markerarrays_sub = n.subscribe("/body_tracking_data", 10, &getMarkerArraysFromKinect);

  goal_task_space_path_position_only_client_ = n.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_position_only");
  goal_joint_space_path_client_ = n.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");

  Arm marvin_right = Arm(RIGHT);

  ros::Rate loop_rate(10);
  while ( ros::ok() ) {
    // if (joint1_fetched && joint2_fetched && joint3_fetched) {
    if (markers_fetched) {
      marvin_right.calculateJointAngle(update_joint_angle);
      setJointSpacePath(1);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}