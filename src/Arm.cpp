#include "Arm.h"

const std::vector<std::string> Arm::joint_name{"joint1", "joint2", "joint3", "joint4"};

Arm::Arm(int _type) {
  type = _type;
  idx_neck = 3;
  idx_spinechest = 2;
	if (type == LEFT) {
		idx_clavicle = 4;
    idx_shoulder = 5;
		idx_elbow = 6;
		idx_wrist = 7;
		idx_hand = 8;
		idx_handtip = 9;
		idx_thumb = 10;
	} else if (type == RIGHT) {
		idx_clavicle = 11;
		idx_shoulder = 12;
		idx_elbow = 13;
		idx_wrist = 14;
		idx_hand = 15;
		idx_handtip = 16;
		idx_thumb = 17;
	}
  update_joint_angle.resize(4, 0);
  present_joint_angle_.resize(5, 0);

  ros::NodeHandle n;

  ros::Subscriber open_manipulator_states_sub_          = n.subscribe("states", 10, &Arm::manipulatorStatesCallback, this);
  ros::Subscriber open_manipulator_joint_states_sub_    = n.subscribe("joint_states", 10, &Arm::jointStatesCallback, this);
  ros::Subscriber open_manipulator_kinematics_pose_sub_ = n.subscribe("kinematics_pose", 10, &Arm::kinematicsPoseCallback, this);

  goal_task_space_path_position_only_client_ = n.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_position_only");
  goal_joint_space_path_client_ = n.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
}


void Arm::calculateJointAngle(const visualization_msgs::MarkerArray markerarray) {
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

	update_joint_angle[0] = acos((link12).dot(ortho_vect));
	update_joint_angle[1] = -acos((plane1).dot(parall_vect)) + M_PI - M_PI/18;
	update_joint_angle[2] = -acos((-1*link12).dot(link23)) + M_PI/2 + M_PI/10;
	// update_joint_angle[3] = atan2((-1*link23).cross(link34).norm(), (-1*link23).dot(link34));
   update_joint_angle[3] = 0;

  //std::cout << update_joint_angle[0]*180/M_PI << " , " << update_joint_angle[1]*180/M_PI << std::endl;
  //std::cout << "---" << std::endl;
  if (type == LEFT) std::cout << "LEFT" << std::endl;
  if (type == RIGHT) std::cout << "RIGHT" << std::endl;
  std::cout << update_joint_angle[0]*180/M_PI << ", " << update_joint_angle[1]*180/M_PI << ", " << update_joint_angle[2]*180/M_PI << ", " << update_joint_angle[3]*180/M_PI << std::endl;
}


Eigen::Vector3f marker2Vector3(const visualization_msgs::MarkerArray& markerarray, int idx) {
	return Eigen::Vector3f(markerarray.markers[idx].pose.position.x, markerarray.markers[idx].pose.position.y, markerarray.markers[idx].pose.position.z);
}


// Update New Joint States
bool Arm::setJointSpacePath(double path_time)
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
void Arm::manipulatorStatesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr &msg)
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
void Arm::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle(NUM_OF_JOINT_AND_TOOL, 0);
  for(int i = 0; i < msg->name.size(); i ++)
  {
    if(!msg->name.at(i).compare("joint1")) temp_angle.at(0) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("gripper"))  temp_angle.at(4) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;
}

// Update Current Pose
void Arm::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);

  present_kinematic_position_ = temp_position;

  kinematics_pose_.pose = msg->pose;
}

// Set Destination for End Effector
bool Arm::setTaskSpacePath(ros::ServiceClient &service, std::vector<double> kinematics_pose, double path_time)
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
