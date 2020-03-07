#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <std_msgs/String.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include "open_manipulator_msgs/OpenManipulatorState.h"
#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
#include "open_manipulator_msgs/SetDrawingTrajectory.h"
#include "open_manipulator_msgs/SetActuatorState.h"

#define LEFT 1
#define RIGHT 2
#define NUM_OF_JOINT_AND_TOOL 5

class Arm {
public:
	Arm(int _type);
	void calculateJointAngle(const visualization_msgs::MarkerArray markerarray);
  bool setJointSpacePath(double path_time);

private:
  static const std::vector<std::string> joint_name;

  ros::ServiceClient goal_task_space_path_position_only_client_;
  ros::ServiceClient goal_joint_space_path_client_ ;

  open_manipulator_msgs::KinematicsPose kinematics_pose_;

  std::vector<double> present_joint_angle_;
  std::vector<double> present_kinematic_position_;
  std::vector<double> update_joint_angle;

  bool open_manipulator_is_moving_ = false;
  bool open_manipulator_actuator_enabled_ = false;

  void manipulatorStatesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr &msg);
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);

  bool setTaskSpacePath(ros::ServiceClient &service, std::vector<double> kinematics_pose, double path_time);

	int type;
	int idx_neck;
	int idx_spinechest;
	int idx_clavicle;
	int idx_shoulder;
	int idx_elbow;
	int idx_wrist;
	int idx_hand;
	int idx_handtip;
	int idx_thumb;
};

Eigen::Vector3f marker2Vector3(const visualization_msgs::MarkerArray& markerarray, int idx);

tf::Vector3 marker2Vector3t(const visualization_msgs::MarkerArray& markerarray, int idx);
