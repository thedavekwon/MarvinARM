#define LEFT 1
#define RIGHT 2
#define NUM_OF_JOINT_AND_TOOL 5

#include <cmath>
#include <sstream>
#include <string>

#include "open_manipulator_msgs/OpenManipulatorState.h"
#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
#include "open_manipulator_msgs/SetDrawingTrajectory.h"
#include "open_manipulator_msgs/SetActuatorState.h"

#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class Arm {
public:
	Arm(int type);

	void calculateJointAngle(std::vector<double> &update_joint_angle);
private:
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