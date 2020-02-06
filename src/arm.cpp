#include "arm.h"

Arm::Arm(int type) {
	if (type == LEFT) {
		idx_neck = 3;
		idx_spinechest = 2;
		idx_clavicle = 4;
		idx_shoulder = 5;
		idx_elbow = 6;
		idx_wrist = 7;
		idx_hand = 8;
		idx_handtip = 9;
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

void Arm::calculateJointAngle(const visualization_msgs::MarkerArray::ConstPtr &msg, std::vector<double> &update_joint_angle) {
	auto tmp_marker_array = *msg;

	auto neck = marker2Vector3(tmp_marker_array, idx_neck);
	auto spinechest = marker2Vector3(tmp_marker_array, idx_spinechest);
	auto clavicle = marker2Vector3(tmp_marker_array, idx_clavicle);
	auto shoulder = marker2Vector3(tmp_marker_array, idx_shoulder);
	auto elbow = marker2Vector3(tmp_marker_array, idx_elbow);
	auto wrist = marker2Vector3(tmp_marker_array, idx_wrist);
	auto hand = marker2Vector3(tmp_marker_array, idx_hand);
	auto handtip = marker2Vector3(tmp_marker_array, idx_handtip);
	auto thumb = marker2Vector3(tmp_marker_array, idx_thumb);

	auto joint1_1 = (neck-spinechest).normalize();
	auto joint1_2 = (neck-elbow).normalize();

	auto joint2_1 = (clavicle-spinechest).normalize();
	auto joint2_2 = (clavicle-shoulder).normalize();

	auto joint3_1 = (elbow-shoulder).normalize();
	auto joint3_2 = (elbow-wrist).normalize();

	auto joint4_1 = (wrist-elbow).normalize();
	auto joint4_2 = (wrist-hand).normalize();

	auto gripper_1 = (hand-handtip).normalize();
	auto gripper_2 = (hand-thumb).normalize();

	update_joint_angle[0] = acos((joint1_1).dot(joint1_2));
	update_joint_angle[1] = acos((joint2_1).dot(joint2_2));
	update_joint_angle[2] = acos((joint3_1).dot(joint3_2))+M_PI/2;
	update_joint_angle[3] = acos((joint4_1).dot(joint4_2));
}

tf::Vector3 marker2Vector3(const visualization_msgs::MarkerArray::ConstPtr &msg, int idx) {
	return 	tf::Vector3(tmp_marker_array.markers[idx].pose.position.x, tmp_marker_array.markers[idx].pose.position.y, tmp_marker_array.markers[idx].pose.position.y);
}


