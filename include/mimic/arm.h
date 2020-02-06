#define LEFT 1
#define RIGHT 2

class Arm {
public:
	Arm(int type);
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

	void calculateJointAngle(const visualization_msgs::MarkerArray::ConstPtr &msg, std::vector<double> &update_joint_angle);
}

tf::Vector3 marker2Vector3(const visualization_msgs::MarkerArray::ConstPtr msg, int idx);