#include <ros/ros.h>
#include <task3/Grasp.h>

void graspCB(const task3::Grasp::ConstPtr& msg) {
	ROS_INFO("ID of item to be grasped: %d", msg->id);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "grasp");
	ros::NodeHandle node;

	// Set ip a subscriber to read id of item to be grasped
	ros::Subscriber grasp_sub = node.subscribe ("grasp", 1000, graspCB);

	ros::spin();

	return 0;

}