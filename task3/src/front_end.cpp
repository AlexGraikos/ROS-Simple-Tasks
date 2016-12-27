#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "front_end");
	ros::NodeHandle node;

	// When called publish END message to front_end topic
	ros::Publisher front_end_pub = node.advertise<std_msgs::String> ("robot/front_end", 1000);

	std_msgs::String msg;
	msg.data = std::string("END");

	ros::Rate delay(1);

	while(ros::ok()) {

		front_end_pub.publish(msg);
		ros::spinOnce();
		delay.sleep();
	}

	return 0;

}