#include <ros/ros.h>
#include <task3/Vision.h>

#include <cstdlib>

int main(int argc, char **argv) {

	if (argc < 4) {
		ROS_INFO("Usage vision x y z id");
		return 1;
	}

	// Initialize node
	ros::init(argc, argv, "vision");
	ros::NodeHandle node;

	// Advertise on vision topic
	ros::Publisher vision_pub = node.advertise<task3::Vision> ("vision", 1000);

	// Send the message containing coordinates from commandline and id
	task3::Vision msg;
	msg.coordinates[0] = atof(argv[1]);
	msg.coordinates[1] = atof(argv[2]);
	msg.coordinates[2] = atof(argv[3]);
	msg.id = atoi(argv[4]);

	ros::Rate publish_rate(1);

	// Publish every second
	while (ros::ok()) {

		publish_rate.sleep();
		vision_pub.publish(msg);
		ros::spinOnce();
	}

	return 0;
}