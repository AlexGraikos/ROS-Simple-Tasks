#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/String.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <task3/MotionAction.h>
#include <task3/Vision.h>
#include <task3/Grasp.h>

class MasterNode {
protected:

	ros::NodeHandle node;
	std::string action_name;
	actionlib::SimpleActionClient<task3::MotionAction> action_client;
	ros::Subscriber master_sub;
	ros::Publisher master_pub;
	ros::Subscriber master_front_end_sub;
	rosbag::Bag bag;
	
	double position[3];
	unsigned int id;

	bool running;

public:

	MasterNode(std::string name) :
	action_client("motion", true),
	action_name(name) {

		// Action server connection
		ROS_INFO("Waiting for action server to start");
		action_client.waitForServer();
		ROS_INFO("Server started");

		// Vision topic subscription
		ROS_INFO("Subscribing to vision topic");
		master_sub = node.subscribe("vision", 1000, &MasterNode::visionCB, this);

		// Grasp topic publishing
		ROS_INFO("Publishing to grasp topic");
		master_pub = node.advertise<task3::Grasp> ("grasp", 1000);

		// Front_End node subscription
		master_front_end_sub = node.subscribe("front_end", 1000, &MasterNode::front_endCB, this);

		// Set running state to false and setup bag file for writing
		running = false;
		bag.open("position.bag", rosbag::bagmode::Write);

	}

	~MasterNode() {
		bag.close();
	}

	// If user requests shutdown of the system set running state to false
	// and cancel currently running actions
	void front_endCB(const std_msgs::String::ConstPtr &msg) {
		if (msg->data == std::string("END")) {
			running = false;
			action_client.cancelAllGoals();
		}
	}

	// When vision data is sent send position data to motion action_server
	void visionCB (const task3::Vision::ConstPtr &msg) {
		ROS_INFO("Received message from vision topic");
		position[0] = msg->coordinates[0];
		position[1] = msg->coordinates[1];
		position[2] = msg->coordinates[2];
		id = msg->id;

		if (!running) {
			sendGoal(position);
		}
	}

	void sendGoal(double position[3]) {
		// Set running state to true and set up starting position
		running = true;
		task3::MotionGoal goal;
		goal.targetPos[0] = position[0];
		goal.targetPos[1] = position[1];
		goal.targetPos[2] = position[2];

		// Send goal and bind callbacks
		action_client.sendGoal(goal, boost::bind(&MasterNode::completedCB, this, _1, _2) , 
			actionlib::SimpleActionClient<task3::MotionAction>::SimpleActiveCallback() ,
			boost::bind(&MasterNode::feedbackCB, this, _1) );

	}

	// When motion is complete publish id to grasp node for database search
	// and set running state to false
	void completedCB(const actionlib::SimpleClientGoalState& state,
	 const task3::MotionResultConstPtr& result) {
		ROS_INFO("Finished in state %s", state.toString().c_str());

		ROS_INFO("Final position [%lf, %lf, %lf]", result->finalPos[0], result->finalPos[1],
			result->finalPos[2]);

		// Publish message ID
		task3::Grasp msg;
		msg.id = id;
		master_pub.publish(msg);
		running = false;
	}

	// Write position feedback data to bag file
	void feedbackCB(const task3::MotionFeedbackConstPtr& feedback) {
		ROS_INFO("Current position [%lf, %lf, %lf]", feedback->currPos[0], feedback->currPos[1],
			feedback->currPos[2]);
		
		bag.write("/motion/feedback", ros::Time::now(), feedback);

	}


};


int main(int argc, char **argv) {
	ros::init(argc, argv, "master");
	MasterNode node("master");
	ros::spin();

	return 0;
}