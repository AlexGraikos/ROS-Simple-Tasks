#include <ros/ros.h>
#include <std_msgs/String.h>
#include <actionlib/server/simple_action_server.h>
#include <task3/MotionAction.h>
#include <cmath>

class MotionNode {
protected:

	// Current position 
	double position[3];

	ros::NodeHandle node;
	actionlib::SimpleActionServer<task3::MotionAction> action_server;
	ros::Subscriber motion_front_end_sub;
	std::string action_name;

	task3::MotionFeedback feedback;
	task3::MotionResult result;

public:

	MotionNode(std::string name) :
	action_server(node, name, boost::bind(&MotionNode::action_callback, this, _1), false),
	action_name(name) {
		// Start action server set position to 0 and subscribe to Front_End node

		action_server.start();
		ROS_INFO("Server started");

		position[0] = position[1] = position[2] = 0.0f;

		motion_front_end_sub = node.subscribe("front_end", 1000, &MotionNode::front_endCB, this);
	}

	~MotionNode() {}

	// If Front_End node requests shutdown set position back to 0
	void front_endCB (const std_msgs::String::ConstPtr &msg) {
		if (msg->data == std::string("END")) {
			position[0] = position[1] = position[2] = 0.0f;
		}
	}

	void action_callback(const task3::MotionGoalConstPtr& goal) {
		ROS_INFO("Received goal");

		// Time to complete trajectory
		double totalTime = 5.0f;

		// Time intervals to calculate dt and printing time
		double time = 0;
		ros::Time last_time = ros::Time::now();
		ros::Time feedback_time = ros::Time::now();

		// Velocities generated from linear trajectory
		double vx = (goal->targetPos[0] - position[0]) / totalTime;
		double vy = (goal->targetPos[1] - position[1]) / totalTime;
		double vz = (goal->targetPos[2] - position[2]) / totalTime;
		double dx, dy, dz, dt;

		bool completed = true;

		// While trajectory is not completed
		while(time < totalTime) {
			
			// If request to action server is canceled or program is stopped
			if(action_server.isPreemptRequested() || !ros::ok()) {
				ROS_INFO("Action preempted");
				action_server.setPreempted();
				completed = false;
				break;
			}

			dt = ros::Time::now().toSec() - last_time.toSec(); 
			last_time = ros::Time::now();

    		// Euler integration
			dx = vx * dt;
			dy = vy * dt;
			dz = vz * dt;
			
			time += dt;

			position[0] += dx;
			position[1] += dy;
			position[2] += dz;

			// Set feedback to current position and publish 
			feedback.currPos[0] = position[0];
			feedback.currPos[1] = position[1];
			feedback.currPos[2] = position[2];
		
			// Delay publishing

			if (ros::Time::now().toSec() - feedback_time.toSec() >= 0.05) {
				action_server.publishFeedback(feedback);
				feedback_time = ros::Time::now();
			}
		}

		// When trajectory is over publish result
		if (completed) {
			result.finalPos[0] = position[0];
			result.finalPos[1] = position[1];
			result.finalPos[2] = position[2];
			action_server.setSucceeded(result);
		}
	}
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "motion");

	MotionNode node("motion");

	ros::spin();

	return 0;
}