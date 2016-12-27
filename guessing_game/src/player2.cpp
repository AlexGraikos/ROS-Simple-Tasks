#include <ros/ros.h>
#include <std_msgs/String.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <guessing_game/GuessAction.h>

#include <sstream>
#include <cstdlib>

int main(int argc, char **argv) {

	if (argc != 2) {
		ROS_INFO("Usage: player2 A");
		return 1;
	}

	// Initialize node, publisher and action_client
	ros::init(argc, argv, "player2");
	ros::NodeHandle node;
	ros::Publisher score_publisher = node.advertise<std_msgs::String>("score", 1000); 
	actionlib::SimpleActionClient<guessing_game::GuessAction> action_client("guessing_game", true);


	// Wait if server is not online
	ROS_INFO("Waiting for server");
	action_client.waitForServer();
	ROS_INFO("Sending goal");

	// Set goal to command line argument
	guessing_game::GuessGoal goal;
	goal.num_guess = atoi(argv[1]);

	// Send goal 
	action_client.sendGoal(goal);

	
	bool won = false;
	std_msgs::String msg;
	std::stringstream stream;
	msg.data = stream.str();

	// Wait until reply is finished or timed out
	bool finished = action_client.waitForResult(ros::Duration(10.0));

	if (finished) {
		guessing_game::GuessResultConstPtr result = action_client.getResult();	
		ROS_INFO("Received %d", result->isRight);

		// Result and set won state if we gave correct answer
		switch (result->isRight) {
			case 0:
				ROS_INFO("Guessed the correct number!");
				won = true;
				break;
			case 1:
				ROS_INFO("Guessed a higher number");
				break;
			case -1:
				ROS_INFO("Guessed a lower number");
				break;
			default:
				ROS_INFO("Result received is not correct");
		}

		// If no more tries left or we won publish the correct
		// message to the /score topic
		if (result->triesLeft == 0 || won) {
			if (won) {
				stream << "Player2 has won with score: " << result->triesLeft;
			} else { 
				ROS_INFO("No more tries left. Game Over.");
				stream << "Player2 has lost";
			}
			msg.data = stream.str();
			score_publisher.publish(msg);
			return 0;

		// Else print tries left
		} else if (result->triesLeft > 0) {
			ROS_INFO("Tries left %d", result->triesLeft);
		}

	// If server timed out
	} else {
		ROS_INFO("Action did not complete before timeout");
	}

	return 0;

}