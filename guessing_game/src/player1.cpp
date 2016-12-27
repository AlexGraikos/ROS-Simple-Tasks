#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <guessing_game/GuessAction.h>

/*
 *  If Number is right    result = 0
 *  If Number is greater  result = 1
 *  If number is less     result = -1
 */

class GuessMyNumber {
protected:


	ros::NodeHandle node;
	actionlib::SimpleActionServer<guessing_game::GuessAction> actionServer;
	std::string action_name;

	guessing_game::GuessFeedback feedback;
	guessing_game::GuessResult result;

	int numberToGuess;
	unsigned int tries;

public:

	// Class constructor
	GuessMyNumber(std::string name) :
		actionServer(node, name, boost::bind(&GuessMyNumber::callback, this ,_1), false),
		action_name(name)
	{
		actionServer.start();
		ROS_INFO("Server started");
		setRandomNumber();
	
		tries = 7;
	}

	~GuessMyNumber(void) {
	}

	// Sets a random number to numberToGuess
	void setRandomNumber() {
		srand(ros::Time::now().toSec());
		numberToGuess = rand() % 100 + 1;
		ROS_INFO("Number to guess %d", numberToGuess);
	}


	void callback(const guessing_game::GuessGoalConstPtr &goal) {

		ROS_INFO("Executing callback");

		// If number given is the correct number
		if (goal->num_guess == numberToGuess) {
			ROS_INFO("Guessed the right number %d", goal->num_guess);
			result.isRight = 0;
			setRandomNumber();
			result.triesLeft = tries;
			actionServer.setSucceeded(result);
			tries = 7;
			return;

		// If number given is > 
		} else if (goal->num_guess > numberToGuess) {
			ROS_INFO("Guessed a bigger number");
			result.isRight = 1;

		// If number given is <
		} else {
			ROS_INFO("Guessed a smaller number");
			result.isRight = -1;
		}

		// Reduce tries 
		tries--;
		result.triesLeft = tries;
		
		// If no more tries left end the game
		if (tries == 0) {
			ROS_INFO("No more tries! Game over");
			setRandomNumber();
			tries = 7;
		}

		actionServer.setSucceeded(result);
	}
};

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "guessing_game");

	GuessMyNumber game("guessing_game");

	ros::spin();

	return 0;
}