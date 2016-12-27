# ROS-Simple-Tasks
Three simple tasks in ROS

They were developed on ROS Kinetic, and can be built with catkin.

## Task1: 
Implements a simple calculator using topics.To use, start up receiver node and then sender with arguments A B Operator.Sender will probe every 1s the operation to be executed and will be canceled with a termination signal.

## Task2:
Implements a simple number guessing game.Player1 "thinks" of a number and player2 has 7 tries to guess it.Start up player1 node and then player2 with a number to guess as an argument.When the game is over (player2 wins or is out of tries), player2 will publish a message to the /score topic with his score and result of the game.

## Task3:
Implements a simple robot with 5 nodes.

The master node reads a message from the vision node on topic /vision and starts up a motion action to move the robot to the desired location.When completed, master nodes publishes a message containing the item id on the /grasp topic.

The vision node sends a message containing the location and id of the item to be grasped on the /vision topic.

The motion node starts at location (0,0,0) and acts as an action server.When it is given a new location to move to, the node calculates a simple linear trajectory to the final position.While integrating the position, it publishes every 50ms the current state as feedback on the action client connected to it.When finished it replies with a result of the final position it has stopped at.

The grasp node will notify on the /grasp topic that it has identified the object and its id.

The front end node can terminate all other nodes and reset the robot's position back to (0,0,0).

To use the above a launch file is included.The launch file requires arguments x,y,z,id that are passed to the vision node.The front end node is not included in the launch file (although it considers the group name of the launch file and can stop execution when started outside of it).
