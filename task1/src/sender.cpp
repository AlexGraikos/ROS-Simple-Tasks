#include "ros/ros.h"
#include "task1/calculator_message.h"

#include <cstdlib>
#include <sstream>


/* Usage: sender A B Op
 * Publishes a calculator_message to topic
 * calculator, as indicated by the arguments
 */

int main(int argc, char **argv) {
  
  if (argc != 4) {
    ROS_INFO("Usage: sender A B Op");
    return 1;
  }

  // Initialize node
  ros::init(argc, argv, "sender");
  ros::NodeHandle node;
  
  // Declare publisher - calculator_message type
  ros::Publisher sender_pub = node.advertise<task1::calculator_message>("calculator", 1000);
  
  // Declare looper 1Hz freq.
  ros::Rate loop(1);

  // Publish every sec
  while(ros::ok()) {
    
    task1::calculator_message msg;
    
    msg.a = atoi(argv[1]);
    msg.b = atoi(argv[2]);
    msg.op = argv[3];
    
    sender_pub.publish(msg);

    ros::spinOnce();

    loop.sleep();
  }

  return 0;
}
