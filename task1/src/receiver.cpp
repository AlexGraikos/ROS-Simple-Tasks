#include "ros/ros.h"
#include "std_msgs/String.h"

#include "task1/calculator_message.h"

/* Usage receiver
 * Received a calculator_message from the calculator
 * topic and prints on screen data + result of a (op) b
 */

void receiver_callback(const task1::calculator_message &msg) {
  
  if (msg.op.length() > 1) {
    ROS_INFO("Operand received not in (+,-,*,/)");
    return;
  }
  

  // Conver basic_string to char
  char op = msg.op.data()[0];
  double result;

  ROS_INFO("Received %ld %ld %s", msg.a, msg.b, msg.op.data());
  
  // Find out operand and execute operation
  switch(op) {
    case '+':
      result = msg.a + msg.b;
      break;
    case '-':
      result = msg.a - msg.b;
      break;
    case '*':
      result = msg.a * msg.b;
      break;
    case '/':
      result =(double) msg.a / (double) msg.b;
      break;
    default:
      ROS_INFO("Operator received not included in (+,-,*,/)");
      return;
  }
  ROS_INFO("Result %.3lf", result);

}

int main(int argc, char **argv) {
  
  // Initialize node
  ros::init(argc, argv, "receiver");
  ros::NodeHandle node;
  
  // Declare a subscriber that receives calculator_message type messages
  ros::Subscriber recv_sub = node.subscribe("calculator", 1000, receiver_callback);

  ros::spin();
  
  return 0;

}
