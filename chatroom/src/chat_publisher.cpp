#include "ros/ros.h"
#include "chatroom/ChatMessage.h"
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "chat_publisher");
  ros::NodeHandle n;
  ros::Publisher chat_pub = n.advertise<chatroom::ChatMessage>("chat_topic", 1000);

  std::string username;
  std::cout << "Enter your username: ";
  std::cin >> username;

  chatroom::ChatMessage msg;
  std::string text;
  while (ros::ok())
  {
    std::cout << username << ": ";
    std::getline(std::cin, text);
    msg.username = username;
    msg.message = text;
    chat_pub.publish(msg);
  }

  return 0;
}
