#include "ros/ros.h"
#include "chatroom/ChatMessage.h"

void chatCallback(const chatroom::ChatMessage::ConstPtr& msg)
{
  ROS_INFO("[%s]: %s", msg->username.c_str(), msg->message.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "chat_subscriber");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chat_topic", 1000, chatCallback);
  ros::spin();
  return 0;
}
