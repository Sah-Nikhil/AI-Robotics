#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>
#include <ctime>

class ChatNode {
public:
    ChatNode() : username("anonymous") {
        // Initialize ROS node handle
        nh = ros::NodeHandle();

        // Subscribe to the chat topic
        chat_subscriber = nh.subscribe("chat", 1000, &ChatNode::chatCallback, this);

        // Publisher for sending messages
        chat_publisher = nh.advertise<std_msgs::String>("chat", 1000);

        // Start the listening thread
        boost::thread listening_thread(&ChatNode::listenForMessages, this);
    }

    void listenForMessages() {
        ros::spin();
    }

    void chatCallback(const std_msgs::String::ConstPtr& msg) {
        std::time_t now = std::time(nullptr);
        std::string message = msg->data;
        std::string sender = message.substr(0, message.find(": "));
        std::string text = message.substr(message.find(": ") + 2);
        ROS_INFO("Received message from %s: %s", sender.c_str(), text.c_str());
        std::cout << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S") << " [" << sender << "] " << text << std::endl;
    }

    void setUsername(const std::string& username) {
        this->username = username;
    }

    void sendMessage(const std::string& message) {
        std_msgs::String msg;
        msg.data = username + ": " + message;
        ROS_INFO("Sending message: %s", msg.data.c_str());
        chat_publisher.publish(msg);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher chat_publisher;
    ros::Subscriber chat_subscriber;
    std::string username;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "chat_node");

    ChatNode chatNode;

    std::string input_username;
    std::cout << "Enter your username: ";
    std::getline(std::cin, input_username);
    chatNode.setUsername(input_username);

    std::string input_message;
    while (ros::ok()) {
        std::getline(std::cin, input_message);
        chatNode.sendMessage(input_message);
    }

    return 0;
}
