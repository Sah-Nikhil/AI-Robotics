#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

ros::Subscriber gpsFixSub,imuSub;
double targetLatitude, targetLongitude;
class Rover
{
public:
    Rover() : nh_("~"), target_x_(0.0), target_y_(0.0), current_x_(0.0), current_y_(0.0), current_yaw_(0.0),
                     Kp_linear_(1.0), Ki_linear_(0.0), Kd_linear_(0.0),
                     Kp_angular_(1.0), Ki_angular_(0.0), Kd_angular_(0.0),
                     prev_linear_error_(0.0), prev_angular_error_(0.0),
                     linear_integral_(0.0), angular_integral_(0.0)
    {
        // Parameters
        nh_.param("target_x", target_x_, targetLatitude);
        nh_.param("target_y", target_y_, targetLongitude);

        // Subscribers
        gpsFixSub = nh_.subscribe<sensor_msgs::NavSatFix>(
            "/fix", 20, [this](const sensor_msgs::NavSatFix::ConstPtr& msg) {
                gpsFixCallback(msg);
            });
        imuSub = nh_.subscribe<sensor_msgs::Imu>(
        "/imu", 20, [this](const sensor_msgs::Imu::ConstPtr& msg) {
            imuCallback(msg);
        });
        // Publishers
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Timer for control loop
        control_timer_ = nh_.createTimer(ros::Duration(0.1), &Rover::controlLoop, this);
    }

private:
    void gpsFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        // Update rover's current location based on received GPS fix
        current_x_ = msg->latitude;
        current_y_ = msg->longitude;


    }
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // Convert orientation quaternion to Euler angles
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(msg->orientation, orientation);
        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Update rover's current yaw value based on received IMU data
        //current_yaw_ = toDegrees(yaw);
        current_yaw_ = yaw;
        // std::cout << "Yaw (degrees): " << currentYaw << std::endl;
    }

    void controlLoop(const ros::TimerEvent &)
    {
        // Calculate the distance and angle to the target
        double dx = target_x_ - current_x_;
        double dy = target_y_ - current_y_;
        double distance_to_goal = std::sqrt(dx * dx + dy * dy);
        double target_yaw = std::atan2(dy, dx);
        double yaw_error = target_yaw - current_yaw_;

        // Normalize yaw error to [-pi, pi]
        if (yaw_error > M_PI)
            yaw_error -= 2 * M_PI;
        else if (yaw_error < -M_PI)
            yaw_error += 2 * M_PI;

        // PID control for linear velocity
        double linear_error = distance_to_goal;
        linear_integral_ += linear_error * 0.1; // dt = 0.1
        double linear_derivative = (linear_error - prev_linear_error_) / 0.1;
        double linear_output = Kp_linear_ * linear_error + Ki_linear_ * linear_integral_ + Kd_linear_ * linear_derivative;
        prev_linear_error_ = linear_error;

        // PID control for angular velocity
        angular_integral_ += yaw_error * 0.1; // dt = 0.1
        double angular_derivative = (yaw_error - prev_angular_error_) / 0.1;
        double angular_output = Kp_angular_ * yaw_error + Ki_angular_ * angular_integral_ + Kd_angular_ * angular_derivative;
        prev_angular_error_ = yaw_error;

        // Create and publish the velocity command
        geometry_msgs::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = linear_output;
        cmd_vel_msg.angular.z = angular_output;
        cmd_vel_pub_.publish(cmd_vel_msg);
        std::cout<<"DIstance to GOal:"<<distance_to_goal<<std::endl;
        // Stop the robot if it is very close to the target
        if (distance_to_goal < 0.1)
        {
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.angular.z = 0.0;
            cmd_vel_pub_.publish(cmd_vel_msg);
            ROS_INFO("Reached the goal!");
            //ros::shutdown();
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Timer control_timer_;

    double target_x_;
    double target_y_;
    double current_x_;
    double current_y_;
    double current_yaw_;

    double Kp_linear_;
    double Ki_linear_;
    double Kd_linear_;
    double Kp_angular_;
    double Ki_angular_;
    double Kd_angular_;

    double prev_linear_error_;
    double prev_angular_error_;
    double linear_integral_;
    double angular_integral_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlebot_pid");
    std::cout << "Enter target latitude: ";
    std::cin >> targetLatitude;
    std::cout << "Enter target longitude: ";
    std::cin >> targetLongitude;
    Rover turtlebot_pid;

    ros::spin();
    return 0;
}

