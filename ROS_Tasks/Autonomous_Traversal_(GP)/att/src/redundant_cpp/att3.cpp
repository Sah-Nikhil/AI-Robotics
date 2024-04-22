#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_listener.h"  // Include the tf header
#include <limits>  // Include for std::numeric_limits

// Constants for Earth's radius in meters
constexpr double earth_rad = 6371000.0;

// Convert degrees to radians
double toRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

// Convert radians to degrees
double toDegrees(double radians) {
    return radians * 180.0 / M_PI;
}

// Calculate distance between two GPS coordinates using Haversine formula
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    double dLat = toRadians(lat2 - lat1);
    double dLon = toRadians(lon2 - lon1);
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(toRadians(lat1)) * cos(toRadians(lat2)) *
               sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return earth_rad * c;
}

// Calculate initial bearing from point A to point B using Haversine formula
double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    double dLon = toRadians(lon2 - lon1);
    double y = sin(dLon) * cos(toRadians(lat2));
    double x = cos(toRadians(lat1)) * sin(toRadians(lat2)) -
               sin(toRadians(lat1)) * cos(toRadians(lat2)) * cos(dLon);
    double bearing = atan2(y, x);
    return toDegrees(bearing);
}

// Class representing the rover
class Rover {
public:
    double currentLatitude; // Current latitude
    double currentLongitude; // Current longitude
    double currentYaw; // Current yaw from IMU
    bool isReached; // Flag to indicate if target is reached

    // Constructor to initialize the rover's position and IMU subscription
    Rover() : isReached(false) {
        // Subscribe to GPS fix topic to receive current location
        gpsFixSub = nh.subscribe<sensor_msgs::NavSatFix>(
            "/fix", 10, boost::bind(&Rover::gpsFixCallback, this, _1));
        
        // Subscribe to IMU topic to receive yaw
        imuSub = nh.subscribe<sensor_msgs::Imu>(
            "/imu", 10, boost::bind(&Rover::imuCallback, this, _1));
    }

    // Callback function for receiving GPS fix
    void gpsFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        // Update rover's current location based on received GPS fix
        currentLatitude = msg->latitude;
        currentLongitude = msg->longitude;
    }

    // Callback function for receiving IMU data
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        double roll, pitch; // Declare roll and pitch variables
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->orientation, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, currentYaw);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber gpsFixSub;
    ros::Subscriber imuSub;
};

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "att");
    ros::NodeHandle nh;

    // Create a rover instance
    Rover rover;

    // Wait for initial GPS fix to set the rover's initial position
    ros::Rate loop_rate(10); // 10 Hz
    // while (ros::ok() ) {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    // Publish Twist messages to move the rover
    ros::Publisher cmdVelPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Input target GPS coordinates
    double targetLatitude, targetLongitude;
    std::cout << "Enter target latitude: ";
    std::cin >> targetLatitude;

    // Flush the input buffer
    // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    std::cout << "Enter target longitude: ";
    std::cin >> targetLongitude; 

    const double tolerance = 0.1; // Tolerance value for latitude and longitude

    // Loop to continuously publish Twist messages
    while (ros::ok() && (!rover.isReached)) {
        // Get the current location and yaw of the rover
        double currentLatitude = rover.currentLatitude;
        double currentLongitude = rover.currentLongitude;
        double currentYaw = rover.currentYaw;

        // Calculate distance to target
        double distance = calculateDistance(targetLatitude, targetLongitude, currentLatitude, currentLongitude);

        // Check if rover has reached target position
        if (distance < 0.1) {
            std::cout << "Rover reached target position." << std::endl;
            geometry_msgs::Twist twistMsg;
            twistMsg.linear.x = 0.0;
            twistMsg.angular.z = 0.0;
            cmdVelPub.publish(twistMsg);
            break; // Exit the loop
        }

        // Check if rover has reached target latitude and longitude
        if (std::abs(currentLatitude - targetLatitude) < tolerance &&
            std::abs(currentLongitude - targetLongitude) < tolerance) {
            rover.isReached = true;
        }

        // Calculate initial bearing from rover's current position to target
        double initialBearing = calculateBearing(currentLatitude, currentLongitude, targetLatitude, targetLongitude);

        // Calculate angle error between initial bearing and current yaw
        double angleError = initialBearing - toDegrees(currentYaw);
        if (angleError > 180.0) {
            angleError -= 360.0;
        } else if (angleError < -180.0) {
            angleError += 360.0;
        }

        // Adjust rover's orientation towards target
        geometry_msgs::Twist twistMsg;
        twistMsg.linear.x = 0.1; // Linear velocity (m/s)
        twistMsg.angular.z = angleError * 0.01; // Angular velocity (rad/s)
        
        cmdVelPub.publish(twistMsg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
