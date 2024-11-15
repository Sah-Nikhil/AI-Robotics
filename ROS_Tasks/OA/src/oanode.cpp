#include <iostream>
#include <cmath>
#include <bits/stdc++.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

class gpscalc{

    private:
        struct coords_init{
            float init_lat; 
            float init_long;
        };
        struct coords_final{
            float fin_lat; 
            float fin_long;
        };
        float fin_lat; 
        float fin_long;
        float d;
        float bearing;
        float earthRad = 6371000.0;
        float phi1;
        float phi2;
        float dphi;
        float dlamda;
        struct gps_coords{
            float current_lat; 
            float current_long;
        };   

    public:
        gps_coords coords;
        coords_init c;
        void set_current_lat_lon(float lat,float lon){
            coords.current_lat=lat;
            coords.current_long=lon;
        }
        void set_fin_lat_lon(float lat,float lon){
            fin_lat=lat;
            fin_long=lon;
        }
        void set_init_lat_lon(float lat,float lon){
            c.init_lat=lat;
            c.init_long=lon;
        }
        
        float get_fin_lat(){
            return fin_lat;
        }
        float get_fin_long(){
            return fin_long;
        }
        float get_distance(float current_lat, float current_long){
            phi1 = current_lat * M_PI/180;
            phi2 = fin_lat * M_PI/180;
            dphi = (fin_lat-current_lat) * M_PI/180;
            dlamda = (fin_long-current_long) * M_PI/180;
            float a = sin(dphi/2) * sin(dphi/2) + cos(phi1) * cos(phi2) *sin(dlamda/2) * sin(dlamda/2);
            float c = 2 * atan2(sqrt(a), sqrt(1-a));
            d = earthRad * c;
            return d;            
        }
        float get_angle(float current_lat, float current_long){
            phi1 = current_lat * M_PI/180;
            phi2 = fin_lat * M_PI/180;
            dphi = (fin_lat-current_lat) * M_PI/180;
            dlamda = (fin_long-current_long) * M_PI/180;
            float y = sin(dlamda)*cos(phi2);
            float x = cos(phi1)*sin(phi2) - sin(phi1)*cos(phi2)*cos(dlamda);
            float theta = M_PI_2-atan2(y, x);
            bearing = fmod((theta*180/M_PI + 360),360);
            return bearing;
        }       
};

class imucalc{

    private:
        struct Quaternion {
            double w, x, y, z;
        };

        struct EulerAngles {
            double roll, pitch, yaw;
        };

        float yaw_c; 
        
    public:
        Quaternion values;
        EulerAngles angle;

        float get_yaw_c(){
            return yaw_c;
        }
        void set_yaw_c(float y){
            yaw_c=y;
        }

        EulerAngles ToEulerAngles(Quaternion q) {
            EulerAngles angles;

            double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
            double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
            angles.roll = std::atan2(sinr_cosp, cosr_cosp);

            double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
            double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
            angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

            double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
            angles.yaw = std::atan2(siny_cosp, cosy_cosp);

            return angles;
        }
};

class lidarcalc{

    private:
    public:
        vector<float> regions;
};

class navigation{
    private:
        ros::Subscriber imu_sub;
        ros::Subscriber gps_sub;
        ros::Subscriber lidar_sub;
        ros::Publisher cmd_vel_pub;
    public:
        navigation(ros::NodeHandle *n){
            imu_sub = n->subscribe("/imu", 100, &navigation::imuCallback,this);
            gps_sub = n->subscribe("/fix", 100, &navigation::gpsCallback,this);
            lidar_sub = n->subscribe("/scan", 100, &navigation::lidarCallback,this);
            cmd_vel_pub = n->advertise<geometry_msgs::Twist>("/cmd_vel", 100);
        }
        gpscalc gps;
        imucalc i;
        lidarcalc l;

        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
            i.values.x=msg->orientation.x;
            i.values.y=msg->orientation.y;
            i.values.z=msg->orientation.z;
            i.values.w=msg->orientation.w;
        }
        void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
            gps.coords.current_lat=msg->latitude;
            gps.coords.current_long=msg->longitude;
        }

        void set_final_coords(){
            float lat_fin,long_fin;
            cout<<"Enter coordinates(lat && long): "<<endl;
            cin>>lat_fin;
            cin>>long_fin;
            gps.set_fin_lat_lon(lat_fin,long_fin);
        }
        geometry_msgs::Twist att(){
            geometry_msgs::Twist msg;
        
            i.angle=i.ToEulerAngles(i.values);        
            float cur_dist=gps.get_distance(gps.coords.current_lat,gps.coords.current_long);
            float target_rad=gps.get_angle(gps.coords.current_lat,gps.coords.current_long)*M_PI/180;

            if (target_rad>=M_PI){
                target_rad=target_rad-2*M_PI;
            }

            i.set_yaw_c(i.angle.yaw);  
            
            if (abs(cur_dist)<=0.1){
                msg.linear.x=0;
            }
            else if (abs(target_rad-i.get_yaw_c())<=0.1){
                msg.linear.x=0.1*abs(cur_dist);
                msg.angular.z=0;
            }
            else{
                msg.angular.z=2.5*(target_rad-i.get_yaw_c());
            }

            return(msg);
        }

        geometry_msgs::Twist turnleft(){
            geometry_msgs::Twist msg;
            msg.linear.x=0;
            msg.angular.z=0.5;
            return(msg);       
        }
        geometry_msgs::Twist turnright(){
            geometry_msgs::Twist msg;
            msg.linear.x=0;
            msg.angular.z=-0.5;
            return(msg);       
        }

        geometry_msgs::Twist wallfollow(){
            geometry_msgs::Twist msg;
            msg.angular.x=0;
            msg.linear.x=0.5;
            return(msg);
        }
        
        geometry_msgs::Twist setstate(int state){
            geometry_msgs::Twist msg;
            if (state==0){
                msg=att();
            }
            else if (state==1){
                msg=turnleft();
            }
            else if (state==2){
                msg=wallfollow();
            }
            else if (state==3){
                msg=turnright();
            }
            return msg;
        }

        geometry_msgs::Twist detect(){
            geometry_msgs::Twist msg; 
            string obstacle;
            float d=1.25;
            
            if (gps.get_distance(gps.coords.current_lat,gps.coords.current_long) < d){
                msg=setstate(0);
            }
            else{
                
                if (l.regions.at(0)>d && l.regions.at(1)>d && l.regions.at(2)>d && l.regions.at(3)>d){
                    obstacle="No Obstacles";
                    msg=setstate(0);
                }
                else if (l.regions.at(0)<1 && l.regions.at(1)>d && l.regions.at(2)>d && l.regions.at(3)>d){
                    obstacle="Obs found: Back_Right";
                    msg=setstate(3);
                }
                else if (l.regions.at(0)>0.75 && l.regions.at(0)<1.25 && l.regions.at(1)>d && l.regions.at(2)>d && l.regions.at(3)>d){
                    obstacle="Obs found: Back_Right";
                    msg=setstate(2);
                }
                else if (l.regions.at(1)>d && l.regions.at(2)<d && l.regions.at(3)>d){
                    obstacle="Obs found: Front";
                    msg=setstate(1);
                }
                else if (l.regions.at(1)<d && l.regions.at(2)>d && l.regions.at(3)>d){
                    obstacle="Obs found: Front_Right";
                    msg=setstate(2);
                }
                else if (l.regions.at(1)>d && l.regions.at(2)>d && l.regions.at(3)<d){
                    obstacle="Obs found: Front_Left";
                    msg=setstate(1);
                }
                else if (l.regions.at(1)<d && l.regions.at(2)<d && l.regions.at(3)>d){
                    obstacle="Obs found: Front and Front_Right";
                    msg=setstate(1);
                }
                else if (l.regions.at(1)>d && l.regions.at(2)<d && l.regions.at(3)<d){
                    obstacle="Obs found: Front and Front_Left";
                    msg=setstate(1);
                }
                else if (l.regions.at(1)<d && l.regions.at(2)>d && l.regions.at(3)<d){
                    obstacle="Obs found: Front_Left and Front_Right";
                    msg=setstate(2);
                }
                else if (l.regions.at(1)<d && l.regions.at(2)<d && l.regions.at(3)<d){
                    obstacle="Obs found: Front_Left, Front and Front_Right";
                    msg=setstate(1);
                }
                
            }
            ROS_INFO("%s",obstacle.c_str());
            return(msg);
        }

        void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
            geometry_msgs::Twist publish_msg;
            const float max=10;
            l.regions = {
                min(*min_element(msg->ranges.begin(),msg->ranges.begin()+108),max),
                min(*min_element(msg->ranges.begin()+108,msg->ranges.begin()+216),max),
                min(*min_element(msg->ranges.begin()+216,msg->ranges.begin()+324),max),
                min(*min_element(msg->ranges.begin()+324,msg->ranges.begin()+432),max),
                min(*min_element(msg->ranges.begin()+432,msg->ranges.begin()+540),max)          
            };
            publish_msg=detect();
            cmd_vel_pub.publish(publish_msg);
        }
};

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "oanode");
    ros::NodeHandle n;
    navigation robot=navigation(&n);
    robot.set_final_coords();    
    ros::spin();
    return 0;
}