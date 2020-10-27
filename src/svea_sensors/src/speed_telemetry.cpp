/* speed_telemetry.cpp
 * Author: Tobias Bolin <tbolin@kth.se>
 *
 * A node that calculates the speed based on the velocity publsihed on 
 * the /odometry/filtered topic
 */

#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"

#include <cmath>
#include <cstdlib>
#include <csignal>

class SpeedTelemetry
{
public: 
    SpeedTelemetry(){
        speed_pub = nh.advertise<geometry_msgs::TwistStamped>("/telemetry/speed", 10);
        odom_sub = nh.subscribe("/odometry/filtered", 10, &SpeedTelemetry::odometryCallback, this);
    }
    
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        double x_vel = msg->twist.twist.linear.x;
        double y_vel = msg->twist.twist.linear.y;
        double z_vel = msg->twist.twist.linear.z;
        
        double speed = pow(x_vel, 2.0) + pow(y_vel, 2.0) + pow(z_vel, 2.0);
        speed = sqrt(speed);
        geometry_msgs::TwistStamped speed_msg;
        speed_msg.twist.linear.x = speed;
        
        speed_msg.header = msg->header;
        speed_pub.publish(speed_msg);
    } 
    
private:
    ros::Publisher speed_pub;
    ros::Subscriber odom_sub;
    ros::NodeHandle nh;  
};

int main(int argc, char *argv[]) {
    
    ros::init(argc, argv, "speed_telemetry");
    ros::NodeHandle nh_priv = ros::NodeHandle("~");
    SpeedTelemetry telemetryObj; 

    ros::spin();
    return EXIT_SUCCESS;
}
