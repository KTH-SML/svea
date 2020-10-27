/* speed_coefficient_estimation.cpp
 * Author: Tobias Bolin <tbolin@kth.se>
 *
 * A node for online estimation of the speed coeffcient for the ESC
 */
 
#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "svea_msgs/lli_ctrl.h"

#include <limits>
#include <cmath>
#include <cstdlib>
#include <csignal>

class SpeedCoefficientEstimator
{
public: 
    SpeedCoefficientEstimator(double estimation_smoothing, 
                              double initial_guess,
                              double input_smoothing,
                              unsigned int dead_zone=10) 
    : estimation_smoothing(estimation_smoothing) 
    , speed_coefficient(initial_guess)
    , input_smoothing(input_smoothing)
    , dead_zone(dead_zone)
    , is_initialized(false)
    , speed_ctrl_input(0.0)
    , speed(std::numeric_limits<double>::infinity())
    , speed_last_update(ros::Time::now())
    , input_last_update(ros::Time::now())
      {
        coefficient_pub = nh.advertise<std_msgs::Float64>("/estimations/speed_coefficient", 10);
        odom_sub = nh.subscribe("/odometry/filtered", 10, &SpeedCoefficientEstimator::odometryCallback, this);
        control_sub = nh.subscribe("/lli/ctrl_actuated", 10, &SpeedCoefficientEstimator::lliCtrlCallback, this);
    }
    
    double calcGain(ros::Time last_update, double nominal_gain)
    {
        ros::Time now = ros::Time::now();
        double dt = (now - last_update).toSec();
        double dt_gain = tanh(nominal_gain * dt);  
        return dt_gain;
    }
    
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        double x_vel = msg->twist.twist.linear.x;
        double y_vel = msg->twist.twist.linear.y;
        double z_vel = msg->twist.twist.linear.z;
        double speed = pow(x_vel, 2.0) + pow(y_vel, 2.0) + pow(z_vel, 2.0);
        if (is_initialized == false){
            if (speed_ctrl_input != 0.0) {
                is_initialized = true; 
            }
        }
    }
    
    void lliCtrlCallback(const svea_msgs::lli_ctrl::ConstPtr& msg)
    {
        double new_input = static_cast<double>(msg->velocity);
        if (abs(new_input) < dead_zone) {
            new_input = 0;
        }
        else if (new_input > dead_zone) {
            new_input = new_input - dead_zone;
        }
        else if (new_input < -dead_zone) {
            new_input = new_input + dead_zone;
        }
        double dt_gain = calcGain(input_last_update, input_smoothing);
        input_last_update = ros::Time::now();
        speed_ctrl_input = dt_gain*new_input + (1-dt_gain)*speed_ctrl_input;
    }
    
    double getSpeedCoefficient()
    {   
        if (is_initialized == true){
            double dt_gain = calcGain(speed_last_update, estimation_smoothing);
            speed_last_update = ros::Time::now();
            speed_coefficient = dt_gain*(speed/speed_ctrl_input) + (1-dt_gain)*speed_coefficient;
        }   
        return speed_coefficient;
    }
    
    void publishSpeedCoefficient()
    {   
        std_msgs::Float64 msg;
        msg.data = getSpeedCoefficient();
        coefficient_pub.publish(msg); 
    }

private:
    ros::Publisher coefficient_pub;
    ros::Subscriber odom_sub;
    ros::Subscriber control_sub;
    ros::NodeHandle nh; 
    double speed_coefficient;
    double estimation_smoothing;
    double input_smoothing;
    double speed_ctrl_input; 
    double speed;
    double initial_guess;
    bool is_initialized;
    ros::Time speed_last_update;
    ros::Time input_last_update;
    unsigned int dead_zone;
};

int main(int argc, char *argv[]) {
    
    ros::init(argc, argv, "speed_coefficient_estimator");
    ros::NodeHandle nh_priv = ros::NodeHandle("~");
    
    double pub_rate;
    nh_priv.getParam("publish_rate", pub_rate);
    ros::Rate rate(pub_rate);
    
    double estimation_smoothing;
    nh_priv.getParam("estimation_smoothing", estimation_smoothing);
    
    double initial_guess = 3.6/115;
    
    double input_smoothing;
    nh_priv.getParam("input_smoothing", input_smoothing);
    
    SpeedCoefficientEstimator speedEstimator(estimation_smoothing, initial_guess, input_smoothing); 
    while (ros::ok()) {
        speedEstimator.publishSpeedCoefficient();
        rate.sleep();
        ros::spinOnce();
    }
    return EXIT_SUCCESS;
}
