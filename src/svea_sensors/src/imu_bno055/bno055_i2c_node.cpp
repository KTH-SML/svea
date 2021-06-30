/* bno055_i2c_node.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 *
 * Instantiates a BNO055I2C Activity class, as well as
 * a Watchdog that causes this node to die if things aren't
 * working.
 */

#include <imu_bno055/bno055_i2c_activity.h>
#include <imu_bno055/watchdog.h>
#include <cstdlib>
#include <ros/ros.h>
#include <csignal>

int main(int argc, char *argv[]) {
    
    ros::init(argc, argv, "bno055_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    imu_bno055::BNO055I2CActivity activity(nh, nh_priv);
    watchdog::Watchdog watchdog;

    activity.start();
    watchdog.start(5000);

    int param_rate;
    nh_priv.param("rate", param_rate, (int)400);

    ros::Rate rate(param_rate);
    // Hack to give the poor IMU some more time to initialize
    rate.sleep();
    rate.sleep();
    rate.sleep();
    rate.sleep();
    rate.sleep();
    rate.sleep();
    rate.sleep();
    rate.sleep();
    // Main loop
    while(ros::ok()) {
        if(activity.spinOnce()) {
            watchdog.refresh();
        }
        ros::spinOnce();
        rate.sleep();
    }
    activity.stop();
    watchdog.stop();
     
    return EXIT_SUCCESS;
}
