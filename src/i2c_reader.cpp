#include "raspi_batmon/batmon_driver.h"
#include "raspi_batmon/smbus.h"
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <errno.h>
#include <cstring>

using namespace std;
/**
 * A ROS Node that pulls data from batmon and publishes it to /BatteryMessage_{serial number}
 * @author Nico Bartholomai
 */

ros::Publisher state_pub;

int main(int argc, char **argv) {
//makes a batmon_driver with address_nr 3
    batmon_driver bat(3, I2CADDRESS3);

//initializing ross stuff
    ros::init(argc, argv, "raspi_batmon");
    ros::NodeHandle nh;

//making a battery messege
    sensor_msgs::BatteryState msg;
    std::string topic = "/BatteryState_" + bat.get_serialnumber();
    state_pub = nh.advertise<sensor_msgs::BatteryState>(topic, 100); //msg buffer of 100
    ros::Rate loop_rate(10); //sets up loop to run at 10hz

//add all the static contents of the message
    msg.serial_number = bat.get_serialnumber();
    msg.power_supply_technology = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    msg.power_supply_status = POWER_SUPPLY_STATUS_UNKNOWN;
    msg.power_supply_health = POWER_SUPPLY_HEALTH_UNKNOWN;

    int count = 0;
    while (ros::ok()) {
        //update all the variable contents of the message
        msg.header.stamp = ros::Time::now();
        msg.voltage = bat.get_voltage();
        msg.current = bat.get_current();
        msg.capacity = bat.get_capacity();
        msg.percentage = bat.get_percentage();
        msg.cell_voltage = bat.get_cell_voltage();
        msg.present = (char) bat.is_present();
        msg.charge = bat.get_charge();
        msg.design_capacity = bat.get_capacity();
        //publish the message
        state_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;

}
