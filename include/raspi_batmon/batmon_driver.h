//
// Created by nico on 5/11/20.
//

#ifndef RASPI_BATMON_BATMON_DRIVER_H
#define RASPI_BATMON_BATMON_DRIVER_H

#include "raspi_batmon/smbus.h"
#include <iostream>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/BatteryState.h>
#include <vector>
#include <string>
#include <sstream>
#include <ros/console.h>

class batmon_driver {
public:
    batmon_driver(int adapter_nr, int addr);

    float get_voltage();

    float get_current();

    float get_charge();

    float get_capacity();

    float get_percentage();

    unsigned int get_status();

    unsigned int get_health();

    unsigned int get_technology();

    bool is_present();

    std::vector<float> get_cell_voltage();

    std::string get_serialnumber();
//    void setup(int adapter_nr);
private:
    __s32 read_16bit_word(int reg);

//    int file;
    int file_num;
    int addr;
};

#endif //RASPI_BATMON_BATMON_DRIVER_H
