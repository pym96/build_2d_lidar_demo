#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <map>
#include <vector>
#include <chrono>
#include "demo1/demo1.hpp"

int main(int argc, char** argv){


    ros::init(argc, argv, "demo1");


    LaserScan laser_scan;

    ros::spin();

    return 0;
}