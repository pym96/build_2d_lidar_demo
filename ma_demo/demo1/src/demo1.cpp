#include "demo1/demo1.hpp"

LaserScan::LaserScan(): private_node_("~")
{
    // Represented by green color in terminal
    ROS_INFO_STREAM("\033[1;32m----> Feature Extraction Started. \033]0m");

    laser_scan_subscriber_ = node_handle_.subscribe("laser_scan", 1, &LaserScan::ScanCallBack, this);

}

LaserScan::~LaserScan(){

}

// Callback
void LaserScan::ScanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_msgs){

    ROS_INFO_STREAM(
        "sequence" << scan_msgs->header.seq <<
        ", time stamp: " << scan_msgs->header.stamp <<
        ",frame_id: " << scan_msgs->header.frame_id <<
        ",angle_min: " << scan_msgs->angle_min <<
        " ,angle_max: " << scan_msgs->angle_max << 
        ",angle_increment: " << scan_msgs->angle_increment <<
        ",scan_time: " << scan_msgs->scan_time <<
        ",range_min: " << scan_msgs->range_min <<
        ",range_max: " << scan_msgs->range_max <<
        ",range_size: " << scan_msgs->ranges.size() <<
        ",intensity: " << scan_msgs->intensities.size());

    // The third points Eucilean frame
    double range = scan_msgs->ranges[2];
    double angle = scan_msgs->angle_min + scan_msgs->angle_increment * 2;
    double x = range * cos(angle);
    double y = range * sin(angle);
    
    ROS_INFO_STREAM(
        "range = " << range<< ", angle=" << angle <<
        ", x= " << x << ", y= " << y;
    );

}

