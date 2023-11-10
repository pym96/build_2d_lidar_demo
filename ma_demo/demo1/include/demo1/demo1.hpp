#ifndef __DEMO1_HPP__
#define __DEMO1_HPP__

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <map>
#include <vector>
#include <chrono>

#define MAX_SCAN_COUNT 1500

struct smoothness_t
{
    float value;
    size_t index;
};

// Computed in the compiling time
struct by_value {
    constexpr bool operator()(const smoothness_t& left, const smoothness_t& right) const {
        return left.value < right.value;
    }
};


class LaserScan
{
    private:
        ros::NodeHandle node_handle_;
        ros::NodeHandle private_node_;
        ros::Subscriber laser_scan_subscriber_;
        ros::Publisher feature_scan_publisher_;

        float edge_threshold_;
    
    public:

        LaserScan();
        ~LaserScan();
        
        LaserScan (const LaserScan&) = delete;
        LaserScan& operator = (const LaserScan& ) = delete;  
    
    public:
        void ScanCallBack(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
        
};



#endif